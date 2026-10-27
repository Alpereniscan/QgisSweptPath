# Simulator class
import time

from threading import Thread
from typing import Optional

from qgis.PyQt.QtCore import QObject, pyqtSignal, QTimer, QEvent
from qgis.gui import QgsMapCanvas
from qgis.PyQt.QtGui import QKeySequence
from qgis.core import Qgis

from .qgis_swept_path_dockwidget_prop import QgisSweptPathDockWidgetProp
from .vehicle import Vehicle
from .qgis_swept_path_enum import SimulationMode


class Simulator(QObject):
    # Signals
    drawVehicle: pyqtSignal = pyqtSignal(name="drawVehicle")
    storePath: pyqtSignal = pyqtSignal(name="storePath")

    def __init__(self, canvas: QgsMapCanvas):
        """Constructor"""
        super().__init__(canvas)
        self._canvas = canvas

        self._vehicle: Vehicle = None
        self._prop: QgisSweptPathDockWidgetProp = None

        self._simulation_running: bool = False
        self._simulation_paused: bool = False
        self._simulation_timer: QTimer = QTimer(self)
        self._simulation_timer.timeout.connect(self._simulate_frame)

        # Number of frames to steer from center to full steering angle
        self._frames_half_steering: float = None
        # Time between steps in seconds
        self._time_between_steps: float = None
        # Counter for distance driven (used for printing path points with frame based simulation)
        self._distance_counter: float = 0.0
        # Counter for steps (used for printing path points with step based simulation)
        self._step_counter: int = 0

        # Flags for steering and speed change
        self._steer_right: bool = False
        self._steer_left: bool = False
        self._speed_up: bool = False
        self._speed_down: bool = False
        # Automatically return steering toward centre when no steering key is pressed
        # TODO: Allow changing this during a running simulation. The plugin currently
        #  locks the checkbox while the simulation is running.
        self._auto_steer: bool = True

    def eventFilter(self, caller: QObject, event: QEvent):
        if event.type() == QEvent.KeyPress:
            if event.key() == QKeySequence(self._prop.key_steer_left):
                self._steer_left = True
                return True
            if event.key() == QKeySequence(self._prop.key_steer_right):
                self._steer_right = True
                return True
            if event.key() == QKeySequence(self._prop.key_speed_up):
                self._speed_up = True
                return True
            if event.key() == QKeySequence(self._prop.key_speed_down):
                self._speed_down = True
                return True
            return False

        if  event.type() == QEvent.KeyRelease:
            if event.key() == QKeySequence(self._prop.key_steer_left):
                self._steer_left = False
                return True
            if event.key() == QKeySequence(self._prop.key_steer_right):
                self._steer_right = False
                return True
            if event.key() == QKeySequence(self._prop.key_speed_up):
                self._speed_up = False
                return True
            if event.key() == QKeySequence(self._prop.key_speed_down):
                self._speed_down = False
                return True
            return False

        return False


    def startSimulation(self):
        self._simulation_running = True
        if self._prop.simulation_mode == SimulationMode.FRAME_BASED:
            self._frames_half_steering = self._prop.steering_time * 0.5 * self._prop.frames
            self._time_between_steps = float(1.0 / self._prop.frames)
            self._distance_counter = 0.0
            self.storePath.emit()
            self._simulation_timer.start(int(self._time_between_steps * 1000))

        elif self._prop.simulation_mode == SimulationMode.STEP_BASED:
            self._step_counter = 0
            self.storePath.emit()
            t = Thread(target=self._simulate_step, args=())
            t.start()

        else:
            self.iface.messageBar().pushMessage(
                "Can't start simulation",
                "Unknown simulation mode",
                level=Qgis.Critical
            )


    def stopSimulation(self):
        if self._prop.simulation_mode == SimulationMode.FRAME_BASED:
            self._simulation_timer.stop()

        self._simulation_running = False
        self._simulation_paused = False

        # Reset flags for steering and speed change
        self._steer_right = False
        self._steer_left = False
        self._speed_up = False
        self._speed_down = False

        self.drawVehicle.emit()
        self.storePath.emit()


    def pauseResumeSimulation(self) -> bool:
        self._simulation_paused = not self._simulation_paused
        return self._simulation_paused


    def speedUp(self):
        if self._prop.simulation_mode == SimulationMode.STEP_BASED:
            self.vehicle.speed_up(self._prop.speed_change_step)


    def speedDown(self):
        if self._prop.simulation_mode == SimulationMode.STEP_BASED:
            self.vehicle.speed_down(self._prop.speed_change_step)


    def steerRight(self):
        if self._prop.simulation_mode == SimulationMode.STEP_BASED:
            self.vehicle.steer_right(self._prop.steer_change_step)


    def steerLeft(self):
        if self._prop.simulation_mode == SimulationMode.STEP_BASED:
            self.vehicle.steer_left(self._prop.steer_change_step)

    def _simulate_frame(self):
        if self._simulation_paused or self._canvas.isDrawing():
            # Skip simulation if simulation is paused or canvas is drawing
            return

        if self._speed_up:
            self._vehicle.speed_up(
                self._time_between_steps * self._prop.acceleration
            )

        if self._speed_down:
            self._vehicle.speed_down(
                self._time_between_steps * self._prop.acceleration
            )

        if self._steer_right:
            self._vehicle.steer_right(
                self._vehicle.max_steering_angle /
                self._frames_half_steering
            )

        elif self._steer_left:
            self._vehicle.steer_left(
                self._vehicle.max_steering_angle /
                self._frames_half_steering
            )

        elif self._auto_steer:
            # Automatically return steering toward centre
            steering_return = (
                    self._vehicle.max_steering_angle /
                    self._frames_half_steering
            )

            if self._vehicle.steering_angle > 0:
                self._vehicle.steer_right(
                    min(steering_return, self._vehicle.steering_angle)
                )

            elif self._vehicle.steering_angle < 0:
                self._vehicle.steer_left(
                    min(steering_return, abs(self._vehicle.steering_angle))
                )

        drive_distance = (
                self._time_between_steps * self._vehicle.speed
        )

        # Use absolute speed for the movement threshold.
        # Keep drive_distance signed so negative speed reverses the vehicle.
        if abs(self._vehicle.speed) > self._prop.minimum_speed:
            self._vehicle.step(drive_distance)

            # Path printing must count travelled distance regardless of direction.
            self._distance_counter += abs(drive_distance)

            if self._distance_counter >= self._prop.print_distance:
                self._distance_counter = 0.0
                self.storePath.emit()

        self.drawVehicle.emit()

    def _simulate_step(self):
        while self._simulation_running:
            if self._simulation_paused or self._canvas.isDrawing():
                # Prevent the loop from consuming a CPU core while waiting.
                time.sleep(0.01)
                continue

            if abs(self._vehicle.speed) > self._prop.minimum_speed:
                # Apply the speed sign to the configured step distance.
                direction = 1.0 if self._vehicle.speed > 0.0 else -1.0
                drive_distance = self._prop.step_distance * direction

                self._vehicle.step(drive_distance)

                self._step_counter += 1

                if self._step_counter >= self._prop.print_interval:
                    self._step_counter = 0
                    self.storePath.emit()

                self.drawVehicle.emit()

                # Sleep duration must always be positive.
                time.sleep(
                    self._prop.step_distance / abs(self._vehicle.speed)
                )
            else:
                # Avoid a busy loop while the vehicle is stationary.
                time.sleep(0.01)

    @property
    def simulation_running(self) -> bool:
        return self._simulation_running


    @property
    def simulation_paused(self) -> bool:
        return self._simulation_paused


    @property
    def properties(self) -> QgisSweptPathDockWidgetProp:
        return self._prop


    @properties.setter
    def properties(self, v: QgisSweptPathDockWidgetProp):
        self._prop = v


    @property
    def vehicle(self) -> Vehicle:
        return self._vehicle


    @vehicle.setter
    def vehicle(self, v: Vehicle):
        self._vehicle = v

    @property
    def auto_steer(self) -> bool:
        return self._auto_steer


    def setAutoSteer(self, enabled: bool):
        self._auto_steer = bool(enabled)
