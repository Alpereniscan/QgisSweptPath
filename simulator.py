# Simulator class
import math
import time

from threading import Thread
from typing import Optional

from qgis.PyQt.QtCore import QObject, pyqtSignal, QTimer, QEvent
from qgis.gui import QgsMapCanvas
from qgis.PyQt.QtGui import QKeySequence
from qgis.core import Qgis
from qgis.utils import iface

from .qgis_swept_path_dockwidget_prop import QgisSweptPathDockWidgetProp
from .vehicle import Vehicle
from .qgis_swept_path_enum import SimulationMode
from .route_follow import Geom, Pilot, Tuning


class Simulator(QObject):
    # Signals
    drawVehicle: pyqtSignal = pyqtSignal(name="drawVehicle")
    storePath: pyqtSignal = pyqtSignal(name="storePath")
    # Emitted when route following ends. True = reached the end, False = cancelled.
    routeEnded: pyqtSignal = pyqtSignal(bool, name="routeEnded")
    # Emitted when the route turns out to be impossible to drive. Argument = reason.
    routeFailed: pyqtSignal = pyqtSignal(str, name="routeFailed")

    # ------------------------------------------------------------------
    # Manual control tuning
    # ------------------------------------------------------------------
    # False = AUTO STEERING: wheel returns to centre when no steering key is held
    #         (your original behaviour).
    # True  = wheel stays where you leave it; press the centre key to straighten.
    STEER_HOLD_ON_RELEASE: bool = False
    # Auto-centre speed relative to normal steering speed (1.0 = your original rate)
    STEER_RETURN_FACTOR: float = 1.0
    # Steering speed multiplier at first key press and after full ramp-up
    STEER_RAMP_START: float = 0.35
    STEER_RAMP_END: float = 1.5
    # Seconds of holding a steering key to reach STEER_RAMP_END
    STEER_RAMP_TIME: float = 1.0
    # Steering gets slower as |speed| rises: scale = 1 / (1 + |speed| / REF)
    STEER_SPEED_REF: float = 10.0
    STEER_SPEED_MIN_SCALE: float = 0.3
    # Brake strength relative to normal acceleration
    BRAKE_FACTOR: float = 3.0
    # Fallback keys if the properties object has no key_brake / key_center
    DEFAULT_KEY_BRAKE: str = "Space"
    DEFAULT_KEY_CENTER: str = "C"

    # ------------------------------------------------------------------
    # Route following (units = map units; speed is map units / s)
    # Path shape / steering / failure tuning lives in route_follow.Tuning
    # ------------------------------------------------------------------
    # True = the wheel jumps straight to the angle the route needs (stop-start assumption,
    # like Autodesk Vehicle Tracking). False = it turns at the normal steering speed.
    ROUTE_STEER_INSTANT: bool = True
    # Steering speed in route mode when ROUTE_STEER_INSTANT is False
    ROUTE_STEER_RATE: float = 1.0
    # Speed along the route (overridden by prop.route_speed / prop.route_reverse_speed if they exist)
    ROUTE_CRUISE_SPEED: float = 3.0
    ROUTE_REVERSE_SPEED: float = 1.5
    # Never slow below this while on the route (so the vehicle does not stall)
    ROUTE_MIN_SPEED: float = 0.5
    # Standstill time at a cusp (direction change), seconds
    ROUTE_CUSP_PAUSE: float = 0.6
    # Pressing a steer / speed key during a route hands control back to you
    ROUTE_MANUAL_OVERRIDE: bool = True
    # No progress along the route for this long (seconds) = route not possible
    ROUTE_STALL_TIME: float = 8.0

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
        self._brake: bool = False
        self._center_steering: bool = False

        # Frames the current steering key has been held (for ramp-up)
        self._steer_hold_frames: int = 0

        # Route following state
        self._route_active: bool = False
        self._pilot: Pilot = None
        self._geom: Geom = None
        self._wait_frames: int = 0
        self._stall_frames: int = 0
        self._stall_ref: tuple = (-1, -1.0)

    # ------------------------------------------------------------------
    # Key handling
    # ------------------------------------------------------------------
    @staticmethod
    def _key_code(seq) -> int:
        """Return the plain Qt key code of a key sequence (no modifiers), or -1."""
        try:
            ks = seq if isinstance(seq, QKeySequence) else QKeySequence(seq)
            if ks.count() == 0:
                return -1
            k = ks[0]
            if hasattr(k, "key"):  # PyQt6: QKeyCombination
                return int(k.key())
            return int(k) & 0x01FFFFFF  # PyQt5: strip modifier bits
        except Exception:
            return -1

    def _key_actions(self) -> dict:
        """Map Qt key code -> flag attribute name."""
        p = self._prop
        mapping = {
            self._key_code(p.key_steer_left): "_steer_left",
            self._key_code(p.key_steer_right): "_steer_right",
            self._key_code(p.key_speed_up): "_speed_up",
            self._key_code(p.key_speed_down): "_speed_down",
            self._key_code(getattr(p, "key_brake", self.DEFAULT_KEY_BRAKE)): "_brake",
            self._key_code(getattr(p, "key_center", self.DEFAULT_KEY_CENTER)): "_center_steering",
        }
        mapping.pop(-1, None)
        return mapping

    def eventFilter(self, caller: QObject, event: QEvent):
        etype = event.type()
        if etype not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False

        flag = self._key_actions().get(event.key())
        if flag is None:
            return False

        # Auto-repeat generates fake press/release pairs on some platforms.
        # Swallow them so the flags stay stable while a key is physically held.
        if event.isAutoRepeat():
            return True

        setattr(self, flag, etype == QEvent.KeyPress)
        return True

    # ------------------------------------------------------------------
    # Simulation start / stop
    # ------------------------------------------------------------------
    def startSimulation(self):
        self._simulation_running = True
        self._steer_hold_frames = 0
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
            iface.messageBar().pushMessage(
                "Can't start simulation",
                "Unknown simulation mode",
                level=Qgis.Critical
            )

    def stopSimulation(self):
        if self._prop.simulation_mode == SimulationMode.FRAME_BASED:
            self._simulation_timer.stop()

        self._simulation_running = False
        self._simulation_paused = False

        # Stopping the simulation also cancels an active route
        self.cancelRoute()

        # Reset flags for steering and speed change
        self._steer_right = False
        self._steer_left = False
        self._speed_up = False
        self._speed_down = False
        self._brake = False
        self._center_steering = False
        self._steer_hold_frames = 0

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

    # ------------------------------------------------------------------
    # Route API
    # ------------------------------------------------------------------
    @property
    def route_active(self) -> bool:
        return self._route_active

    @staticmethod
    def _normalize_runs(route) -> list:
        """
        route: [(points, direction), ...] from the route tool (direction 1 = forward,
        -1 = reverse; points are QgsPointXY or (x, y) and do NOT include the start),
        or a plain list of points (one forward run).
        """
        route = list(route)
        if (route and isinstance(route[0], (tuple, list)) and len(route[0]) == 2
                and isinstance(route[0][0], (tuple, list))):
            raw_runs = [(pts, d) for pts, d in route]
        else:
            raw_runs = [(route, 1)]

        runs = []
        for pts_in, direction in raw_runs:
            pts = []
            for p in pts_in:
                if callable(getattr(p, "x", None)):
                    xy = (float(p.x()), float(p.y()))
                else:
                    xy = (float(p[0]), float(p[1]))
                if not pts or math.hypot(xy[0] - pts[-1][0], xy[1] - pts[-1][1]) > 1e-9:
                    pts.append(xy)
            if pts:
                runs.append({"dir": -1 if direction < 0 else 1, "pts": pts})
        return runs

    def setRoute(self, route) -> bool:
        """
        Arm route following with a hand-drawn route. Returns True if the route was accepted.
        If the simulation is already running the vehicle starts following immediately;
        otherwise it starts as soon as the simulation starts.
        """
        runs = self._normalize_runs(route)
        if not runs or self._vehicle is None:
            iface.messageBar().pushMessage(
                "Route", "A route needs at least one point",
                level=Qgis.Warning, duration=5
            )
            return False

        pose = self._read_pose()
        if pose is None:
            return False

        self._geom = Geom.from_vehicle(self._vehicle)
        self._pilot = Pilot(self._geom, runs, Tuning())
        self._pilot.start(*pose)
        self._wait_frames = 0
        self._stall_frames = 0
        self._stall_ref = (-1, -1.0)
        self._route_active = True
        return True

    def _clear_route(self):
        self._route_active = False
        self._pilot = None
        self._wait_frames = 0

    def cancelRoute(self):
        """Stop following the route (no-op if no route is active)."""
        if not self._route_active:
            return
        self._clear_route()
        self.routeEnded.emit(False)

    def _stop_vehicle(self):
        """Bring the vehicle to a standstill immediately."""
        v = self._vehicle
        if v.speed > 0.0:
            v.speed_down(v.speed)
        elif v.speed < 0.0:
            v.speed_up(-v.speed)

    def _finish_route(self):
        self._stop_vehicle()
        self._clear_route()
        self.routeEnded.emit(True)

    def _fail_route(self, reason: str):
        self._stop_vehicle()
        self._clear_route()
        self.routeFailed.emit(reason)

    # ------------------------------------------------------------------
    # Route following
    # ------------------------------------------------------------------
    def _read_pose(self):
        """
        Return (front_x, front_y, heading) of the vehicle: vehicle.f.x, vehicle.f.y, vehicle.a
        (radians, counter-clockwise from +X), or None.
        """
        v = self._vehicle
        try:
            return float(v.f.x), float(v.f.y), float(v.a)
        except (AttributeError, TypeError, ValueError):
            iface.messageBar().pushMessage(
                "Route following",
                "Can't read the vehicle position/heading (expected vehicle.f.x, vehicle.f.y, vehicle.a).",
                level=Qgis.Critical
            )
            return None

    def _set_steering(self, target_rad: float):
        """Steer to an absolute angle (radians). Instant unless ROUTE_STEER_INSTANT is False."""
        v = self._vehicle
        unit = self._geom.steer_unit
        max_units = v.max_steering_angle
        target = max(-max_units, min(max_units, target_rad / unit))
        diff = target - v.steering_angle

        if not self.ROUTE_STEER_INSTANT:
            step = (max_units / self._frames_half_steering) * self.ROUTE_STEER_RATE
            diff = max(-step, min(step, diff))

        if diff > 0.0:
            v.steer_left(diff)
        elif diff < 0.0:
            v.steer_right(-diff)

    def _update_route_control(self):
        pose = self._read_pose()
        if pose is None:
            self._clear_route()
            self.routeEnded.emit(False)
            return

        v = self._vehicle
        dt = self._time_between_steps
        cmd = self._pilot.update(*pose)

        if cmd.status == "done":
            self._finish_route()
            return
        if cmd.status == "fail":
            self._fail_route(cmd.reason)
            return
        if cmd.status == "cusp":
            # Stop, and keep the wheel turning to the angle needed for the next run
            self._stop_vehicle()
            self._wait_frames = max(1, int(round(self.ROUTE_CUSP_PAUSE / dt)))

        # --- steering ----------------------------------------------------------
        self._set_steering(cmd.steer)

        # --- speed -------------------------------------------------------------
        accel_step = dt * self._prop.acceleration
        decel = self._prop.acceleration * self.BRAKE_FACTOR
        floor = max(self.ROUTE_MIN_SPEED, 1.5 * self._prop.minimum_speed)

        if cmd.direction < 0:
            cruise = getattr(self._prop, "route_reverse_speed", self.ROUTE_REVERSE_SPEED)
        else:
            cruise = getattr(self._prop, "route_speed", self.ROUTE_CRUISE_SPEED)

        if cmd.stops_at_end:
            v_end = math.sqrt(2.0 * decel * max(cmd.remaining - 0.3, 0.0))
        else:
            v_end = cruise

        magnitude = max(floor, min(cruise, cmd.v_curve, v_end))

        waiting = self._wait_frames > 0
        if waiting:
            self._wait_frames -= 1
        if waiting or self._brake:
            magnitude = 0.0

        target = cmd.direction * magnitude
        dv = target - v.speed
        if abs(dv) > 1e-9:
            braking = v.speed * dv < 0.0
            amount = min(abs(dv), accel_step * (self.BRAKE_FACTOR if braking else 1.0))
            if dv > 0.0:
                v.speed_up(amount)
            else:
                v.speed_down(amount)

        # --- stalled? ------------------------------------------------------------
        if waiting or self._brake or self._simulation_paused:
            self._stall_frames = 0
        elif cmd.seg != self._stall_ref[0] or cmd.progress > self._stall_ref[1] + 0.2:
            self._stall_ref = (cmd.seg, cmd.progress)
            self._stall_frames = 0
        else:
            self._stall_frames += 1
            if self._stall_frames * dt >= self.ROUTE_STALL_TIME:
                self._fail_route("The vehicle is not making progress along the route.")

    # ------------------------------------------------------------------
    # Manual control helpers (frame based)
    # ------------------------------------------------------------------
    def _update_speed(self):
        v = self._vehicle
        dt = self._time_between_steps
        accel_step = dt * self._prop.acceleration

        if self._brake:
            # Brake smoothly towards zero from either direction, never overshoot.
            brake_step = accel_step * self.BRAKE_FACTOR
            if v.speed > 0.0:
                v.speed_down(min(v.speed, brake_step))
            elif v.speed < 0.0:
                v.speed_up(min(-v.speed, brake_step))
            return

        if self._speed_up:
            v.speed_up(accel_step)
        if self._speed_down:
            v.speed_down(accel_step)

    def _update_steering(self):
        v = self._vehicle
        base_step = v.max_steering_angle / self._frames_half_steering

        # Right = +1, left = -1; both pressed cancel out
        direction = int(self._steer_right) - int(self._steer_left)

        if self._center_steering:
            direction = 0

        if direction != 0:
            self._steer_hold_frames += 1
            held_seconds = self._steer_hold_frames * self._time_between_steps
            t = min(1.0, held_seconds / self.STEER_RAMP_TIME)
            ramp = self.STEER_RAMP_START + (self.STEER_RAMP_END - self.STEER_RAMP_START) * t

            speed_scale = max(
                self.STEER_SPEED_MIN_SCALE,
                1.0 / (1.0 + abs(v.speed) / self.STEER_SPEED_REF)
            )
            delta = base_step * ramp * speed_scale

            if direction > 0:
                v.steer_right(delta)
            else:
                v.steer_left(delta)
            return

        # No steering input
        self._steer_hold_frames = 0

        if self._center_steering:
            rate = base_step
        elif not self.STEER_HOLD_ON_RELEASE:
            # AUTO STEERING: return toward centre
            rate = base_step * self.STEER_RETURN_FACTOR
        else:
            return  # keep the current steering angle

        angle = v.steering_angle
        if angle > 0.0:
            v.steer_right(min(rate, angle))
        elif angle < 0.0:
            v.steer_left(min(rate, -angle))

    # ------------------------------------------------------------------
    # Simulation loops
    # ------------------------------------------------------------------
    def _simulate_frame(self):
        if self._simulation_paused or self._canvas.isDrawing():
            # Skip simulation if simulation is paused or canvas is drawing
            return

        if self._route_active and self.ROUTE_MANUAL_OVERRIDE and (
                self._steer_left or self._steer_right or self._speed_up or self._speed_down
        ):
            # Driver took over: leave route mode and drive manually
            self.cancelRoute()

        if self._route_active:
            self._update_route_control()
            if not self._simulation_running:
                # Route ended and the plugin stopped the simulation
                return
        else:
            self._update_speed()
            self._update_steering()

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

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
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
