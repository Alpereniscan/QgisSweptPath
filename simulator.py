# Simulator class
import math
import time

from bisect import bisect_left
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


class Simulator(QObject):
    # Signals
    drawVehicle: pyqtSignal = pyqtSignal(name="drawVehicle")
    storePath: pyqtSignal = pyqtSignal(name="storePath")
    # Emitted when route following ends. True = reached the end, False = cancelled.
    routeEnded: pyqtSignal = pyqtSignal(bool, name="routeEnded")

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
    # Route following tuning (units = map units; speed is map units / s)
    # ------------------------------------------------------------------
    # Speed along the route (overridden by prop.route_speed if it exists)
    ROUTE_CRUISE_SPEED: float = 3.0
    # Never slow below this while on the route (so the vehicle does not stall)
    ROUTE_MIN_SPEED: float = 0.5
    # Max lateral acceleration used to slow down for curves
    ROUTE_MAX_LAT_ACCEL: float = 1.5
    # Lookahead distance = MIN + GAIN * |speed|, clamped to MAX
    ROUTE_LOOKAHEAD_MIN: float = 2.0
    ROUTE_LOOKAHEAD_GAIN: float = 0.8
    ROUTE_LOOKAHEAD_MAX: float = 12.0
    # Route is considered finished when this close to the end
    ROUTE_END_TOLERANCE: float = 0.4
    # Route resampling spacing and corner smoothing (Chaikin iterations, 0 = off)
    ROUTE_SPACING: float = 0.25
    ROUTE_SMOOTHING_ITERATIONS: int = 2
    # Steering actuator speed in route mode, relative to normal steering speed
    ROUTE_STEER_RATE: float = 1.0
    # Pressing a steer / speed key during a route hands control back to you
    ROUTE_MANUAL_OVERRIDE: bool = True
    # How many route points ahead of the current one are searched for progress
    ROUTE_SEARCH_WINDOW: int = 200

    # ------------------------------------------------------------------
    # Vehicle conventions used by route following. Adjust to match vehicle.py
    # ------------------------------------------------------------------
    # "math_rad"    : radians, counter-clockwise from +X (east)
    # "math_deg"    : degrees, counter-clockwise from +X
    # "azimuth_deg" : degrees, clockwise from north
    # "azimuth_rad" : radians, clockwise from north
    HEADING_CONVENTION: str = "math_rad"
    # None = auto (max_steering_angle > 1.6 means degrees), or True / False
    #STEERING_IN_DEGREES: Optional[bool] = None
    STEERING_IN_DEGREES: Optional[bool] = False
    # Used only if the vehicle has no readable wheelbase attribute
    ROUTE_WHEELBASE_FALLBACK: float = 3.0

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
        self._route_pts: list = []
        self._route_s: list = []
        self._route_length: float = 0.0
        self._route_end_dir: tuple = (1.0, 0.0)
        self._route_idx: int = 0

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
    def _chaikin(pts: list, iterations: int) -> list:
        """Corner cutting; endpoints are kept."""
        for _ in range(iterations):
            if len(pts) < 3:
                break
            new = [pts[0]]
            for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
                new.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
                new.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
            new.append(pts[-1])
            pts = new
        return pts

    @staticmethod
    def _resample(pts: list, spacing: float) -> list:
        """Resample a polyline at a fixed spacing (last point is kept)."""
        seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])]
        total = sum(seg)
        if total <= 0.0:
            return []

        out = [pts[0]]
        i = 0
        acc = 0.0
        s = spacing
        while s < total:
            while i < len(seg) - 1 and acc + seg[i] < s:
                acc += seg[i]
                i += 1
            t = (s - acc) / seg[i] if seg[i] > 0.0 else 0.0
            a, b = pts[i], pts[i + 1]
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            s += spacing
        out.append(pts[-1])
        return out

    def setRoute(self, points) -> bool:
        """
        Arm route following with a hand-drawn route.

        points: list of QgsPointXY or (x, y) tuples in map units.
        Returns True if the route was accepted. If the simulation is already
        running the vehicle starts following immediately; otherwise it starts
        as soon as the simulation starts.
        """
        raw = []
        for p in points:
            if callable(getattr(p, "x", None)):
                xy = (float(p.x()), float(p.y()))
            else:
                xy = (float(p[0]), float(p[1]))
            if not raw or math.hypot(xy[0] - raw[-1][0], xy[1] - raw[-1][1]) > 1e-9:
                raw.append(xy)

        if len(raw) < 2:
            iface.messageBar().pushMessage(
                "Route", "A route needs at least two different points",
                level=Qgis.Warning, duration=5
            )
            return False

        pts = self._chaikin(raw, self.ROUTE_SMOOTHING_ITERATIONS)
        pts = self._resample(pts, self.ROUTE_SPACING)
        if len(pts) < 2:
            return False

        s = [0.0]
        for a, b in zip(pts[:-1], pts[1:]):
            s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

        # Direction of the last segment (used to extrapolate the lookahead target)
        ex, ey = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        el = math.hypot(ex, ey) or 1.0

        self._route_pts = pts
        self._route_s = s
        self._route_length = s[-1]
        self._route_end_dir = (ex / el, ey / el)
        self._route_idx = 0

        # Start from the route point closest to the vehicle
        pose = self._read_pose() if self._vehicle is not None else None
        if pose is not None:
            x, y, _h, _l = pose
            self._route_idx = min(
                range(len(pts)),
                key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            )

        self._route_active = True
        return True

    def cancelRoute(self):
        """Stop following the route (no-op if no route is active)."""
        if not self._route_active:
            return
        self._route_active = False
        self._route_pts = []
        self._route_s = []
        self.routeEnded.emit(False)

    def _finish_route(self):
        # Bring the vehicle to a standstill
        v = self._vehicle
        if v.speed > 0.0:
            v.speed_down(v.speed)
        elif v.speed < 0.0:
            v.speed_up(-v.speed)

        self._route_active = False
        self._route_pts = []
        self._route_s = []
        self.routeEnded.emit(True)

    # ------------------------------------------------------------------
    # Route following helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _attr(obj, names):
        """Return the first existing attribute (property or no-arg method) from names."""
        for name in names:
            if not hasattr(obj, name):
                continue
            val = getattr(obj, name)
            if callable(val):
                try:
                    val = val()
                except TypeError:
                    continue
            if val is not None:
                return val
        return None

    def _read_pose(self):
        """Return (x, y, heading_rad_math, wheelbase) of the vehicle, or None."""
        v = self._vehicle
        try:
            x = float(v.f.x)
            y = float(v.f.y)
            h = float(v.a)  # radians, counter-clockwise from +X
        except (AttributeError, TypeError, ValueError):
            iface.messageBar().pushMessage(
                "Route following",
                "Can't read the vehicle position/heading (expected vehicle.f.x, vehicle.f.y, vehicle.a).",
                level=Qgis.Critical
            )
            return None

        wheelbase = self._attr(v, ("wheelbase", "wheel_base", "axle_distance", "length_wheelbase"))
        wheelbase = float(wheelbase) if wheelbase else self.ROUTE_WHEELBASE_FALLBACK

        return x, y, h, wheelbase

    def _steering_in_degrees(self) -> bool:
        if self.STEERING_IN_DEGREES is not None:
            return self.STEERING_IN_DEGREES
        return self._vehicle.max_steering_angle > 1.6

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _update_route_control(self):
        pose = self._read_pose()
        if pose is None:
            self._route_active = False
            self.routeEnded.emit(False)
            return

        x, y, heading, wheelbase = pose
        v = self._vehicle
        pts = self._route_pts
        s = self._route_s
        n = len(pts)

        # --- progress along the route (monotonic, local search) -------------
        hi = min(n, self._route_idx + self.ROUTE_SEARCH_WINDOW)
        best = self._route_idx
        best_d2 = float("inf")
        for i in range(self._route_idx, hi):
            d2 = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if d2 < best_d2:
                best, best_d2 = i, d2
        self._route_idx = best

        remaining = self._route_length - s[best]
        if remaining <= self.ROUTE_END_TOLERANCE:
            self._finish_route()
            return

        # --- pure pursuit steering -------------------------------------------
        lookahead = min(
            self.ROUTE_LOOKAHEAD_MAX,
            self.ROUTE_LOOKAHEAD_MIN + self.ROUTE_LOOKAHEAD_GAIN * abs(v.speed)
        )
        target_s = s[best] + lookahead
        if target_s >= self._route_length:
            over = target_s - self._route_length
            tx = pts[-1][0] + self._route_end_dir[0] * over
            ty = pts[-1][1] + self._route_end_dir[1] * over
        else:
            j = min(bisect_left(s, target_s), n - 1)
            tx, ty = pts[j]

        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-6)
        alpha = self._wrap_pi(math.atan2(dy, dx) - heading)

        in_deg = self._steering_in_degrees()
        units_per_rad = math.degrees(1.0) if in_deg else 1.0
        max_steer = v.max_steering_angle

        desired = math.atan2(2.0 * wheelbase * math.sin(alpha), dist) * units_per_rad
        desired = max(-max_steer, min(max_steer, desired))

        # Move the steering wheel towards the desired angle at the normal steering speed
        step = (max_steer / self._frames_half_steering) * self.ROUTE_STEER_RATE
        diff = desired - v.steering_angle
        if diff > 0.0:
            v.steer_left(min(diff, step))
        elif diff < 0.0:
            v.steer_right(min(-diff, step))

        # --- speed -------------------------------------------------------------
        accel_step = self._time_between_steps * self._prop.acceleration
        decel = self._prop.acceleration * self.BRAKE_FACTOR
        floor = max(self.ROUTE_MIN_SPEED, 1.5 * self._prop.minimum_speed)

        cruise = getattr(self._prop, "route_speed", self.ROUTE_CRUISE_SPEED)

        # Slow down for curves ahead (curvature of the pursuit arc)
        kappa = abs(2.0 * math.sin(alpha) / dist)
        v_curve = math.sqrt(self.ROUTE_MAX_LAT_ACCEL / kappa) if kappa > 1e-6 else cruise

        # Slow down for the end of the route
        v_end = math.sqrt(2.0 * decel * max(remaining - self.ROUTE_END_TOLERANCE, 0.0))

        target_speed = max(floor, min(cruise, v_curve, v_end))

        # Holding the brake key pauses the route; releasing it resumes
        if self._brake:
            target_speed = 0.0

        dv = target_speed - v.speed
        if dv > 0.0:
            v.speed_up(min(dv, accel_step))
        elif dv < 0.0:
            v.speed_down(min(-dv, accel_step * self.BRAKE_FACTOR))

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