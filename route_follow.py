"""
Route following core for QgisSweptPath.  No QGIS imports, so it can be tested anywhere.

The SAME Pilot is used by
  * the Simulator (real run, drives the real vehicle) and
  * the RouteTool (live preview while you draw),
so the preview shows exactly what the run will do.

Conventions
  - Angles are radians, counter-clockwise from +X (vehicle.a).
  - Steering angle: positive = left (radians).
  - F = front axle centre (vehicle.f), R = rear axle centre = F - wheelbase * (cos a, sin a).
  - FORWARD runs: the FRONT axle follows the line you draw.
  - REVERSE runs: the REAR axle follows the line you draw.
  - Every run starts automatically at the vehicle (or at the end of the previous run),
    so you only click the points AFTER the start.
  - Steering is applied instantly (stop-start assumption), as in Autodesk Vehicle Tracking.
"""
import copy
import math
from bisect import bisect_left


class Tuning(object):
    # --- path preparation -------------------------------------------------
    SPACING = 0.25                 # resampling distance of the drawn line (map units)
    SMOOTHING = 2                  # corner rounding (Chaikin iterations, 0 = off)
    LEADIN = 1.0                   # straight run-in along the vehicle axis at the start of a run
    # --- steering ---------------------------------------------------------
    FWD_SOFTEN = 1.2               # forward: smaller = pulls back onto the line harder
    REV_LOOKAHEAD_FACTOR = 0.6     # reverse: lookahead = factor * wheelbase ...
    REV_LOOKAHEAD_MIN = 1.2        # ... but never less than this
    # --- run end / failure ------------------------------------------------
    END_TOL = 0.3                  # a run is finished this close to its end
    MAX_DEVIATION = 1.0            # further than this from the line counts as "off the route" ...
    FAIL_DISTANCE = 2.0            # ... and after this much travel off the line the route is "not possible"
    # --- speed ------------------------------------------------------------
    LAT_ACCEL = 1.5                # slow down in curves so that v^2 * curvature <= this
    CURVE_LOOK = 5.0               # how far ahead curvature is checked for slowing down
    SEARCH_WINDOW = 60             # path points ahead searched when tracking progress


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ----------------------------------------------------------------------------
# Vehicle geometry
# ----------------------------------------------------------------------------
def _num(obj, names, default=None):
    for name in names:
        try:
            val = getattr(obj, name)
            if callable(val):
                val = val()
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                return float(val)
        except Exception:
            continue
    return default


class Geom(object):
    """Geometry of the (main part of the) vehicle."""

    def __init__(self, wheelbase, width, length, front_over, max_steer, steer_unit=1.0):
        self.wheelbase = float(wheelbase)
        self.width = float(width)
        self.length = float(length)
        self.front_over = float(front_over)               # body ahead of the front axle
        self.rear_over = max(0.0, self.length - self.front_over - self.wheelbase)  # body behind the rear axle
        self.max_steer = float(max_steer)                 # radians
        self.steer_unit = float(steer_unit)               # radians per vehicle steering unit

    @classmethod
    def from_vehicle(cls, v):
        fa = _num(v, ("front_axle_ref_pos", "_front_axle_ref_pos"))
        ra = _num(v, ("rear_axle_ref_pos", "_rear_axle_ref_pos"))
        wheelbase = None
        if fa is not None and ra is not None and ra - fa > 0.1:
            wheelbase = ra - fa
        if wheelbase is None:
            wheelbase = _num(v, ("wheelbase", "_wheelbase", "wheel_base"))
        if wheelbase is None:
            wheelbase = 3.0

        width = _num(v, ("body_width", "_body_width"), 2.0)
        length = _num(v, ("body_length", "_body_length"), wheelbase + 1.8)
        front_over = fa if fa is not None else 0.9

        mx = abs(_num(v, ("max_steering_angle", "_max_steering_angle"), 0.6))
        unit = 1.0
        if mx > 1.6:                       # looks like degrees
            unit = math.pi / 180.0
            mx = mx * unit
        return cls(wheelbase, width, length, front_over, mx, unit)

    def rear_from_front(self, fx, fy, th):
        return fx - self.wheelbase * math.cos(th), fy - self.wheelbase * math.sin(th)

    def front_from_rear(self, rx, ry, th):
        return rx + self.wheelbase * math.cos(th), ry + self.wheelbase * math.sin(th)

    def outline(self, rx, ry, th):
        """Body corners [front-left, front-right, rear-right, rear-left] for rear axle pose (rx, ry, th)."""
        c, s = math.cos(th), math.sin(th)
        xf = self.wheelbase + self.front_over
        xr = -self.rear_over
        w = self.width / 2.0
        pts = []
        for lx, ly in ((xf, w), (xf, -w), (xr, -w), (xr, w)):
            pts.append((rx + lx * c - ly * s, ry + lx * s + ly * c))
        return pts


# ----------------------------------------------------------------------------
# Path
# ----------------------------------------------------------------------------
def chaikin(pts, iterations):
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


def resample(pts, spacing):
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])]
    total = sum(seg)
    if total <= 1e-9:
        return list(pts[:1]) * 2 if pts else []
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


class Loc(object):
    __slots__ = ("k", "s", "e", "phi", "dist")


class Path(object):
    def __init__(self, pts):
        self.pts = pts
        n = len(pts)
        s = [0.0]
        for a, b in zip(pts[:-1], pts[1:]):
            s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.s = s
        self.length = s[-1]

        phi = []
        for i in range(n):
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, n - 1)]
            phi.append(math.atan2(b[1] - a[1], b[0] - a[0]))
        self.phi = phi

        kap = [0.0] * n
        for i in range(1, n - 1):
            ds = s[i + 1] - s[i - 1]
            if ds > 1e-9:
                kap[i] = wrap_pi(phi[i + 1] - phi[i - 1]) / ds
        if n > 2:
            kap[0] = kap[1]
            kap[-1] = kap[-2]
        sm = []
        for i in range(n):
            lo, hi = max(0, i - 3), min(n, i + 4)
            sm.append(sum(kap[lo:hi]) / (hi - lo))
        self.kappa = sm

    def locate(self, x, y, k0, window):
        pts = self.pts
        n = len(pts)
        k0 = min(k0, n - 1)
        hi = min(n, k0 + window)
        best, bd = k0, float("inf")
        for i in range(k0, hi):
            d2 = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if d2 < bd:
                best, bd = i, d2

        loc = Loc()
        loc.k = best
        loc.s = self.s[best]
        loc.phi = self.phi[best]
        loc.e = 0.0
        loc.dist = math.sqrt(bd)
        found = float("inf")
        for j in (best - 1, best):
            if j < 0 or j >= n - 1:
                continue
            ax, ay = pts[j]
            bx, by = pts[j + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            if l2 <= 0.0:
                continue
            seg_len = math.sqrt(l2)
            t = ((x - ax) * dx + (y - ay) * dy) / l2
            tc = min(1.0, max(0.0, t))
            d = math.hypot(x - (ax + tc * dx), y - (ay + tc * dy))
            if d < found:
                found = d
                loc.s = self.s[j] + tc * seg_len
                loc.phi = math.atan2(dy, dx)
                loc.e = (dx * (y - ay) - dy * (x - ax)) / seg_len   # left of the line = positive
                loc.dist = d
        return loc

    def point_at(self, s_target):
        """Point at arc length s_target; extrapolates straight beyond the end."""
        pts = self.pts
        if s_target >= self.length:
            over = s_target - self.length
            a, b = pts[-2], pts[-1]
            ang = math.atan2(b[1] - a[1], b[0] - a[0])
            return b[0] + over * math.cos(ang), b[1] + over * math.sin(ang)
        j = min(bisect_left(self.s, s_target), len(pts) - 1)
        return pts[j]

    def max_curvature(self, k, look, spacing):
        n = max(1, int(look / max(spacing, 1e-6)))
        seg = self.kappa[k:k + n]
        return max(abs(c) for c in seg) if seg else 0.0


# ----------------------------------------------------------------------------
# Pilot
# ----------------------------------------------------------------------------
class Cmd(object):
    """What the controller wants right now."""

    def __init__(self):
        self.status = "run"        # run | cusp | done | fail
        self.reason = ""
        self.steer = 0.0           # desired steering angle (radians, + = left)
        self.direction = 1         # +1 forward, -1 reverse
        self.v_curve = float("inf")
        self.remaining = 0.0       # distance to the end of the current run
        self.stops_at_end = True
        self.deviation = 0.0
        self.seg = 0
        self.progress = 0.0


class Pilot(object):
    """
    runs: [{"dir": 1 or -1, "pts": [(x, y), ...]}, ...]  - the points you clicked (without the start).
    """

    def __init__(self, geom, runs, tuning=None):
        self.g = geom
        self.t = tuning or Tuning()
        self.runs = runs
        self.i = -1
        self.path = None
        self.k = 0
        self.off = 0.0
        self._last = None

    def start(self, fx, fy, th):
        self._last = None
        self._activate(0, fx, fy, th)

    # ------------------------------------------------------------------
    def _activate(self, index, fx, fy, th):
        g, t = self.g, self.t
        run = self.runs[index]
        d = run["dir"]
        if d > 0:
            ax, ay = fx, fy
        else:
            ax, ay = g.rear_from_front(fx, fy, th)

        pts = [(ax, ay)]
        raw = [p for p in run["pts"]]
        if raw:
            ux, uy = math.cos(th) * d, math.sin(th) * d
            dist = math.hypot(raw[0][0] - ax, raw[0][1] - ay)
            lead = min(t.LEADIN, 0.5 * dist)
            if lead > 0.05:
                pts.append((ax + ux * lead, ay + uy * lead))
            pts.extend(raw)

        if len(pts) < 2:
            pts = [(ax, ay), (ax + 1e-3 * math.cos(th) * d, ay + 1e-3 * math.sin(th) * d)]
        pts = chaikin(pts, t.SMOOTHING)
        pts = resample(pts, t.SPACING)
        self.i = index
        self.path = Path(pts)
        self.k = 0
        self.off = 0.0

    def _tracked_point(self, fx, fy, th):
        if self.runs[self.i]["dir"] > 0:
            return fx, fy
        return self.g.rear_from_front(fx, fy, th)

    def _locate(self, fx, fy, th):
        px, py = self._tracked_point(fx, fy, th)
        loc = self.path.locate(px, py, self.k, self.t.SEARCH_WINDOW)
        self.k = loc.k
        return loc, self.path.length - loc.s

    # ------------------------------------------------------------------
    def update(self, fx, fy, th):
        """Call once per frame with the current front axle position and heading."""
        moved = 0.0 if self._last is None else math.hypot(fx - self._last[0], fy - self._last[1])
        self._last = (fx, fy)

        status = "run"
        loc, remaining = self._locate(fx, fy, th)
        guard = 0
        while remaining <= self.t.END_TOL:
            if self.i + 1 >= len(self.runs):
                cmd = Cmd()
                cmd.status = "done"
                cmd.direction = self.runs[self.i]["dir"]
                return cmd
            prev_dir = self.runs[self.i]["dir"]
            self._activate(self.i + 1, fx, fy, th)
            if self.runs[self.i]["dir"] != prev_dir:
                status = "cusp"
            loc, remaining = self._locate(fx, fy, th)
            guard += 1
            if guard > len(self.runs) + 1:
                break
        return self._command(fx, fy, th, loc, remaining, moved, status)

    def _command(self, fx, fy, th, loc, remaining, moved, status):
        g, t, path = self.g, self.t, self.path
        run = self.runs[self.i]
        d = run["dir"]
        cmd = Cmd()
        cmd.status = status
        cmd.direction = d
        cmd.remaining = remaining
        cmd.deviation = loc.dist
        cmd.seg = self.i
        cmd.progress = loc.s
        cmd.stops_at_end = (self.i + 1 >= len(self.runs)) or (self.runs[self.i + 1]["dir"] != d)

        if d > 0:
            # Front axle follows the line: steer the front wheel along the line, minus the cross-track correction
            delta = wrap_pi(loc.phi - th) - math.atan2(loc.e, t.FWD_SOFTEN)
        else:
            # Rear axle follows the line (pure pursuit, mirrored because the car goes backwards)
            rx, ry = g.rear_from_front(fx, fy, th)
            ld = max(t.REV_LOOKAHEAD_MIN, t.REV_LOOKAHEAD_FACTOR * g.wheelbase)
            tx, ty = path.point_at(loc.s + ld)
            dist = max(math.hypot(tx - rx, ty - ry), 1e-6)
            alpha = wrap_pi(math.atan2(ty - ry, tx - rx) - (th + math.pi))
            kappa = 2.0 * math.sin(alpha) / dist
            delta = -math.atan(g.wheelbase * kappa)
        cmd.steer = max(-g.max_steer, min(g.max_steer, delta))

        kmax = path.max_curvature(loc.k, t.CURVE_LOOK, t.SPACING)
        if kmax > 1e-4:
            cmd.v_curve = math.sqrt(t.LAT_ACCEL / kmax)

        if loc.dist > t.MAX_DEVIATION:
            self.off += moved
        else:
            self.off = 0.0
        if self.off > t.FAIL_DISTANCE:
            cmd.status = "fail"
            cmd.reason = ("The vehicle can't stay on the route (it is {:.1f} m off the line). "
                          "A turn is probably too tight for this vehicle.").format(loc.dist)
        return cmd


# ----------------------------------------------------------------------------
# Offline simulation (used for the live preview while drawing)
# ----------------------------------------------------------------------------
class SimResult(object):
    def __init__(self):
        self.samples = []     # (rx, ry, th, steer, direction, run index)
        self.ghosts = []      # (rx, ry, th) at the start, at every cusp and at the end
        self.fail_index = None
        self.fail_reason = ""
        self.complete = False


def simulate(geom, fx, fy, th, runs, step=0.1, tuning=None):
    """Drive the route with the real controller and a kinematic bicycle model."""
    res = SimResult()
    runs = [r for r in runs if r["pts"]]
    if not runs:
        rx, ry = geom.rear_from_front(fx, fy, th)
        res.ghosts.append((rx, ry, th))
        return res

    t = tuning or Tuning()
    pilot = Pilot(geom, runs, t)
    pilot.start(fx, fy, th)
    rx, ry = geom.rear_from_front(fx, fy, th)
    res.ghosts.append((rx, ry, th))

    # Upper bound for the loop: roughly the drawn length, with margin
    approx = 0.0
    prev = (fx, fy)
    for r in runs:
        for p in r["pts"]:
            approx += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
    max_steps = int(3.0 * approx / step) + 2000

    L = geom.wheelbase
    for _ in range(max_steps):
        fx, fy = geom.front_from_rear(rx, ry, th)
        cmd = pilot.update(fx, fy, th)
        if cmd.status == "done":
            res.complete = True
            break
        if cmd.status == "fail":
            res.fail_index = len(res.samples)
            res.fail_reason = cmd.reason
            break
        if cmd.status == "cusp":
            res.ghosts.append((rx, ry, th))
        delta = cmd.steer
        dd = cmd.direction * step
        rx += dd * math.cos(th)
        ry += dd * math.sin(th)
        th += dd * math.tan(delta) / L
        res.samples.append((rx, ry, th, delta, cmd.direction, cmd.seg))

    res.ghosts.append((rx, ry, th))
    return res


# ----------------------------------------------------------------------------
# Undo support: snapshot / restore of the complete state of a vehicle
# ----------------------------------------------------------------------------
_SIMPLE = (type(None), bool, int, float, complex, str, bytes)
_EXTERNAL_PREFIXES = ("PyQt", "qgis", "sip", "_thread", "threading")


def _all_parts(vehicle):
    parts = []
    seen = set()
    candidates = [vehicle]
    try:
        candidates += list(getattr(vehicle, "vehicle_parts", None) or [])
    except Exception:
        pass
    for p in candidates:
        if id(p) not in seen:
            seen.add(id(p))
            parts.append(p)
    return parts


def _attr_names(obj):
    names = []
    d = getattr(obj, "__dict__", None)
    if d:
        names.extend(d.keys())
    for klass in type(obj).__mro__:
        slots = klass.__dict__.get("__slots__", ())
        if isinstance(slots, str):
            slots = (slots,)
        for s in slots:
            if s in ("__dict__", "__weakref__"):
                continue
            if s.startswith("__") and not s.endswith("__"):
                s = "_{}{}".format(klass.__name__.lstrip("_"), s)
            if s not in names:
                try:
                    getattr(obj, s)
                except AttributeError:
                    continue
                names.append(s)
    return names


def _copy_value(value, memo):
    if isinstance(value, _SIMPLE):
        return value
    module = type(value).__module__ or ""
    if module.startswith(_EXTERNAL_PREFIXES):
        return type(value)(value)
    return copy.deepcopy(value, memo)


def _pose(vehicle):
    sig = []
    for part in _all_parts(vehicle):
        item = []
        for getter in (lambda p: p.f.x, lambda p: p.f.y, lambda p: p.a,
                       lambda p: p.steering_angle, lambda p: p.speed):
            try:
                item.append(round(float(getter(part)), 9))
            except Exception:
                item.append(None)
        sig.append(tuple(item))
    return sig


def capture_vehicle_state(vehicle):
    """Return an opaque snapshot of the vehicle (all parts), or None."""
    try:
        parts = _all_parts(vehicle)
        memo = dict((id(p), p) for p in parts)
        items = []
        skipped = []
        for part in parts:
            for name in _attr_names(part):
                try:
                    value = object.__getattribute__(part, name)
                except AttributeError:
                    continue
                if any(value is p for p in parts):
                    continue
                try:
                    items.append((part, name, _copy_value(value, memo)))
                except Exception:
                    skipped.append("{}.{}".format(type(part).__name__, name))
        if not items:
            return None
        return {"vehicle": vehicle, "pose": _pose(vehicle), "items": items, "skipped": skipped}
    except Exception:
        return None


def apply_vehicle_state(snap):
    """Put a snapshot back. Returns (ok, details). ok is True only if the pose was verified."""
    if not snap:
        return False, "no snapshot was taken"

    restored = 0
    errors = []
    for obj, name, value in snap["items"]:
        try:
            d = getattr(obj, "__dict__", None)
            if d is not None and name in d:
                d[name] = value
            else:
                object.__setattr__(obj, name, value)
            restored += 1
        except Exception as e:
            errors.append("{}.{} ({})".format(type(obj).__name__, name, type(e).__name__))

    after = _pose(snap["vehicle"])
    verified = (after == snap["pose"])
    details = []
    if errors:
        details.append("write errors: " + ", ".join(errors[:6]))
    if not verified:
        details.append("pose expected {} but is {}".format(snap["pose"], after))
    if snap["skipped"]:
        details.append("not copied: " + ", ".join(snap["skipped"][:6]))
    return (restored > 0 and verified), "; ".join(details)
