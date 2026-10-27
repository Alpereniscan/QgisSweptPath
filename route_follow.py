"""
Route following core for QgisSweptPath.  No QGIS imports, so it can be tested anywhere.
VERSION 4 (trailer aware)

The SAME Pilot is used by
  * the Simulator (real run, drives the real vehicle) and
  * the RouteTool (live preview while you draw),
so the preview shows exactly what the run will do.

Conventions
  - Angles are radians, counter-clockwise from +X (vehicle.a).
  - Steering angle: positive = left (radians).
  - F = front axle centre (vehicle.f), R = rear axle centre = F - wheelbase * (cos a, sin a).
  - FORWARD runs: the FRONT axle follows the line you draw.
  - REVERSE runs, no trailer: the REAR axle follows the line you draw.
  - REVERSE runs WITH a trailer: the TRAILER's rear axle follows the line you draw.
  - Every run starts automatically at the vehicle (or at the end of the previous run),
    so you only click the points AFTER the start.
  - Steering is applied instantly (stop-start assumption), as in Autodesk Vehicle Tracking.

Reversing with a trailer is unstable (the trailer folds up = jackknife) unless the angle
between tractor and trailer is controlled.  The Pilot does that with two nested loops:
  outer loop : pure pursuit of the trailer axle  ->  wanted trailer curvature
               -> wanted articulation angle (beta*), capped well below the bending limit
  inner loop : steers the tractor so that the measured articulation angle follows beta*
"""
import copy
import math
import weakref
from bisect import bisect_left


class Tuning(object):
    # --- path preparation -------------------------------------------------
    SPACING = 0.25                 # resampling distance of the drawn line (map units)
    SMOOTHING = 2                  # corner rounding (Chaikin iterations, 0 = off)
    LEADIN = 1.0                   # straight run-in along the vehicle axis at the start of a run
    # --- steering ---------------------------------------------------------
    FWD_SOFTEN = 1.2               # forward: smaller = pulls back onto the line harder
    REV_LOOKAHEAD_FACTOR = 0.6     # reverse (no trailer): lookahead = factor * wheelbase ...
    REV_LOOKAHEAD_MIN = 1.2        # ... but never less than this
    # --- run end / failure ------------------------------------------------
    END_TOL = 0.3                  # a run is finished this close to its end
    MAX_DEVIATION = 1.0            # further than this from the line counts as "off the route" ...
    FAIL_DISTANCE = 2.0            # ... and after this much travel off the line the route is "not possible"
    # --- speed ------------------------------------------------------------
    LAT_ACCEL = 1.5                # slow down in curves so that v^2 * curvature <= this
    CURVE_LOOK = 5.0               # how far ahead curvature is checked for slowing down
    SEARCH_WINDOW = 60             # path points ahead searched when tracking progress
    # --- trailer ------------------------------------------------------------
    # Used when the vehicle does not state its maximum bending angle
    TRAILER_MAX_ARTICULATION = math.radians(45.0)
    # Fraction of the limit (and of what the steering can hold) that is ever commanded
    TRAILER_SAFETY = 0.80
    # The run is stopped as "jackknife" at this fraction of the bending limit
    TRAILER_FAIL_FRACTION = 0.97
    # Inner loop: how fast the articulation angle is brought to its target (1/m travelled)
    TRAILER_INNER_GAIN = 1.5
    # Outer loop: trailer pure pursuit lookahead = max(MIN, FACTOR * hitch-to-axle distance)
    TRAILER_LOOKAHEAD_FACTOR = 0.5
    TRAILER_LOOKAHEAD_MIN = 2.0
    # The wanted articulation angle never runs further ahead of the real one than this (radians),
    # so the steering is not asked for more than it can deliver (prevents overshoot)
    TRAILER_BETA_LEAD = math.radians(20.0)
    # Used if the trailer's axle points can't be read
    TRAILER_LENGTH_FALLBACK = 3.5
    # Hitch re-estimation: blend factor per frame, used when |sin(beta)| is above this
    HITCH_BLEND = 0.4
    HITCH_MIN_SINBETA = 0.08


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ----------------------------------------------------------------------------
# Reading points / numbers from vehicle objects (no assumptions about their classes)
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


def _pt(p):
    """(x, y) of a point-like object (attributes or methods), or None."""
    try:
        x, y = p.x, p.y
        if callable(x):
            x, y = x(), y()
        return float(x), float(y)
    except Exception:
        return None


def _axle_mid(part, left, right):
    a = _pt(getattr(part, left, None))
    b = _pt(getattr(part, right, None))
    if a is None or b is None:
        return None
    return (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0


# ----------------------------------------------------------------------------
# Trailer geometry
# ----------------------------------------------------------------------------
# Hitch geometry found for a trailer object is remembered, so it stays known when the
# vehicle is straight again (the hitch can only be measured while it is articulated).
_HITCH_CACHE = weakref.WeakKeyDictionary()


def find_trailer(v):
    """The (first) trailer part attached to the vehicle, or None."""
    tr = getattr(v, "trailer", None)
    if tr is not None and not callable(tr):
        return tr
    try:
        for p in list(getattr(v, "vehicle_parts", None) or []):
            if p is not v and not getattr(p, "is_main_vehicle", False):
                return p
    except Exception:
        pass
    return None


def estimate_hitch(R, th1, T, th2):
    """
    Hitch position from the two axle centres and headings: the hitch is where the
    tractor's centre line (through R) meets the trailer's centre line (through T).
    Returns (c, d): c = hitch distance BEHIND the tractor rear axle, d = hitch to trailer axle.
    None if the articulation angle is too small to tell.
    """
    u1x, u1y = math.cos(th1), math.sin(th1)
    u2x, u2y = math.cos(th2), math.sin(th2)
    det = math.sin(th1 - th2)
    if abs(det) < 0.05:
        return None
    rx, ry = T[0] - R[0], T[1] - R[1]
    s = (-rx * u2y + u2x * ry) / det     # H = R + s * u1
    t = (u1x * ry - u1y * rx) / det      # H = T + t * u2
    c, d = -s, t
    if 0.3 <= d <= 25.0 and -4.0 <= c <= 8.0:
        return c, d
    return None


def read_articulation(v, geom):
    """
    Current tractor/trailer state, or None if there is no (readable) trailer.
    Returns (tractor rear axle R, tractor heading, trailer axle T, trailer heading, hitch hint, trailer part).
    """
    tr = find_trailer(v)
    if tr is None:
        return None
    try:
        th1 = float(v.a)
        th2 = float(tr.a)
        fx, fy = float(v.f.x), float(v.f.y)
    except Exception:
        return None

    R = _axle_mid(v, "rwl", "rwr")
    if R is None:
        R = geom.rear_from_front(fx, fy, th1)
    T = _axle_mid(tr, "rwl", "rwr")
    H = _pt(getattr(tr, "f", None))
    return {"R": R, "th1": th1, "T": T, "th2": th2, "H": H, "tr": tr}


def _local_outline(tr, T, th2):
    """Trailer body corners in the trailer frame (origin = trailer axle, +x forward), or None."""
    pts = [_pt(getattr(tr, n, None)) for n in ("fl", "fr", "br", "bl")]
    if T is None or any(p is None for p in pts):
        return None
    c, s = math.cos(th2), math.sin(th2)
    return [((p[0] - T[0]) * c + (p[1] - T[1]) * s, -(p[0] - T[0]) * s + (p[1] - T[1]) * c) for p in pts]


def _find_limit(tr, v, default):
    names = ("max_bending_angle", "_max_bending_angle", "bending_angle_max", "max_bend_angle",
             "max_hinge_angle", "_max_hinge_angle", "max_angle", "_max_angle")
    for obj in (tr, v):
        val = _num(obj, names)
        if val is not None and val > 0.0:
            if val > 3.2:                  # degrees
                val = math.radians(val)
            return min(val, math.radians(90.0))
    return default


class TrailerGeom(object):
    """Hitch c metres behind the tractor rear axle, trailer axle d metres behind the hitch."""

    def __init__(self, c, d, outline, limit, key=None, known=False):
        self.c = float(c)
        self.d = float(d)
        self.outline = outline          # [(x, y)] in the trailer frame
        self.limit = float(limit)       # bending limit (radians)
        self.known = known
        self._key = key

    @classmethod
    def from_vehicle(cls, v, geom, tuning=None):
        t = tuning or Tuning()
        st = read_articulation(v, geom)
        if st is None:
            return None
        tr, R, th1, T, th2, H = st["tr"], st["R"], st["th1"], st["T"], st["th2"], st["H"]
        u1 = (math.cos(th1), math.sin(th1))
        u2 = (math.cos(th2), math.sin(th2))

        c = d = None
        known = False
        cached = None
        try:
            cached = _HITCH_CACHE.get(tr)
        except TypeError:
            pass

        if cached is not None:
            c, d = cached
            known = True
        elif T is not None:
            est = estimate_hitch(R, th1, T, th2)
            if est is not None:
                c, d = est
                known = True
            elif H is not None:
                # Straight: use the trailer's own reference point if it lies on both centre lines
                cc = (R[0] - H[0]) * u1[0] + (R[1] - H[1]) * u1[1]
                dd = (H[0] - T[0]) * u2[0] + (H[1] - T[1]) * u2[1]
                lat1 = abs(-(H[0] - R[0]) * u1[1] + (H[1] - R[1]) * u1[0])
                lat2 = abs(-(H[0] - T[0]) * u2[1] + (H[1] - T[1]) * u2[0])
                if lat1 < 0.08 and lat2 < 0.08 and dd > 0.3 and -3.0 < cc < 6.0:
                    c, d = cc, dd
            if c is None:
                c, d = 0.0, max(1.0, math.hypot(T[0] - R[0], T[1] - R[1]))
        else:
            # No trailer axle points: build the axle from the hitch hint and a default length
            c = 0.0
            d = t.TRAILER_LENGTH_FALLBACK
            if H is not None:
                c = (R[0] - H[0]) * u1[0] + (R[1] - H[1]) * u1[1]
                c = max(-3.0, min(6.0, c))
            T = (R[0] - c * u1[0] - d * u2[0] * 1.0, R[1] - c * u1[1] - d * u2[1] * 1.0)

        outline = _local_outline(tr, T, th2)
        if outline is None:
            w = geom.width / 2.0
            outline = [(d + 0.6, w), (d + 0.6, -w), (-0.9, -w), (-0.9, w)]

        limit = _find_limit(tr, v, t.TRAILER_MAX_ARTICULATION)
        key = tr
        return cls(c, d, outline, limit, key, known)

    def set_hitch(self, c, d):
        self.c = float(c)
        self.d = float(d)
        self.known = True
        if self._key is not None:
            try:
                _HITCH_CACHE[self._key] = (self.c, self.d)
            except TypeError:
                pass

    def sync(self):
        """Pick up a hitch geometry that was learned while driving."""
        try:
            cached = _HITCH_CACHE.get(self._key)
        except TypeError:
            cached = None
        if cached is not None:
            self.c, self.d = cached
            self.known = True

    def axle_from(self, rx, ry, th1, th2):
        """Trailer axle centre from the tractor rear axle (rx, ry) and both headings."""
        return (rx - self.c * math.cos(th1) - self.d * math.cos(th2),
                ry - self.c * math.sin(th1) - self.d * math.sin(th2))

    def outline_world(self, tx, ty, th2):
        c, s = math.cos(th2), math.sin(th2)
        return [(tx + x * c - y * s, ty + x * s + y * c) for x, y in self.outline]

    def describe(self):
        return ("hitch {:.2f} m behind the rear axle, trailer axle {:.2f} m behind the hitch, "
                "bending limit {:.0f} deg").format(self.c, self.d, math.degrees(self.limit))


# ----------------------------------------------------------------------------
# Vehicle geometry
# ----------------------------------------------------------------------------
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
        self.trailer = None                               # TrailerGeom or None

    @classmethod
    def from_vehicle(cls, v):
        fa = _num(v, ("front_axle_ref_pos", "_front_axle_ref_pos"))
        ra = _num(v, ("rear_axle_ref_pos", "_rear_axle_ref_pos"))
        wheelbase = None
        if fa is not None and ra is not None and ra - fa > 0.1:
            wheelbase = ra - fa
        if wheelbase is None:
            # Exact from the axle points if the vehicle has them
            f_mid = _axle_mid(v, "fwl", "fwr")
            r_mid = _axle_mid(v, "rwl", "rwr")
            if f_mid is not None and r_mid is not None:
                wb = math.hypot(f_mid[0] - r_mid[0], f_mid[1] - r_mid[1])
                if wb > 0.1:
                    wheelbase = wb
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
        g = cls(wheelbase, width, length, front_over, mx, unit)
        try:
            g.trailer = TrailerGeom.from_vehicle(v, g)
        except Exception:
            g.trailer = None
        return g

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

    def trailer_outline(self, rx, ry, th1, th2):
        tx, ty = self.trailer.axle_from(rx, ry, th1, th2)
        return self.trailer.outline_world(tx, ty, th2)


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
# Articulation maths
# ----------------------------------------------------------------------------
def beta_from_curvature(k2, c, d):
    """
    Steady-state articulation angle (tractor heading minus trailer heading) that makes the
    trailer axle move on a circle of curvature k2 (+ = left, in the trailer's own forward sense).
    From  k2 = sin(beta) / (d cos(beta) + c).
    """
    g = math.atan(k2 * d)
    s = k2 * c / math.sqrt(1.0 + (k2 * d) ** 2)
    s = max(-1.0, min(1.0, s))
    return g + math.asin(s)


def beta_steering_limit(L, c, d, max_steer):
    """Largest articulation angle the tractor's steering can still hold in a steady turn."""
    tm = math.tan(max_steer)
    best = 0.0
    for i in range(1, 181):
        b = math.radians(0.5 * i)
        denom = d + c * math.cos(b)
        if denom <= 1e-6:
            break
        if L * math.sin(b) / denom <= tm:
            best = b
        else:
            break
    return best


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
        self.with_trailer = False
        self.beta = 0.0            # measured articulation angle (radians)
        self.beta_target = 0.0


class Pilot(object):
    """
    runs: [{"dir": 1 or -1, "pts": [(x, y), ...]}, ...]  - the points you clicked (without the start).
    art:  None, or (trailer axle x, y, trailer heading, tractor rear axle x, y) when a trailer is attached.
    """

    def __init__(self, geom, runs, tuning=None, refine=True):
        self.g = geom
        self.t = tuning or Tuning()
        self.runs = runs
        self.refine = refine
        self.i = -1
        self.path = None
        self.k = 0
        self.off = 0.0
        self._last = None
        self._tr_run = False
        self._cap_key = None
        self._cap = 0.0

    def start(self, fx, fy, th, art=None):
        self._last = None
        self._activate(0, fx, fy, th, art)

    # ------------------------------------------------------------------
    def _has_trailer(self, art):
        return self.g.trailer is not None and art is not None

    def _activate(self, index, fx, fy, th, art):
        g, t = self.g, self.t
        run = self.runs[index]
        d = run["dir"]
        tr_run = d < 0 and self._has_trailer(art)
        if d > 0:
            ax, ay, ah = fx, fy, th
        elif tr_run:
            ax, ay, ah = art[0], art[1], art[2]       # the trailer axle leads when reversing
        else:
            ax, ay = g.rear_from_front(fx, fy, th)
            ah = th

        pts = [(ax, ay)]
        raw = [p for p in run["pts"]]
        if raw:
            ux, uy = math.cos(ah) * d, math.sin(ah) * d
            dist = math.hypot(raw[0][0] - ax, raw[0][1] - ay)
            lead = min(t.LEADIN, 0.5 * dist)
            if lead > 0.05:
                pts.append((ax + ux * lead, ay + uy * lead))
            pts.extend(raw)

        if len(pts) < 2:
            pts = [(ax, ay), (ax + 1e-3 * math.cos(ah) * d, ay + 1e-3 * math.sin(ah) * d)]
        pts = chaikin(pts, t.SMOOTHING)
        pts = resample(pts, t.SPACING)
        self.i = index
        self._tr_run = tr_run
        self.path = Path(pts)
        self.k = 0
        self.off = 0.0

    def _tracked_point(self, fx, fy, th, art):
        d = self.runs[self.i]["dir"]
        if d > 0:
            return fx, fy
        if self._tr_run and art is not None:
            return art[0], art[1]
        return self.g.rear_from_front(fx, fy, th)

    def _locate(self, fx, fy, th, art):
        px, py = self._tracked_point(fx, fy, th, art)
        loc = self.path.locate(px, py, self.k, self.t.SEARCH_WINDOW)
        self.k = loc.k
        return loc, self.path.length - loc.s

    def _refine_hitch(self, th, art):
        """Re-measure the hitch whenever the articulation is large enough to see it."""
        tg = self.g.trailer
        R = (art[3], art[4])
        T = (art[0], art[1])
        if abs(math.sin(th - art[2])) < self.t.HITCH_MIN_SINBETA:
            return
        est = estimate_hitch(R, th, T, art[2])
        if est is None:
            return
        b = self.t.HITCH_BLEND
        tg.set_hitch(tg.c + b * (est[0] - tg.c), tg.d + b * (est[1] - tg.d))

    def _angle_cap(self):
        """Largest articulation angle that is ever commanded."""
        tg, g = self.g.trailer, self.g
        key = (round(tg.c, 2), round(tg.d, 2))
        if key != self._cap_key:
            sat = beta_steering_limit(g.wheelbase, tg.c, tg.d, g.max_steer)
            self._cap = self.t.TRAILER_SAFETY * min(tg.limit, sat)
            self._cap_key = key
        return self._cap

    # ------------------------------------------------------------------
    def update(self, fx, fy, th, art=None):
        """Call once per frame with the current front axle position, heading and trailer state."""
        if self._has_trailer(art) and self.refine:
            self._refine_hitch(th, art)

        moved = 0.0 if self._last is None else math.hypot(fx - self._last[0], fy - self._last[1])
        self._last = (fx, fy)

        status = "run"
        loc, remaining = self._locate(fx, fy, th, art)
        guard = 0
        while remaining <= self.t.END_TOL:
            if self.i + 1 >= len(self.runs):
                cmd = Cmd()
                cmd.status = "done"
                cmd.direction = self.runs[self.i]["dir"]
                return cmd
            prev_dir = self.runs[self.i]["dir"]
            self._activate(self.i + 1, fx, fy, th, art)
            if self.runs[self.i]["dir"] != prev_dir:
                status = "cusp"
            loc, remaining = self._locate(fx, fy, th, art)
            guard += 1
            if guard > len(self.runs) + 1:
                break
        return self._command(fx, fy, th, art, loc, remaining, moved, status)

    def _command(self, fx, fy, th, art, loc, remaining, moved, status):
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
        elif self._tr_run and art is not None:
            delta = self._reverse_with_trailer(cmd, th, art, loc)
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

        if cmd.status != "fail":
            if loc.dist > t.MAX_DEVIATION:
                self.off += moved
            else:
                self.off = 0.0
            if self.off > t.FAIL_DISTANCE:
                cmd.status = "fail"
                who = "trailer" if (self._tr_run and art is not None) else "vehicle"
                cmd.reason = ("The {} can't stay on the route (it is {:.1f} m off the line). "
                              "A turn is probably too tight for this vehicle.").format(who, loc.dist)
        return cmd

    def _reverse_with_trailer(self, cmd, th, art, loc):
        """Tractor steering angle (radians) for reversing with the trailer axle on the line."""
        g, t, path, tg = self.g, self.t, self.path, self.g.trailer
        Tx, Ty, th2 = art[0], art[1], art[2]
        L, c, d = g.wheelbase, tg.c, tg.d
        beta = wrap_pi(th - th2)
        cmd.with_trailer = True
        cmd.beta = beta

        # Outer loop: pure pursuit of the trailer axle (motion heading = trailer heading + 180 deg)
        ld = max(t.TRAILER_LOOKAHEAD_MIN, t.TRAILER_LOOKAHEAD_FACTOR * d)
        tx, ty = path.point_at(loc.s + ld)
        dist = max(math.hypot(tx - Tx, ty - Ty), 1e-6)
        alpha = wrap_pi(math.atan2(ty - Ty, tx - Tx) - (th2 + math.pi))
        k_motion = 2.0 * math.sin(alpha) / dist
        k2 = -k_motion                       # trailer curvature in its own forward sense

        cap = self._angle_cap()
        beta_star = max(-cap, min(cap, beta_from_curvature(k2, c, d)))
        lead = t.TRAILER_BETA_LEAD
        beta_star = beta + max(-lead, min(lead, beta_star - beta))
        beta_star = max(-cap, min(cap, beta_star))
        cmd.beta_target = beta_star

        # Jackknife: stop before the vehicle's bending limit is reached
        if abs(beta) > tg.limit * t.TRAILER_FAIL_FRACTION:
            cmd.status = "fail"
            cmd.reason = ("The trailer would jackknife ({:.0f} deg between tractor and trailer, "
                          "limit {:.0f} deg). Use a wider turn or drive forward first.").format(
                              abs(math.degrees(beta)), math.degrees(tg.limit))

        # Inner loop: steer so that the articulation angle follows beta_star (reversing: dir = -1)
        A = (1.0 / L) * (1.0 + (c / d) * math.cos(beta))
        tan_delta = (math.sin(beta) / d - t.TRAILER_INNER_GAIN * (beta_star - beta)) / max(A, 1e-6)
        return math.atan(tan_delta)


# ----------------------------------------------------------------------------
# Offline simulation (used for the live preview while drawing)
# ----------------------------------------------------------------------------
class SimResult(object):
    def __init__(self):
        self.samples = []     # (rx, ry, th, steer, direction, run index, trailer heading)
        self.ghosts = []      # (rx, ry, th, trailer heading) at the start, at every cusp and at the end
        self.max_beta = 0.0
        self.fail_index = None
        self.fail_reason = ""
        self.complete = False


def simulate(geom, fx, fy, th, runs, step=0.1, tuning=None, th2=None):
    """Drive the route with the real controller and a kinematic model (bicycle + one trailer)."""
    res = SimResult()
    runs = [r for r in runs if r["pts"]]
    tg = geom.trailer
    if tg is not None and th2 is None:
        th2 = th
    if tg is None:
        th2 = th
    rx, ry = geom.rear_from_front(fx, fy, th)
    if not runs:
        res.ghosts.append((rx, ry, th, th2))
        return res

    t = tuning or Tuning()
    pilot = Pilot(geom, runs, t, refine=False)

    def art_now():
        if tg is None:
            return None
        Tx, Ty = tg.axle_from(rx, ry, th, th2)
        return (Tx, Ty, th2, rx, ry)

    pilot.start(fx, fy, th, art_now())
    res.ghosts.append((rx, ry, th, th2))

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
        cmd = pilot.update(fx, fy, th, art_now())
        if cmd.status == "done":
            res.complete = True
            break
        if cmd.status == "fail":
            res.fail_index = len(res.samples)
            res.fail_reason = cmd.reason
            break
        if cmd.status == "cusp":
            res.ghosts.append((rx, ry, th, th2))
        delta = cmd.steer
        dd = cmd.direction * step
        if tg is None:
            rx += dd * math.cos(th)
            ry += dd * math.sin(th)
            th += dd * math.tan(delta) / L
            th2 = th
        else:
            tand = math.tan(delta)
            c, d = tg.c, tg.d

            def rates(a1, a2):
                b = a1 - a2
                return tand / L, (math.sin(b) - (c / L) * tand * math.cos(b)) / d

            k1 = rates(th, th2)
            m1, m2 = th + k1[0] * dd / 2.0, th2 + k1[1] * dd / 2.0
            k2 = rates(m1, m2)
            rx += dd * math.cos(m1)
            ry += dd * math.sin(m1)
            th += k2[0] * dd
            th2 += k2[1] * dd
            res.max_beta = max(res.max_beta, abs(wrap_pi(th - th2)))
        res.samples.append((rx, ry, th, delta, cmd.direction, cmd.seg, th2))

    res.ghosts.append((rx, ry, th, th2))
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
