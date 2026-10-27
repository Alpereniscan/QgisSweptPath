# Map tool for drawing a route by hand, with a live swept-path preview
import math
import time

from qgis.PyQt.QtCore import Qt, pyqtSignal
from qgis.PyQt.QtGui import QColor
from qgis.core import Qgis, QgsPointXY, QgsGeometry, QgsWkbTypes
from qgis.gui import QgsMapTool, QgsMapCanvas, QgsRubberBand
from qgis.utils import iface

from .route_follow import Geom, simulate, read_articulation

FORWARD = 1
REVERSE = -1

try:
    GEOM_LINE = Qgis.GeometryType.Line
    GEOM_POLYGON = Qgis.GeometryType.Polygon
except AttributeError:
    GEOM_LINE = QgsWkbTypes.LineGeometry
    GEOM_POLYGON = QgsWkbTypes.PolygonGeometry


class RouteTool(QgsMapTool):
    """
    Draw the route the vehicle should follow. A live preview shows how the vehicle will
    really steer, forward and in reverse (same controller as the real run):

      orange line  : path of the FRONT axle (forward parts)
      blue line    : path of the REAR axle (reverse parts); with a trailer: the TRAILER's rear axle
      shaded area  : swept area, its edge = the outer lines of the vehicle body (and trailer)
      outlines     : the vehicle at the start, at every direction change and at the end
      red          : the vehicle can't follow the route from here
      grey dashed  : the points you clicked

    You only click the points AFTER the start. Each part starts at the vehicle
    (or at the end of the previous part).

      Left click / drag : add points (drag = freehand)
      R                 : switch between forward and reverse (creates a stop)
      Backspace         : remove the last point
      Right click/Enter : finish
      Esc               : cancel
    """

    # Emitted with a list of runs: [(list of QgsPointXY, direction), ...]; direction 1 = forward, -1 = reverse
    routeFinished: pyqtSignal = pyqtSignal(list, name="routeFinished")
    routeCancelled: pyqtSignal = pyqtSignal(name="routeCancelled")

    # Minimum screen distance (pixels) between freehand vertices / duplicate clicks
    FREEHAND_MIN_PIXELS: int = 8
    DUPLICATE_PIXELS: int = 2
    # Preview refresh limit while the mouse moves (seconds)
    PREVIEW_INTERVAL: float = 0.07
    # Distance between body outlines used for the swept area (map units):
    # coarse while the mouse moves, fine after a click / when finished
    ENVELOPE_STEP: float = 0.3
    ENVELOPE_STEP_FINE: float = 0.12
    # Never union more outlines than this (keeps long routes responsive)
    ENVELOPE_MAX_OUTLINES: int = 450
    # Simulation step of the preview (map units)
    PREVIEW_STEP: float = 0.1

    def __init__(self, canvas: QgsMapCanvas, vehicle_getter=None):
        """vehicle_getter: callable returning the current Vehicle (or None). Without it there is no preview."""
        super().__init__(canvas)
        self._canvas = canvas
        self._vehicle_getter = vehicle_getter
        self._geom: Geom = None
        self._runs: list = [{"dir": FORWARD, "pts": []}]
        self._finished: bool = False
        self._dragging: bool = False
        self._last_px = None
        self._last_cursor = None
        self._last_preview_time: float = 0.0
        self._result = None

        self._band_envelope = self._make_band(GEOM_POLYGON, QColor(30, 60, 90, 200), QColor(70, 130, 200, 55), 1)
        self._band_guide = self._make_band(GEOM_LINE, QColor(120, 120, 120, 200), None, 1, Qt.PenStyle.DashLine)
        self._band_fwd = self._make_band(GEOM_LINE, QColor(255, 120, 0, 240), None, 3)
        self._band_rev = self._make_band(GEOM_LINE, QColor(0, 110, 255, 240), None, 3)
        self._band_ghost = self._make_band(GEOM_POLYGON, QColor(30, 30, 30, 230), QColor(0, 0, 0, 0), 1,
                                           Qt.PenStyle.SolidLine, no_brush=True)
        self._band_fail = self._make_band(GEOM_LINE, QColor(230, 0, 0, 255), None, 4)
        self._band_fail_ghost = self._make_band(GEOM_POLYGON, QColor(230, 0, 0, 255), QColor(230, 0, 0, 40), 2)
        self._bands = [
            (self._band_envelope, GEOM_POLYGON), (self._band_guide, GEOM_LINE),
            (self._band_fwd, GEOM_LINE), (self._band_rev, GEOM_LINE),
            (self._band_ghost, GEOM_POLYGON), (self._band_fail, GEOM_LINE),
            (self._band_fail_ghost, GEOM_POLYGON),
        ]

        self.setCursor(Qt.CursorShape.CrossCursor)

    def _make_band(self, geom_type, stroke, fill, width, style=Qt.PenStyle.SolidLine, no_brush=False):
        band = QgsRubberBand(self._canvas, geom_type)
        band.setStrokeColor(stroke)
        if fill is not None:
            band.setFillColor(fill)
        if no_brush:
            band.setBrushStyle(Qt.BrushStyle.NoBrush)
        band.setWidth(width)
        band.setLineStyle(style)
        return band

    # ------------------------------------------------------------------
    # Tool life cycle
    # ------------------------------------------------------------------
    def activate(self):
        super().activate()
        self.reset()
        v = self._vehicle()
        self._geom = Geom.from_vehicle(v) if v is not None else None
        iface.messageBar().pushMessage(
            "Draw route",
            "Click or drag to add points (the start is the vehicle). R = forward/reverse. "
            "Right-click to finish, Backspace = undo point, Esc = cancel.",
            level=Qgis.Info,
            duration=8
        )
        self._refresh(fine=True)

    def deactivate(self):
        # Keep the preview visible while the vehicle drives the finished route
        if not self._finished:
            self._clear_bands()
        super().deactivate()

    def reset(self):
        """Clear the drawing."""
        self._runs = [{"dir": FORWARD, "pts": []}]
        self._finished = False
        self._dragging = False
        self._last_px = None
        self._last_cursor = None
        self._result = None
        self._clear_bands()

    def _clear_bands(self):
        for band, geom_type in self._bands:
            band.reset(geom_type)

    # ------------------------------------------------------------------
    def _vehicle(self):
        if self._vehicle_getter is None:
            return None
        try:
            v = self._vehicle_getter()
            if v is not None and getattr(v, "f", None) is not None:
                return v
        except Exception:
            pass
        return None

    @property
    def direction(self) -> int:
        return self._runs[-1]["dir"]

    # ------------------------------------------------------------------
    # Drawing the preview
    # ------------------------------------------------------------------
    @staticmethod
    def _add_part(band, pts, part):
        last = len(pts) - 1
        for i, (x, y) in enumerate(pts):
            band.addPoint(QgsPointXY(x, y), i == last, part)

    def _refresh(self, cursor: QgsPointXY = None, fine: bool = False):
        self._last_preview_time = time.monotonic()
        self._clear_bands()

        # Points as (direction, [(x, y), ...]), with the cursor as a provisional last point
        raw = [(r["dir"], [(p.x(), p.y()) for p in r["pts"]]) for r in self._runs]
        if cursor is not None and not self._finished:
            pts = raw[-1][1]
            if not pts or math.hypot(cursor.x() - pts[-1][0], cursor.y() - pts[-1][1]) > 1e-9:
                pts.append((cursor.x(), cursor.y()))

        v = self._vehicle()
        if v is None or self._geom is None:
            self._draw_guide(raw, None)
            return

        try:
            fx, fy, th = float(v.f.x), float(v.f.y), float(v.a)
        except (AttributeError, TypeError, ValueError):
            self._draw_guide(raw, None)
            return

        # Trailer: pick up what the real run has learned about the hitch, and the current angle
        th2 = None
        tg = self._geom.trailer
        if tg is not None:
            tg.sync()
            st = read_articulation(v, self._geom)
            if st is not None:
                th2 = st["th2"]

        sim_runs = [{"dir": d, "pts": pts} for d, pts in raw if pts]
        res = simulate(self._geom, fx, fy, th, sim_runs, step=self.PREVIEW_STEP, th2=th2)
        self._result = res

        self._draw_guide(raw, (fx, fy, th))
        self._draw_paths(res)
        self._draw_envelope(res, fine)
        self._draw_ghosts(res)
        self._draw_failure(res)

        status = iface.mainWindow().statusBar()
        name = "REVERSE" if self.direction == REVERSE else "FORWARD"
        text = "Route direction: " + name
        if tg is not None and res.samples:
            text += "  |  max trailer angle {:.0f} deg (limit {:.0f})".format(
                math.degrees(res.max_beta), math.degrees(tg.limit))
        if res.fail_index is not None:
            status.showMessage("Route not possible from the red part: " + res.fail_reason)
        else:
            status.showMessage(text)

    def _draw_guide(self, raw, start):
        part = 0
        for d, pts in raw:
            if not pts:
                continue
            line = list(pts)
            if start is not None and part == 0:
                line = [(start[0], start[1])] + line
            self._add_part(self._band_guide, line, part)
            part += 1
        if part:
            self._band_guide.show()

    def _tracked(self, rx, ry, th, th2, d):
        """The point that follows the drawn line: front axle forward, rear axle (trailer axle) in reverse."""
        g = self._geom
        if d > 0:
            return g.front_from_rear(rx, ry, th)
        if g.trailer is not None:
            return g.trailer.axle_from(rx, ry, th, th2)
        return (rx, ry)

    def _draw_paths(self, res):
        parts = {FORWARD: [], REVERSE: []}
        current = None
        cur_dir = None
        end = res.fail_index if res.fail_index is not None else None
        for (rx, ry, th, _delta, d, _seg, th2) in res.samples[:end]:
            if cur_dir != d or current is None:
                current = []
                parts[d].append(current)
                cur_dir = d
            current.append(self._tracked(rx, ry, th, th2, d))
        for d, band in ((FORWARD, self._band_fwd), (REVERSE, self._band_rev)):
            idx = 0
            for pts in parts[d]:
                if len(pts) >= 2:
                    self._add_part(band, pts, idx)
                    idx += 1
            if idx:
                band.show()

    def _bodies(self, rx, ry, th, th2):
        """Outlines of the tractor and, if present, the trailer."""
        g = self._geom
        out = [g.outline(rx, ry, th)]
        if g.trailer is not None:
            out.append(g.trailer_outline(rx, ry, th, th2))
        return out

    def _draw_envelope(self, res, fine=False):
        """Swept area of the vehicle body; its edge shows the outer lines of the vehicle."""
        end = res.fail_index if res.fail_index is not None else None
        samples = res.samples[:end]
        if not samples:
            return
        step = self.ENVELOPE_STEP_FINE if fine else self.ENVELOPE_STEP
        every = max(1, int(round(step / self.PREVIEW_STEP)))
        every = max(every, int(math.ceil(len(samples) / float(self.ENVELOPE_MAX_OUTLINES))))
        picks = samples[::every]
        if picks[-1] is not samples[-1]:
            picks.append(samples[-1])

        polys = []
        for (rx, ry, th, _d, _dir, _seg, th2) in picks:
            for body in self._bodies(rx, ry, th, th2):
                ring = [QgsPointXY(x, y) for x, y in body]
                ring.append(ring[0])
                polys.append(QgsGeometry.fromPolygonXY([ring]))

        union = None
        try:
            union = QgsGeometry.unaryUnion(polys)
        except Exception:
            union = None

        rings = []
        if union is not None and not union.isEmpty():
            try:
                parts = union.asMultiPolygon() if union.isMultipart() else [union.asPolygon()]
                for poly in parts:
                    if poly and poly[0]:
                        rings.append([(p.x(), p.y()) for p in poly[0]])
            except Exception:
                rings = []
        if not rings:
            # Fallback: outline of every sampled position
            rings = [[(p.x(), p.y()) for p in poly.asPolygon()[0]] for poly in polys[::3]]

        for i, ring in enumerate(rings):
            self._add_part(self._band_envelope, ring, i)
        self._band_envelope.show()

    def _draw_ghosts(self, res):
        idx = 0
        for (rx, ry, th, th2) in res.ghosts:
            for body in self._bodies(rx, ry, th, th2):
                self._add_part(self._band_ghost, body, idx)
                idx += 1
        if idx:
            self._band_ghost.show()

    def _draw_failure(self, res):
        if res.fail_index is None or not res.samples:
            return
        # The last stretch before the failure, as a red line, and the vehicle where it gave up
        tail = res.samples[max(0, res.fail_index - 40): res.fail_index]
        pts = [self._tracked(rx, ry, th, th2, d) for (rx, ry, th, _dl, d, _seg, th2) in tail]
        if len(pts) >= 2:
            self._add_part(self._band_fail, pts, 0)
            self._band_fail.show()
        last = res.samples[min(res.fail_index, len(res.samples)) - 1]
        for i, body in enumerate(self._bodies(last[0], last[1], last[2], last[6])):
            self._add_part(self._band_fail_ghost, body, i)
        self._band_fail_ghost.show()

    # ------------------------------------------------------------------
    # Editing the route
    # ------------------------------------------------------------------
    def _add_point(self, pt: QgsPointXY, px):
        if self._last_px is not None:
            dx = px.x() - self._last_px.x()
            dy = px.y() - self._last_px.y()
            if dx * dx + dy * dy < self.DUPLICATE_PIXELS ** 2:
                return
        self._runs[-1]["pts"].append(QgsPointXY(pt))
        self._last_px = px

    def _toggle_direction(self):
        cur = self._runs[-1]
        if cur["pts"]:
            # Start a new part in the opposite direction (the vehicle stops where the last part ended)
            self._runs.append({"dir": -cur["dir"], "pts": []})
        elif len(self._runs) > 1:
            # Toggled again before drawing anything: continue the previous part
            self._runs.pop()
        else:
            # Nothing drawn yet: just choose the starting direction
            cur["dir"] = -cur["dir"]
        self._last_px = None

    def _finish(self):
        segments = [(list(r["pts"]), r["dir"]) for r in self._runs if r["pts"]]
        if not segments:
            iface.messageBar().pushMessage(
                "Draw route", "Click at least one point.",
                level=Qgis.Warning, duration=4
            )
            return
        self._finished = True
        self._dragging = False
        self._refresh(fine=True)
        if self._result is not None and self._result.fail_index is not None:
            iface.messageBar().pushMessage(
                "Draw route",
                "Part of this route can't be driven (shown in red). The vehicle will stop there.",
                level=Qgis.Warning, duration=6
            )
        self.routeFinished.emit(segments)

    # ------------------------------------------------------------------
    # Mouse and keyboard
    # ------------------------------------------------------------------
    def canvasPressEvent(self, event):
        if self._finished:
            return
        if event.button() == Qt.MouseButton.LeftButton:
            self._add_point(event.mapPoint(), event.pixelPoint())
            self._dragging = True
            self._refresh(event.mapPoint(), fine=True)
        elif event.button() == Qt.MouseButton.RightButton:
            self._finish()

    def canvasMoveEvent(self, event):
        if self._finished:
            return
        self._last_cursor = event.mapPoint()
        if self._dragging and self._last_px is not None:
            px = event.pixelPoint()
            dx = px.x() - self._last_px.x()
            dy = px.y() - self._last_px.y()
            if dx * dx + dy * dy >= self.FREEHAND_MIN_PIXELS ** 2:
                self._add_point(event.mapPoint(), px)
        if time.monotonic() - self._last_preview_time >= self.PREVIEW_INTERVAL:
            self._refresh(event.mapPoint())

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            if not self._finished:
                self._refresh(event.mapPoint(), fine=True)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key.Key_Escape:
            self.reset()
            self.routeCancelled.emit()
            event.accept()
        elif key == Qt.Key.Key_R:
            if not self._finished:
                self._toggle_direction()
                self._refresh(self._last_cursor, fine=True)
            event.accept()
        elif key == Qt.Key.Key_Backspace:
            if not self._finished:
                cur = self._runs[-1]
                if cur["pts"]:
                    cur["pts"].pop()
                elif len(self._runs) > 1:
                    self._runs.pop()
                self._last_px = None
                self._refresh(self._last_cursor, fine=True)
            event.accept()
        elif key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if not self._finished:
                self._finish()
            event.accept()
        else:
            event.ignore()
