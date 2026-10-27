# Map tool for drawing a route by hand
from qgis.PyQt.QtCore import Qt, pyqtSignal
from qgis.PyQt.QtGui import QColor
from qgis.core import Qgis, QgsPointXY, QgsWkbTypes
from qgis.gui import QgsMapTool, QgsMapCanvas, QgsRubberBand
from qgis.utils import iface


class RouteTool(QgsMapTool):
    """
    Draw the route the vehicle should follow.

    - Left click          : add a vertex
    - Left press and drag : freehand drawing
    - Right click / Enter : finish the route
    - Backspace           : remove the last vertex
    - Esc                 : cancel
    """

    # Emitted with a list of QgsPointXY (map canvas CRS)
    routeFinished: pyqtSignal = pyqtSignal(list, name="routeFinished")
    routeCancelled: pyqtSignal = pyqtSignal(name="routeCancelled")

    # Minimum screen distance (pixels) between freehand vertices / duplicate clicks
    FREEHAND_MIN_PIXELS: int = 8
    DUPLICATE_PIXELS: int = 2

    def __init__(self, canvas: QgsMapCanvas):
        super().__init__(canvas)
        self._canvas = canvas
        self._points: list = []
        self._finished: bool = False
        self._dragging: bool = False
        self._last_px = None

        try:
            self._geom_type = Qgis.GeometryType.Line
        except AttributeError:
            self._geom_type = QgsWkbTypes.LineGeometry

        self._band = QgsRubberBand(canvas, self._geom_type)
        self._band.setColor(QColor(255, 120, 0, 220))
        self._band.setWidth(2)
        self._band.setLineStyle(Qt.PenStyle.DashLine)

        self.setCursor(Qt.CursorShape.CrossCursor)

    # ------------------------------------------------------------------
    def activate(self):
        super().activate()
        self.reset()
        iface.messageBar().pushMessage(
            "Draw route",
            "Click or drag to draw. Right-click to finish, Backspace to undo, Esc to cancel.",
            level=Qgis.Info,
            duration=6
        )

    def reset(self):
        """Clear the drawing."""
        self._points = []
        self._finished = False
        self._dragging = False
        self._last_px = None
        self._band.reset(self._geom_type)

    # ------------------------------------------------------------------
    def _refresh(self, cursor: QgsPointXY = None):
        self._band.reset(self._geom_type)
        pts = list(self._points)
        if cursor is not None and pts:
            pts.append(cursor)
        for i, p in enumerate(pts):
            self._band.addPoint(p, i == len(pts) - 1)

    def _add_point(self, pt: QgsPointXY, px):
        if self._last_px is not None:
            dx = px.x() - self._last_px.x()
            dy = px.y() - self._last_px.y()
            if dx * dx + dy * dy < self.DUPLICATE_PIXELS ** 2:
                return
        self._points.append(QgsPointXY(pt))
        self._last_px = px

    def _finish(self):
        if len(self._points) < 2:
            iface.messageBar().pushMessage(
                "Draw route", "A route needs at least two points.",
                level=Qgis.Warning, duration=4
            )
            return
        self._finished = True
        self._dragging = False
        self._refresh()
        self.routeFinished.emit(list(self._points))

    # ------------------------------------------------------------------
    def canvasPressEvent(self, event):
        if self._finished:
            return
        if event.button() == Qt.MouseButton.LeftButton:
            self._add_point(event.mapPoint(), event.pixelPoint())
            self._dragging = True
            self._refresh(event.mapPoint())
        elif event.button() == Qt.MouseButton.RightButton:
            self._finish()

    def canvasMoveEvent(self, event):
        if self._finished:
            return
        if self._dragging and self._last_px is not None:
            px = event.pixelPoint()
            dx = px.x() - self._last_px.x()
            dy = px.y() - self._last_px.y()
            if dx * dx + dy * dy >= self.FREEHAND_MIN_PIXELS ** 2:
                self._add_point(event.mapPoint(), px)
        self._refresh(event.mapPoint())

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key.Key_Escape:
            self.reset()
            self.routeCancelled.emit()
            event.accept()
        elif key == Qt.Key.Key_Backspace:
            if self._points and not self._finished:
                self._points.pop()
                self._last_px = None
                self._refresh()
            event.accept()
        elif key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            if not self._finished:
                self._finish()
            event.accept()
        else:
            event.ignore()