from __future__ import annotations

from enum import StrEnum

from PySide6.QtCore import QPoint, QPointF, QRect, QRectF, Qt, Signal
from PySide6.QtGui import (
    QBrush,
    QColor,
    QKeyEvent,
    QMouseEvent,
    QPainter,
    QPainterPath,
    QPen,
    QWheelEvent,
)
from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QWidget,
)

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
    SearchArea,
)


class ToolMode(StrEnum):
    SELECT = "select"
    BASE = "base"
    DRONE = "drone"
    OBSTACLE = "obstacle"
    NO_FLY = "no_fly"
    TASK = "task"
    SEARCH_AREA = "search_area"
    DELETE = "delete"


class MapView(QGraphicsView):
    create_point_requested = Signal(str, float, float)
    create_rect_requested = Signal(str, float, float, float, float)
    object_selected = Signal(str)
    delete_requested = Signal(str)
    coordinates_changed = Signal(float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("MapView")
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._model = MapModel()
        self._mode = ToolMode.SELECT
        self._zoom = 1.0
        self._space_down = False
        self._panning = False
        self._pan_start = QPoint()
        self._drag_origin: QPointF | None = None
        self._preview: QGraphicsRectItem | None = None
        self._coverage_progress: dict[str, float] = {}
        self._coverage_cells: dict[str, tuple[tuple[Point, int], ...]] = {}
        self._coverage_resolutions: dict[str, float] = {}
        self._communication_links: tuple[tuple[Point, Point], ...] = ()
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing
            | QPainter.RenderHint.TextAntialiasing
            | QPainter.RenderHint.SmoothPixmapTransform
        )
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.SmartViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QColor("#0a1019"))
        self.setFrameShape(QGraphicsView.Shape.NoFrame)
        self.setSceneRect(0, 0, self._model.width, self._model.height)

    @property
    def mode(self) -> ToolMode:
        return self._mode

    def set_mode(self, mode: ToolMode) -> None:
        self._mode = mode
        self.setDragMode(
            QGraphicsView.DragMode.RubberBandDrag
            if mode == ToolMode.SELECT
            else QGraphicsView.DragMode.NoDrag
        )
        cursors = {
            ToolMode.SELECT: Qt.CursorShape.ArrowCursor,
            ToolMode.DELETE: Qt.CursorShape.ForbiddenCursor,
        }
        self.viewport().setCursor(cursors.get(mode, Qt.CursorShape.CrossCursor))

    def set_model(self, model: MapModel) -> None:
        self._model = model
        self.setSceneRect(0, 0, model.width, model.height)
        self.render_model()

    def render_model(self) -> None:
        self._scene.clear()
        for area in self._model.search_areas:
            self._add_search_area_item(area)
        self._add_coverage_overlay()
        self._add_communication_links()
        for obstacle in self._model.obstacles:
            self._add_obstacle_item(obstacle)
        for zone in self._model.no_fly_zones:
            self._add_no_fly_item(zone)
        for index, drone in enumerate(self._model.drones):
            if drone.planned_path:
                self._add_route(drone, index)
        for base in self._model.bases:
            self._add_base_item(base)
        for task in self._model.tasks:
            self._add_task_item(task)
        for drone in self._model.drones:
            self._add_drone_item(drone)

    def set_coverage_overlay(
        self,
        progress: dict[str, float],
        cells: dict[str, tuple[tuple[Point, int], ...]],
        resolutions: dict[str, float],
    ) -> None:
        self._coverage_progress = dict(progress)
        self._coverage_cells = dict(cells)
        self._coverage_resolutions = dict(resolutions)

    def clear_coverage_overlay(self) -> None:
        self._coverage_progress.clear()
        self._coverage_cells.clear()
        self._coverage_resolutions.clear()

    def set_communication_links(self, links: tuple[tuple[Point, Point], ...]) -> None:
        self._communication_links = links

    def clear_communication_links(self) -> None:
        self._communication_links = ()

    def reset_view(self) -> None:
        self.resetTransform()
        self._zoom = 1.0
        padded = self.sceneRect().adjusted(-30, -30, 30, 30)
        self.fitInView(padded, Qt.AspectRatioMode.KeepAspectRatio)
        self._zoom = self.transform().m11()

    def world_to_scene(self, point: Point) -> QPointF:
        return QPointF(point.x, point.y)

    def scene_to_world(self, point: QPointF) -> Point:
        return Point(point.x(), point.y())

    def screen_to_world(self, point: QPoint) -> Point:
        return self.scene_to_world(self.mapToScene(point))

    def drawBackground(self, painter: QPainter, rect: QRectF | QRect) -> None:
        view_rect = QRectF(rect)
        painter.fillRect(view_rect, QColor("#0a1019"))
        grid = max(5.0, self._model.grid_size)
        left = int(view_rect.left() // grid) * grid
        top = int(view_rect.top() // grid) * grid
        minor_pen = QPen(QColor(36, 49, 68, 115), 0)
        major_pen = QPen(QColor(52, 70, 96, 150), 0)
        x = left
        while x < view_rect.right():
            painter.setPen(major_pen if int(x / grid) % 4 == 0 else minor_pen)
            painter.drawLine(QPointF(x, view_rect.top()), QPointF(x, view_rect.bottom()))
            x += grid
        y = top
        while y < view_rect.bottom():
            painter.setPen(major_pen if int(y / grid) % 4 == 0 else minor_pen)
            painter.drawLine(QPointF(view_rect.left(), y), QPointF(view_rect.right(), y))
            y += grid
        painter.setPen(QPen(QColor("#3a4b64"), 1.5))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(self.sceneRect())

    def wheelEvent(self, event: QWheelEvent) -> None:
        factor = 1.18 if event.angleDelta().y() > 0 else 1 / 1.18
        next_zoom = self._zoom * factor
        if 0.2 <= next_zoom <= 6.0:
            self.scale(factor, factor)
            self._zoom = next_zoom
        event.accept()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_Space:
            self._space_down = True
            self.viewport().setCursor(Qt.CursorShape.OpenHandCursor)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_Space:
            self._space_down = False
            self.viewport().setCursor(Qt.CursorShape.ArrowCursor)
        super().keyReleaseEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.MiddleButton or (
            event.button() == Qt.MouseButton.LeftButton and self._space_down
        ):
            self._panning = True
            self._pan_start = event.position().toPoint()
            self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        world = self.screen_to_world(event.position().toPoint())
        if event.button() == Qt.MouseButton.LeftButton and self._inside(world):
            if self._mode in {ToolMode.BASE, ToolMode.DRONE, ToolMode.TASK}:
                self.create_point_requested.emit(self._mode.value, world.x, world.y)
                event.accept()
                return
            if self._mode in {ToolMode.OBSTACLE, ToolMode.NO_FLY, ToolMode.SEARCH_AREA}:
                self._drag_origin = QPointF(world.x, world.y)
                self._preview = self._scene.addRect(
                    QRectF(self._drag_origin, self._drag_origin),
                    QPen(QColor("#ffb54d"), 2, Qt.PenStyle.DashLine),
                    QBrush(QColor(255, 181, 77, 45)),
                )
                event.accept()
                return
            item = self.itemAt(event.position().toPoint())
            object_id = self._object_id(item)
            if self._mode == ToolMode.DELETE and object_id:
                self.delete_requested.emit(object_id)
                event.accept()
                return
            if object_id:
                self.object_selected.emit(object_id)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        world = self.screen_to_world(event.position().toPoint())
        self.coordinates_changed.emit(world.x, world.y)
        if self._panning:
            delta = event.position().toPoint() - self._pan_start
            self._pan_start = event.position().toPoint()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        if self._drag_origin is not None and self._preview is not None:
            self._preview.setRect(QRectF(self._drag_origin, QPointF(world.x, world.y)).normalized())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if self._panning:
            self._panning = False
            self.viewport().setCursor(
                Qt.CursorShape.OpenHandCursor if self._space_down else Qt.CursorShape.ArrowCursor
            )
            event.accept()
            return
        if self._drag_origin is not None and event.button() == Qt.MouseButton.LeftButton:
            world = self.screen_to_world(event.position().toPoint())
            rect = QRectF(self._drag_origin, QPointF(world.x, world.y)).normalized()
            if self._preview is not None:
                self._scene.removeItem(self._preview)
            self._preview = None
            self._drag_origin = None
            if rect.width() >= 5 and rect.height() >= 5:
                self.create_rect_requested.emit(
                    self._mode.value, rect.x(), rect.y(), rect.width(), rect.height()
                )
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _inside(self, point: Point) -> bool:
        return self._model.validate_position(point)

    @staticmethod
    def _object_id(item: QGraphicsItem | None) -> str | None:
        current = item
        while current is not None:
            value = current.data(0)
            if value:
                return str(value)
            current = current.parentItem()
        return None

    @staticmethod
    def _tag(item: QGraphicsItem, object_id: str) -> None:
        item.setData(0, object_id)
        item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

    def _add_label(self, text: str, x: float, y: float, color: str = "#ced9e9") -> None:
        label = QGraphicsSimpleTextItem(text)
        label.setBrush(QColor(color))
        label.setPos(x, y)
        label.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self._scene.addItem(label)

    def _add_base_item(self, base: BaseStation) -> None:
        ring = QGraphicsEllipseItem(-16, -16, 32, 32)
        ring.setPos(base.position.x, base.position.y)
        ring.setPen(QPen(QColor("#55d6be"), 2.5))
        ring.setBrush(QColor(17, 68, 67, 210))
        self._tag(ring, base.id)
        self._scene.addItem(ring)
        inner = QGraphicsRectItem(-6, -6, 12, 12, ring)
        inner.setPen(QPen(QColor("#a7fff0"), 1.5))
        inner.setBrush(QColor("#46bba6"))
        self._add_label(base.id, base.position.x + 19, base.position.y - 11, "#86efdc")

    def _add_drone_item(self, drone: Drone) -> None:
        path = QPainterPath()
        path.moveTo(0, -13)
        path.lineTo(11, 10)
        path.lineTo(0, 6)
        path.lineTo(-11, 10)
        path.closeSubpath()
        item = QGraphicsPathItem(path)
        item.setPos(drone.position.x, drone.position.y)
        failed = drone.status.value in {"failed", "emergency"}
        item.setPen(QPen(QColor("#ff9aa6" if failed else "#87aefe"), 2))
        item.setBrush(QColor("#b83449" if failed else "#2f6de0"))
        self._tag(item, drone.id)
        self._scene.addItem(item)
        halo = QGraphicsEllipseItem(-17, -17, 34, 34, item)
        halo.setPen(QPen(QColor(239, 106, 121, 150) if failed else QColor(77, 141, 247, 90), 1))
        halo.setBrush(Qt.BrushStyle.NoBrush)
        label = f"{drone.id}  FAILED" if failed else drone.id
        self._add_label(
            label,
            drone.position.x + 16,
            drone.position.y - 12,
            "#ff9aa6" if failed else "#a9c5ff",
        )

    def _add_task_item(self, task: MissionTask) -> None:
        outer = QGraphicsEllipseItem(-10, -10, 20, 20)
        outer.setPos(task.position.x, task.position.y)
        outer.setPen(QPen(QColor("#f9ca5b"), 2))
        outer.setBrush(QColor(101, 73, 18, 175))
        self._tag(outer, task.id)
        self._scene.addItem(outer)
        dot = QGraphicsEllipseItem(-3, -3, 6, 6, outer)
        dot.setPen(Qt.PenStyle.NoPen)
        dot.setBrush(QColor("#ffe396"))
        self._add_label(task.id, task.position.x + 13, task.position.y - 11, "#ffe396")

    def _add_obstacle_item(self, obstacle: Obstacle) -> None:
        bounds = obstacle.bounds.normalized
        item = QGraphicsRectItem(bounds.x, bounds.y, bounds.width, bounds.height)
        item.setPen(QPen(QColor("#ef6a79"), 1.8))
        item.setBrush(QColor(134, 40, 55, 150))
        self._tag(item, obstacle.id)
        self._scene.addItem(item)
        self._add_label(obstacle.id, bounds.x + 6, bounds.y + 5, "#ff9aa6")

    def _add_no_fly_item(self, zone: NoFlyZone) -> None:
        bounds = zone.bounds.normalized
        item = QGraphicsRectItem(bounds.x, bounds.y, bounds.width, bounds.height)
        item.setPen(QPen(QColor("#c77dff"), 2, Qt.PenStyle.DashLine))
        item.setBrush(QColor(100, 45, 135, 100))
        self._tag(item, zone.id)
        self._scene.addItem(item)
        self._add_label(zone.id, bounds.x + 6, bounds.y + 5, "#dda8ff")

    def _add_search_area_item(self, area: SearchArea) -> None:
        polygon = area.polygon()
        if not polygon:
            return
        path = QPainterPath(QPointF(polygon[0].x, polygon[0].y))
        for point in polygon[1:]:
            path.lineTo(point.x, point.y)
        path.closeSubpath()
        item = QGraphicsPathItem(path)
        item.setPen(QPen(QColor("#4ce0d2"), 2, Qt.PenStyle.DashLine))
        item.setBrush(QColor(35, 148, 140, 35))
        item.setZValue(-6)
        self._tag(item, area.id)
        self._scene.addItem(item)
        anchor = min(polygon, key=lambda point: (point.y, point.x))
        progress = self._coverage_progress.get(area.id, 0.0)
        self._add_label(
            f"{area.id}  •  {progress:.1%} covered",
            anchor.x + 7,
            anchor.y + 7,
            "#78f1e5",
        )

    def _add_coverage_overlay(self) -> None:
        for area_id, cells in self._coverage_cells.items():
            resolution = self._coverage_resolutions.get(area_id, 0.0)
            if resolution <= 0:
                continue
            size = resolution * 0.82
            for center, visit_count in cells:
                item = QGraphicsRectItem(
                    center.x - size / 2,
                    center.y - size / 2,
                    size,
                    size,
                )
                if visit_count >= 2:
                    item.setBrush(QColor(249, 202, 91, 105))
                else:
                    item.setBrush(QColor(85, 214, 190, 95))
                item.setPen(Qt.PenStyle.NoPen)
                item.setZValue(-5)
                item.setData(0, area_id)
                self._scene.addItem(item)

    def _add_communication_links(self) -> None:
        for start, end in self._communication_links:
            path = QPainterPath(QPointF(start.x, start.y))
            path.lineTo(end.x, end.y)
            item = QGraphicsPathItem(path)
            item.setPen(QPen(QColor(85, 214, 190, 105), 1.5, Qt.PenStyle.DashLine))
            item.setZValue(-3)
            self._scene.addItem(item)

    def _add_route(self, drone: Drone, index: int) -> None:
        if len(drone.planned_path) < 2:
            return
        colors = ["#4d8df7", "#55d6be", "#f9ca5b", "#c77dff", "#ff7a90"]
        route = QPainterPath(QPointF(drone.planned_path[0].x, drone.planned_path[0].y))
        for point in drone.planned_path[1:]:
            route.lineTo(point.x, point.y)
        halo = QGraphicsPathItem(route)
        halo.setPen(
            QPen(
                QColor(7, 12, 20, 210),
                6.5,
                Qt.PenStyle.SolidLine,
                Qt.PenCapStyle.RoundCap,
                Qt.PenJoinStyle.RoundJoin,
            )
        )
        halo.setZValue(-1)
        self._scene.addItem(halo)
        item = QGraphicsPathItem(route)
        item.setPen(
            QPen(
                QColor(colors[index % len(colors)]),
                2.8,
                Qt.PenStyle.SolidLine,
                Qt.PenCapStyle.RoundCap,
                Qt.PenJoinStyle.RoundJoin,
            )
        )
        item.setZValue(0)
        self._scene.addItem(item)
