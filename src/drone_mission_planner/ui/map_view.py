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
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask, Obstacle


class ToolMode(StrEnum):
    SELECT = "select"
    BASE = "base"
    DRONE = "drone"
    OBSTACLE = "obstacle"
    TASK = "task"
    DELETE = "delete"


class MapView(QGraphicsView):
    create_point_requested = Signal(str, float, float)
    create_rect_requested = Signal(float, float, float, float)
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
        for obstacle in self._model.obstacles:
            self._add_obstacle_item(obstacle)
        for base in self._model.bases:
            self._add_base_item(base)
        for task in self._model.tasks:
            self._add_task_item(task)
        for drone in self._model.drones:
            self._add_drone_item(drone)

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
            if self._mode == ToolMode.OBSTACLE:
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
                self.create_rect_requested.emit(rect.x(), rect.y(), rect.width(), rect.height())
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
        item.setPen(QPen(QColor("#87aefe"), 2))
        item.setBrush(QColor("#2f6de0"))
        self._tag(item, drone.id)
        self._scene.addItem(item)
        halo = QGraphicsEllipseItem(-17, -17, 34, 34, item)
        halo.setPen(QPen(QColor(77, 141, 247, 90), 1))
        halo.setBrush(Qt.BrushStyle.NoBrush)
        self._add_label(drone.id, drone.position.x + 16, drone.position.y - 12, "#a9c5ff")

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
