from __future__ import annotations

from enum import StrEnum
from itertools import pairwise
from math import cos, radians, sin

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

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
    SearchArea,
)

ISO_COS = cos(radians(30.0))
ISO_SIN = sin(radians(30.0))


class ToolMode(StrEnum):
    SELECT = "select"
    BASE = "base"
    DRONE = "drone"
    OBSTACLE = "obstacle"
    NO_FLY = "no_fly"
    TASK = "task"
    SEARCH_AREA = "search_area"
    DELETE = "delete"


class RenderMode(StrEnum):
    TWO_D = "2d"
    TERRAIN_25D = "terrain_25d"


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
        self._render_mode = RenderMode.TWO_D
        self._zoom = 1.0
        self._iso_origin = QPointF(0.0, 0.0)
        self._z_scale = 0.45
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

    @property
    def render_mode(self) -> RenderMode:
        return self._render_mode

    def set_mode(self, mode: ToolMode) -> None:
        if self._render_mode == RenderMode.TERRAIN_25D and mode != ToolMode.SELECT:
            mode = ToolMode.SELECT
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
        self._sync_scene_rect()
        self.render_model()

    def set_render_mode(self, mode: RenderMode) -> None:
        self._render_mode = mode
        if mode == RenderMode.TERRAIN_25D:
            self.set_mode(ToolMode.SELECT)
        self._sync_scene_rect()
        self.render_model()
        self.reset_view()

    def render_model(self) -> None:
        self._scene.clear()
        self._sync_scene_rect()
        if self._render_mode == RenderMode.TERRAIN_25D:
            self._render_model_25d()
            return
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
        if self._render_mode == RenderMode.TERRAIN_25D:
            return self._project(point, self._terrain_altitude(point))
        return QPointF(point.x, point.y)

    def scene_to_world(self, point: QPointF) -> Point:
        if self._render_mode == RenderMode.TERRAIN_25D:
            sx = point.x() - self._iso_origin.x()
            sy = point.y() - self._iso_origin.y()
            x_plus_y = sy / ISO_SIN
            x_minus_y = sx / ISO_COS
            return Point((x_plus_y + x_minus_y) / 2.0, (x_plus_y - x_minus_y) / 2.0)
        return Point(point.x(), point.y())

    def screen_to_world(self, point: QPoint) -> Point:
        return self.scene_to_world(self.mapToScene(point))

    def drawBackground(self, painter: QPainter, rect: QRectF | QRect) -> None:
        view_rect = QRectF(rect)
        painter.fillRect(view_rect, QColor("#0a1019"))
        if self._render_mode == RenderMode.TERRAIN_25D:
            painter.setPen(QPen(QColor(42, 58, 77, 150), 1.3))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(self.sceneRect())
            return
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
            if self._render_mode == RenderMode.TERRAIN_25D and self._mode != ToolMode.SELECT:
                event.accept()
                return
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

    def _sync_scene_rect(self) -> None:
        if self._render_mode == RenderMode.TWO_D:
            self.setSceneRect(0, 0, self._model.width, self._model.height)
            return
        altitude_candidates = [
            self._model.terrain.max_altitude,
            *(drone.cruise_altitude for drone in self._model.drones),
            *(obstacle.height for obstacle in self._model.obstacles),
            *(zone.ceiling_altitude for zone in self._model.no_fly_zones),
        ]
        max_altitude = max(altitude_candidates, default=0.0)
        self._iso_origin = QPointF(
            self._model.height * ISO_COS + 90.0,
            max_altitude * self._z_scale + 90.0,
        )
        corners = [
            Point(0.0, 0.0),
            Point(float(self._model.width), 0.0),
            Point(float(self._model.width), float(self._model.height)),
            Point(0.0, float(self._model.height)),
        ]
        projected = [self._project(point, 0.0) for point in corners]
        projected.extend(self._project(point, max_altitude + 80.0) for point in corners)
        min_x = min(point.x() for point in projected) - 80.0
        max_x = max(point.x() for point in projected) + 80.0
        min_y = min(point.y() for point in projected) - 80.0
        max_y = max(point.y() for point in projected) + 80.0
        self.setSceneRect(min_x, min_y, max_x - min_x, max_y - min_y)

    def _render_model_25d(self) -> None:
        self._add_terrain_mesh()
        for area in self._model.search_areas:
            self._add_search_area_item_25d(area)
        self._add_coverage_overlay_25d()
        self._add_communication_links_25d()
        for obstacle in self._model.obstacles:
            self._add_obstacle_item_25d(obstacle)
        for zone in self._model.no_fly_zones:
            self._add_no_fly_item_25d(zone)
        for index, drone in enumerate(self._model.drones):
            if drone.planned_path:
                self._add_route_25d(drone, index)
        for base in self._model.bases:
            self._add_base_item_25d(base)
        for task in self._model.tasks:
            self._add_task_item_25d(task)
        for drone in self._model.drones:
            self._add_drone_item_25d(drone)

    def _project(self, point: Point, altitude: float = 0.0) -> QPointF:
        return QPointF(
            (point.x - point.y) * ISO_COS + self._iso_origin.x(),
            (point.x + point.y) * ISO_SIN - altitude * self._z_scale + self._iso_origin.y(),
        )

    def _terrain_altitude(self, point: Point) -> float:
        return self._model.terrain.altitude_at(point.x, point.y)

    def _flight_altitude(self, drone: Drone, point: Point) -> float:
        return max(
            drone.cruise_altitude,
            self._terrain_altitude(point) + drone.min_clearance,
        )

    def _add_terrain_mesh(self) -> None:
        step = max(25.0, self._model.terrain.resolution, self._model.grid_size * 2.0)
        x_values = _sample_axis(float(self._model.width), step)
        y_values = _sample_axis(float(self._model.height), step)
        for y1, y2 in pairwise(y_values):
            for x1, x2 in pairwise(x_values):
                corners = [
                    Point(x1, y1),
                    Point(x2, y1),
                    Point(x2, y2),
                    Point(x1, y2),
                ]
                altitudes = [self._terrain_altitude(point) for point in corners]
                path = _closed_path(
                    [
                        self._project(point, altitude)
                        for point, altitude in zip(corners, altitudes, strict=True)
                    ]
                )
                item = QGraphicsPathItem(path)
                item.setPen(QPen(QColor(30, 45, 58, 80), 0))
                item.setBrush(self._terrain_brush(sum(altitudes) / len(altitudes)))
                item.setZValue(-30 + (y1 + y2) / max(self._model.height, 1.0))
                self._scene.addItem(item)

    def _terrain_brush(self, altitude: float) -> QColor:
        terrain = self._model.terrain
        span = max(1.0, terrain.max_altitude - terrain.min_altitude)
        ratio = max(0.0, min(1.0, (altitude - terrain.min_altitude) / span))
        if ratio < 0.45:
            local = ratio / 0.45
            return _mix_color(QColor("#234b3a"), QColor("#6d8f54"), local)
        if ratio < 0.8:
            local = (ratio - 0.45) / 0.35
            return _mix_color(QColor("#6d8f54"), QColor("#9a7653"), local)
        local = (ratio - 0.8) / 0.2
        return _mix_color(QColor("#9a7653"), QColor("#eef2f5"), local)

    def _add_obstacle_item_25d(self, obstacle: Obstacle) -> None:
        self._add_prism(
            obstacle.bounds.normalized,
            top_altitude=obstacle.height,
            color=QColor("#b64652"),
            outline=QColor("#ff9aa6"),
            object_id=obstacle.id,
        )
        anchor = self._project(
            Point(obstacle.bounds.normalized.x, obstacle.bounds.normalized.y),
            obstacle.height,
        )
        self._add_label(obstacle.id, anchor.x() + 5, anchor.y() - 8, "#ffb2bc")

    def _add_no_fly_item_25d(self, zone: NoFlyZone) -> None:
        self._add_prism(
            zone.bounds.normalized,
            top_altitude=zone.ceiling_altitude,
            color=QColor(141, 73, 190, 115),
            outline=QColor("#dda8ff"),
            object_id=zone.id,
        )
        anchor = self._project(
            Point(zone.bounds.normalized.x, zone.bounds.normalized.y),
            zone.ceiling_altitude,
        )
        self._add_label(zone.id, anchor.x() + 5, anchor.y() - 8, "#e4bdff")

    def _add_prism(
        self,
        rect: QRectF | Rect,
        *,
        top_altitude: float,
        color: QColor,
        outline: QColor,
        object_id: str,
    ) -> None:
        bounds = (
            rect if isinstance(rect, Rect) else Rect(rect.x(), rect.y(), rect.width(), rect.height())
        )
        corners = [
            Point(bounds.x, bounds.y),
            Point(bounds.x + bounds.width, bounds.y),
            Point(bounds.x + bounds.width, bounds.y + bounds.height),
            Point(bounds.x, bounds.y + bounds.height),
        ]
        base = [self._project(point, self._terrain_altitude(point)) for point in corners]
        top = [
            self._project(point, self._terrain_altitude(point) + top_altitude)
            for point in corners
        ]
        side_color = QColor(color)
        side_color.setAlpha(max(60, min(190, color.alpha() - 20)))
        for index in range(4):
            next_index = (index + 1) % 4
            side = QGraphicsPathItem(
                _closed_path([base[index], base[next_index], top[next_index], top[index]])
            )
            side.setPen(QPen(outline, 0.8))
            side.setBrush(side_color)
            side.setZValue(-5 + index * 0.01)
            self._tag(side, object_id)
            self._scene.addItem(side)
        top_item = QGraphicsPathItem(_closed_path(top))
        top_item.setPen(QPen(outline, 1.5))
        top_item.setBrush(color)
        top_item.setZValue(2)
        self._tag(top_item, object_id)
        self._scene.addItem(top_item)

    def _add_search_area_item_25d(self, area: SearchArea) -> None:
        polygon = area.polygon()
        if not polygon:
            return
        path = _closed_path(
            [self._project(point, self._terrain_altitude(point) + 1.0) for point in polygon]
        )
        item = QGraphicsPathItem(path)
        item.setPen(QPen(QColor("#4ce0d2"), 2, Qt.PenStyle.DashLine))
        item.setBrush(QColor(35, 148, 140, 35))
        item.setZValue(-4)
        self._tag(item, area.id)
        self._scene.addItem(item)
        anchor = min(polygon, key=lambda point: (point.y, point.x))
        projected = self._project(anchor, self._terrain_altitude(anchor) + 2.0)
        progress = self._coverage_progress.get(area.id, 0.0)
        self._add_label(
            f"{area.id}  •  {progress:.1%} covered",
            projected.x() + 7,
            projected.y() + 7,
            "#78f1e5",
        )

    def _add_coverage_overlay_25d(self) -> None:
        for area_id, cells in self._coverage_cells.items():
            resolution = self._coverage_resolutions.get(area_id, 0.0)
            if resolution <= 0:
                continue
            size = max(3.0, min(9.0, resolution * 0.15))
            for center, visit_count in cells:
                altitude = self._terrain_altitude(center) + 2.0
                projected = self._project(center, altitude)
                item = QGraphicsEllipseItem(-size / 2, -size / 2, size, size)
                item.setPos(projected)
                item.setBrush(
                    QColor(249, 202, 91, 125)
                    if visit_count >= 2
                    else QColor(85, 214, 190, 110)
                )
                item.setPen(Qt.PenStyle.NoPen)
                item.setZValue(-2)
                item.setData(0, area_id)
                item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
                self._scene.addItem(item)

    def _add_communication_links_25d(self) -> None:
        for start, end in self._communication_links:
            path = QPainterPath(self._project(start, self._terrain_altitude(start) + 20.0))
            path.lineTo(self._project(end, self._terrain_altitude(end) + 20.0))
            item = QGraphicsPathItem(path)
            item.setPen(QPen(QColor(85, 214, 190, 105), 1.4, Qt.PenStyle.DashLine))
            item.setZValue(4)
            self._scene.addItem(item)

    def _add_route_25d(self, drone: Drone, index: int) -> None:
        if len(drone.planned_path) < 2:
            return
        colors = ["#4d8df7", "#55d6be", "#f9ca5b", "#c77dff", "#ff7a90"]
        first = drone.planned_path[0]
        route = QPainterPath(self._project(first, self._flight_altitude(drone, first)))
        for point in drone.planned_path[1:]:
            route.lineTo(self._project(point, self._flight_altitude(drone, point)))
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
        halo.setZValue(6)
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
        item.setZValue(7)
        self._scene.addItem(item)

    def _add_base_item_25d(self, base: BaseStation) -> None:
        projected = self._project(base.position, self._terrain_altitude(base.position) + 4.0)
        item = QGraphicsEllipseItem(-8, -8, 16, 16)
        item.setPos(projected)
        item.setPen(QPen(QColor("#a7fff0"), 2))
        item.setBrush(QColor("#46bba6"))
        item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self._tag(item, base.id)
        self._scene.addItem(item)
        self._add_label(base.id, projected.x() + 11, projected.y() - 13, "#86efdc")

    def _add_task_item_25d(self, task: MissionTask) -> None:
        altitude = max(task.target_altitude, self._terrain_altitude(task.position) + 8.0)
        projected = self._project(task.position, altitude)
        item = QGraphicsEllipseItem(-6, -6, 12, 12)
        item.setPos(projected)
        item.setPen(QPen(QColor("#ffe396"), 2))
        item.setBrush(QColor("#9e7118"))
        item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self._tag(item, task.id)
        self._scene.addItem(item)
        self._add_label(
            f"{task.id}  {altitude:.0f}m",
            projected.x() + 9,
            projected.y() - 12,
            "#ffe396",
        )

    def _add_drone_item_25d(self, drone: Drone) -> None:
        altitude = self._flight_altitude(drone, drone.position)
        projected = self._project(drone.position, altitude)
        failed = drone.status.value in {"failed", "emergency"}
        path = QPainterPath()
        path.moveTo(0, -9)
        path.lineTo(8, 7)
        path.lineTo(0, 3)
        path.lineTo(-8, 7)
        path.closeSubpath()
        item = QGraphicsPathItem(path)
        item.setPos(projected)
        item.setPen(QPen(QColor("#ff9aa6" if failed else "#a9c5ff"), 2))
        item.setBrush(QColor("#b83449" if failed else "#2f6de0"))
        item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        item.setZValue(12)
        self._tag(item, drone.id)
        self._scene.addItem(item)
        ground = self._project(drone.position, self._terrain_altitude(drone.position))
        tether = QPainterPath(ground)
        tether.lineTo(projected)
        line = QGraphicsPathItem(tether)
        line.setPen(QPen(QColor(169, 197, 255, 110), 1.1, Qt.PenStyle.DotLine))
        line.setZValue(5)
        self._scene.addItem(line)
        label = f"{drone.id}  FAILED" if failed else f"{drone.id}  {altitude:.0f}m"
        self._add_label(
            label,
            projected.x() + 12,
            projected.y() - 14,
            "#ff9aa6" if failed else "#a9c5ff",
        )

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


def _sample_axis(limit: float, step: float) -> list[float]:
    values = [0.0]
    cursor = step
    while cursor < limit:
        values.append(cursor)
        cursor += step
    if values[-1] != limit:
        values.append(limit)
    return values


def _closed_path(points: list[QPointF]) -> QPainterPath:
    if not points:
        return QPainterPath()
    path = QPainterPath(points[0])
    for point in points[1:]:
        path.lineTo(point)
    path.closeSubpath()
    return path


def _mix_color(first: QColor, second: QColor, ratio: float) -> QColor:
    clamped = max(0.0, min(1.0, ratio))
    return QColor(
        round(first.red() + (second.red() - first.red()) * clamped),
        round(first.green() + (second.green() - first.green()) * clamped),
        round(first.blue() + (second.blue() - first.blue()) * clamped),
        round(first.alpha() + (second.alpha() - first.alpha()) * clamped),
    )
