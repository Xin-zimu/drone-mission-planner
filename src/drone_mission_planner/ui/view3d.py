"""Read-only software-rendered 3D mission view.

The view projects the deterministic scene from
:mod:`drone_mission_planner.ui.scene3d_export` with an orbit camera and
paints it with QPainter using a painter's algorithm. No external 3D or web
dependency is required, and the view never mutates mission data.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from math import cos, hypot, radians, sin, sqrt

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (
    QColor,
    QMouseEvent,
    QPainter,
    QPainterPath,
    QPaintEvent,
    QPen,
    QPolygonF,
    QWheelEvent,
)
from PySide6.QtWidgets import QLabel, QWidget

from drone_mission_planner.domain.geometry import Point

from .scene3d_export import (
    COVERAGE_COLOR_COVERED,
    COVERAGE_COLOR_UNCOVERED,
    WIND_COLOR,
    Scene3D,
    Scene3DMarker,
    Scene3DRoute,
)

BACKGROUND_COLOR = "#0a1019"
LABEL_TEXT_COLOR = "#dce5f3"
HIGHLIGHT_COLOR = "#f9ca5b"
SELECTED_PEN_WIDTH = 3.5
ROUTE_PEN_WIDTH = 2.0

VIEW_PRESETS: tuple[str, ...] = ("top", "iso", "side", "follow")
LAYERS: tuple[str, ...] = (
    "terrain",
    "routes",
    "obstacles",
    "no_fly",
    "coverage",
    "risks",
    "labels",
)

_FOCAL_FACTOR = 1.2
_MIN_PITCH_DEG = 3.0
_MAX_PITCH_DEG = 89.0
_MIN_DISTANCE = 40.0
_PICK_RADIUS_MARKER = 14.0
_PICK_RADIUS_ROUTE = 8.0


@dataclass(slots=True)
class _Camera:
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 50.0
    distance: float = 1200.0


class _Projection:
    """Perspective projection for the current camera and viewport."""

    def __init__(self, camera: _Camera, width: float, height: float) -> None:
        yaw = radians(camera.yaw_deg)
        pitch = radians(camera.pitch_deg)
        cp, sp = cos(pitch), sin(pitch)
        cy, sy = cos(yaw), sin(yaw)
        self._cam = (
            camera.target_x - camera.distance * cp * sy,
            camera.target_y + camera.distance * cp * cy,
            camera.target_z + camera.distance * sp,
        )
        self._right = (cy, sy, 0.0)
        self._up = (sp * sy, -sp * cy, cp)
        self._forward = (cp * sy, -cp * cy, -sp)
        self._focal = _FOCAL_FACTOR * max(1.0, min(width, height))
        self._width = width
        self._height = height

    def project(self, x: float, y: float, z: float) -> tuple[float, float, float] | None:
        vx = x - self._cam[0]
        vy = y - self._cam[1]
        vz = z - self._cam[2]
        depth = vx * self._forward[0] + vy * self._forward[1] + vz * self._forward[2]
        if depth < 1.0:
            return None
        right = vx * self._right[0] + vy * self._right[1] + vz * self._right[2]
        up = vx * self._up[0] + vy * self._up[1] + vz * self._up[2]
        scale = self._focal / depth
        return (
            self._width / 2.0 + right * scale,
            self._height / 2.0 - up * scale,
            depth,
        )


@dataclass(slots=True)
class _HitTarget:
    """Pickable object recorded during the last paint."""

    object_id: str
    kind: str
    points: tuple[tuple[float, float], ...]


class ThreeDView(QWidget):
    """Software 3D projection of terrain, routes, volumes, and drones."""

    object_selected = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(320, 240)
        self.setMouseTracking(False)
        self._scene: Scene3D | None = None
        self._camera = _Camera()
        self._selected_id: str | None = None
        self._layers = {layer: True for layer in LAYERS}
        self._colors: dict[str, QColor] = {}
        self._press_pos = QPointF(0.0, 0.0)
        self._press_button: Qt.MouseButton | None = None
        self._dragged = False
        self._hits: list[_HitTarget] = []
        self._current_painter: QPainter | None = None
        self._highlighted_waypoint: tuple[str, int] | None = None
        self._replay_markers: dict[str, tuple[float, float, float, str]] | None = None
        self._badge = QLabel("3D MISSION VIEW  •  READ ONLY", self)
        self._badge.setStyleSheet(
            "background: rgba(12,19,30,210); color: #8ea0b8; border: 1px solid #2b3a50; "
            "border-radius: 6px; padding: 7px 11px; font-size: 9pt;"
        )
        self._badge.setFixedWidth(240)
        self._badge.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self._badge.move(18, 18)

    # ------------------------------------------------------------------ API

    def set_scene(self, scene: Scene3D) -> None:
        """Replace the rendered scene, keeping the camera when possible."""

        previous = self._scene
        self._scene = scene
        self._colors = {}
        if previous is None or previous.width != scene.width or previous.height != scene.height:
            self._fit_camera(scene)
        self.update()

    def set_selected_object(self, object_id: str | None) -> None:
        self._selected_id = object_id
        self.update()

    def show_replay_markers(
        self, markers: Mapping[str, tuple[float, float, float, str]] | None
    ) -> None:
        """Display historical drone states; None returns to the live scene."""

        self._replay_markers = dict(markers) if markers else None
        self.update()

    def set_highlighted_waypoint(self, drone_id: str | None, index: int | None = None) -> None:
        """Highlight one waypoint of a drone route, or clear the highlight."""

        self._highlighted_waypoint = (
            (drone_id, index) if drone_id is not None and index is not None else None
        )
        self.update()

    @property
    def selected_object(self) -> str | None:
        return self._selected_id

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        if layer in self._layers:
            self._layers[layer] = visible
            self.update()

    def is_layer_visible(self, layer: str) -> bool:
        return self._layers.get(layer, False)

    def apply_view_preset(self, preset: str) -> None:
        if self._scene is None:
            return
        if preset == "top":
            self._camera.yaw_deg = 0.0
            self._camera.pitch_deg = 88.0
            self._fit_distance()
        elif preset == "side":
            self._camera.yaw_deg = 0.0
            self._camera.pitch_deg = 8.0
            self._fit_distance()
        elif preset == "iso":
            self._camera.yaw_deg = -35.0
            self._camera.pitch_deg = 45.0
            self._fit_distance()
        elif preset == "follow":
            marker = self._follow_marker()
            if marker is not None:
                self._camera.target_x = marker.x
                self._camera.target_y = marker.y
                self._camera.target_z = marker.z
                self._camera.yaw_deg = -35.0
                self._camera.pitch_deg = 35.0
                self._camera.distance = 320.0
        self.update()

    def update_live_positions(self, positions: Mapping[str, tuple[Point, float]]) -> None:
        """Move drone markers to live simulation positions."""

        if self._scene is None or self._replay_markers is not None:
            return
        if self._scene is None:
            return
        for marker in self._scene.markers:
            live = positions.get(marker.object_id)
            if live is not None:
                marker.x, marker.y, marker.z = live[0].x, live[0].y, live[1]
        self.update()

    def project(self, x: float, y: float, z: float) -> QPointF | None:
        """Project a world point to widget coordinates."""

        projected = self._projection().project(x, y, z)
        return QPointF(projected[0], projected[1]) if projected else None

    def pick_at(self, x: float, y: float) -> str | None:
        """Return the object id under widget coordinates, if any."""

        marker_hits = [hit for hit in self._hits if hit.kind == "marker"]
        if marker_hits:
            nearest = min(
                marker_hits,
                key=lambda hit: (hypot(hit.points[0][0] - x, hit.points[0][1] - y), hit.object_id),
            )
            if hypot(nearest.points[0][0] - x, nearest.points[0][1] - y) <= _PICK_RADIUS_MARKER:
                return nearest.object_id
        route_hits = [hit for hit in self._hits if hit.kind == "route"]
        if route_hits:
            best: tuple[float, str] | None = None
            for hit in route_hits:
                for index in range(len(hit.points) - 1):
                    distance = _point_segment_distance(
                        x, y, hit.points[index], hit.points[index + 1]
                    )
                    if distance <= _PICK_RADIUS_ROUTE and (
                        best is None or (distance, hit.object_id) < best
                    ):
                        best = (distance, hit.object_id)
            if best is not None:
                return best[1]
        for hit in self._hits:
            if hit.kind != "volume":
                continue
            polygon = QPolygonF([QPointF(px, py) for px, py in hit.points])
            if polygon.containsPoint(QPointF(x, y), Qt.FillRule.OddEvenFill):
                return hit.object_id
        return None

    # ------------------------------------------------------------- painting

    def paintEvent(self, event: QPaintEvent) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(BACKGROUND_COLOR))
        scene = self._scene
        self._hits = []
        if scene is None:
            painter.end()
            return
        self._current_painter = painter
        projection = self._projection()
        ground: list[tuple[float, int, Callable[[], None]]] = []
        air: list[tuple[float, int, Callable[[], None]]] = []
        if self._layers["terrain"]:
            self._queue_terrain(scene, projection, ground)
        if self._layers["coverage"] and scene.coverage is not None:
            self._queue_coverage(scene, projection, ground)
        if self._layers["routes"]:
            self._queue_routes(scene, projection, air)
        if self._layers["obstacles"]:
            self._queue_volumes(scene, projection, air, kind="obstacle")
        if self._layers["no_fly"]:
            self._queue_volumes(scene, projection, air, kind="no_fly")
        self._queue_markers(scene, projection, air)
        self._queue_waypoint_highlight(scene, projection, air)
        self._queue_wind(scene, projection, air)
        for queue in (ground, air):
            queue.sort(key=lambda item: (-item[0], item[1]))
            for _, _, draw in queue:
                draw()
        painter.end()
        self._current_painter = None

    def _queue_terrain(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        terrain = scene.terrain
        columns = len(terrain.xs)
        order = 0
        for row in range(len(terrain.ys) - 1):
            for column in range(columns - 1):
                corners = (
                    (terrain.xs[column], terrain.ys[row], terrain.corner_altitude(column, row)),
                    (
                        terrain.xs[column + 1],
                        terrain.ys[row],
                        terrain.corner_altitude(column + 1, row),
                    ),
                    (
                        terrain.xs[column + 1],
                        terrain.ys[row + 1],
                        terrain.corner_altitude(column + 1, row + 1),
                    ),
                    (
                        terrain.xs[column],
                        terrain.ys[row + 1],
                        terrain.corner_altitude(column, row + 1),
                    ),
                )
                projected = [projection.project(*corner) for corner in corners]
                if any(point is None for point in projected):
                    continue
                solid = [point for point in projected if point is not None]
                polygon = QPolygonF([QPointF(point[0], point[1]) for point in solid])
                depth = sum(point[2] for point in solid) / len(solid)
                color = QColor(terrain.cell_colors[row * (columns - 1) + column])
                shaded = _shaded_color(color, _surface_normal(corners))
                queue.append(
                    (depth, order, self._make_polygon_draw(polygon, shaded, shaded.darker(150)))
                )
                order += 1

    def _queue_coverage(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        coverage = scene.coverage
        if coverage is None:
            return
        half = coverage.cell_size / 2.0
        order = 0
        for color, cells in (
            (COVERAGE_COLOR_COVERED, coverage.covered),
            (COVERAGE_COLOR_UNCOVERED, coverage.uncovered),
        ):
            fill = QColor(color)
            fill.setAlpha(110)
            for x, y, z in cells:
                corners = (
                    (x - half, y - half, z),
                    (x + half, y - half, z),
                    (x + half, y + half, z),
                    (x - half, y + half, z),
                )
                projected = [projection.project(*corner) for corner in corners]
                if any(point is None for point in projected):
                    continue
                solid = [point for point in projected if point is not None]
                polygon = QPolygonF([QPointF(point[0], point[1]) for point in solid])
                depth = sum(point[2] for point in solid) / len(solid)
                queue.append(
                    (depth, 10000 + order, self._make_polygon_draw(polygon, fill, None))
                )
                order += 1

    def _queue_routes(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        for drone_index, route in enumerate(scene.routes):
            projected = [projection.project(*point) for point in route.points]
            if any(point is None for point in projected):
                continue
            solid = [point for point in projected if point is not None]
            if len(solid) < 2:
                continue
            screen = tuple((point[0], point[1]) for point in solid)
            self._hits.append(_HitTarget(route.object_id, "route", screen))
            selected = self._selected_id == route.object_id
            path = QPainterPath(QPointF(screen[0][0], screen[0][1]))
            for point in screen[1:]:
                path.lineTo(QPointF(point[0], point[1]))
            pen = QPen(QColor(route.color), SELECTED_PEN_WIDTH if selected else ROUTE_PEN_WIDTH)
            depth = sum(point[2] for point in solid) / len(solid)
            queue.append((depth, 20000 + drone_index, self._make_path_draw(path, pen)))
            if self._layers["risks"] and route.risk_colors:
                self._queue_risk_segments(route, solid, queue, drone_index)

    def _queue_risk_segments(
        self,
        route: Scene3DRoute,
        solid: list[tuple[float, float, float]],
        queue: list[tuple[float, int, Callable[[], None]]],
        drone_index: int,
    ) -> None:
        if len(solid) != len(route.points):
            return
        for segment_index, color in sorted(route.risk_colors.items()):
            if segment_index < 1 or segment_index >= len(solid):
                continue
            start = solid[segment_index - 1]
            end = solid[segment_index]
            path = QPainterPath(QPointF(start[0], start[1]))
            path.lineTo(QPointF(end[0], end[1]))
            pen = QPen(QColor(color), SELECTED_PEN_WIDTH + 1.5)
            depth = (start[2] + end[2]) / 2.0
            queue.append((depth, 21000 + drone_index, self._make_path_draw(path, pen)))

    def _queue_volumes(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
        *,
        kind: str,
    ) -> None:
        for volume_index, volume in enumerate(scene.volumes):
            if volume.kind != kind:
                continue
            footprint = volume.footprint
            bottom_points = [projection.project(x, y, volume.bottom) for x, y in footprint]
            top_points = [projection.project(x, y, volume.top) for x, y in footprint]
            if any(point is None for point in bottom_points) or any(
                point is None for point in top_points
            ):
                continue
            solid_top = [point for point in top_points if point is not None]
            fill_rgb = volume.fill_rgba[:3]
            alpha = volume.fill_rgba[3]
            edge = QColor(volume.edge_color)
            selected = self._selected_id == volume.object_id
            pen_width = 2.5 if selected else 1.2
            count = len(footprint)
            for index in range(count):
                next_index = (index + 1) % count
                world_side = (
                    (footprint[index][0], footprint[index][1], volume.bottom),
                    (footprint[next_index][0], footprint[next_index][1], volume.bottom),
                    (footprint[next_index][0], footprint[next_index][1], volume.top),
                    (footprint[index][0], footprint[index][1], volume.top),
                )
                projected_side = [projection.project(*corner) for corner in world_side]
                if any(point is None for point in projected_side):
                    continue
                solid_side = [point for point in projected_side if point is not None]
                polygon = QPolygonF([QPointF(point[0], point[1]) for point in solid_side])
                depth = sum(point[2] for point in solid_side) / len(solid_side)
                shaded = _shaded_color(QColor(*fill_rgb), _surface_normal(world_side))
                shaded.setAlpha(alpha)
                queue.append(
                    (
                        depth,
                        30000 + volume_index * 50 + index,
                        self._make_polygon_draw(polygon, shaded, edge, pen_width=pen_width),
                    )
                )
            top_polygon = QPolygonF([QPointF(point[0], point[1]) for point in solid_top])
            top_depth = sum(point[2] for point in solid_top) / len(solid_top)
            top_fill = QColor(*fill_rgb)
            top_fill.setAlpha(min(255, alpha + 30))
            self._hits.append(
                _HitTarget(
                    volume.object_id,
                    "volume",
                    tuple((point[0], point[1]) for point in solid_top),
                )
            )
            queue.append(
                (
                    top_depth,
                    31000 + volume_index * 50,
                    self._make_polygon_draw(
                        top_polygon, top_fill, edge, pen_width=pen_width
                    ),
                )
            )

    def _queue_markers(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        if self._replay_markers is not None:
            self._queue_replay_markers(scene, projection, queue)
            return
        for marker_index, marker in enumerate(scene.markers):
            projected = projection.project(marker.x, marker.y, marker.z)
            if projected is None:
                continue
            screen = QPointF(projected[0], projected[1])
            depth = projected[2]
            color = QColor(marker.color)
            selected = self._selected_id == marker.object_id
            self._hits.append(
                _HitTarget(marker.object_id, "marker", ((projected[0], projected[1]),))
            )

            def draw(
                painter: QPainter | None,
                screen: QPointF = screen,
                color: QColor = color,
                selected: bool = selected,
            ) -> None:
                if painter is None:
                    return
                radius = 9.0 if selected else 6.0
                pen = QPen(color)
                pen.setWidthF(2.5 if selected else 1.5)
                painter.setPen(pen)
                painter.setBrush(color)
                painter.drawEllipse(screen, radius, radius)
                if selected:
                    painter.setPen(QPen(QColor("#ffffff"), 1.2))
                    painter.setBrush(Qt.BrushStyle.NoBrush)
                    painter.drawEllipse(screen, radius + 4.0, radius + 4.0)

            queue.append((depth, 40000 + marker_index, self._bind_painter(draw)))
            if self._layers["labels"]:
                self._queue_label(
                    f"{marker.object_id} {marker.status}",
                    screen,
                    depth,
                    45000 + marker_index,
                    queue,
                )

    def _queue_replay_markers(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        colors = {marker.object_id: marker.color for marker in scene.markers}
        replay_markers = self._replay_markers or {}
        for index, drone_id in enumerate(sorted(replay_markers)):
            x, y, z, status = replay_markers[drone_id]
            projected = projection.project(x, y, z)
            if projected is None:
                continue
            screen = QPointF(projected[0], projected[1])
            color = QColor(colors.get(drone_id, "#55d6be"))
            self._hits.append(_HitTarget(drone_id, "marker", ((projected[0], projected[1]),)))

            def draw(
                painter: QPainter | None,
                screen: QPointF = screen,
                color: QColor = color,
                drone_id: str = drone_id,
            ) -> None:
                if painter is None:
                    return
                pen = QPen(color)
                pen.setWidthF(2.0)
                painter.setPen(pen)
                painter.setBrush(color)
                painter.drawEllipse(screen, 6.0, 6.0)
                painter.setPen(QColor("#f9ca5b"))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(screen, 10.0, 10.0)
                del drone_id

            queue.append((projected[2], 40000 + index, self._bind_painter(draw)))
            if self._layers["labels"]:
                self._queue_label(
                    f"{drone_id} {status} (replay)",
                    screen,
                    projected[2],
                    45000 + index,
                    queue,
                )

    def _queue_label(
        self,
        text: str,
        screen: QPointF,
        depth: float,
        order: int,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        def draw(
            painter: QPainter | None,
            screen: QPointF = screen,
            text: str = text,
        ) -> None:
            if painter is None:
                return
            metrics = painter.fontMetrics()
            width = metrics.horizontalAdvance(text) + 10
            height = metrics.height() + 4
            rect = QRectF(
                screen.x() - width / 2.0,
                screen.y() - 20.0 - height / 2.0,
                float(width),
                float(height),
            )
            painter.setPen(QPen(QColor("#2b3a50")))
            painter.setBrush(QColor(12, 19, 30, 205))
            painter.drawRoundedRect(rect, 4.0, 4.0)
            painter.setPen(QColor(LABEL_TEXT_COLOR))
            painter.drawText(rect, int(Qt.AlignmentFlag.AlignCenter), text)

        queue.append((depth, order, self._bind_painter(draw)))

    def _queue_waypoint_highlight(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        highlighted = self._highlighted_waypoint
        if highlighted is None:
            return
        drone_id, index = highlighted
        route = next((item for item in scene.routes if item.object_id == drone_id), None)
        if route is None or not 0 <= index < len(route.points):
            return
        point = route.points[index]
        projected = projection.project(*point)
        if projected is None:
            return
        screen = QPointF(projected[0], projected[1])
        depth = projected[2]

        def draw(painter: QPainter | None, screen: QPointF = screen) -> None:
            if painter is None:
                return
            pen = QPen(QColor(HIGHLIGHT_COLOR))
            pen.setWidthF(2.5)
            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(screen, 9.0, 9.0)

        queue.append((depth, 48000, self._bind_painter(draw)))

    def _queue_wind(
        self,
        scene: Scene3D,
        projection: _Projection,
        queue: list[tuple[float, int, Callable[[], None]]],
    ) -> None:
        wind = scene.wind
        if wind is None:
            return
        scale = 12.0
        start_projected = projection.project(wind.x, wind.y, wind.z)
        end_projected = projection.project(
            wind.x + wind.dx * scale, wind.y + wind.dy * scale, wind.z
        )
        if start_projected is None or end_projected is None:
            return
        start = QPointF(start_projected[0], start_projected[1])
        end_point = QPointF(end_projected[0], end_projected[1])
        path = QPainterPath(start)
        path.lineTo(end_point)
        pen = QPen(QColor(WIND_COLOR), 2.5)
        queue.append((start_projected[2], 50000, self._make_path_draw(path, pen)))
        arrow = QPainterPath(end_point)
        arrow.lineTo(QPointF(end_point.x() - 10.0, end_point.y() - 4.0))
        arrow.moveTo(end_point)
        arrow.lineTo(QPointF(end_point.x() - 10.0, end_point.y() + 4.0))
        queue.append((start_projected[2], 50001, self._make_path_draw(arrow, pen)))

    def _bind_painter(
        self,
        draw: Callable[[QPainter | None], None],
    ) -> Callable[[], None]:
        def call() -> None:
            draw(self._current_painter)

        return call

    def _make_polygon_draw(
        self,
        polygon: QPolygonF,
        fill: QColor,
        edge: QColor | None,
        pen_width: float = 1.0,
    ) -> Callable[[], None]:
        def draw(painter: QPainter | None) -> None:
            if painter is None:
                return
            painter.setPen(QPen(edge, pen_width) if edge else QPen(Qt.PenStyle.NoPen))
            painter.setBrush(fill)
            painter.drawPolygon(polygon)

        return self._bind_painter(draw)

    def _make_path_draw(self, path: QPainterPath, pen: QPen) -> Callable[[], None]:
        def draw(painter: QPainter | None) -> None:
            if painter is None:
                return
            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawPath(path)

        return self._bind_painter(draw)

    def _projection(self) -> _Projection:
        return _Projection(self._camera, self.width(), self.height())

    # --------------------------------------------------------------- camera

    def _fit_camera(self, scene: Scene3D) -> None:
        terrain = scene.terrain
        centre_column = (len(terrain.xs) - 1) // 2
        centre_row = (len(terrain.ys) - 1) // 2
        self._camera.target_x = scene.width / 2.0
        self._camera.target_y = scene.height / 2.0
        self._camera.target_z = max(20.0, terrain.corner_altitude(centre_column, centre_row))
        self._fit_distance()

    def _fit_distance(self) -> None:
        scene = self._scene
        span = hypot(scene.width, scene.height) if scene else 1200.0
        self._camera.distance = max(_MIN_DISTANCE, span * 0.85)

    def _follow_marker(self) -> Scene3DMarker | None:
        scene = self._scene
        if scene is None or not scene.markers:
            return None
        if self._selected_id is not None:
            for marker in scene.markers:
                if marker.object_id == self._selected_id:
                    return marker
        return scene.markers[0]

    # --------------------------------------------------------------- events

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._press_pos = event.position()
        self._press_button = event.button()
        self._dragged = False

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._scene is None or self._press_button is None:
            return
        dx = event.position().x() - self._press_pos.x()
        dy = event.position().y() - self._press_pos.y()
        if abs(dx) < 3.0 and abs(dy) < 3.0 and not self._dragged:
            return
        self._dragged = True
        if self._press_button == Qt.MouseButton.LeftButton:
            self._camera.yaw_deg -= dx * 0.35
            self._camera.pitch_deg = max(
                _MIN_PITCH_DEG,
                min(_MAX_PITCH_DEG, self._camera.pitch_deg + dy * 0.3),
            )
        else:
            self._pan_target(dx, dy)
        self._press_pos = event.position()
        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if (
            event.button() == Qt.MouseButton.LeftButton
            and not self._dragged
        ):
            object_id = self.pick_at(event.position().x(), event.position().y())
            if object_id is not None:
                self.object_selected.emit(object_id)
                self.set_selected_object(object_id)
        self._press_button = None
        self._dragged = False

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        if not delta:
            return
        factor = 1.15 ** (-delta / 120.0)
        span = hypot(self._scene.width, self._scene.height) * 2.0 if self._scene else 4000.0
        self._camera.distance = max(_MIN_DISTANCE, min(span, self._camera.distance * factor))
        self.update()

    def _pan_target(self, dx: float, dy: float) -> None:
        yaw = radians(self._camera.yaw_deg)
        cy, sy = cos(yaw), sin(yaw)
        scale = self._camera.distance / (
            _FOCAL_FACTOR * max(1.0, min(self.width(), self.height()))
        )
        self._camera.target_x += (-dx * cy - dy * sy) * scale
        self._camera.target_y += (-dx * sy + dy * cy) * scale
        if self._scene is not None:
            self._camera.target_x = max(
                -100.0, min(self._scene.width + 100.0, self._camera.target_x)
            )
            self._camera.target_y = max(
                -100.0, min(self._scene.height + 100.0, self._camera.target_y)
            )


def _surface_normal(
    corners: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = corners[0], corners[1], corners[2]
    ux, uy, uz = x2 - x1, y2 - y1, z2 - z1
    vx, vy, vz = x3 - x1, y3 - y1, z3 - z1
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-9:
        return (0.0, 0.0, 1.0)
    normal = (nx / length, ny / length, nz / length)
    return normal if normal[2] >= 0.0 else (-normal[0], -normal[1], -normal[2])


def _shaded_color(color: QColor, normal: tuple[float, float, float]) -> QColor:
    light = (-0.35, -0.45, 0.82)
    dot = normal[0] * light[0] + normal[1] * light[1] + normal[2] * light[2]
    factor = min(1.18, 0.72 + 0.42 * max(0.0, dot))
    shaded = QColor(
        min(255, round(color.red() * factor)),
        min(255, round(color.green() * factor)),
        min(255, round(color.blue() * factor)),
    )
    shaded.setAlpha(color.alpha())
    return shaded


def _point_segment_distance(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    sx, sy = start
    ex, ey = end
    vx, vy = ex - sx, ey - sy
    span = vx * vx + vy * vy
    if span < 1e-9:
        return hypot(x - sx, y - sy)
    t = max(0.0, min(1.0, ((x - sx) * vx + (y - sy) * vy) / span))
    return hypot(x - (sx + t * vx), y - (sy + t * vy))
