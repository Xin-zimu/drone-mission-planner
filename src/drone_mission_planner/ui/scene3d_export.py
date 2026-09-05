"""Deterministic 3D scene construction from mission and planning models.

The scene description is renderer-agnostic and imports no UI toolkit, so it
can be built and verified headlessly. :class:`ThreeDView` renders the
dataclasses directly and :meth:`Scene3D.to_json_dict` exports the same
content as stable JSON for debugging and future exporters.

Terrain and risk colors deliberately reuse the palette of the existing
2.5D map view so both projections read as one product.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, fields, is_dataclass
from math import cos, isfinite, pi, sin
from typing import Any

from drone_mission_planner.domain.enums import ObstacleShape
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import Drone, MapModel
from drone_mission_planner.domain.waypoint import Waypoint, waypoint_msl_altitude
from drone_mission_planner.planning.altitude_validator import (
    AltitudeRiskSeverity,
    validate_model_altitudes,
)

RISK_COLOR_CRITICAL = "#ef6a79"
RISK_COLOR_WARNING = "#f9ca5b"
COVERAGE_COLOR_COVERED = "#55d6be"
COVERAGE_COLOR_UNCOVERED = "#ef6a79"
OUTLINE_COLOR_SEARCH = "#55d6be"
WIND_COLOR = "#9fc2ff"

DRONE_COLORS: tuple[str, ...] = (
    "#55d6be",
    "#ffb54d",
    "#6aa9ff",
    "#dda8ff",
    "#8fd460",
    "#ff9aa6",
    "#7fd8e8",
    "#f2b8cd",
)

_CIRCLE_SIDES = 16
_MAX_MESH_SIZE = 65


@dataclass(slots=True)
class Scene3DTerrain:
    """Regular terrain mesh over the mission map, row-major from origin."""

    xs: list[float]
    ys: list[float]
    altitudes: list[float]
    cell_colors: list[str]

    def corner_altitude(self, column: int, row: int) -> float:
        return self.altitudes[row * len(self.xs) + column]


@dataclass(slots=True)
class Scene3DRoute:
    """Three-dimensional flight path of one drone."""

    object_id: str
    label: str
    color: str
    points: list[tuple[float, float, float]]
    risk_colors: dict[int, str] = field(default_factory=dict)


@dataclass(slots=True)
class Scene3DVolume:
    """Prism volume standing on the terrain, for obstacles and no-fly zones."""

    object_id: str
    label: str
    kind: str
    footprint: list[tuple[float, float]]
    bottom: float
    top: float
    fill_rgba: tuple[int, int, int, int]
    edge_color: str


@dataclass(slots=True)
class Scene3DMarker:
    """Drone position marker with its current status."""

    object_id: str
    label: str
    color: str
    x: float
    y: float
    z: float
    status: str


@dataclass(slots=True)
class Scene3DOutline:
    """Closed ground outline following the terrain, used for search areas."""

    object_id: str
    label: str
    color: str
    points: list[tuple[float, float, float]]


@dataclass(slots=True)
class Scene3DCoverage:
    """Coverage cells drawn on the terrain surface."""

    cell_size: float
    covered: list[tuple[float, float, float]]
    uncovered: list[tuple[float, float, float]]


@dataclass(slots=True)
class Scene3DWindArrow:
    """Global wind vector rendered as an arrow above the map centre."""

    x: float
    y: float
    z: float
    dx: float
    dy: float
    speed: float


@dataclass(slots=True)
class Scene3D:
    """Complete read-only 3D scene description of the current mission."""

    width: float
    height: float
    terrain: Scene3DTerrain
    routes: list[Scene3DRoute] = field(default_factory=list)
    volumes: list[Scene3DVolume] = field(default_factory=list)
    markers: list[Scene3DMarker] = field(default_factory=list)
    outlines: list[Scene3DOutline] = field(default_factory=list)
    coverage: Scene3DCoverage | None = None
    wind: Scene3DWindArrow | None = None

    def to_json_dict(self) -> dict[str, Any]:
        """Return the scene as plain JSON-compatible dictionaries."""

        def encode(value: Any) -> Any:
            if is_dataclass(value) and not isinstance(value, type):
                return {item.name: encode(getattr(value, item.name)) for item in fields(value)}
            if isinstance(value, (list, tuple)):
                return [encode(item) for item in value]
            if isinstance(value, dict):
                return {str(key): encode(item) for key, item in value.items()}
            return value

        encoded = encode(self)
        assert isinstance(encoded, dict)
        return encoded


def build_scene3d(
    map_model: MapModel,
    *,
    covered_cells: Mapping[str, Sequence[Point]] | None = None,
    uncovered_cells: Mapping[str, Sequence[Point]] | None = None,
    coverage_cell_size: float = 0.0,
    live_positions: Mapping[str, tuple[Point, float]] | None = None,
    include_altitude_risks: bool = True,
) -> Scene3D:
    """Build the 3D scene for ``map_model`` with deterministic ordering."""

    drones = sorted(map_model.drones, key=lambda drone: drone.id)
    risks = _risk_colors_by_drone(map_model) if include_altitude_risks else {}
    routes: list[Scene3DRoute] = []
    for index, drone in enumerate(drones):
        points = _drone_route_points(map_model, drone)
        if points:
            routes.append(_build_route(drone, points, risks.get(drone.id, {}), index))
    return Scene3D(
        width=float(map_model.width),
        height=float(map_model.height),
        terrain=_build_terrain(map_model),
        routes=routes,
        volumes=[*_obstacle_volumes(map_model), *_no_fly_volumes(map_model)],
        markers=[
            _build_marker(map_model, drone, live_positions, index)
            for index, drone in enumerate(drones)
        ],
        outlines=_search_outlines(map_model),
        coverage=_build_coverage(
            map_model,
            covered_cells or {},
            uncovered_cells or {},
            coverage_cell_size,
        ),
        wind=_build_wind(map_model),
    )


def terrain_color(altitude: float, min_altitude: float, max_altitude: float) -> str:
    """Map an altitude to the shared 2.5D terrain gradient."""

    span = max(1.0, max_altitude - min_altitude)
    ratio = max(0.0, min(1.0, (altitude - min_altitude) / span))
    stops = (
        (0x23, 0x4B, 0x3A),
        (0x6D, 0x8F, 0x54),
        (0x9A, 0x76, 0x53),
        (0xEE, 0xF2, 0xF5),
    )
    if ratio < 0.45:
        color = _mix(stops[0], stops[1], ratio / 0.45)
    elif ratio < 0.8:
        color = _mix(stops[1], stops[2], (ratio - 0.45) / 0.35)
    else:
        color = _mix(stops[2], stops[3], (ratio - 0.8) / 0.2)
    return f"#{color[0]:02x}{color[1]:02x}{color[2]:02x}"


def waypoint_render_altitude(waypoint: Waypoint, map_model: MapModel) -> float:
    """Deprecated alias for the domain MSL conversion."""

    return waypoint_msl_altitude(waypoint, map_model.terrain)


def _mix(
    first: tuple[int, int, int],
    second: tuple[int, int, int],
    ratio: float,
) -> tuple[int, int, int]:
    return (
        round(first[0] + (second[0] - first[0]) * ratio),
        round(first[1] + (second[1] - first[1]) * ratio),
        round(first[2] + (second[2] - first[2]) * ratio),
    )


def _build_terrain(map_model: MapModel) -> Scene3DTerrain:
    terrain = map_model.terrain
    step = max(25.0, terrain.resolution, map_model.grid_size * 2.0)
    columns = max(2, min(_MAX_MESH_SIZE, int(map_model.width / step) + 1))
    rows = max(2, min(_MAX_MESH_SIZE, int(map_model.height / step) + 1))
    xs = [map_model.width * column / (columns - 1) for column in range(columns)]
    ys = [map_model.height * row / (rows - 1) for row in range(rows)]
    altitudes = [terrain.altitude_at(x, y) for y in ys for x in xs]
    cell_colors: list[str] = []
    for row in range(rows - 1):
        for column in range(columns - 1):
            corners = (
                altitudes[row * columns + column],
                altitudes[row * columns + column + 1],
                altitudes[(row + 1) * columns + column],
                altitudes[(row + 1) * columns + column + 1],
            )
            mean = sum(corners) / 4.0
            cell_colors.append(terrain_color(mean, terrain.min_altitude, terrain.max_altitude))
    return Scene3DTerrain(xs=xs, ys=ys, altitudes=altitudes, cell_colors=cell_colors)


def _drone_route_points(
    map_model: MapModel,
    drone: Drone,
) -> list[tuple[float, float, float]]:
    if drone.waypoints:
        return [
            (waypoint.x, waypoint.y, waypoint_msl_altitude(waypoint, map_model.terrain))
            for waypoint in drone.waypoints
        ]
    points: list[tuple[float, float, float]] = []
    for point in drone.planned_path:
        altitude = max(
            drone.cruise_altitude,
            map_model.terrain.altitude_at(point.x, point.y) + drone.min_clearance,
        )
        points.append((point.x, point.y, altitude))
    return points


def _build_route(
    drone: Drone,
    points: list[tuple[float, float, float]],
    risk_colors: dict[int, str],
    color_index: int,
) -> Scene3DRoute:
    return Scene3DRoute(
        object_id=drone.id,
        label=drone.name or drone.id,
        color=DRONE_COLORS[color_index % len(DRONE_COLORS)],
        points=points,
        risk_colors=dict(sorted(risk_colors.items())),
    )


def _risk_colors_by_drone(map_model: MapModel) -> dict[str, dict[int, str]]:
    by_drone: dict[str, dict[int, str]] = {}
    for risk in validate_model_altitudes(map_model):
        color = (
            RISK_COLOR_CRITICAL
            if risk.severity == AltitudeRiskSeverity.CRITICAL
            else RISK_COLOR_WARNING
        )
        segment_colors = by_drone.setdefault(risk.drone_id, {})
        if segment_colors.get(risk.segment_index) != RISK_COLOR_CRITICAL:
            segment_colors[risk.segment_index] = color
    return by_drone


def _footprint_points(
    bounds: Rect,
    shape: ObstacleShape,
    points: Sequence[Point],
    radius: float,
) -> list[tuple[float, float]]:
    if shape == ObstacleShape.CIRCLE:
        centre_x = bounds.x + bounds.width / 2.0
        centre_y = bounds.y + bounds.height / 2.0
        circle_radius = radius if radius > 0.0 else bounds.width / 2.0
        return [
            (
                centre_x + circle_radius * cos(2.0 * pi * index / _CIRCLE_SIDES),
                centre_y + circle_radius * sin(2.0 * pi * index / _CIRCLE_SIDES),
            )
            for index in range(_CIRCLE_SIDES)
        ]
    if shape == ObstacleShape.POLYGON and points:
        return [(point.x, point.y) for point in points]
    rect = bounds.normalized
    return [
        (rect.x, rect.y),
        (rect.x + rect.width, rect.y),
        (rect.x + rect.width, rect.y + rect.height),
        (rect.x, rect.y + rect.height),
    ]


def _obstacle_volumes(map_model: MapModel) -> list[Scene3DVolume]:
    terrain = map_model.terrain
    volumes: list[Scene3DVolume] = []
    for obstacle in sorted(map_model.obstacles, key=lambda item: item.id):
        footprint = _footprint_points(
            obstacle.bounds,
            obstacle.shape,
            obstacle.points,
            obstacle.radius,
        )
        if not footprint:
            continue
        bottom = min(terrain.altitude_at(x, y) for x, y in footprint)
        top = bottom + max(0.0, obstacle.height)
        volumes.append(
            Scene3DVolume(
                object_id=obstacle.id,
                label=obstacle.name or obstacle.id,
                kind="obstacle",
                footprint=footprint,
                bottom=bottom,
                top=top,
                fill_rgba=(0xB6, 0x46, 0x52, 190),
                edge_color="#ff9aa6",
            )
        )
    return volumes


def _no_fly_volumes(map_model: MapModel) -> list[Scene3DVolume]:
    terrain = map_model.terrain
    volumes: list[Scene3DVolume] = []
    for zone in sorted(map_model.no_fly_zones, key=lambda item: item.id):
        footprint = _footprint_points(zone.bounds, zone.shape, zone.points, 0.0)
        if not footprint:
            continue
        bottom = min(terrain.altitude_at(x, y) for x, y in footprint)
        top = max(terrain.altitude_at(x, y) for x, y in footprint) + max(
            0.0, zone.ceiling_altitude
        )
        volumes.append(
            Scene3DVolume(
                object_id=zone.id,
                label=zone.name or zone.id,
                kind="no_fly",
                footprint=footprint,
                bottom=bottom,
                top=max(top, bottom + 1.0),
                fill_rgba=(141, 73, 190, 70),
                edge_color="#dda8ff",
            )
        )
    return volumes


def _search_outlines(map_model: MapModel) -> list[Scene3DOutline]:
    terrain = map_model.terrain
    outlines: list[Scene3DOutline] = []
    for area in sorted(map_model.search_areas, key=lambda item: item.id):
        corners = _footprint_points(area.bounds, ObstacleShape.POLYGON, area.points, 0.0)
        if not corners:
            continue
        outlines.append(
            Scene3DOutline(
                object_id=area.id,
                label=area.name or area.id,
                color=OUTLINE_COLOR_SEARCH,
                points=[(x, y, terrain.altitude_at(x, y) + 0.5) for x, y in corners],
            )
        )
    return outlines


def _build_marker(
    map_model: MapModel,
    drone: Drone,
    live_positions: Mapping[str, tuple[Point, float]] | None,
    color_index: int,
) -> Scene3DMarker:
    live = live_positions.get(drone.id) if live_positions else None
    if live is not None:
        position, z = live
    else:
        position = drone.position
        z = max(
            drone.cruise_altitude,
            map_model.terrain.altitude_at(position.x, position.y) + drone.min_clearance,
        )
    return Scene3DMarker(
        object_id=drone.id,
        label=drone.name or drone.id,
        color=DRONE_COLORS[color_index % len(DRONE_COLORS)],
        x=position.x,
        y=position.y,
        z=z,
        status=drone.status.value,
    )


def _build_coverage(
    map_model: MapModel,
    covered_cells: Mapping[str, Sequence[Point]],
    uncovered_cells: Mapping[str, Sequence[Point]],
    cell_size: float,
) -> Scene3DCoverage | None:
    terrain = map_model.terrain
    covered = [
        (point.x, point.y, terrain.altitude_at(point.x, point.y) + 0.4)
        for area_id in sorted(covered_cells)
        for point in covered_cells[area_id]
    ]
    uncovered = [
        (point.x, point.y, terrain.altitude_at(point.x, point.y) + 0.5)
        for area_id in sorted(uncovered_cells)
        for point in uncovered_cells[area_id]
    ]
    if not covered and not uncovered:
        return None
    size = cell_size if cell_size > 0.0 else map_model.grid_size
    return Scene3DCoverage(cell_size=size, covered=covered, uncovered=uncovered)


def _build_wind(map_model: MapModel) -> Scene3DWindArrow | None:
    wind = map_model.wind
    if not wind.enabled or not isfinite(wind.speed) or wind.speed <= 0.0:
        return None
    dx, dy = wind.wind_vector()
    if dx == 0.0 and dy == 0.0:
        return None
    centre_x = map_model.width / 2.0
    centre_y = map_model.height / 2.0
    return Scene3DWindArrow(
        x=centre_x,
        y=centre_y,
        z=map_model.terrain.altitude_at(centre_x, centre_y) + 80.0,
        dx=dx,
        dy=dy,
        speed=wind.speed,
    )
