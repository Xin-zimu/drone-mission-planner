from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
from math import ceil

from drone_mission_planner.domain.enums import ObstacleShape
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
)
from drone_mission_planner.domain.waypoint import path_from_waypoints


class AltitudeRiskKind(StrEnum):
    TERRAIN_CLEARANCE = "terrain_clearance"
    OBSTACLE_HEIGHT = "obstacle_height"
    NO_FLY_CEILING = "no_fly_ceiling"
    NO_FLY_POLICY = "no_fly_policy"
    TASK_ALTITUDE = "task_altitude"


class AltitudeRiskSeverity(StrEnum):
    WARNING = "warning"
    CRITICAL = "critical"


@dataclass(frozen=True, slots=True)
class AltitudeRisk:
    drone_id: str
    segment_index: int
    kind: AltitudeRiskKind
    severity: AltitudeRiskSeverity
    position: Point
    required_altitude: float
    flight_altitude: float
    object_id: str | None = None
    message: str = ""

    def summary(self) -> str:
        subject = f" {self.object_id}" if self.object_id else ""
        detail = self.message or self.kind.value.replace("_", " ")
        return (
            f"leg {self.segment_index}{subject}: {detail} "
            f"({self.flight_altitude:.1f}/{self.required_altitude:.1f} m)"
        )


def validate_altitude_path(
    map_model: MapModel,
    drone: Drone,
    path: list[Point] | tuple[Point, ...],
    *,
    sample_spacing: float | None = None,
    allow_no_fly_overflight: bool = True,
) -> tuple[AltitudeRisk, ...]:
    """Sample a planned 2D path and report altitude-related safety risks.

    Heights are checked against the commanded cruise/target altitude. The existing
    2D A* geometry remains authoritative for horizontal obstacle avoidance.
    """

    if len(path) < 2:
        return ()
    spacing = sample_spacing or max(5.0, min(map_model.grid_size, 25.0))
    tasks = _tasks_by_position(map_model, drone)
    risks: list[AltitudeRisk] = []
    seen: set[tuple[int, AltitudeRiskKind, str | None]] = set()
    for segment_index, (start, end) in enumerate(pairwise(path), start=1):
        end_task = _matching_task(end, tasks)
        for sample, ratio in _sample_segment(start, end, spacing):
            flight_altitude = _commanded_altitude(drone, ratio, end_task)
            _add_terrain_risk(
                risks,
                seen,
                map_model,
                drone,
                segment_index,
                sample,
                flight_altitude,
            )
            _add_obstacle_risks(
                risks,
                seen,
                map_model,
                drone,
                segment_index,
                sample,
                flight_altitude,
            )
            _add_no_fly_risks(
                risks,
                seen,
                map_model,
                drone,
                segment_index,
                sample,
                flight_altitude,
                allow_no_fly_overflight=allow_no_fly_overflight,
            )
        if end_task is not None:
            _add_task_altitude_risk(
                risks,
                seen,
                map_model,
                drone,
                segment_index,
                end,
                end_task,
            )
    return tuple(risks)


def validate_model_altitudes(
    map_model: MapModel,
    *,
    allow_no_fly_overflight: bool = True,
) -> tuple[AltitudeRisk, ...]:
    risks: list[AltitudeRisk] = []
    for drone in sorted(map_model.drones, key=lambda item: item.id):
        risks.extend(
            validate_altitude_path(
                map_model,
                drone,
                drone.planned_path or path_from_waypoints(drone.waypoints),
                allow_no_fly_overflight=allow_no_fly_overflight,
            )
        )
    return tuple(risks)


def _sample_segment(start: Point, end: Point, spacing: float) -> tuple[tuple[Point, float], ...]:
    distance = start.distance_to(end)
    if distance <= 1e-9:
        return ((start, 1.0),)
    steps = max(1, ceil(distance / max(spacing, 1e-9)))
    return tuple((start.lerp(end, index / steps), index / steps) for index in range(steps + 1))


def _tasks_by_position(map_model: MapModel, drone: Drone) -> tuple[MissionTask, ...]:
    return tuple(
        task
        for task in map_model.tasks
        if task.assigned_drone_id in {None, drone.id} or task.id in drone.assigned_tasks
    )


def _matching_task(point: Point, tasks: tuple[MissionTask, ...]) -> MissionTask | None:
    return next((task for task in tasks if task.position.distance_to(point) <= 1e-6), None)


def _commanded_altitude(drone: Drone, ratio: float, end_task: MissionTask | None) -> float:
    if end_task is None:
        return drone.cruise_altitude
    clamped = max(0.0, min(1.0, ratio))
    return drone.cruise_altitude + (end_task.target_altitude - drone.cruise_altitude) * clamped


def _add_terrain_risk(
    risks: list[AltitudeRisk],
    seen: set[tuple[int, AltitudeRiskKind, str | None]],
    map_model: MapModel,
    drone: Drone,
    segment_index: int,
    point: Point,
    flight_altitude: float,
) -> None:
    terrain_altitude = map_model.terrain.altitude_at(point.x, point.y)
    required = terrain_altitude + drone.min_clearance
    if flight_altitude + 1e-9 >= required:
        return
    _append_once(
        risks,
        seen,
        AltitudeRisk(
            drone.id,
            segment_index,
            AltitudeRiskKind.TERRAIN_CLEARANCE,
            AltitudeRiskSeverity.WARNING,
            point,
            required,
            flight_altitude,
            message="terrain clearance below minimum",
        ),
    )


def _add_obstacle_risks(
    risks: list[AltitudeRisk],
    seen: set[tuple[int, AltitudeRiskKind, str | None]],
    map_model: MapModel,
    drone: Drone,
    segment_index: int,
    point: Point,
    flight_altitude: float,
) -> None:
    terrain_altitude = map_model.terrain.altitude_at(point.x, point.y)
    for obstacle in map_model.obstacles:
        if not _contains_obstacle(obstacle, point):
            continue
        required = terrain_altitude + obstacle.height + drone.min_clearance
        if flight_altitude + 1e-9 >= required:
            continue
        _append_once(
            risks,
            seen,
            AltitudeRisk(
                drone.id,
                segment_index,
                AltitudeRiskKind.OBSTACLE_HEIGHT,
                AltitudeRiskSeverity.CRITICAL,
                point,
                required,
                flight_altitude,
                obstacle.id,
                "obstacle top plus clearance is above commanded altitude",
            ),
        )


def _add_no_fly_risks(
    risks: list[AltitudeRisk],
    seen: set[tuple[int, AltitudeRiskKind, str | None]],
    map_model: MapModel,
    drone: Drone,
    segment_index: int,
    point: Point,
    flight_altitude: float,
    *,
    allow_no_fly_overflight: bool,
) -> None:
    terrain_altitude = map_model.terrain.altitude_at(point.x, point.y)
    for zone in map_model.no_fly_zones:
        if not _contains_zone(zone, point):
            continue
        required = terrain_altitude + zone.ceiling_altitude
        if not allow_no_fly_overflight:
            _append_once(
                risks,
                seen,
                AltitudeRisk(
                    drone.id,
                    segment_index,
                    AltitudeRiskKind.NO_FLY_POLICY,
                    AltitudeRiskSeverity.CRITICAL,
                    point,
                    required,
                    flight_altitude,
                    zone.id,
                    "no-fly policy forbids overflight",
                ),
            )
            continue
        if flight_altitude + 1e-9 >= required:
            continue
        _append_once(
            risks,
            seen,
            AltitudeRisk(
                drone.id,
                segment_index,
                AltitudeRiskKind.NO_FLY_CEILING,
                AltitudeRiskSeverity.CRITICAL,
                point,
                required,
                flight_altitude,
                zone.id,
                "below no-fly ceiling",
            ),
        )


def _add_task_altitude_risk(
    risks: list[AltitudeRisk],
    seen: set[tuple[int, AltitudeRiskKind, str | None]],
    map_model: MapModel,
    drone: Drone,
    segment_index: int,
    point: Point,
    task: MissionTask,
) -> None:
    required = map_model.terrain.altitude_at(point.x, point.y) + drone.min_clearance
    if task.target_altitude + 1e-9 >= required:
        return
    _append_once(
        risks,
        seen,
        AltitudeRisk(
            drone.id,
            segment_index,
            AltitudeRiskKind.TASK_ALTITUDE,
            AltitudeRiskSeverity.WARNING,
            point,
            required,
            task.target_altitude,
            task.id,
            "task target altitude is below minimum clearance",
        ),
    )


def _append_once(
    risks: list[AltitudeRisk],
    seen: set[tuple[int, AltitudeRiskKind, str | None]],
    risk: AltitudeRisk,
) -> None:
    key = (risk.segment_index, risk.kind, risk.object_id)
    if key in seen:
        return
    seen.add(key)
    risks.append(risk)


def _contains_obstacle(obstacle: Obstacle, point: Point) -> bool:
    if obstacle.shape == ObstacleShape.CIRCLE:
        center = _shape_center(obstacle.bounds)
        radius = (
            obstacle.radius
            or min(obstacle.bounds.normalized.width, obstacle.bounds.normalized.height) / 2.0
        )
        return center.distance_to(point) <= radius
    if obstacle.shape == ObstacleShape.POLYGON and obstacle.points:
        return _point_in_polygon(point, obstacle.points)
    return obstacle.bounds.contains(point)


def _contains_zone(zone: NoFlyZone, point: Point) -> bool:
    if zone.shape == ObstacleShape.CIRCLE:
        center = _shape_center(zone.bounds)
        radius = min(zone.bounds.normalized.width, zone.bounds.normalized.height) / 2.0
        return center.distance_to(point) <= radius
    if zone.shape == ObstacleShape.POLYGON and zone.points:
        return _point_in_polygon(point, zone.points)
    return zone.bounds.contains(point)


def _shape_center(rect: Rect) -> Point:
    bounds = rect.normalized
    return Point(bounds.x + bounds.width / 2.0, bounds.y + bounds.height / 2.0)


def _point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(point, previous, current):
            return True
        crosses = (current.y > point.y) != (previous.y > point.y)
        if crosses:
            cross_x = (previous.x - current.x) * (point.y - current.y) / (
                previous.y - current.y
            ) + current.x
            if point.x < cross_x:
                inside = not inside
        previous = current
    return inside


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    cross = (point.y - start.y) * (end.x - start.x) - (point.x - start.x) * (end.y - start.y)
    if abs(cross) > 1e-8:
        return False
    return (
        min(start.x, end.x) - 1e-8 <= point.x <= max(start.x, end.x) + 1e-8
        and min(start.y, end.y) - 1e-8 <= point.y <= max(start.y, end.y) + 1e-8
    )
