from __future__ import annotations

from dataclasses import dataclass, field
from math import floor

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, SearchArea

from .grid import GridMap
from .route_planner import RoutePlanner


@dataclass(frozen=True, slots=True)
class CoveragePass:
    start: Point
    end: Point


@dataclass(frozen=True, slots=True)
class CoverageStrip:
    index: int
    drone_id: str
    min_x: float
    max_x: float
    passes: tuple[CoveragePass, ...]


@dataclass(slots=True)
class CoveragePlanResult:
    area_id: str
    strips: tuple[CoverageStrip, ...]
    drone_paths: dict[str, list[Point]] = field(default_factory=dict)
    drone_distances: dict[str, float] = field(default_factory=dict)
    failures: dict[str, str] = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return bool(self.drone_paths) and not self.failures

    @property
    def total_distance(self) -> float:
        return sum(self.drone_distances.values())


def polygon_area(points: list[Point]) -> float:
    if len(points) < 3:
        return 0.0
    twice_area = sum(
        point.x * points[(index + 1) % len(points)].y
        - points[(index + 1) % len(points)].x * point.y
        for index, point in enumerate(points)
    )
    return abs(twice_area) / 2.0


def point_in_polygon(point: Point, polygon: list[Point]) -> bool:
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


def scanline_intervals(polygon: list[Point], y: float) -> list[tuple[float, float]]:
    intersections: list[float] = []
    previous = polygon[-1]
    for current in polygon:
        if (previous.y <= y < current.y) or (current.y <= y < previous.y):
            ratio = (y - previous.y) / (current.y - previous.y)
            intersections.append(previous.x + ratio * (current.x - previous.x))
        previous = current
    intersections.sort()
    return [
        (intersections[index], intersections[index + 1])
        for index in range(0, len(intersections) - 1, 2)
    ]


class CoveragePlanner:
    """Partition a polygon into vertical strips and build obstacle-safe lawnmower routes."""

    def __init__(self, route_planner: RoutePlanner | None = None) -> None:
        self.route_planner = route_planner or RoutePlanner()

    def plan(
        self, map_model: MapModel, area: SearchArea, drones: list[Drone] | None = None
    ) -> CoveragePlanResult:
        selected = sorted(drones or map_model.drones, key=lambda item: item.id)
        if not selected:
            return CoveragePlanResult(area.id, (), failures={"area": "No drones available"})
        polygon = area.polygon()
        if polygon_area(polygon) <= 1e-6:
            return CoveragePlanResult(area.id, (), failures={"area": "Search area has no area"})

        strips = self.build_strips(map_model, area, selected)
        result = CoveragePlanResult(area.id, strips)
        for drone, strip in zip(selected, strips, strict=True):
            if not strip.passes:
                result.failures[drone.id] = "Assigned strip contains no reachable scan passes"
                continue
            targets = [
                point
                for coverage_pass in strip.passes
                for point in (coverage_pass.start, coverage_pass.end)
            ]
            home = next(
                (base.position for base in map_model.bases if base.id == drone.home_base_id),
                drone.position,
            )
            targets.append(home)
            path = [drone.position]
            distance = 0.0
            cursor = drone.position
            for target in targets:
                leg = self.route_planner.plan_between(map_model, drone, cursor, target)
                if not leg.success:
                    result.failures[drone.id] = leg.failure_reason or "No safe connecting route"
                    break
                distance += leg.total_distance
                path.extend(leg.waypoints[1:])
                cursor = target
            else:
                result.drone_paths[drone.id] = _deduplicate(path)
                result.drone_distances[drone.id] = distance
        return result

    def build_strips(
        self, map_model: MapModel, area: SearchArea, drones: list[Drone]
    ) -> tuple[CoverageStrip, ...]:
        polygon = area.polygon()
        min_x = min(point.x for point in polygon)
        max_x = max(point.x for point in polygon)
        min_y = min(point.y for point in polygon)
        max_y = max(point.y for point in polygon)
        width = max_x - min_x
        spacing = max(1.0, area.scan_spacing)
        margin = max(0.0, area.boundary_margin)
        y_start = min_y + margin
        y_end = max_y - margin
        if y_start > y_end:
            y_start = y_end = (min_y + max_y) / 2.0
        row_count = max(1, floor((y_end - y_start) / spacing) + 1)
        rows = [y_start + index * spacing for index in range(row_count)]
        if rows[-1] < y_end - spacing * 0.35:
            rows.append(y_end)

        strips: list[CoverageStrip] = []
        for index, drone in enumerate(drones):
            strip_min = min_x + width * index / len(drones)
            strip_max = min_x + width * (index + 1) / len(drones)
            grid = GridMap.from_map(map_model, safety_radius=drone.safety_radius)
            passes: list[CoveragePass] = []
            reverse = bool(index % 2)
            for y in rows:
                row_segments: list[tuple[Point, Point]] = []
                for interval_min, interval_max in scanline_intervals(polygon, y):
                    left = max(strip_min, interval_min + margin)
                    right = min(strip_max, interval_max - margin)
                    endpoints = _free_endpoints(grid, left, right, y)
                    if endpoints is not None:
                        row_segments.append(endpoints)
                if not row_segments:
                    continue
                ordered = list(reversed(row_segments)) if reverse else row_segments
                for left_point, right_point in ordered:
                    start, end = (right_point, left_point) if reverse else (left_point, right_point)
                    passes.append(CoveragePass(start, end))
                reverse = not reverse
            strips.append(CoverageStrip(index, drone.id, strip_min, strip_max, tuple(passes)))
        return tuple(strips)


def _free_endpoints(
    grid: GridMap, left: float, right: float, y: float
) -> tuple[Point, Point] | None:
    if right - left < grid.resolution * 0.35:
        return None
    step = max(1.0, grid.resolution * 0.25)
    left_point: Point | None = None
    right_point: Point | None = None
    candidate = left
    while candidate <= right:
        point = Point(candidate, y)
        if not grid.is_blocked(grid.world_to_cell(point)):
            left_point = point
            break
        candidate += step
    candidate = right
    while candidate >= left:
        point = Point(candidate, y)
        if not grid.is_blocked(grid.world_to_cell(point)):
            right_point = point
            break
        candidate -= step
    if left_point is None or right_point is None or left_point.x >= right_point.x:
        return None
    return left_point, right_point


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    cross = (point.y - start.y) * (end.x - start.x) - (point.x - start.x) * (end.y - start.y)
    if abs(cross) > 1e-8:
        return False
    return (
        min(start.x, end.x) - 1e-8 <= point.x <= max(start.x, end.x) + 1e-8
        and min(start.y, end.y) - 1e-8 <= point.y <= max(start.y, end.y) + 1e-8
    )


def _deduplicate(points: list[Point]) -> list[Point]:
    result: list[Point] = []
    for point in points:
        if not result or point.distance_to(result[-1]) > 1e-6:
            result.append(point)
    return result
