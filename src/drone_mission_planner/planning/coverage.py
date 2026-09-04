from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field
from math import ceil, floor

from drone_mission_planner.domain.enums import DroneStatus, ObstacleShape
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import Drone, MapModel, SearchArea

from .energy import estimate_path_energy
from .grid import GridMap
from .route_planner import RoutePlanner

type CoverageCell = tuple[int, int]


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
    drone_energies: dict[str, float] = field(default_factory=dict)
    failures: dict[str, str] = field(default_factory=dict)
    incremental: bool = False
    target_cells: int = 0
    remaining_cells: int = 0
    covered_input_cells: int = 0

    @property
    def success(self) -> bool:
        return (bool(self.drone_paths) or self.remaining_cells == 0) and not self.failures

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


def coverage_resolution_for(map_model: MapModel, area: SearchArea) -> float:
    return max(5.0, min(map_model.grid_size, area.scan_spacing / 2.0))


def world_to_coverage_cell(point: Point, resolution: float) -> CoverageCell:
    return floor(point.x / resolution), floor(point.y / resolution)


def coverage_cell_center(cell: CoverageCell, resolution: float) -> Point:
    return Point((cell[0] + 0.5) * resolution, (cell[1] + 0.5) * resolution)


def target_cells_for_area(
    map_model: MapModel,
    area: SearchArea,
    resolution: float | None = None,
) -> frozenset[CoverageCell]:
    cell_resolution = resolution or coverage_resolution_for(map_model, area)
    polygon = area.polygon()
    min_x = min(point.x for point in polygon)
    max_x = max(point.x for point in polygon)
    min_y = min(point.y for point in polygon)
    max_y = max(point.y for point in polygon)
    targets: set[CoverageCell] = set()
    for x in range(floor(min_x / cell_resolution), ceil(max_x / cell_resolution)):
        for y in range(floor(min_y / cell_resolution), ceil(max_y / cell_resolution)):
            cell = (x, y)
            center = coverage_cell_center(cell, cell_resolution)
            if point_in_polygon(center, polygon) and not _blocked_for_coverage(map_model, center):
                targets.add(cell)
    return frozenset(targets)


class CoveragePlanner:
    """Partition a polygon into scan passes and build obstacle-safe lawnmower routes."""

    def __init__(self, route_planner: RoutePlanner | None = None) -> None:
        self.route_planner = route_planner or RoutePlanner()

    def plan(
        self,
        map_model: MapModel,
        area: SearchArea,
        drones: list[Drone] | None = None,
        *,
        covered_cells: Iterable[CoverageCell] | None = None,
        coverage_resolution: float | None = None,
    ) -> CoveragePlanResult:
        selected = sorted(
            (
                drone
                for drone in (drones or map_model.drones)
                if drone.status not in {DroneStatus.FAILED, DroneStatus.EMERGENCY}
            ),
            key=lambda item: item.id,
        )
        if not selected:
            return CoveragePlanResult(area.id, (), failures={"area": "No drones available"})
        polygon = area.polygon()
        if polygon_area(polygon) <= 1e-6:
            return CoveragePlanResult(area.id, (), failures={"area": "Search area has no area"})

        resolution = coverage_resolution or coverage_resolution_for(map_model, area)
        targets = target_cells_for_area(map_model, area, resolution)
        covered = set(covered_cells or ())
        incremental = covered_cells is not None
        remaining = targets - covered if incremental else targets
        if incremental and not remaining:
            return CoveragePlanResult(
                area.id,
                (),
                incremental=True,
                target_cells=len(targets),
                remaining_cells=0,
                covered_input_cells=len(covered & targets),
            )

        if incremental:
            safety_radius = max(drone.safety_radius for drone in selected)
            grid = GridMap.from_map(map_model, safety_radius=safety_radius)
            strips = self._build_incremental_strips(remaining, resolution, selected, grid)
        else:
            strips = self.build_strips(map_model, area, selected)
        result = CoveragePlanResult(
            area.id,
            strips,
            incremental=incremental,
            target_cells=len(targets),
            remaining_cells=len(remaining),
            covered_input_cells=len(covered & targets),
        )
        self._route_strips(
            map_model,
            selected,
            result,
            require_every_drone=not incremental,
        )
        return result

    def coverage_resolution_for(self, map_model: MapModel, area: SearchArea) -> float:
        return coverage_resolution_for(map_model, area)

    def target_cells_for_area(
        self,
        map_model: MapModel,
        area: SearchArea,
        resolution: float | None = None,
    ) -> frozenset[CoverageCell]:
        return target_cells_for_area(map_model, area, resolution)

    def coverage_cell_center(self, cell: CoverageCell, resolution: float) -> Point:
        return coverage_cell_center(cell, resolution)

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
                    row_segments.extend(_free_segments(grid, left, right, y))
                if not row_segments:
                    continue
                ordered = list(reversed(row_segments)) if reverse else row_segments
                for left_point, right_point in ordered:
                    start, end = (right_point, left_point) if reverse else (left_point, right_point)
                    passes.append(CoveragePass(start, end))
                    reverse = not reverse
            strips.append(CoverageStrip(index, drone.id, strip_min, strip_max, tuple(passes)))
        return tuple(strips)

    def _build_incremental_strips(
        self,
        remaining_cells: frozenset[CoverageCell],
        resolution: float,
        drones: list[Drone],
        grid: GridMap,
    ) -> tuple[CoverageStrip, ...]:
        loads = {drone.id: 0.0 for drone in drones}
        strips: list[CoverageStrip] = []
        clusters = sorted(_cluster_cells(remaining_cells), key=_cluster_sort_key)
        for index, cluster in enumerate(clusters):
            passes = _passes_for_cluster(cluster, resolution, grid)
            if not passes:
                continue
            center = _cluster_center(cluster, resolution)
            drone = min(
                drones,
                key=lambda item: (
                    item.position.distance_to(center) + loads[item.id] * 0.4,
                    item.id,
                ),
            )
            loads[drone.id] += _estimate_pass_load(drone.position, passes)
            min_x, max_x = _cluster_x_bounds(cluster, resolution)
            strips.append(CoverageStrip(index, drone.id, min_x, max_x, tuple(passes)))
        return tuple(strips)

    def _route_strips(
        self,
        map_model: MapModel,
        drones: list[Drone],
        result: CoveragePlanResult,
        *,
        require_every_drone: bool,
    ) -> None:
        strips_by_drone: dict[str, list[CoverageStrip]] = defaultdict(list)
        for strip in result.strips:
            strips_by_drone[strip.drone_id].append(strip)

        for drone in drones:
            passes = [
                coverage_pass
                for strip in strips_by_drone.get(drone.id, [])
                for coverage_pass in strip.passes
            ]
            if not passes:
                if require_every_drone:
                    result.failures[drone.id] = "Assigned strip contains no reachable scan passes"
                continue
            targets = [
                point
                for coverage_pass in passes
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
                route = _deduplicate(path)
                result.drone_paths[drone.id] = route
                result.drone_distances[drone.id] = distance
                result.drone_energies[drone.id] = estimate_path_energy(
                    drone,
                    route,
                    terrain=map_model.terrain,
                    wind=map_model.wind,
                ).energy


def _free_segments(grid: GridMap, left: float, right: float, y: float) -> list[tuple[Point, Point]]:
    if right - left < grid.resolution * 0.35:
        return []
    step = max(1.0, grid.resolution * 0.25)
    segments: list[tuple[Point, Point]] = []
    segment_start: Point | None = None
    previous_free: Point | None = None
    candidate = left
    while candidate <= right:
        point = Point(candidate, y)
        free = not grid.is_blocked(grid.world_to_cell(point))
        if free:
            if segment_start is None:
                segment_start = point
            previous_free = point
        elif segment_start is not None and previous_free is not None:
            if previous_free.x - segment_start.x >= grid.resolution * 0.35:
                segments.append((segment_start, previous_free))
            segment_start = None
            previous_free = None
        candidate += step
    if segment_start is not None and previous_free is not None:
        end = Point(right, y) if not grid.is_blocked(grid.world_to_cell(Point(right, y))) else previous_free
        if end.x - segment_start.x >= grid.resolution * 0.35:
            segments.append((segment_start, end))
    return segments


def _cluster_cells(cells: frozenset[CoverageCell]) -> list[frozenset[CoverageCell]]:
    remaining = set(cells)
    clusters: list[frozenset[CoverageCell]] = []
    while remaining:
        seed = min(remaining, key=lambda cell: (cell[1], cell[0]))
        stack = [seed]
        remaining.remove(seed)
        cluster = {seed}
        while stack:
            current = stack.pop()
            for neighbor in _neighbors(current):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                cluster.add(neighbor)
                stack.append(neighbor)
        clusters.append(frozenset(cluster))
    return clusters


def _neighbors(cell: CoverageCell) -> tuple[CoverageCell, ...]:
    x, y = cell
    return (
        (x - 1, y - 1),
        (x, y - 1),
        (x + 1, y - 1),
        (x - 1, y),
        (x + 1, y),
        (x - 1, y + 1),
        (x, y + 1),
        (x + 1, y + 1),
    )


def _cluster_sort_key(cells: frozenset[CoverageCell]) -> tuple[int, int, int]:
    return (
        min(cell[1] for cell in cells),
        min(cell[0] for cell in cells),
        -len(cells),
    )


def _passes_for_cluster(
    cells: frozenset[CoverageCell],
    resolution: float,
    grid: GridMap | None = None,
) -> tuple[CoveragePass, ...]:
    rows: dict[int, list[int]] = defaultdict(list)
    for x, y in cells:
        rows[y].append(x)
    passes: list[CoveragePass] = []
    reverse = False
    for y in sorted(rows):
        chunks = _contiguous_chunks(sorted(rows[y]))
        ordered = list(reversed(chunks)) if reverse else chunks
        for min_x, max_x in ordered:
            left = coverage_cell_center((min_x, y), resolution)
            right = coverage_cell_center((max_x, y), resolution)
            if grid is None:
                start, end = (right, left) if reverse else (left, right)
                passes.append(CoveragePass(start, end))
                reverse = not reverse
                continue
            free_segments = _free_segments(
                grid, left.x - resolution / 2.0, right.x + resolution / 2.0, left.y
            )
            for free_start, free_end in free_segments:
                start, end = (free_end, free_start) if reverse else (free_start, free_end)
                passes.append(CoveragePass(start, end))
                reverse = not reverse
    return tuple(passes)


def _contiguous_chunks(values: list[int]) -> list[tuple[int, int]]:
    if not values:
        return []
    chunks: list[tuple[int, int]] = []
    start = end = values[0]
    for value in values[1:]:
        if value == end + 1:
            end = value
            continue
        chunks.append((start, end))
        start = end = value
    chunks.append((start, end))
    return chunks


def _cluster_center(cells: frozenset[CoverageCell], resolution: float) -> Point:
    centers = [coverage_cell_center(cell, resolution) for cell in cells]
    return Point(
        sum(point.x for point in centers) / len(centers),
        sum(point.y for point in centers) / len(centers),
    )


def _cluster_x_bounds(cells: frozenset[CoverageCell], resolution: float) -> tuple[float, float]:
    min_x = min(cell[0] for cell in cells) * resolution
    max_x = (max(cell[0] for cell in cells) + 1) * resolution
    return min_x, max_x


def _estimate_pass_load(origin: Point, passes: tuple[CoveragePass, ...]) -> float:
    total = 0.0
    cursor = origin
    for coverage_pass in passes:
        total += cursor.distance_to(coverage_pass.start)
        total += coverage_pass.start.distance_to(coverage_pass.end)
        cursor = coverage_pass.end
    return total


def _blocked_for_coverage(map_model: MapModel, point: Point) -> bool:
    return any(
        _contains_shape(
            item.shape,
            item.bounds,
            item.points,
            item.radius,
            point,
        )
        for item in map_model.obstacles
    ) or any(
        _contains_shape(
            item.shape,
            item.bounds,
            item.points,
            min(item.bounds.normalized.width, item.bounds.normalized.height) / 2.0,
            point,
        )
        for item in map_model.no_fly_zones
    )


def _contains_shape(
    shape: ObstacleShape,
    bounds: Rect,
    points: list[Point],
    radius: float,
    point: Point,
) -> bool:
    rect = bounds.normalized
    if shape == ObstacleShape.CIRCLE:
        center = Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0)
        return center.distance_to(point) <= max(0.0, radius)
    if shape == ObstacleShape.POLYGON and points:
        return point_in_polygon(point, points)
    return rect.contains(point)


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
