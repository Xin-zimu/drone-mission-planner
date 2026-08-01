from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, floor

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MapModel, SearchArea
from drone_mission_planner.planning.coverage import point_in_polygon

type CoverageCell = tuple[int, int]


@dataclass(frozen=True, slots=True)
class AreaCoverageSnapshot:
    area_id: str
    target_cells: int
    covered_cells: int
    repeat_covered_cells: int

    @property
    def coverage(self) -> float:
        return self.covered_cells / self.target_cells if self.target_cells else 0.0

    @property
    def repeat_coverage(self) -> float:
        return self.repeat_covered_cells / self.target_cells if self.target_cells else 0.0


@dataclass(slots=True)
class _AreaState:
    area: SearchArea
    resolution: float
    targets: set[CoverageCell]
    visits: dict[CoverageCell, set[str]] = field(default_factory=dict)


class CoverageMonitor:
    """Track unique and repeat coverage over accessible cells in each search area."""

    def __init__(self, map_model: MapModel) -> None:
        self.map_model = map_model
        self._states = {area.id: self._create_state(area) for area in map_model.search_areas}

    def update(self, drone_positions: dict[str, Point]) -> None:
        for state in self._states.values():
            sensor_radius = max(state.resolution, state.area.scan_spacing / 2.0)
            cell_radius = ceil(sensor_radius / state.resolution)
            for drone_id, position in drone_positions.items():
                center = self._world_to_cell(position, state.resolution)
                for offset_x in range(-cell_radius, cell_radius + 1):
                    for offset_y in range(-cell_radius, cell_radius + 1):
                        cell = (center[0] + offset_x, center[1] + offset_y)
                        if cell not in state.targets:
                            continue
                        cell_center = self._cell_center(cell, state.resolution)
                        if position.distance_to(cell_center) <= sensor_radius:
                            state.visits.setdefault(cell, set()).add(drone_id)

    def reset(self) -> None:
        for state in self._states.values():
            state.visits.clear()

    def snapshot(self) -> tuple[AreaCoverageSnapshot, ...]:
        return tuple(
            AreaCoverageSnapshot(
                area_id,
                len(state.targets),
                len(state.visits),
                sum(1 for visitors in state.visits.values() if len(visitors) >= 2),
            )
            for area_id, state in sorted(self._states.items())
        )

    def render_cells(self) -> dict[str, tuple[tuple[Point, int], ...]]:
        return {
            area_id: tuple(
                (self._cell_center(cell, state.resolution), len(visitors))
                for cell, visitors in sorted(state.visits.items())
            )
            for area_id, state in self._states.items()
        }

    def resolution(self, area_id: str) -> float:
        state = self._states.get(area_id)
        return state.resolution if state is not None else 0.0

    def _create_state(self, area: SearchArea) -> _AreaState:
        resolution = max(5.0, min(self.map_model.grid_size, area.scan_spacing / 2.0))
        polygon = area.polygon()
        min_x = min(point.x for point in polygon)
        max_x = max(point.x for point in polygon)
        min_y = min(point.y for point in polygon)
        max_y = max(point.y for point in polygon)
        targets: set[CoverageCell] = set()
        for x in range(floor(min_x / resolution), ceil(max_x / resolution)):
            for y in range(floor(min_y / resolution), ceil(max_y / resolution)):
                cell = (x, y)
                center = self._cell_center(cell, resolution)
                if point_in_polygon(center, polygon) and not self._blocked(center):
                    targets.add(cell)
        return _AreaState(area, resolution, targets)

    def _blocked(self, point: Point) -> bool:
        return any(item.bounds.contains(point) for item in self.map_model.obstacles) or any(
            item.bounds.contains(point) for item in self.map_model.no_fly_zones
        )

    @staticmethod
    def _world_to_cell(point: Point, resolution: float) -> CoverageCell:
        return floor(point.x / resolution), floor(point.y / resolution)

    @staticmethod
    def _cell_center(cell: CoverageCell, resolution: float) -> Point:
        return Point((cell[0] + 0.5) * resolution, (cell[1] + 0.5) * resolution)
