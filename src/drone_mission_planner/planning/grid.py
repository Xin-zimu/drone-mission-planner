from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, hypot

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import MapModel

type Cell = tuple[int, int]


@dataclass(slots=True)
class GridMap:
    width: int
    height: int
    resolution: float
    blocked: set[Cell]

    @classmethod
    def from_map(cls, model: MapModel, *, safety_radius: float = 0.0) -> GridMap:
        resolution = model.grid_size
        grid = cls(
            width=ceil(model.width / resolution),
            height=ceil(model.height / resolution),
            resolution=resolution,
            blocked=set(),
        )
        for obstacle in model.obstacles:
            grid.block_rect(obstacle.bounds, inflation=safety_radius)
        for zone in model.no_fly_zones:
            grid.block_rect(zone.bounds, inflation=safety_radius)
        return grid

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def is_blocked(self, cell: Cell) -> bool:
        return not self.in_bounds(cell) or cell in self.blocked

    def world_to_cell(self, point: Point) -> Cell:
        return floor(point.x / self.resolution), floor(point.y / self.resolution)

    def cell_to_world(self, cell: Cell) -> Point:
        return Point((cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution)

    def block_rect(self, rect: Rect, *, inflation: float = 0.0) -> None:
        bounds = rect.normalized
        expanded = Rect(
            bounds.x - inflation,
            bounds.y - inflation,
            bounds.width + 2 * inflation,
            bounds.height + 2 * inflation,
        )
        min_x = max(0, floor(expanded.x / self.resolution))
        min_y = max(0, floor(expanded.y / self.resolution))
        max_x = min(self.width - 1, floor((expanded.x + expanded.width) / self.resolution))
        max_y = min(self.height - 1, floor((expanded.y + expanded.height) / self.resolution))
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                self.blocked.add((x, y))

    def segment_is_clear(self, start: Point, end: Point) -> bool:
        distance = start.distance_to(end)
        steps = max(1, ceil(distance / (self.resolution * 0.35)))
        for index in range(steps + 1):
            point = start.lerp(end, index / steps)
            if self.is_blocked(self.world_to_cell(point)):
                return False
        return True

    def neighbors(self, cell: Cell) -> list[tuple[Cell, float]]:
        result: list[tuple[Cell, float]] = []
        directions = [
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (-1, 1),
            (-1, -1),
            (1, -1),
        ]
        for dx, dy in directions:
            target = (cell[0] + dx, cell[1] + dy)
            if self.is_blocked(target):
                continue
            if dx and dy:
                if self.is_blocked((cell[0] + dx, cell[1])) or self.is_blocked(
                    (cell[0], cell[1] + dy)
                ):
                    continue
                cost = hypot(dx, dy)
            else:
                cost = 1.0
            result.append((target, cost))
        return result
