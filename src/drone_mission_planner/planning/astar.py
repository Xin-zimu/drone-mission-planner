from __future__ import annotations

import heapq
from itertools import count, pairwise
from math import sqrt

from drone_mission_planner.domain.geometry import Point

from .grid import Cell, GridMap
from .path_smoothing import smooth_path
from .result import PathResult


class AStarPlanner:
    def plan(
        self,
        grid: GridMap,
        start: Point,
        goal: Point,
        *,
        max_speed: float = 1.0,
        energy_per_meter: float = 0.0,
        smoothing: bool = True,
        max_expansions: int = 500_000,
    ) -> PathResult:
        start_cell = grid.world_to_cell(start)
        goal_cell = grid.world_to_cell(goal)
        if not grid.in_bounds(start_cell):
            return PathResult.failure("Start position is outside the map")
        if not grid.in_bounds(goal_cell):
            return PathResult.failure("Goal position is outside the map")
        if grid.is_blocked(start_cell):
            return PathResult.failure("Start position is inside an obstacle or no-fly zone")
        if grid.is_blocked(goal_cell):
            return PathResult.failure("Goal position is inside an obstacle or no-fly zone")
        if start_cell == goal_cell:
            waypoints = [start, goal] if start != goal else [start]
            distance = start.distance_to(goal)
            return PathResult(
                True,
                waypoints=waypoints,
                total_distance=distance,
                estimated_time=distance / max(max_speed, 1e-9),
                estimated_energy=distance * energy_per_meter,
                raw_waypoint_count=len(waypoints),
            )

        frontier: list[tuple[float, float, int, Cell]] = []
        sequence = count()
        start_h = self._octile(start_cell, goal_cell)
        heapq.heappush(frontier, (start_h, start_h, next(sequence), start_cell))
        came_from: dict[Cell, Cell] = {}
        cost_so_far: dict[Cell, float] = {start_cell: 0.0}
        expanded = 0

        while frontier:
            _, _, _, current = heapq.heappop(frontier)
            if current == goal_cell:
                cells = self._reconstruct(came_from, current)
                raw = [grid.cell_to_world(cell) for cell in cells]
                raw[0] = start
                raw[-1] = goal
                waypoints = smooth_path(raw, grid) if smoothing else raw
                distance = sum(first.distance_to(second) for first, second in pairwise(waypoints))
                return PathResult(
                    True,
                    waypoints=waypoints,
                    total_distance=distance,
                    estimated_time=distance / max(max_speed, 1e-9),
                    estimated_energy=distance * energy_per_meter,
                    expanded_nodes=expanded,
                    raw_waypoint_count=len(raw),
                )
            expanded += 1
            if expanded > max_expansions:
                return PathResult.failure(
                    f"Planning exceeded the {max_expansions} node safety limit",
                    expanded_nodes=expanded,
                )
            for neighbor, move_cost in grid.neighbors(current):
                new_cost = cost_so_far[current] + move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heuristic = self._octile(neighbor, goal_cell)
                    heapq.heappush(
                        frontier,
                        (new_cost + heuristic, heuristic, next(sequence), neighbor),
                    )
        return PathResult.failure(
            "No safe path connects the start and goal", expanded_nodes=expanded
        )

    @staticmethod
    def _octile(first: Cell, second: Cell) -> float:
        dx = abs(first[0] - second[0])
        dy = abs(first[1] - second[1])
        return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

    @staticmethod
    def _reconstruct(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
        result = [current]
        while current in came_from:
            current = came_from[current]
            result.append(current)
        result.reverse()
        return result
