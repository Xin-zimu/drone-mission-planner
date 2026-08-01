from __future__ import annotations

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel

from .astar import AStarPlanner
from .grid import GridMap
from .result import PathResult
from .validator import validate_path


class RoutePlanner:
    def __init__(self, astar: AStarPlanner | None = None) -> None:
        self.astar = astar or AStarPlanner()

    def plan(self, map_model: MapModel, drone: Drone, goal: Point) -> PathResult:
        return self.plan_between(map_model, drone, drone.position, goal)

    def plan_between(
        self, map_model: MapModel, drone: Drone, start: Point, goal: Point
    ) -> PathResult:
        grid = GridMap.from_map(map_model, safety_radius=drone.safety_radius)
        result = self.astar.plan(
            grid,
            start,
            goal,
            max_speed=drone.max_speed,
            energy_per_meter=drone.energy_per_meter,
        )
        if result.success:
            valid, reason = validate_path(result.waypoints, grid)
            if not valid:
                return PathResult.failure(f"Post-planning validation failed: {reason}")
        return result
