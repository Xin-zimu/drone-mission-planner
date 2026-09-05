from __future__ import annotations

from drone_mission_planner.domain.enums import TaskType, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint, waypoints_from_path

from .altitude_validator import validate_altitude_path
from .astar import AStarPlanner
from .energy import estimate_path_energy, flight_altitude_at
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
            profile = estimate_path_energy(
                drone,
                result.waypoints,
                terrain=map_model.terrain,
                wind=map_model.wind,
            )
            result.estimated_time = profile.time
            result.estimated_energy = profile.energy
            result.altitude_risks = validate_altitude_path(
                map_model,
                drone,
                result.waypoints,
            )
            result.flight_waypoints = _build_flight_waypoints(map_model, drone, result.waypoints, goal)
        return result


def _build_flight_waypoints(
    map_model: MapModel,
    drone: Drone,
    path: list[Point],
    goal: Point,
) -> list[Waypoint]:
    task = _matching_goal_task(map_model, drone, goal)
    last_index = len(path) - 1
    flight_waypoints = waypoints_from_path(
        path,
        altitude_provider=lambda point, index: flight_altitude_at(
            drone,
            point,
            map_model.terrain,
            target_altitude=task.target_altitude if task is not None and index == last_index else None,
        ),
    )
    if task is not None and flight_waypoints:
        final_waypoint = flight_waypoints[-1]
        final_waypoint.task_id = task.id
        final_waypoint.action = _task_action(task)
        final_waypoint.hold_seconds = task.execution_duration
    return flight_waypoints


def _matching_goal_task(map_model: MapModel, drone: Drone, goal: Point) -> MissionTask | None:
    for task in sorted(map_model.tasks, key=lambda item: item.id):
        if task.position.distance_to(goal) > 1e-6:
            continue
        if task.assigned_drone_id not in {None, drone.id} and task.id not in drone.assigned_tasks:
            continue
        return task
    return None


def _task_action(task: MissionTask) -> WaypointAction:
    if task.task_type == TaskType.AREA_SEARCH:
        return WaypointAction.SCAN
    if task.task_type == TaskType.INSPECTION:
        return WaypointAction.TAKE_PHOTO
    if task.task_type == TaskType.RETURN_HOME:
        return WaypointAction.RETURN_TO_LAUNCH
    if task.execution_duration > 0:
        return WaypointAction.HOVER
    return WaypointAction.FLY_TO
