from __future__ import annotations

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import MapModel, Obstacle
from drone_mission_planner.planning.astar import AStarPlanner
from drone_mission_planner.planning.grid import GridMap
from drone_mission_planner.planning.validator import validate_path


def test_open_map_path_is_direct_after_smoothing() -> None:
    grid = GridMap(20, 20, 10.0, blocked=set())
    result = AStarPlanner().plan(grid, Point(15, 15), Point(155, 125), max_speed=10.0)
    assert result.success
    assert result.waypoints == [Point(15, 15), Point(155, 125)]
    assert result.estimated_time > 0


def test_path_routes_around_obstacle_and_validates() -> None:
    model = MapModel(width=300, height=200, grid_size=10.0)
    model.obstacles.append(Obstacle("O-01", "Wall", bounds=Rect(120, 0, 30, 150)))
    grid = GridMap.from_map(model, safety_radius=5.0)
    result = AStarPlanner().plan(grid, Point(30, 50), Point(250, 50))
    assert result.success
    assert len(result.waypoints) >= 3
    assert validate_path(result.waypoints, grid) == (True, None)


def test_unreachable_goal_reports_reason() -> None:
    blocked = {(x, 5) for x in range(10)}
    grid = GridMap(10, 10, 10.0, blocked=blocked)
    result = AStarPlanner().plan(grid, Point(15, 15), Point(15, 85))
    assert not result.success
    assert result.failure_reason == "No safe path connects the start and goal"


def test_blocked_endpoints_report_specific_error() -> None:
    grid = GridMap(10, 10, 10.0, blocked={(1, 1), (8, 8)})
    start_result = AStarPlanner().plan(grid, Point(15, 15), Point(50, 50))
    goal_result = AStarPlanner().plan(grid, Point(50, 50), Point(85, 85))
    assert start_result.failure_reason and start_result.failure_reason.startswith("Start position")
    assert goal_result.failure_reason and goal_result.failure_reason.startswith("Goal position")


def test_planning_is_deterministic() -> None:
    grid = GridMap(30, 30, 10.0, blocked={(15, y) for y in range(4, 25)})
    planner = AStarPlanner()
    first = planner.plan(grid, Point(20, 20), Point(280, 280), smoothing=False)
    second = planner.plan(grid, Point(20, 20), Point(280, 280), smoothing=False)
    assert first.waypoints == second.waypoints
    assert first.expanded_nodes == second.expanded_nodes
