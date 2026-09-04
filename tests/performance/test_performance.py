from __future__ import annotations

from time import perf_counter

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import Drone, MapModel, Obstacle
from drone_mission_planner.planning.grid import GridMap
from drone_mission_planner.planning.route_planner import RoutePlanner


def test_normal_route_plans_below_one_second() -> None:
    model = MapModel(width=1000, height=700, grid_size=10)
    model.obstacles.append(Obstacle("O-01", "Block", bounds=Rect(400, 100, 140, 480)))
    start = perf_counter()
    result = RoutePlanner().plan(model, Drone("D-01", "Drone", Point(30, 30)), Point(950, 650))
    elapsed = perf_counter() - start
    assert result.success
    assert elapsed < 1.0


def test_maximum_grid_rasterizes_below_one_second() -> None:
    model = MapModel(width=5000, height=5000, grid_size=10)
    model.obstacles.append(Obstacle("O-01", "Block", bounds=Rect(1000, 1000, 800, 2500)))
    start = perf_counter()
    grid = GridMap.from_map(model, safety_radius=8)
    elapsed = perf_counter() - start
    assert (grid.width, grid.height) == (500, 500)
    assert elapsed < 1.0
