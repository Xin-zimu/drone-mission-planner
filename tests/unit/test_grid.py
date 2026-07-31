from __future__ import annotations

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import MapModel, NoFlyZone, Obstacle
from drone_mission_planner.planning.grid import GridMap


def test_obstacles_and_no_fly_zones_are_rasterized() -> None:
    model = MapModel(width=200, height=150, grid_size=10.0)
    model.obstacles.append(Obstacle("O-01", "Block", bounds=Rect(50, 40, 20, 30)))
    model.no_fly_zones.append(NoFlyZone("N-01", "Zone", bounds=Rect(120, 80, 20, 20)))
    grid = GridMap.from_map(model)
    assert grid.is_blocked(grid.world_to_cell(Point(55, 45)))
    assert grid.is_blocked(grid.world_to_cell(Point(125, 85)))
    assert not grid.is_blocked(grid.world_to_cell(Point(20, 20)))


def test_safety_inflation_blocks_nearby_cells() -> None:
    model = MapModel(width=200, height=150, grid_size=10.0)
    model.obstacles.append(Obstacle("O-01", "Block", bounds=Rect(50, 40, 20, 30)))
    grid = GridMap.from_map(model, safety_radius=12.0)
    assert grid.is_blocked(grid.world_to_cell(Point(42, 45)))
    assert grid.is_blocked(grid.world_to_cell(Point(75, 45)))


def test_diagonal_neighbors_do_not_cut_corners() -> None:
    grid = GridMap(5, 5, 10.0, blocked={(2, 1)})
    neighbors = dict(grid.neighbors((1, 1)))
    assert (2, 2) not in neighbors
    assert (1, 2) in neighbors
