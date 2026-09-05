from __future__ import annotations

from itertools import pairwise

import pytest

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, Obstacle, SearchArea
from drone_mission_planner.planning.coverage import (
    CoveragePlanner,
    point_in_polygon,
    polygon_area,
    target_cells_for_area,
)
from drone_mission_planner.planning.grid import GridMap
from drone_mission_planner.planning.validator import validate_path
from drone_mission_planner.simulation.engine import SimulationEngine


def coverage_map() -> tuple[MapModel, SearchArea]:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40, 200)))
    model.drones.extend(
        [
            Drone("D-01", "Alpha", Point(40, 180), "B-01", safety_radius=5),
            Drone("D-02", "Bravo", Point(40, 220), "B-01", safety_radius=5),
        ]
    )
    model.obstacles.append(Obstacle("O-01", "Tower", bounds=Rect(250, 140, 55, 90)))
    area = SearchArea(
        "S-01",
        "Ridge search",
        Rect(100, 70, 430, 260),
        scan_spacing=35,
        boundary_margin=12,
    )
    model.search_areas.append(area)
    return model, area


def test_polygon_geometry_handles_edges_and_area() -> None:
    polygon = [Point(0, 0), Point(8, 0), Point(8, 6), Point(0, 6)]
    assert polygon_area(polygon) == 48
    assert point_in_polygon(Point(4, 3), polygon)
    assert point_in_polygon(Point(0, 2), polygon)
    assert not point_in_polygon(Point(9, 3), polygon)


def test_strips_partition_area_and_alternate_pass_direction() -> None:
    model, area = coverage_map()
    strips = CoveragePlanner().build_strips(model, area, model.drones)

    assert len(strips) == 2
    assert strips[0].max_x == pytest.approx(strips[1].min_x)
    assert strips[0].min_x == pytest.approx(100)
    assert strips[1].max_x == pytest.approx(530)
    directions = [coverage_pass.end.x - coverage_pass.start.x for coverage_pass in strips[0].passes]
    assert directions
    assert all(first * second < 0 for first, second in pairwise(directions))


def test_coverage_routes_are_safe_and_return_home() -> None:
    model, area = coverage_map()
    result = CoveragePlanner().plan(model, area)

    assert result.success, result.failures
    assert set(result.drone_paths) == {"D-01", "D-02"}
    for drone in model.drones:
        path = result.drone_paths[drone.id]
        assert path[0] == drone.position
        assert path[-1] == model.bases[0].position
        grid = GridMap.from_map(model, safety_radius=drone.safety_radius)
        assert validate_path(path, grid) == (True, None)
        assert result.drone_distances[drone.id] > 500


def test_planned_sweep_reaches_target_coverage_in_simulation() -> None:
    model, area = coverage_map()
    result = CoveragePlanner().plan(model, area)
    for drone in model.drones:
        drone.planned_path = result.drone_paths[drone.id]
        drone.waypoints = result.drone_waypoints[drone.id]

    engine = SimulationEngine(model)
    engine.run_until_complete()
    coverage = engine.snapshot().coverage[0]

    assert coverage.coverage >= area.target_coverage
    assert coverage.repeat_coverage < coverage.coverage


def incremental_map() -> tuple[MapModel, SearchArea]:
    model = MapModel(width=100, height=60, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(5, 5)))
    model.drones.extend(
        [
            Drone("D-01", "Alpha", Point(5, 5), "B-01", safety_radius=6),
            Drone("D-02", "Bravo", Point(5, 30), "B-01", safety_radius=6),
        ]
    )
    area = SearchArea("S-01", "Patch", Rect(10, 10, 60, 30), scan_spacing=25)
    model.search_areas.append(area)
    return model, area


def test_incremental_with_no_covered_cells_targets_the_whole_area() -> None:
    model, area = incremental_map()
    targets = target_cells_for_area(model, area)

    result = CoveragePlanner().plan(model, area, covered_cells=())

    assert result.incremental
    assert result.target_cells == len(targets)
    assert result.remaining_cells == len(targets)
    assert result.covered_input_cells == 0
    assert result.success, result.failures


def test_incremental_with_full_coverage_returns_empty_success() -> None:
    model, area = incremental_map()
    targets = target_cells_for_area(model, area)

    result = CoveragePlanner().plan(model, area, covered_cells=targets)

    assert result.incremental
    assert result.remaining_cells == 0
    assert result.drone_paths == {}
    assert result.failures == {}
    assert result.success


def test_incremental_remaining_cells_excludes_covered_input() -> None:
    model, area = incremental_map()
    targets = target_cells_for_area(model, area)
    covered = set(sorted(targets)[: len(targets) // 2])

    result = CoveragePlanner().plan(model, area, covered_cells=covered)

    assert result.incremental
    assert result.remaining_cells == len(targets) - len(covered)
    assert result.covered_input_cells == len(covered)
    assert result.success, result.failures


def test_incremental_passes_endpoints_stay_on_the_inflated_free_grid() -> None:
    model, area = incremental_map()
    # Obstacle overlaps the right-hand target columns so raw cell centers in the
    # inflated grid become blocked; incremental strips must clamp onto free segments.
    model.obstacles.append(Obstacle("O-01", "Tower", bounds=Rect(50, 8, 30, 12), height=45))
    planner = CoveragePlanner()
    grid = GridMap.from_map(model, safety_radius=6)

    result = planner.plan(model, area, covered_cells=())

    assert result.success, result.failures
    assert result.drone_paths
    for drone_id, path in result.drone_paths.items():
        valid, reason = validate_path(path, grid)
        assert valid, f"{drone_id} incremental route is unsafe: {reason}"


def test_incremental_does_not_require_every_drone_when_work_is_small() -> None:
    model, area = incremental_map()
    targets = target_cells_for_area(model, area)
    covered = targets - {min(targets)}

    result = CoveragePlanner().plan(model, area, covered_cells=covered)

    assert result.remaining_cells == 1
    assert result.success
    assert result.failures == {}
    assert len(result.drone_paths) == 1
