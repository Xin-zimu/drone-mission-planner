from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, SearchArea
from drone_mission_planner.planning.coverage import (
    CoveragePlanner,
    target_cells_for_area,
)


def _model_with_area(holes: list[list[Point]] | None = None) -> tuple[MapModel, SearchArea]:
    model = MapModel(width=300, height=300, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    drone = Drone("D-01", "Alpha", Point(10, 10), "B-01", max_speed=10)
    model.drones.append(drone)
    area = SearchArea("S-01", "Field", Rect(50, 50, 200, 200), scan_spacing=25.0)
    if holes:
        area.holes = holes
    model.search_areas.append(area)
    return model, area


def test_hole_cells_are_not_coverage_targets() -> None:
    plain_model, plain_area = _model_with_area()
    holed_model, holed_area = _model_with_area(
        holes=[[Point(120.0, 120.0), Point(180.0, 120.0), Point(180.0, 180.0), Point(120.0, 180.0)]]
    )

    plain_targets = target_cells_for_area(plain_model, plain_area)
    holed_targets = target_cells_for_area(holed_model, holed_area)

    assert plain_targets
    from drone_mission_planner.planning.coverage import (
        coverage_cell_center,
        coverage_resolution_for,
    )

    hole_cells = 0
    for cell in holed_targets:
        center = coverage_cell_center(cell, coverage_resolution_for(holed_model, holed_area))
        if 120.0 < center.x < 180.0 and 120.0 < center.y < 180.0:
            hole_cells += 1
    assert hole_cells == 0
    assert len(holed_targets) < len(plain_targets)


def test_strips_do_not_enter_the_hole() -> None:
    model, area = _model_with_area(
        holes=[[Point(120.0, 120.0), Point(180.0, 120.0), Point(180.0, 180.0), Point(120.0, 180.0)]]
    )
    result = CoveragePlanner().plan(model, area)
    assert not result.failures

    def inside_hole(point: Point) -> bool:
        return 120.0 < point.x < 180.0 and 120.0 < point.y < 180.0

    endpoints = [
        point
        for strip in result.strips
        for coverage_pass in strip.passes
        for point in (coverage_pass.start, coverage_pass.end)
    ]
    assert endpoints
    assert not any(inside_hole(point) for point in endpoints)


def test_vertical_scan_direction_rotates_the_pattern() -> None:
    model, area = _model_with_area()
    horizontal = CoveragePlanner().plan(model, area)
    area.scan_direction = "vertical"
    vertical = CoveragePlanner().plan(model, area)

    horizontal_ends = [
        (coverage_pass.start, coverage_pass.end)
        for strip in horizontal.strips
        for coverage_pass in strip.passes
    ]
    vertical_ends = [
        (coverage_pass.start, coverage_pass.end)
        for strip in vertical.strips
        for coverage_pass in strip.passes
    ]
    assert horizontal_ends
    assert vertical_ends
    assert all(start.y == pytest.approx(end.y) for start, end in horizontal_ends[:3])
    assert all(start.x == pytest.approx(end.x) for start, end in vertical_ends[:3])


def test_plan_all_areas_respects_priority_order() -> None:
    model = MapModel(width=400, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(Drone("D-01", "Alpha", Point(10, 10), "B-01", max_speed=10))
    low = SearchArea("S-LOW", "Low", Rect(50, 50, 100, 100), scan_spacing=25.0, priority=0)
    high = SearchArea("S-HIGH", "High", Rect(220, 50, 100, 100), scan_spacing=25.0, priority=5)
    model.search_areas.extend([low, high])

    results = CoveragePlanner().plan_all_areas(model)

    assert results["S-HIGH"].drone_paths.get("D-01")
    assert not results["S-LOW"].drone_paths.get("D-01")
    assert "No drones available" in results["S-LOW"].failures.values()


def test_incremental_rescan_is_shorter_than_full_replan() -> None:
    model, area = _model_with_area()
    planner = CoveragePlanner()
    full = planner.plan(model, area)
    ordered_targets = sorted(target_cells_for_area(model, area), reverse=True)
    covered = frozenset(ordered_targets[: len(ordered_targets) // 2])

    partial = planner.plan(model, area, covered_cells=covered)

    assert partial.incremental
    assert partial.total_distance < full.total_distance
