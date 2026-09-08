"""OPT-05 (plan §11.7): coverage blocks and cross-area chaining.

A block owns its entry, exit, scan direction, trajectory, duration and coverage
benefit. A single aircraft must be able to fly area A and then area B when its
cumulative time and energy allow it, and an entry/exit pair that does not
correspond to a real scan block must be refused.
"""

from __future__ import annotations

from collections.abc import Mapping

import pytest

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, SearchArea
from drone_mission_planner.planning.coverage import CoveragePass, CoveragePlanner
from drone_mission_planner.planning.coverage_blocks import (
    CoverageBlock,
    block_nodes,
    block_tasks,
    blocks_for_area,
    blocks_for_areas,
    cumulative_energy,
    scan_direction_of,
)
from drone_mission_planner.planning.optimization import (
    AssignmentProblem,
    SolverSettings,
    SolverStatus,
    SolverVehicle,
    evaluate_routes,
)


def _model(remaining_battery: float = 500.0) -> MapModel:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10.0,
            air_speed=10.0,
            battery_capacity=500.0,
            remaining_battery=remaining_battery,
        )
    )
    return model


def _area(area_id: str, x: float, *, direction: str = "horizontal") -> SearchArea:
    area = SearchArea(area_id, area_id, Rect(x, 50, 100, 100), scan_spacing=25.0)
    area.scan_direction = direction
    return area


def test_one_drone_produces_one_block_per_area() -> None:
    model = _model()
    drone = model.drones[0]

    blocks = blocks_for_area(model, _area("S-01", 50.0), drone)

    assert len(blocks) == 1
    block = blocks[0]
    assert block.entry == block.passes[0].start
    assert block.exit == block.passes[-1].end
    assert block.distance > 0
    assert block.duration_seconds > 0
    assert block.energy > 0
    assert block.coverage_benefit > 0


def test_block_trajectory_runs_from_entry_to_exit() -> None:
    model = _model()
    block = blocks_for_area(model, _area("S-01", 50.0), model.drones[0])[0]

    assert block.trajectory[0] == block.entry
    assert block.trajectory[-1] == block.exit


def test_vertical_area_yields_a_vertical_block() -> None:
    model = _model()
    block = blocks_for_area(
        model, _area("S-01", 50.0, direction="vertical"), model.drones[0]
    )[0]

    assert block.scan_direction == "vertical"
    assert all(item.start.x == pytest.approx(item.end.x) for item in block.passes)


def test_a_mismatched_entry_exit_pair_is_refused() -> None:
    """Plan §11.7: an entry/exit combination that is not a real block is invalid."""

    horizontal = CoveragePass(Point(0.0, 0.0), Point(10.0, 0.0))

    with pytest.raises(ValueError, match="does not match the pass geometry"):
        CoverageBlock(
            block_id="S-01:00",
            area_id="S-01",
            drone_id="D-01",
            scan_direction="vertical",
            entry=horizontal.start,
            exit=horizontal.end,
            passes=(horizontal,),
            duration_seconds=1.0,
            distance=10.0,
            energy=1.0,
            coverage_benefit=1,
        )


def test_ambiguous_pass_geometry_is_refused() -> None:
    with pytest.raises(ValueError, match="ambiguous or mixed"):
        scan_direction_of((CoveragePass(Point(0.0, 0.0), Point(0.0, 0.0)),))


def test_entry_must_be_the_first_pass_start() -> None:
    horizontal = CoveragePass(Point(0.0, 0.0), Point(10.0, 0.0))

    with pytest.raises(ValueError, match="first pass start"):
        CoverageBlock(
            block_id="S-01:00",
            area_id="S-01",
            drone_id="D-01",
            scan_direction="horizontal",
            entry=Point(5.0, 5.0),
            exit=horizontal.end,
            passes=(horizontal,),
            duration_seconds=1.0,
            distance=10.0,
            energy=1.0,
            coverage_benefit=1,
        )


def test_blocks_for_two_areas_are_keyed_by_area() -> None:
    model = _model()
    areas = [_area("S-01", 50.0), _area("S-02", 250.0)]

    grouped = blocks_for_areas(model, areas, model.drones[0])

    assert set(grouped) == {"S-01", "S-02"}
    assert all(len(items) == 1 for items in grouped.values())


def test_one_aircraft_can_chain_two_areas_within_its_energy() -> None:
    """Plan §11.10 acceptance: a single aircraft serves two independent areas."""

    model = _model()
    drone = model.drones[0]
    first = blocks_for_area(model, _area("S-01", 50.0), drone)[0]
    second = blocks_for_area(model, _area("S-02", 250.0), drone)[0]
    entries, exits = block_nodes((first, second), base=0)
    matrix = _matrix((first, second), entries, exits, node_count=6)
    tasks = block_tasks((first, second), node_of=entries.__getitem__,
                        exit_node_of=exits.__getitem__)
    problem = AssignmentProblem(
        node_count=6,
        travel_ticks=matrix,
        vehicles=(
            SolverVehicle(
                "D-01",
                0,
                5,
                energy_capacity=drone.battery_capacity,
                energy_per_tick=1.0,
            ),
        ),
        tasks=tasks,
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=1.0),
    )

    outcome = evaluate_routes(problem, [("D-01", (first.block_id, second.block_id))])

    assert outcome.status is SolverStatus.FEASIBLE
    assert cumulative_energy((first, second)) == pytest.approx(first.energy + second.energy)
    # The chained route must be longer than either block alone.
    assert outcome.makespan > first.duration_seconds + second.duration_seconds


def test_chained_areas_are_refused_when_energy_does_not_add_up() -> None:
    model = _model()
    drone = model.drones[0]
    first = blocks_for_area(model, _area("S-01", 50.0), drone)[0]
    second = blocks_for_area(model, _area("S-02", 250.0), drone)[0]
    entries, exits = block_nodes((first, second), base=0)
    problem = AssignmentProblem(
        node_count=6,
        travel_ticks=_matrix((first, second), entries, exits, node_count=6),
        vehicles=(
            SolverVehicle("D-01", 0, 5, energy_capacity=first.energy + 0.5, energy_per_tick=1.0),
        ),
        tasks=block_tasks((first, second), node_of=entries.__getitem__,
                          exit_node_of=exits.__getitem__),
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=1.0),
    )

    outcome = evaluate_routes(problem, [("D-01", (first.block_id, second.block_id))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "consumes" in dict(outcome.unassigned)["D-01"]


def test_plan_all_areas_serves_both_areas_with_one_drone() -> None:
    model = _model(remaining_battery=500.0)
    model.search_areas.extend([_area("S-01", 50.0), _area("S-02", 250.0)])

    results = CoveragePlanner().plan_all_areas(model)

    assert results["S-01"].drone_paths.get("D-01")
    assert results["S-02"].drone_paths.get("D-01")


def _matrix(
    blocks: tuple[CoverageBlock, ...],
    entries: Mapping[str, int],
    exits: Mapping[str, int],
    *,
    node_count: int,
) -> list[list[int]]:
    """Tick matrix over base 0, block entries/exits and a shared end base."""

    points: dict[int, Point] = {0: Point(10.0, 10.0), node_count - 1: Point(10.0, 10.0)}
    for block in blocks:
        points[entries[block.block_id]] = block.entry
        points[exits[block.block_id]] = block.exit
    matrix = [[0] * node_count for _ in range(node_count)]
    for i in range(node_count):
        for j in range(node_count):
            if i == j:
                continue
            matrix[i][j] = max(1, round(points[i].distance_to(points[j]) / 10.0))
    return matrix
