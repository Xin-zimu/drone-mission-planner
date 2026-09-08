"""F3-a: solver interface, directed cost matrix, VRPTW and baseline comparison."""

from __future__ import annotations

from itertools import permutations, product

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.planning import optimization as opt
from drone_mission_planner.planning.optimization import (
    AssignmentProblem,
    ORToolsAssignmentSolver,
    SolverSettings,
    SolverStatus,
    SolverTask,
    SolverVehicle,
    evaluate_routes,
    greedy_routes,
    solve_with_baseline,
    ticks_down,
    ticks_up,
)
from drone_mission_planner.planning.travel_costs import (
    TravelCostProvider,
    drone_profile_key,
    environment_revision,
)

BIG = opt._BIG

# node 0 is the shared base, nodes 1-3 are missions
MATRIX = [
    [0, 10, 20, 30],
    [10, 0, 5, 15],
    [20, 5, 0, 10],
    [30, 15, 10, 0],
]


def _problem(
    matrix: list[list[int]] = MATRIX,
    *,
    vehicles: int = 2,
    mandatory: tuple[bool, ...] | None = None,
    settings: SolverSettings | None = None,
    requires: tuple[str, ...] = (),
) -> AssignmentProblem:
    task_count = len(matrix) - 1
    flags = mandatory or (True,) * task_count
    return AssignmentProblem(
        node_count=len(matrix),
        travel_ticks=matrix,
        vehicles=tuple(
            SolverVehicle(f"D-{index + 1:02d}", 0, 0, capacity=10.0) for index in range(vehicles)
        ),
        tasks=tuple(
            SolverTask(f"T-{node:02d}", node, mandatory=flags[node - 1])
            for node in range(1, task_count + 1)
        ),
        settings=settings or SolverSettings(time_limit_seconds=2.0, tick_seconds=1.0),
        requires=requires,
    )


def test_status_enum_covers_five_explicit_outcomes() -> None:
    assert {status.value for status in SolverStatus} == {
        "feasible",
        "infeasible",
        "timeout",
        "cancelled",
        "unsupported_constraint",
    }


def test_unsupported_constraint_is_reported_not_ignored() -> None:
    problem = _problem(requires=("cross_vehicle_synchronisation",))

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.UNSUPPORTED_CONSTRAINT
    assert outcome.unsupported == ("cross_vehicle_synchronisation",)
    assert "cross_vehicle_synchronisation" in outcome.message
    assert outcome.routes == ()
    assert not outcome.feasible


def test_dependency_energy_and_return_are_supported_since_opt04() -> None:
    """OPT-04 closed the gap this test used to encode (plan §11.2/§11.10)."""

    assert {"predecessor_ids", "energy", "return_energy"} <= set(opt.SUPPORTED_CONSTRAINTS)
    problem = _problem(requires=("predecessor_ids", "energy", "return_energy"))

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is not SolverStatus.UNSUPPORTED_CONSTRAINT


def test_missing_ortools_degrades_to_unsupported(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(opt, "ORTOOLS_AVAILABLE", False)

    outcome = ORToolsAssignmentSolver().solve(_problem())

    assert outcome.status is SolverStatus.UNSUPPORTED_CONSTRAINT
    assert outcome.unsupported == ("ortools",)
    # The baseline is still available without OR-Tools.
    baseline = evaluate_routes(_problem(), greedy_routes(_problem()))
    assert baseline.status is SolverStatus.FEASIBLE


def test_timeout_is_not_reported_as_infeasible(monkeypatch: pytest.MonkeyPatch) -> None:
    # Two mandatory missions that cannot both fit one aircraft's capacity, so no
    # solution exists, and the clock is forced past the budget: the controlled
    # timeout branch must answer, never an infeasibility claim (plan §11.3).
    problem = AssignmentProblem(
        node_count=3,
        travel_ticks=[[0, 1, 2], [1, 0, 1], [2, 1, 0]],
        vehicles=(SolverVehicle("D-01", 0, 0, capacity=1.0),),
        tasks=(SolverTask("T-01", 1, demand=1.0), SolverTask("T-02", 2, demand=1.0)),
        settings=SolverSettings(time_limit_seconds=0.001, tick_seconds=1.0),
    )
    clock = iter([0.0, 100.0])
    monkeypatch.setattr(opt, "perf_counter", lambda: next(clock))

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.TIMEOUT
    assert "not a proof" in outcome.message


def test_cancellation_is_reported() -> None:
    outcome = ORToolsAssignmentSolver().solve(_problem(), cancel=lambda: True)

    assert outcome.status is SolverStatus.CANCELLED
    assert not outcome.feasible


def test_mandatory_unreachable_mission_is_infeasible_with_reason() -> None:
    matrix = [row[:] for row in MATRIX]
    for row in matrix:
        row[3] = BIG
    problem = _problem(matrix)

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.INFEASIBLE
    assert any(task_id == "T-03" for task_id, _ in outcome.unassigned)
    assert any("reach" in reason for _, reason in outcome.unassigned)


def _exhaustive_best(problem: AssignmentProblem) -> float:
    task_ids = [task.task_id for task in problem.tasks]
    vehicle_ids = [vehicle.drone_id for vehicle in problem.vehicles]
    best = float("inf")
    for assignment in product(vehicle_ids, repeat=len(task_ids)):
        loads: dict[str, list[str]] = {vehicle_id: [] for vehicle_id in vehicle_ids}
        for task_id, vehicle_id in zip(task_ids, assignment, strict=True):
            loads[vehicle_id].append(task_id)
        for orders in product(*(list(permutations(loads[v])) for v in vehicle_ids)):
            routes = [(vehicle_ids[i], orders[i]) for i in range(len(vehicle_ids))]
            outcome = evaluate_routes(problem, routes)
            if outcome.status is SolverStatus.FEASIBLE:
                best = min(best, outcome.makespan)
    return best


def test_small_instance_matches_the_exhaustive_optimum() -> None:
    problem = _problem()

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.FEASIBLE
    best = _exhaustive_best(problem)
    assert outcome.makespan == pytest.approx(best)


def test_baseline_is_never_replaced_by_a_worse_candidate() -> None:
    problem = _problem()
    greedy = evaluate_routes(problem, greedy_routes(problem))

    chosen, kept_baseline = solve_with_baseline(problem)

    assert chosen.status is SolverStatus.FEASIBLE
    assert chosen.makespan <= greedy.makespan + 1e-9
    if kept_baseline:
        assert chosen.makespan == pytest.approx(greedy.makespan)
    else:
        assert chosen.makespan < greedy.makespan - 1e-9


def test_rounding_is_conservative_for_time_and_deadlines() -> None:
    assert ticks_up(0.95, 0.1) == 10
    assert ticks_up(1.0, 0.1) == 10
    assert ticks_down(1.05, 0.1) == 10
    assert ticks_down(0.99, 0.1) == 9


def test_an_empty_window_after_rounding_is_reported() -> None:
    problem = AssignmentProblem(
        node_count=2,
        travel_ticks=[[0, 1], [1, 0]],
        vehicles=(SolverVehicle("D-01", 0, 0),),
        tasks=(SolverTask("T-01", 1, earliest_start=1.05, deadline=1.04, service_seconds=0.05),),
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=0.1),
    )

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.INFEASIBLE
    assert any("window" in reason for _, reason in outcome.unassigned)


def _map(wind_speed: float = 0.0) -> MapModel:
    model = MapModel(width=400, height=200, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(20.0, 20.0)))
    model.wind = WindModel(direction_to_deg=90.0, speed=wind_speed, enabled=wind_speed > 0)
    return model


def _drone() -> Drone:
    return Drone(
        "D-01",
        "Alpha",
        Point(20.0, 20.0),
        "B-01",
        max_speed=10.0,
        air_speed=10.0,
        battery_capacity=500.0,
        remaining_battery=500.0,
    )


def test_leg_costs_are_directional_and_cached_by_environment() -> None:
    model = _map(wind_speed=6.0)
    provider = TravelCostProvider()
    drone = _drone()
    a, b = Point(40.0, 40.0), Point(300.0, 150.0)

    forward = provider.leg(model, drone, a, b)
    backward = provider.leg(model, drone, b, a)

    assert forward.reachable and backward.reachable
    assert forward.time != pytest.approx(backward.time)

    provider.leg(model, drone, a, b)
    stats = provider.stats()
    assert stats.hits == 1
    assert stats.misses == 2

    model.wind.speed = 9.0
    provider.leg(model, drone, a, b)
    assert provider.stats().misses == 3


def test_equivalent_aircraft_share_a_profile_key() -> None:
    first, second = _drone(), _drone()
    second.id = "D-02"

    assert drone_profile_key(first) == drone_profile_key(second)
    second.max_speed = 25.0
    assert drone_profile_key(first) != drone_profile_key(second)


def test_environment_revision_changes_with_terrain_and_wind() -> None:
    model = _map()
    before = environment_revision(model)

    model.wind = WindModel(direction_to_deg=0.0, speed=5.0, enabled=True)

    assert environment_revision(model) != before
