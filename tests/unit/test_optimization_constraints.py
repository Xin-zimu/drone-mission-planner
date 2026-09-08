"""OPT-04 (plan §11.2): dependencies, energy and return as hard constraints.

The independent verifier ``evaluate_routes`` must reject a plan on its own
evidence — with numbers in the reason — and a solver answer must never be
accepted without that check.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import pytest

from drone_mission_planner.planning import optimization as opt
from drone_mission_planner.planning.optimization import (
    SUPPORTED_CONSTRAINTS,
    AssignmentOutcome,
    AssignmentProblem,
    ORToolsAssignmentSolver,
    RouteAssignment,
    SolverSettings,
    SolverStatus,
    SolverTask,
    SolverVehicle,
    evaluate_routes,
    greedy_routes,
    solve_with_baseline,
    verify_outcome,
)

BIG = opt._BIG

# node 0 = start base, 1 = T-01, 2 = T-02, 3 = end base
MATRIX = [
    [0, 10, 20, 0],
    [10, 0, 5, 10],
    [20, 5, 0, 20],
    [0, 10, 20, 0],
]


def _problem(
    *,
    tasks: tuple[SolverTask, ...] | None = None,
    vehicle: SolverVehicle | None = None,
    matrix: Sequence[Sequence[int]] = MATRIX,
) -> AssignmentProblem:
    return AssignmentProblem(
        node_count=len(matrix),
        travel_ticks=matrix,
        vehicles=(vehicle or SolverVehicle("D-01", 0, 3),),
        tasks=tasks
        or (
            SolverTask("T-01", 1, service_seconds=0.0),
            SolverTask("T-02", 2, service_seconds=0.0),
        ),
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=1.0),
    )


def _reasons(outcome: AssignmentOutcome) -> str:
    return " | ".join(reason for _, reason in outcome.unassigned)


def test_supported_constraints_cover_the_opt04_families() -> None:
    assert len(SUPPORTED_CONSTRAINTS) == 6
    assert {"predecessor_ids", "energy", "return_energy"} <= set(SUPPORTED_CONSTRAINTS)


def test_dependency_order_violation_is_rejected_with_times() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1, service_seconds=0.0),
            SolverTask("T-02", 2, service_seconds=0.0, predecessor_ids=("T-01",)),
        )
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-02", "T-01"))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "before predecessor T-01 finishes" in _reasons(outcome)


def test_dependency_satisfied_order_is_accepted() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1, service_seconds=0.0),
            SolverTask("T-02", 2, service_seconds=0.0, predecessor_ids=("T-01",)),
        )
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert outcome.status is SolverStatus.FEASIBLE


def test_minimum_lag_is_enforced() -> None:
    tasks = (
        SolverTask("T-01", 1, service_seconds=0.0),
        SolverTask("T-02", 2, service_seconds=0.0, predecessor_ids=("T-01",), min_lag_seconds=10.0),
    )
    problem = _problem(tasks=tasks)

    late = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert late.status is SolverStatus.INFEASIBLE
    assert "plus lag 10.0 s" in _reasons(late)

    relaxed = _problem(
        tasks=(
            SolverTask("T-01", 1, service_seconds=0.0),
            SolverTask(
                "T-02", 2, service_seconds=0.0, predecessor_ids=("T-01",), min_lag_seconds=5.0
            ),
        )
    )
    assert evaluate_routes(relaxed, [("D-01", ("T-01", "T-02"))]).status is SolverStatus.FEASIBLE


def test_dependency_cycle_is_reported_with_its_path() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1, predecessor_ids=("T-02",)),
            SolverTask("T-02", 2, predecessor_ids=("T-01",)),
        )
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "T-01 -> T-02 -> T-01" in outcome.message


def test_missing_predecessor_is_rejected() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1, predecessor_ids=("T-99",)),
            SolverTask("T-02", 2),
        )
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "predecessor T-99 is not served" in _reasons(outcome)


def test_route_energy_counts_travel_service_and_return() -> None:
    vehicle = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, energy_per_tick=1.0
    )
    problem = _problem(
        vehicle=vehicle,
        tasks=(
            SolverTask("T-01", 1, energy_demand=5.0),
            SolverTask("T-02", 2, energy_demand=5.0),
        ),
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert outcome.status is SolverStatus.FEASIBLE
    # 0->1 (10) + 1->2 (5) + 2->3 (20) travel, plus 5 + 5 service.
    assert dict(outcome.energy)["D-01"] == pytest.approx(45.0)


def test_energy_over_budget_is_rejected_with_numbers() -> None:
    vehicle = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, reserve_energy=80.0, energy_per_tick=1.0
    )
    problem = _problem(
        vehicle=vehicle,
        tasks=(
            SolverTask("T-01", 1, energy_demand=5.0),
            SolverTask("T-02", 2, energy_demand=5.0),
        ),
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01", "T-02"))])

    assert outcome.status is SolverStatus.INFEASIBLE
    reason = _reasons(outcome)
    assert "consumes 45.0" in reason
    assert "usable 20.0" in reason


def test_airborne_wait_energy_is_charged() -> None:
    vehicle = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, energy_per_tick=1.0,
        hover_energy_per_second=0.5,
    )
    no_wait = _problem(vehicle=vehicle)
    waiting = _problem(
        vehicle=vehicle,
        tasks=(
            SolverTask("T-01", 1, earliest_start=20.0),
            SolverTask("T-02", 2),
        ),
    )

    base = dict(evaluate_routes(no_wait, [("D-01", ("T-01", "T-02"))]).energy)["D-01"]
    held = dict(evaluate_routes(waiting, [("D-01", ("T-01", "T-02"))]).energy)["D-01"]

    # 10 s of hovering in place at 0.5 energy/s.
    assert held - base == pytest.approx(5.0)


def test_return_leg_alone_can_exhaust_the_reserve() -> None:
    vehicle = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, reserve_energy=85.0, energy_per_tick=1.0
    )
    problem = _problem(vehicle=vehicle, tasks=(SolverTask("T-01", 1),))

    outcome = evaluate_routes(problem, [("D-01", ("T-01",))])

    # 0->1 (10) + return 1->3 (10) = 20 against 15 usable.
    assert outcome.status is SolverStatus.INFEASIBLE
    assert "consumes 20.0" in _reasons(outcome)


def test_unreachable_return_leg_is_named() -> None:
    matrix = [row[:] for row in MATRIX]
    for row in matrix:
        row[3] = BIG
    problem = _problem(matrix=matrix, tasks=(SolverTask("T-01", 1),))

    outcome = evaluate_routes(problem, [("D-01", ("T-01",))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "no safe return leg" in _reasons(outcome)


def test_energy_constraint_is_off_when_capacity_is_zero() -> None:
    problem = _problem(tasks=(SolverTask("T-01", 1, energy_demand=10_000.0),))

    outcome = evaluate_routes(problem, [("D-01", ("T-01",))])

    assert outcome.status is SolverStatus.FEASIBLE
    assert outcome.energy == ()


def test_optional_mission_is_skipped_with_a_reason() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1),
            SolverTask("T-02", 2, mandatory=False, deadline=1.0, optional_penalty=3.0),
        )
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01",))])

    assert outcome.status is SolverStatus.FEASIBLE
    assert any(task_id == "T-02" for task_id, _ in outcome.skipped)
    assert "penalty 3.0" in dict(outcome.skipped)["T-02"]


def test_mandatory_mission_is_never_silently_dropped() -> None:
    problem = _problem(
        tasks=(SolverTask("T-01", 1), SolverTask("T-02", 2, deadline=1.0))
    )

    outcome = evaluate_routes(problem, [("D-01", ("T-01",))])

    assert outcome.status is SolverStatus.INFEASIBLE
    assert "mandatory mission is not assigned" in _reasons(outcome)


def test_solver_returns_a_dependency_feasible_plan() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1),
            SolverTask("T-02", 2, predecessor_ids=("T-01",)),
        )
    )

    outcome = ORToolsAssignmentSolver().solve(problem)

    assert outcome.status is SolverStatus.FEASIBLE
    route = next(route for route in outcome.routes if route.drone_id == "D-01")
    assert route.task_ids == ("T-01", "T-02")
    assert verify_outcome(problem, outcome).status is SolverStatus.FEASIBLE


def test_solver_respects_the_energy_budget() -> None:
    vehicle = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, reserve_energy=60.0, energy_per_tick=1.0
    )
    problem = _problem(
        vehicle=vehicle,
        tasks=(
            SolverTask("T-01", 1, energy_demand=5.0),
            SolverTask("T-02", 2, energy_demand=5.0),
        ),
    )

    outcome = ORToolsAssignmentSolver().solve(problem)

    # 45 units are needed but only 40 are usable, so the missions cannot both fly.
    assert outcome.status is SolverStatus.INFEASIBLE
    assert "no feasible assignment" in outcome.message or outcome.unassigned


class _RogueSolver:
    """Pretends to be feasible while returning a dependency-violating route."""

    supported_constraints: tuple[str, ...] = SUPPORTED_CONSTRAINTS

    def solve(
        self, problem: AssignmentProblem, *, cancel: Callable[[], bool] | None = None
    ) -> AssignmentOutcome:
        route = RouteAssignment(
            "D-01", ("T-02", "T-01"), (20.0, 25.0), (20.0, 25.0), (20.0, 25.0)
        )
        return AssignmentOutcome(SolverStatus.FEASIBLE, routes=(route,), makespan=35.0)


def test_solver_answer_is_reverified_by_the_independent_checker() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1),
            SolverTask("T-02", 2, predecessor_ids=("T-01",)),
        )
    )

    checked = verify_outcome(problem, _RogueSolver().solve(problem))

    assert checked.status is SolverStatus.INFEASIBLE
    assert "independent verifier" in checked.message

    chosen, kept_baseline = solve_with_baseline(problem, solver=_RogueSolver())
    assert chosen.status is SolverStatus.FEASIBLE
    assert kept_baseline
    assert chosen.routes[0].task_ids == ("T-01", "T-02")


def test_greedy_baseline_places_predecessors_first() -> None:
    problem = _problem(
        tasks=(
            SolverTask("T-01", 1, predecessor_ids=()),
            SolverTask("T-02", 2, predecessor_ids=("T-01",)),
        )
    )

    routes = greedy_routes(problem)

    assert routes[0][1] == ("T-01", "T-02")
    assert evaluate_routes(problem, routes).status is SolverStatus.FEASIBLE
