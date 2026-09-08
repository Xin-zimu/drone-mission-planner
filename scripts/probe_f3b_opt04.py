"""F3-b OPT-04 evidence probe: measure the constraint kernel after the change.

Prints the numbers the step predicted, so the convergence record cites measured
values instead of test-internal expectations. Read-only.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_mission_planner.planning import optimization as opt  # noqa: E402
from drone_mission_planner.planning.optimization import (  # noqa: E402
    AssignmentProblem,
    SolverSettings,
    SolverTask,
    SolverVehicle,
    evaluate_routes,
)

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
) -> AssignmentProblem:
    return AssignmentProblem(
        node_count=4,
        travel_ticks=MATRIX,
        vehicles=(vehicle or SolverVehicle("D-01", 0, 3),),
        tasks=tasks
        or (
            SolverTask("T-01", 1, service_seconds=0.0),
            SolverTask("T-02", 2, service_seconds=0.0),
        ),
        settings=SolverSettings(time_limit_seconds=1.0, tick_seconds=1.0),
    )


def main() -> None:
    print("supported_constraints_count", len(opt.SUPPORTED_CONSTRAINTS))
    print("supported_constraints", list(opt.SUPPORTED_CONSTRAINTS))

    dependency = _problem(
        tasks=(
            SolverTask("T-01", 1),
            SolverTask("T-02", 2, predecessor_ids=("T-01",)),
        )
    )
    violated = evaluate_routes(dependency, [("D-01", ("T-02", "T-01"))])
    print("dependency_status", violated.status.value)
    print("dependency_reason", dict(violated.unassigned).get("T-02"))

    metered = SolverVehicle(
        "D-01", 0, 3, energy_capacity=100.0, energy_per_tick=1.0
    )
    energy_problem = _problem(
        vehicle=metered,
        tasks=(
            SolverTask("T-01", 1, energy_demand=5.0),
            SolverTask("T-02", 2, energy_demand=5.0),
        ),
    )
    feasible = evaluate_routes(energy_problem, [("D-01", ("T-01", "T-02"))])
    print("feasible_route_energy", dict(feasible.energy).get("D-01"))

    tight = _problem(
        vehicle=SolverVehicle(
            "D-01", 0, 3, energy_capacity=100.0, reserve_energy=80.0, energy_per_tick=1.0
        ),
        tasks=(
            SolverTask("T-01", 1, energy_demand=5.0),
            SolverTask("T-02", 2, energy_demand=5.0),
        ),
    )
    over = evaluate_routes(tight, [("D-01", ("T-01", "T-02"))])
    print("energy_rejection_status", over.status.value)
    print("energy_rejection_reason", dict(over.unassigned).get("D-01"))

    hover = _problem(
        vehicle=SolverVehicle(
            "D-01", 0, 3, energy_capacity=100.0, energy_per_tick=1.0,
            hover_energy_per_second=0.5,
        )
    )
    waiting = _problem(
        vehicle=SolverVehicle(
            "D-01", 0, 3, energy_capacity=100.0, energy_per_tick=1.0,
            hover_energy_per_second=0.5,
        ),
        tasks=(SolverTask("T-01", 1, earliest_start=20.0), SolverTask("T-02", 2)),
    )
    base = dict(evaluate_routes(hover, [("D-01", ("T-01", "T-02"))]).energy)["D-01"]
    held = dict(evaluate_routes(waiting, [("D-01", ("T-01", "T-02"))]).energy)["D-01"]
    print("wait_energy_contribution", round(held - base, 3))

    reserve = _problem(
        vehicle=SolverVehicle(
            "D-01", 0, 3, energy_capacity=100.0, reserve_energy=85.0, energy_per_tick=1.0
        ),
        tasks=(SolverTask("T-01", 1),),
    )
    reserve_outcome = evaluate_routes(reserve, [("D-01", ("T-01",))])
    print("return_reserve_status", reserve_outcome.status.value)
    print("return_reserve_reason", dict(reserve_outcome.unassigned).get("D-01"))


if __name__ == "__main__":
    main()
