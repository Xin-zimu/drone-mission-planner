"""Global multi-vehicle assignment (plan §11).

The module separates four things the plan insists on keeping apart:

* ``AssignmentProblem`` — a solver-independent, tick-based view of the instance.
* ``AssignmentSolver`` — the interface, with an explicit outcome status so a
  timeout is never reported as "infeasible" and an unsupported hard constraint
  is reported instead of ignored.
* ``evaluate_routes`` — an independent schedule/feasibility checker used for the
  greedy baseline and for re-checking solver output.
* ``ORToolsAssignmentSolver`` — a heterogeneous VRPTW over the cost matrix.

Dependencies, energy ledgers and coverage blocks are *not* modelled here yet
(plan OPT-04/05); a caller that needs them declares them in ``requires`` and
receives ``unsupported_constraint`` rather than a silently wrong answer.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from math import ceil, floor, inf
from time import perf_counter
from typing import Protocol

try:  # OR-Tools is optional: its absence degrades to an explicit status.
    from ortools.constraint_solver import (  # type: ignore[import-untyped]
        pywrapcp,
        routing_enums_pb2,
    )

    ORTOOLS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised by monkeypatching the flag
    pywrapcp = None
    routing_enums_pb2 = None
    ORTOOLS_AVAILABLE = False


class SolverStatus(StrEnum):
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    UNSUPPORTED_CONSTRAINT = "unsupported_constraint"


SUPPORTED_CONSTRAINTS = ("capacity", "time_window", "heterogeneous_fleet")


@dataclass(frozen=True, slots=True)
class SolverSettings:
    time_limit_seconds: float = 5.0
    tick_seconds: float = 0.1
    max_repair_rounds: int = 3

    def __post_init__(self) -> None:
        if self.time_limit_seconds <= 0:
            raise ValueError("time limit must be positive")
        if self.tick_seconds <= 0:
            raise ValueError("tick must be positive")
        if self.max_repair_rounds < 0:
            raise ValueError("repair rounds cannot be negative")


@dataclass(frozen=True, slots=True)
class SolverVehicle:
    drone_id: str
    start_node: int
    end_node: int
    capacity: float = 0.0
    available_from: float = 0.0
    available_until: float | None = None


@dataclass(frozen=True, slots=True)
class SolverTask:
    task_id: str
    node: int
    demand: float = 0.0
    earliest_start: float | None = None
    deadline: float | None = None
    service_seconds: float = 0.0
    mandatory: bool = True


@dataclass(frozen=True, slots=True)
class AssignmentProblem:
    """Solver-independent instance; travel times are directed integer ticks."""

    node_count: int
    travel_ticks: Sequence[Sequence[int]]
    vehicles: tuple[SolverVehicle, ...]
    tasks: tuple[SolverTask, ...]
    settings: SolverSettings = field(default_factory=SolverSettings)
    requires: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.node_count <= 0:
            raise ValueError("node_count must be positive")
        if len(self.travel_ticks) != self.node_count:
            raise ValueError("travel matrix must have node_count rows")
        for row in self.travel_ticks:
            if len(row) != self.node_count:
                raise ValueError("travel matrix must be square")
        if not self.vehicles:
            raise ValueError("at least one vehicle is required")
        if not self.tasks:
            raise ValueError("at least one task is required")


@dataclass(frozen=True, slots=True)
class RouteAssignment:
    drone_id: str
    task_ids: tuple[str, ...]
    arrival_times: tuple[float, ...]
    start_times: tuple[float, ...]
    finish_times: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class AssignmentOutcome:
    status: SolverStatus
    routes: tuple[RouteAssignment, ...] = ()
    unassigned: tuple[tuple[str, str], ...] = ()
    makespan: float = 0.0
    objective: float = 0.0
    unsupported: tuple[str, ...] = ()
    message: str = ""

    @property
    def feasible(self) -> bool:
        return self.status is SolverStatus.FEASIBLE


class AssignmentSolver(Protocol):
    """Interface every solver (greedy, OR-Tools, future CP-SAT) implements."""

    supported_constraints: tuple[str, ...]

    def solve(
        self,
        problem: AssignmentProblem,
        *,
        cancel: Callable[[], bool] | None = None,
    ) -> AssignmentOutcome: ...


def ticks_up(seconds: float, tick: float) -> int:
    """Round a duration up to whole ticks so travel is never under-estimated."""

    return ceil(seconds / tick - 1e-9)


def ticks_down(seconds: float, tick: float) -> int:
    """Round a deadline down to whole ticks so it is never relaxed."""

    return floor(seconds / tick + 1e-9)


def evaluate_routes(
    problem: AssignmentProblem,
    routes: Sequence[tuple[str, Sequence[str]]],
    *,
    require_all: bool = True,
) -> AssignmentOutcome:
    """Independent feasibility/schedule check of explicit routes.

    Used for the greedy baseline and to re-verify solver output; it never
    consults the solver, so it cannot inherit the solver's own assumptions.
    ``require_all=False`` checks only the given routes, which the incremental
    greedy baseline needs while it is still building the full assignment.
    """

    tick = problem.settings.tick_seconds
    by_id = {task.task_id: task for task in problem.tasks}
    vehicles = {vehicle.drone_id: vehicle for vehicle in problem.vehicles}
    assigned: set[str] = set()
    results: list[RouteAssignment] = []
    problems: list[tuple[str, str]] = []
    makespan = 0.0
    for drone_id, task_ids in routes:
        vehicle = vehicles.get(drone_id)
        if vehicle is None:
            problems.append((drone_id, "unknown vehicle"))
            continue
        node = vehicle.start_node
        clock = vehicle.available_from
        load = 0.0
        arrivals: list[float] = []
        starts: list[float] = []
        finishes: list[float] = []
        for task_id in task_ids:
            task = by_id.get(task_id)
            if task is None:
                problems.append((task_id, "unknown task"))
                break
            travel = problem.travel_ticks[node][task.node] * tick
            arrival = clock + travel
            earliest = task.earliest_start if task.earliest_start is not None else 0.0
            start = max(arrival, earliest)
            finish = start + task.service_seconds
            if task.deadline is not None and finish > task.deadline + 1e-9:
                problems.append(
                    (task_id, f"finishes at {finish:.1f} s after deadline {task.deadline:.1f} s")
                )
            load += task.demand
            if load > vehicle.capacity + 1e-9:
                problems.append((task_id, f"load {load:.1f} exceeds capacity {vehicle.capacity:.1f}"))
            arrivals.append(arrival)
            starts.append(start)
            finishes.append(finish)
            assigned.add(task_id)
            clock = finish
            node = task.node
        if finishes:
            clock += problem.travel_ticks[node][vehicle.end_node] * tick
            makespan = max(makespan, clock)
        results.append(RouteAssignment(drone_id, tuple(task_ids), tuple(arrivals),
                                       tuple(starts), tuple(finishes)))
    for task in problem.tasks:
        if task.task_id in assigned:
            continue
        if any(item[0] == task.task_id for item in problems):
            continue
        if require_all:
            problems.append((task.task_id, "not assigned to any vehicle"))
    return AssignmentOutcome(
        status=SolverStatus.FEASIBLE if not problems else SolverStatus.INFEASIBLE,
        routes=tuple(results),
        unassigned=tuple(problems),
        makespan=makespan,
    )


def greedy_routes(problem: AssignmentProblem) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Deterministic baseline: earliest-deadline task onto the least-loaded vehicle."""

    ordered = sorted(
        problem.tasks,
        key=lambda task: (task.deadline if task.deadline is not None else inf, task.task_id),
    )
    loads: dict[str, list[str]] = {vehicle.drone_id: [] for vehicle in problem.vehicles}
    makespans = {vehicle.drone_id: 0.0 for vehicle in problem.vehicles}
    for task in ordered:
        best: tuple[float, str] | None = None
        for vehicle in problem.vehicles:
            candidate = (*loads[vehicle.drone_id], task.task_id)
            outcome = evaluate_routes(
                problem, [(vehicle.drone_id, candidate)], require_all=False
            )
            if outcome.status is not SolverStatus.FEASIBLE:
                continue
            key = (outcome.makespan, vehicle.drone_id)
            if best is None or key < best:
                best = key
        if best is None:
            continue
        drone_id = best[1]
        loads[drone_id].append(task.task_id)
        makespans[drone_id] = best[0]
    return tuple(
        (vehicle.drone_id, tuple(loads[vehicle.drone_id])) for vehicle in problem.vehicles
    )


class ORToolsAssignmentSolver:
    """Heterogeneous VRPTW over the directed cost matrix (plan §11.2/§11.5)."""

    supported_constraints = SUPPORTED_CONSTRAINTS

    def solve(
        self,
        problem: AssignmentProblem,
        *,
        cancel: Callable[[], bool] | None = None,
    ) -> AssignmentOutcome:
        if cancel is not None and cancel():
            return AssignmentOutcome(SolverStatus.CANCELLED, message="cancelled before solve")
        missing = tuple(
            name for name in problem.requires if name not in self.supported_constraints
        )
        if missing:
            return AssignmentOutcome(
                SolverStatus.UNSUPPORTED_CONSTRAINT,
                unsupported=missing,
                message="the solver cannot model these hard constraints: " + ", ".join(missing),
            )
        if not ORTOOLS_AVAILABLE:
            return AssignmentOutcome(
                SolverStatus.UNSUPPORTED_CONSTRAINT,
                unsupported=("ortools",),
                message="OR-Tools is not installed; the greedy baseline remains available",
            )
        unreachable = _mandatory_unreachable(problem)
        if unreachable:
            return AssignmentOutcome(
                SolverStatus.INFEASIBLE,
                unassigned=unreachable,
                message="mandatory missions have no reachable vehicle",
            )
        settings = problem.settings
        tick = settings.tick_seconds
        starts = [vehicle.start_node for vehicle in problem.vehicles]
        ends = [vehicle.end_node for vehicle in problem.vehicles]
        manager = pywrapcp.RoutingIndexManager(problem.node_count, len(problem.vehicles), starts, ends)
        routing = pywrapcp.RoutingModel(manager)
        service_ticks = {
            task.node: ticks_up(task.service_seconds, tick) for task in problem.tasks
        }
        travel = problem.travel_ticks

        def transit(from_index: int, to_index: int) -> int:
            from_node = int(manager.IndexToNode(from_index))
            to_node = int(manager.IndexToNode(to_index))
            return travel[from_node][to_node] + service_ticks.get(from_node, 0)

        transit_index = routing.RegisterTransitCallback(transit)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_index)
        horizon = _horizon(problem)
        routing.AddDimension(transit_index, horizon, horizon, True, "Time")
        time_dimension = routing.GetDimensionOrDie("Time")

        def demand_callback(from_index: int) -> int:
            task = _task_at_node(problem, manager.IndexToNode(from_index))
            return round(task.demand) if task is not None else 0

        demand_index = routing.RegisterUnaryTransitCallback(demand_callback)
        capacities = [round(vehicle.capacity) for vehicle in problem.vehicles]
        routing.AddDimensionWithVehicleCapacity(demand_index, 0, capacities, True, "Load")

        for task in problem.tasks:
            index = manager.NodeToIndex(task.node)
            earliest = ticks_up(task.earliest_start or 0.0, tick)
            latest = (
                ticks_down(task.deadline, tick) - service_ticks[task.node]
                if task.deadline is not None
                else horizon
            )
            if latest < earliest:
                return AssignmentOutcome(
                    SolverStatus.INFEASIBLE,
                    unassigned=((task.task_id, "time window is empty after rounding"),),
                    message=f"{task.task_id} cannot be served inside its window",
                )
            time_dimension.CumulVar(index).SetRange(earliest, latest)
            if not task.mandatory:
                routing.AddDisjunction([index], 1)
        for vehicle_index, vehicle in enumerate(problem.vehicles):
            start_index = routing.Start(vehicle_index)
            end_index = routing.End(vehicle_index)
            time_dimension.CumulVar(start_index).SetRange(
                ticks_up(vehicle.available_from, tick), horizon
            )
            time_dimension.CumulVar(end_index).SetRange(0, horizon)
            if vehicle.available_until is not None:
                time_dimension.CumulVar(end_index).SetMax(
                    ticks_down(vehicle.available_until, tick)
                )
        time_dimension.SetGlobalSpanCostCoefficient(1000)

        parameters = pywrapcp.DefaultRoutingSearchParameters()
        parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        parameters.time_limit.FromMilliseconds(int(settings.time_limit_seconds * 1000))
        started = perf_counter()
        solution = routing.SolveWithParameters(parameters)
        elapsed = perf_counter() - started
        if solution is None:
            if cancel is not None and cancel():
                return AssignmentOutcome(SolverStatus.CANCELLED, message="cancelled during solve")
            if elapsed >= settings.time_limit_seconds * 0.99:
                return AssignmentOutcome(
                    SolverStatus.TIMEOUT,
                    message=(
                        f"no feasible solution within {settings.time_limit_seconds:.3f} s; "
                        "this is not a proof of infeasibility"
                    ),
                )
            return AssignmentOutcome(
                SolverStatus.INFEASIBLE,
                unassigned=_unassigned(problem, set()),
                message="no feasible assignment exists for this instance",
            )
        if cancel is not None and cancel():
            return AssignmentOutcome(SolverStatus.CANCELLED, message="cancelled after solve")
        routes, makespan = _extract_routes(problem, manager, routing, solution, time_dimension)
        return AssignmentOutcome(
            SolverStatus.FEASIBLE,
            routes=routes,
            unassigned=_unassigned(problem, {task for route in routes for task in route.task_ids}),
            makespan=makespan,
            objective=float(solution.ObjectiveValue()),
            message="feasible (optimality not proven within the time budget)",
        )


def solve_with_baseline(
    problem: AssignmentProblem,
    *,
    solver: AssignmentSolver | None = None,
    cancel: Callable[[], bool] | None = None,
) -> tuple[AssignmentOutcome, bool]:
    """Run both the greedy baseline and the optimiser; keep the better one.

    Returns ``(chosen_outcome, kept_baseline)``. The optimiser only wins when it
    is feasible and its makespan is strictly better, so a worse or equal
    candidate never replaces the baseline (plan §11.9).
    """

    active = solver or ORToolsAssignmentSolver()
    baseline = evaluate_routes(problem, greedy_routes(problem))
    optimised = active.solve(problem, cancel=cancel)
    if (
        optimised.status is SolverStatus.FEASIBLE
        and baseline.status is SolverStatus.FEASIBLE
        and optimised.makespan < baseline.makespan - 1e-9
    ):
        return optimised, False
    return baseline, True


def _mandatory_unreachable(problem: AssignmentProblem) -> tuple[tuple[str, str], ...]:
    blocked: list[tuple[str, str]] = []
    for task in problem.tasks:
        if not task.mandatory:
            continue
        if all(problem.travel_ticks[vehicle.start_node][task.node] >= _BIG for vehicle in problem.vehicles):
            blocked.append((task.task_id, "no vehicle can reach this mandatory mission"))
    return tuple(blocked)


_BIG = 10**12


def _horizon(problem: AssignmentProblem) -> int:
    """A safe upper bound on any useful arrival tick."""

    longest_leg = max((max(row) for row in problem.travel_ticks if row), default=0)
    span = longest_leg * (len(problem.tasks) + 1)
    limits = [
        ticks_down(task.deadline, problem.settings.tick_seconds)
        for task in problem.tasks
        if task.deadline is not None
    ]
    limits.extend(
        ticks_down(vehicle.available_until, problem.settings.tick_seconds)
        for vehicle in problem.vehicles
        if vehicle.available_until is not None
    )
    return max(span, max(limits, default=0), 1)


def _task_at_node(problem: AssignmentProblem, node: int) -> SolverTask | None:
    return next((task for task in problem.tasks if task.node == node), None)


def _unassigned(
    problem: AssignmentProblem, assigned: set[str]
) -> tuple[tuple[str, str], ...]:
    return tuple(
        (task.task_id, "left unassigned by the solver")
        for task in problem.tasks
        if task.task_id not in assigned
    )


def _extract_routes(
    problem: AssignmentProblem,
    manager: object,
    routing: object,
    solution: object,
    time_dimension: object,
) -> tuple[tuple[RouteAssignment, ...], float]:
    """Rebuild routes and the makespan **including each aircraft's return leg**."""

    tick = problem.settings.tick_seconds
    by_node = {task.node: task for task in problem.tasks}
    routes: list[RouteAssignment] = []
    makespan = 0.0
    for vehicle_index, vehicle in enumerate(problem.vehicles):
        index = routing.Start(vehicle_index)  # type: ignore[attr-defined]
        task_ids: list[str] = []
        arrivals: list[float] = []
        starts: list[float] = []
        finishes: list[float] = []
        while not routing.IsEnd(index):  # type: ignore[attr-defined]
            node = manager.IndexToNode(index)  # type: ignore[attr-defined]
            task = by_node.get(node)
            if task is not None:
                start = solution.Value(time_dimension.CumulVar(index)) * tick  # type: ignore[attr-defined]
                task_ids.append(task.task_id)
                arrivals.append(start)
                starts.append(start)
                finishes.append(start + task.service_seconds)
            index = solution.Value(routing.NextVar(index))  # type: ignore[attr-defined]
        end_time = solution.Value(time_dimension.CumulVar(index)) * tick  # type: ignore[attr-defined]
        makespan = max(makespan, end_time)
        routes.append(
            RouteAssignment(vehicle.drone_id, tuple(task_ids), tuple(arrivals),
                            tuple(starts), tuple(finishes))
        )
    return tuple(routes), makespan
