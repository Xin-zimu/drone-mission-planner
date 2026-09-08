"""Global multi-vehicle assignment (plan §11).

The module separates four things the plan insists on keeping apart:

* ``AssignmentProblem`` — a solver-independent, tick-based view of the instance.
* ``AssignmentSolver`` — the interface, with an explicit outcome status so a
  timeout is never reported as "infeasible" and an unsupported hard constraint
  is reported instead of ignored.
* ``evaluate_routes`` — an independent schedule/feasibility checker used for the
  greedy baseline and for re-checking solver output.
* ``ORToolsAssignmentSolver`` — a heterogeneous VRPTW over the cost matrix.

Hard constraints modelled here (plan §11.2, OPT-04): capacity, time windows,
global predecessor dependencies with minimum lag, whole-route energy including
service, airborne waiting and the return leg, and a landing reserve. The
independent checker ``evaluate_routes`` enforces them itself, so a solver
answer can never pass on the strength of the solver's own assumptions.

Waiting energy is the one place a static model cannot be exact: the solver uses
a conservative bound (it ignores wait energy) and the exact ledger is re-applied
by ``evaluate_routes`` afterwards (plan §11.8). Coverage blocks are OPT-05.
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


SUPPORTED_CONSTRAINTS = (
    "capacity",
    "time_window",
    "heterogeneous_fleet",
    "predecessor_ids",
    "energy",
    "return_energy",
)

ENERGY_SCALE = 1000  # abstract energy units -> integer for the solver


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
    energy_capacity: float = 0.0
    reserve_energy: float = 0.0
    energy_per_tick: float = 0.0
    hover_energy_per_second: float = 0.0

    @property
    def usable_energy(self) -> float:
        """Energy the route may spend and still land with the reserve intact."""

        return self.energy_capacity - self.reserve_energy

    @property
    def energy_limited(self) -> bool:
        return self.energy_capacity > 0


@dataclass(frozen=True, slots=True)
class SolverTask:
    task_id: str
    node: int
    demand: float = 0.0
    earliest_start: float | None = None
    deadline: float | None = None
    service_seconds: float = 0.0
    mandatory: bool = True
    predecessor_ids: tuple[str, ...] = ()
    min_lag_seconds: float = 0.0
    energy_demand: float = 0.0
    optional_penalty: float = 0.0
    exit_node: int | None = None


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
        for task in self.tasks:
            if task.exit_node is not None and not 0 <= task.exit_node < self.node_count:
                raise ValueError("task exit_node must index the travel matrix")


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
    skipped: tuple[tuple[str, str], ...] = ()
    energy: tuple[tuple[str, float], ...] = ()

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

    Enforces every hard constraint this module claims to model — capacity, time
    windows, global predecessor dependencies, whole-route energy including
    service, airborne waiting and the return leg, and the landing reserve —
    without consulting a solver, so it cannot inherit the solver's assumptions
    (plan §11.3).

    ``require_all=True`` additionally requires every *mandatory* mission to be
    assigned; optional missions left out land in ``skipped`` with a reason
    instead of failing the plan (plan §11.2). With ``require_all=False`` only the
    given routes are checked, which the incremental greedy baseline needs: a
    predecessor outside those routes counts as not yet scheduled, not as a
    violation.
    """

    tick = problem.settings.tick_seconds
    by_id = {task.task_id: task for task in problem.tasks}
    vehicles = {vehicle.drone_id: vehicle for vehicle in problem.vehicles}
    results: list[RouteAssignment] = []
    problems: list[tuple[str, str]] = []
    skipped: list[tuple[str, str]] = []
    energies: list[tuple[str, float]] = []
    assigned: set[str] = set()
    starts: dict[str, float] = {}
    finishes: dict[str, float] = {}
    makespan = 0.0

    cycle = _dependency_cycle(problem.tasks)
    if cycle:
        return AssignmentOutcome(
            SolverStatus.INFEASIBLE,
            unassigned=((" -> ".join(cycle), "predecessor dependencies form a cycle"),),
            message="predecessor dependencies form a cycle: " + " -> ".join(cycle),
        )

    for drone_id, task_ids in routes:
        vehicle = vehicles.get(drone_id)
        if vehicle is None:
            problems.append((drone_id, "unknown vehicle"))
            continue
        node = vehicle.start_node
        clock = vehicle.available_from
        load = 0.0
        energy = 0.0
        arrivals: list[float] = []
        start_times: list[float] = []
        finish_times: list[float] = []
        for task_id in task_ids:
            task = by_id.get(task_id)
            if task is None:
                problems.append((task_id, "unknown task"))
                break
            leg_ticks = problem.travel_ticks[node][task.node]
            if leg_ticks >= _BIG:
                problems.append((task_id, f"no safe leg from node {node} to node {task.node}"))
                break
            travel = leg_ticks * tick
            arrival = clock + travel
            earliest = task.earliest_start if task.earliest_start is not None else 0.0
            start = max(arrival, earliest)
            finish = start + task.service_seconds
            wait = start - arrival
            if task.deadline is not None and finish > task.deadline + 1e-9:
                problems.append(
                    (task_id, f"finishes at {finish:.1f} s after deadline {task.deadline:.1f} s")
                )
            load += task.demand
            if load > vehicle.capacity + 1e-9:
                problems.append((task_id, f"load {load:.1f} exceeds capacity {vehicle.capacity:.1f}"))
            energy += travel * vehicle.energy_per_tick
            energy += wait * vehicle.hover_energy_per_second
            energy += task.energy_demand
            arrivals.append(arrival)
            start_times.append(start)
            finish_times.append(finish)
            assigned.add(task_id)
            starts[task_id] = start
            finishes[task_id] = finish
            clock = finish
            node = task.exit_node if task.exit_node is not None else task.node
        return_ticks = problem.travel_ticks[node][vehicle.end_node]
        if return_ticks >= _BIG:
            problems.append(
                (drone_id, f"no safe return leg from node {node} to base {vehicle.end_node}")
            )
        else:
            energy += return_ticks * tick * vehicle.energy_per_tick
            clock += return_ticks * tick
        if finish_times:
            makespan = max(makespan, clock)
        if vehicle.energy_limited:
            energies.append((drone_id, energy))
            if energy > vehicle.usable_energy + 1e-9:
                problems.append(
                    (
                        drone_id,
                        f"route consumes {energy:.1f} energy against "
                        f"usable {vehicle.usable_energy:.1f} "
                        f"(capacity {vehicle.energy_capacity:.1f}, "
                        f"reserve {vehicle.reserve_energy:.1f})",
                    )
                )
        results.append(RouteAssignment(drone_id, tuple(task_ids), tuple(arrivals),
                                       tuple(start_times), tuple(finish_times)))

    for task in problem.tasks:
        if task.task_id not in starts:
            continue
        for predecessor in task.predecessor_ids:
            if predecessor not in finishes:
                if require_all:
                    problems.append(
                        (task.task_id, f"predecessor {predecessor} is not served")
                    )
                continue
            floor = finishes[predecessor] + task.min_lag_seconds
            if starts[task.task_id] + 1e-9 < floor:
                lag = f" plus lag {task.min_lag_seconds:.1f} s" if task.min_lag_seconds else ""
                problems.append(
                    (
                        task.task_id,
                        f"{task.task_id} starts at {starts[task.task_id]:.1f} s before "
                        f"predecessor {predecessor} finishes at {finishes[predecessor]:.1f} s"
                        f"{lag}",
                    )
                )

    for task in problem.tasks:
        if task.task_id in assigned:
            continue
        if any(item[0] == task.task_id for item in problems):
            continue
        if task.mandatory:
            if require_all:
                problems.append((task.task_id, "mandatory mission is not assigned to any vehicle"))
        else:
            skipped.append(
                (task.task_id, f"optional mission left unassigned (penalty {task.optional_penalty:.1f})")
            )
    return AssignmentOutcome(
        status=SolverStatus.FEASIBLE if not problems else SolverStatus.INFEASIBLE,
        routes=tuple(results),
        unassigned=tuple(problems),
        makespan=makespan,
        skipped=tuple(skipped),
        energy=tuple(energies),
    )


def greedy_routes(problem: AssignmentProblem) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Deterministic baseline: dependency order, earliest deadline, least loaded.

    Tasks are placed so a predecessor is always scheduled before its successors
    (plan §11.2); ties fall back to deadline and id so the baseline is stable.
    """

    ordered = _topological_tasks(problem)
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
        cycle = _dependency_cycle(problem.tasks)
        if cycle:
            return AssignmentOutcome(
                SolverStatus.INFEASIBLE,
                unassigned=((" -> ".join(cycle), "predecessor dependencies form a cycle"),),
                message="predecessor dependencies form a cycle: " + " -> ".join(cycle),
            )
        no_return = _vehicles_without_return(problem)
        if no_return:
            return AssignmentOutcome(
                SolverStatus.INFEASIBLE,
                unassigned=no_return,
                message="some aircraft have no safe return leg to the end base",
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
        by_id = {task.task_id: task for task in problem.tasks}
        windows = _propagate_dependencies(problem)
        active = {
            task.node: routing.ActiveVar(manager.NodeToIndex(task.node))
            for task in problem.tasks
        }

        def transit(from_index: int, to_index: int) -> int:
            from_node = int(manager.IndexToNode(from_index))
            to_node = int(manager.IndexToNode(to_index))
            departure = _departure_node(problem, from_node)
            return travel[departure][to_node] + service_ticks.get(from_node, 0)

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
            window = windows.get(task.task_id)
            earliest_seconds = window[0] if window else (task.earliest_start or 0.0)
            latest_seconds = window[1] if window else task.deadline
            earliest = ticks_up(earliest_seconds, tick)
            latest = (
                ticks_down(latest_seconds, tick) - service_ticks[task.node]
                if latest_seconds is not None
                else horizon
            )
            if latest < earliest:
                if task.mandatory:
                    return AssignmentOutcome(
                        SolverStatus.INFEASIBLE,
                        unassigned=((task.task_id, "time window is empty after rounding"),),
                        message=f"{task.task_id} cannot be served inside its window",
                    )
                # An optional mission whose window closed is simply not flown.
                routing.AddDisjunction([index], max(1, round(task.optional_penalty * ENERGY_SCALE)))
                routing.solver().Add(active[task.node] == 0)
                continue
            time_dimension.CumulVar(index).SetRange(earliest, latest)
            if not task.mandatory:
                routing.AddDisjunction([index], max(1, round(task.optional_penalty * ENERGY_SCALE)))
        for task in problem.tasks:
            if not task.predecessor_ids:
                continue
            successor_active = active[task.node]
            successor_index = manager.NodeToIndex(task.node)
            for predecessor_id in task.predecessor_ids:
                predecessor = by_id.get(predecessor_id)
                if predecessor is None:
                    return AssignmentOutcome(
                        SolverStatus.INFEASIBLE,
                        unassigned=((task.task_id, f"predecessor {predecessor_id} does not exist"),),
                        message=f"{task.task_id} references a missing predecessor",
                    )
                # A successor may only be flown if its predecessor is flown
                # (plan §11.2). When both are mandatory the ordering is exact;
                # otherwise the time windows were already propagated and the
                # independent verifier re-checks the real timing (plan §11.3).
                routing.solver().Add(successor_active <= active[predecessor.node])
                if task.mandatory and predecessor.mandatory:
                    routing.solver().Add(
                        time_dimension.CumulVar(successor_index)
                        >= time_dimension.CumulVar(manager.NodeToIndex(predecessor.node))
                        + service_ticks[predecessor.node]
                        + ticks_up(task.min_lag_seconds, tick)
                    )
        if any(vehicle.energy_limited for vehicle in problem.vehicles):
            routing.AddDimensionWithVehicleTransits(
                [_energy_callback(routing, manager, problem, vehicle) for vehicle in problem.vehicles],
                0,
                _ENERGY_BIG,
                True,
                "Energy",
            )
            energy_dimension = routing.GetDimensionOrDie("Energy")
            for vehicle_index, vehicle in enumerate(problem.vehicles):
                if vehicle.energy_limited:
                    # The end cumul is travel + service energy including the
                    # return leg; capping it at usable energy is the reserve.
                    energy_dimension.CumulVar(routing.End(vehicle_index)).SetMax(
                        _energy_units(vehicle.usable_energy)
                    )
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
        assigned = {task for route in routes for task in route.task_ids}
        return AssignmentOutcome(
            SolverStatus.FEASIBLE,
            routes=routes,
            unassigned=_unassigned(problem, assigned),
            makespan=makespan,
            objective=float(solution.ObjectiveValue()),
            message="feasible (optimality not proven within the time budget)",
            skipped=_skipped(problem, assigned),
        )


def verify_outcome(
    problem: AssignmentProblem, outcome: AssignmentOutcome
) -> AssignmentOutcome:
    """Re-check a solver answer with the independent evaluator (plan §11.3).

    A candidate that fails the exact energy, dependency or window check is
    reported as infeasible with the verifier's own reasons — the solver's
    ``feasible`` status never overrides the verifier.
    """

    if outcome.status is not SolverStatus.FEASIBLE:
        return outcome
    checked = evaluate_routes(
        problem, [(route.drone_id, route.task_ids) for route in outcome.routes]
    )
    if checked.status is SolverStatus.FEASIBLE:
        return outcome
    return AssignmentOutcome(
        SolverStatus.INFEASIBLE,
        routes=checked.routes,
        unassigned=checked.unassigned,
        skipped=checked.skipped,
        energy=checked.energy,
        makespan=checked.makespan,
        message="solver answer rejected by the independent verifier",
    )


def solve_with_baseline(
    problem: AssignmentProblem,
    *,
    solver: AssignmentSolver | None = None,
    cancel: Callable[[], bool] | None = None,
) -> tuple[AssignmentOutcome, bool]:
    """Run both the greedy baseline and the optimiser; keep the better one.

    Returns ``(chosen_outcome, kept_baseline)``. The optimiser candidate is first
    re-checked by ``evaluate_routes``, so only a verified answer can win, and it
    wins only when its makespan is strictly better — a worse or equal candidate
    never replaces the baseline (plan §11.9).
    """

    active = solver or ORToolsAssignmentSolver()
    baseline = evaluate_routes(problem, greedy_routes(problem))
    optimised = verify_outcome(problem, active.solve(problem, cancel=cancel))
    if (
        optimised.status is SolverStatus.FEASIBLE
        and baseline.status is SolverStatus.FEASIBLE
        and optimised.makespan < baseline.makespan - 1e-9
    ):
        return optimised, False
    return baseline, True


def _dependency_cycle(tasks: Sequence[SolverTask]) -> tuple[str, ...]:
    """Return a concrete predecessor cycle path, or an empty tuple."""

    known = {task.task_id for task in tasks}
    edges = {
        task.task_id: [item for item in task.predecessor_ids if item in known]
        for task in tasks
    }
    state: dict[str, int] = {}
    path: list[str] = []

    def visit(node: str) -> tuple[str, ...]:
        if state.get(node) == 1:
            return tuple([*path[path.index(node):], node])
        if state.get(node) == 2:
            return ()
        state[node] = 1
        path.append(node)
        for predecessor in edges.get(node, ()):
            found = visit(predecessor)
            if found:
                return found
        path.pop()
        state[node] = 2
        return ()

    for task_id in edges:
        found = visit(task_id)
        if found:
            return found
    return ()


def _topological_tasks(problem: AssignmentProblem) -> tuple[SolverTask, ...]:
    """Predecessors first; ties broken by deadline then id (stable baseline)."""

    by_id = {task.task_id: task for task in problem.tasks}

    def key(task_id: str) -> tuple[float, str]:
        task = by_id[task_id]
        return (task.deadline if task.deadline is not None else inf, task_id)

    pending = {
        task.task_id: {item for item in task.predecessor_ids if item in by_id}
        for task in problem.tasks
    }
    ordered: list[SolverTask] = []
    while pending:
        ready = [task_id for task_id, deps in pending.items() if not deps]
        if not ready:
            # A cycle: keep a deterministic order and let evaluation report it.
            ordered.extend(by_id[task_id] for task_id in sorted(pending, key=key))
            break
        chosen = min(ready, key=key)
        ordered.append(by_id[chosen])
        del pending[chosen]
        for deps in pending.values():
            deps.discard(chosen)
    return tuple(ordered)


def _propagate_dependencies(
    problem: AssignmentProblem,
) -> dict[str, tuple[float, float | None]]:
    """Sound earliest/latest start windows after propagating predecessor links.

    Forward propagation raises a successor's earliest start to the predecessor's
    earliest finish plus the lag; backward propagation lowers a predecessor's
    latest start by the same amount. Both are *necessary* conditions only, so
    they never cut off a feasible plan; the exact timing is enforced afterwards
    by ``evaluate_routes`` (plan §11.3).
    """

    by_id = {task.task_id: task for task in problem.tasks}
    order = _topological_tasks(problem)
    earliest = {task.task_id: (task.earliest_start or 0.0) for task in problem.tasks}
    for task in order:
        for predecessor_id in task.predecessor_ids:
            predecessor = by_id.get(predecessor_id)
            if predecessor is None:
                continue
            floor = earliest[predecessor_id] + predecessor.service_seconds + task.min_lag_seconds
            earliest[task.task_id] = max(earliest[task.task_id], floor)
    latest: dict[str, float | None] = {task.task_id: task.deadline for task in problem.tasks}
    for task in reversed(order):
        ceiling = latest[task.task_id]
        if ceiling is None:
            continue
        for predecessor_id in task.predecessor_ids:
            predecessor = by_id.get(predecessor_id)
            if predecessor is None:
                continue
            bound = ceiling - predecessor.service_seconds - task.min_lag_seconds
            current = latest[predecessor_id]
            latest[predecessor_id] = bound if current is None else min(current, bound)
    return {task_id: (earliest[task_id], latest[task_id]) for task_id in earliest}


def _departure_node(problem: AssignmentProblem, node: int) -> int:
    """Node an aircraft actually leaves after serving the task at ``node``.

    A coverage block enters at one point and leaves at another, so the following
    leg starts at the exit (plan §11.7); plain point tasks leave where they are.
    """

    for task in problem.tasks:
        if task.node == node:
            return task.exit_node if task.exit_node is not None else node
    return node


def _energy_units(value: float) -> int:
    """Integer energy units, rounded up so the solver never under-counts."""

    return max(0, ceil(value * ENERGY_SCALE - 1e-9))


def _energy_callback(
    routing: object,
    manager: object,
    problem: AssignmentProblem,
    vehicle: SolverVehicle,
) -> int:
    """Per-vehicle transit energy: travel leg plus the service demand at the origin.

    Airborne waiting is deliberately absent — the solver uses this conservative
    bound and ``evaluate_routes`` re-applies the exact ledger (plan §11.8).
    """

    def energy(from_index: int, to_index: int) -> int:
        from_node = int(manager.IndexToNode(from_index))  # type: ignore[attr-defined]
        to_node = int(manager.IndexToNode(to_index))  # type: ignore[attr-defined]
        departure = _departure_node(problem, from_node)
        leg = problem.travel_ticks[departure][to_node] * vehicle.energy_per_tick
        task = _task_at_node(problem, from_node)
        demand = task.energy_demand if task is not None else 0.0
        return _energy_units(leg + demand)

    return routing.RegisterTransitCallback(energy)  # type: ignore[attr-defined,no-any-return]


def _vehicles_without_return(problem: AssignmentProblem) -> tuple[tuple[str, str], ...]:
    blocked: list[tuple[str, str]] = []
    for vehicle in problem.vehicles:
        if problem.travel_ticks[vehicle.start_node][vehicle.end_node] >= _BIG:
            blocked.append((vehicle.drone_id, f"no safe return leg to base {vehicle.end_node}"))
    return tuple(blocked)


def _mandatory_unreachable(problem: AssignmentProblem) -> tuple[tuple[str, str], ...]:
    blocked: list[tuple[str, str]] = []
    for task in problem.tasks:
        if not task.mandatory:
            continue
        if all(problem.travel_ticks[vehicle.start_node][task.node] >= _BIG for vehicle in problem.vehicles):
            blocked.append((task.task_id, "no vehicle can reach this mandatory mission"))
    return tuple(blocked)


_BIG = 10**12
_ENERGY_BIG = 10**12


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
    """Mandatory missions the solver failed to place — never silently dropped."""

    return tuple(
        (task.task_id, "mandatory mission left unassigned by the solver")
        for task in problem.tasks
        if task.mandatory and task.task_id not in assigned
    )


def _skipped(
    problem: AssignmentProblem, assigned: set[str]
) -> tuple[tuple[str, str], ...]:
    """Optional missions the solver chose not to fly, with their penalty (plan §11.2)."""

    return tuple(
        (task.task_id, f"optional mission left unassigned (penalty {task.optional_penalty:.1f})")
        for task in problem.tasks
        if not task.mandatory and task.task_id not in assigned
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
