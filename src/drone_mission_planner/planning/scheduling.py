"""Deterministic mission scheduling: time propagation and dependency validation.

Implements plan §10.2-§10.5 as pure Python (no PySide6, no numpy): a task's
service window is derived from the previous departure on the same aircraft, its
own ``earliest_start``, and the earliest time its predecessors allow. The module
is the single source of truth for those numbers so planning, simulation and
reports cannot each invent their own rule.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from math import inf, isfinite

from drone_mission_planner.domain.enums import DeadlinePolicy, TaskStatus
from drone_mission_planner.domain.models import MissionTask

TIME_TOLERANCE = 1e-9

#: ``wait_reasons`` values (plan §10.2): a wait is attributed to the binding bound.
WAIT_TIME_WINDOW = "time_window"
WAIT_PREDECESSOR = "predecessor"
WAIT_CONFLICT = "conflict"
WAIT_RESOURCE = "resource"

STATUS_SCHEDULED = "scheduled"
STATUS_COMPLETED = "completed"
STATUS_BLOCKED = "blocked"
STATUS_UNSCHEDULED = "unscheduled"


class ScheduleError(ValueError):
    """A structural or numeric problem that prevents schedule evaluation."""


@dataclass(frozen=True, slots=True)
class DependencyReport:
    """Structural findings of a dependency graph (plan §10.5)."""

    order: tuple[str, ...] = ()
    missing: tuple[tuple[str, str], ...] = ()
    self_references: tuple[str, ...] = ()
    duplicates: tuple[tuple[str, str], ...] = ()
    cycle: tuple[str, ...] = ()
    blocked: tuple[tuple[str, str], ...] = ()

    @property
    def has_structural_errors(self) -> bool:
        return bool(self.missing or self.self_references or self.duplicates or self.cycle)

    def issues(self) -> tuple[str, ...]:
        messages: list[str] = []
        for task_id, predecessor_id in self.missing:
            messages.append(f"{task_id} depends on missing mission {predecessor_id}")
        for task_id in self.self_references:
            messages.append(f"{task_id} depends on itself")
        for task_id, predecessor_id in self.duplicates:
            messages.append(f"{task_id} lists dependency {predecessor_id} more than once")
        if self.cycle:
            messages.append("dependency cycle: " + " → ".join(self.cycle))
        for task_id, predecessor_id in self.blocked:
            messages.append(f"{task_id} is blocked by {predecessor_id}")
        return tuple(messages)


@dataclass(frozen=True, slots=True)
class ScheduledTask:
    """One mission's derived timeline (plan §10.2 field table)."""

    task_id: str
    drone_id: str | None
    status: str
    travel_seconds: float
    arrival_time: float
    start_time: float
    wait_seconds: float
    wait_reasons: tuple[str, ...]
    finish_time: float
    departure_time: float
    deadline: float | None
    deadline_policy: DeadlinePolicy
    lateness_seconds: float
    deadline_ok: bool
    blocked_by: tuple[str, ...] = ()

    @property
    def scheduled(self) -> bool:
        return self.status in {STATUS_SCHEDULED, STATUS_COMPLETED}


@dataclass(frozen=True, slots=True)
class ScheduleResult:
    """Evaluated schedule with explicit feasibility and violation reporting."""

    entries: tuple[ScheduledTask, ...] = ()
    feasible: bool = True
    hard_violations: tuple[str, ...] = ()
    blocked_tasks: tuple[str, ...] = ()
    makespan: float = 0.0
    issues: tuple[str, ...] = ()

    def entry(self, task_id: str) -> ScheduledTask | None:
        return next((entry for entry in self.entries if entry.task_id == task_id), None)

    def scheduled(self) -> tuple[ScheduledTask, ...]:
        return tuple(entry for entry in self.entries if entry.scheduled)

    def total_wait(self) -> float:
        return sum(entry.wait_seconds for entry in self.entries)

    def late_tasks(self) -> tuple[ScheduledTask, ...]:
        return tuple(entry for entry in self.entries if entry.lateness_seconds > TIME_TOLERANCE)


TravelProvider = Mapping[str, float] | Callable[[MissionTask], float]


def validate_dependencies(tasks: Sequence[MissionTask]) -> DependencyReport:
    """Check dependency references, duplicates, cycles and blocked predecessors."""

    by_id = {task.id: task for task in tasks}
    missing: list[tuple[str, str]] = []
    self_references: list[str] = []
    duplicates: list[tuple[str, str]] = []
    blocked: list[tuple[str, str]] = []
    edges: dict[str, list[str]] = {}

    for task in tasks:
        seen: set[str] = set()
        valid: list[str] = []
        for predecessor_id in task.predecessor_ids:
            if predecessor_id == task.id:
                if task.id not in self_references:
                    self_references.append(task.id)
                continue
            if predecessor_id in seen:
                duplicates.append((task.id, predecessor_id))
                continue
            seen.add(predecessor_id)
            predecessor = by_id.get(predecessor_id)
            if predecessor is None:
                missing.append((task.id, predecessor_id))
                continue
            if predecessor.status in {TaskStatus.CANCELLED, TaskStatus.FAILED}:
                blocked.append((task.id, predecessor_id))
            valid.append(predecessor_id)
        edges[task.id] = valid

    order, cycle = _topological_order(tasks, edges)
    return DependencyReport(
        order=order,
        missing=tuple(missing),
        self_references=tuple(self_references),
        duplicates=tuple(duplicates),
        cycle=cycle,
        blocked=tuple(blocked),
    )


def resolve_start(
    arrival: float,
    earliest_start: float | None,
    predecessor_floor: float | None = None,
    *,
    tolerance: float = TIME_TOLERANCE,
) -> tuple[float, float, tuple[str, ...]]:
    """Return ``(start, wait, reasons)`` for an arrival against both lower bounds.

    The single implementation of ``start = max(arrival, earliest_start,
    predecessor_floor)`` used by the evaluator and by assignment scoring, so the
    two can never disagree about when a mission may begin.
    """

    earliest = earliest_start if earliest_start is not None else -inf
    floor = predecessor_floor if predecessor_floor is not None else -inf
    start = max(arrival, earliest, floor)
    wait = max(0.0, start - arrival)
    reasons: list[str] = []
    if earliest > arrival + tolerance and start <= earliest + tolerance:
        reasons.append(WAIT_TIME_WINDOW)
    if floor > arrival + tolerance and start <= floor + tolerance:
        reasons.append(WAIT_PREDECESSOR)
    return start, wait, tuple(reasons)


def evaluate_schedule(
    tasks: Sequence[MissionTask],
    *,
    travel_seconds: TravelProvider,
    assignments: Mapping[str, str] | None = None,
    completed: Mapping[str, float] | None = None,
    post_service_hold: Mapping[str, float] | None = None,
    start_time: float = 0.0,
    tolerance: float = TIME_TOLERANCE,
) -> ScheduleResult:
    """Propagate the timeline of ``tasks`` (plan §10.3).

    ``travel_seconds`` supplies the leg time into each mission; ``assignments``
    maps task ids to drone ids (falling back to ``MissionTask.assigned_drone_id``);
    ``completed`` maps finished missions to their actual finish time so a replan
    inherits real history instead of re-deriving it.
    """

    if not isfinite(start_time) or start_time < 0:
        raise ScheduleError("schedule start time must be finite and non-negative")
    if not isfinite(tolerance) or tolerance < 0:
        raise ScheduleError("schedule tolerance must be finite and non-negative")

    completed = dict(completed or {})
    post_service_hold = dict(post_service_hold or {})
    report = validate_dependencies(tasks)
    by_id = {task.id: task for task in tasks}
    cycle_members = set(report.cycle)

    blocked_reasons: dict[str, list[str]] = {}
    blocked_ids: set[str] = set()
    for task_id, predecessor_id in report.missing:
        blocked_reasons.setdefault(task_id, []).append(f"missing mission {predecessor_id}")
    for task_id in report.self_references:
        blocked_reasons.setdefault(task_id, []).append("depends on itself")
    for task_id, predecessor_id in report.duplicates:
        blocked_reasons.setdefault(task_id, []).append(
            f"duplicate dependency {predecessor_id}"
        )
    for task_id, predecessor_id in report.blocked:
        blocked_reasons.setdefault(task_id, []).append(
            f"predecessor {predecessor_id} is cancelled or failed"
        )
    for task_id in report.cycle:
        blocked_reasons.setdefault(task_id, []).append(
            "dependency cycle: " + " → ".join(report.cycle)
        )

    order = list(report.order) + [task_id for task_id in cycle_members if task_id not in report.order]
    clock: dict[str, float] = {}
    entries: list[ScheduledTask] = []
    hard_violations: list[str] = []
    finished: dict[str, float] = {}

    for task_id in order:
        task = by_id[task_id]
        policy = task.deadline_policy
        hold = post_service_hold.get(task_id, 0.0)
        if not isfinite(hold) or hold < 0:
            raise ScheduleError(f"{task_id} post-service hold must be finite and non-negative")

        if task_id in completed:
            actual_finish = completed[task_id]
            if not isfinite(actual_finish):
                raise ScheduleError(f"{task_id} completed finish time must be finite")
            finished[task_id] = actual_finish
            entries.append(
                ScheduledTask(
                    task_id=task_id,
                    drone_id=assignments.get(task_id) if assignments else task.assigned_drone_id,
                    status=STATUS_COMPLETED,
                    travel_seconds=0.0,
                    arrival_time=actual_finish - task.execution_duration,
                    start_time=actual_finish - task.execution_duration,
                    wait_seconds=0.0,
                    wait_reasons=(),
                    finish_time=actual_finish,
                    departure_time=actual_finish + hold,
                    deadline=task.deadline,
                    deadline_policy=policy,
                    lateness_seconds=0.0,
                    deadline_ok=True,
                )
            )
            continue

        reasons = list(blocked_reasons.get(task_id, ()))
        for predecessor_id in task.predecessor_ids:
            if predecessor_id in blocked_ids:
                reasons.append(f"predecessor {predecessor_id} is blocked or unscheduled")
        if reasons:
            blocked_reasons.setdefault(task_id, []).extend(
                reason for reason in reasons if reason not in blocked_reasons.get(task_id, ())
            )
            blocked_ids.add(task_id)
            entries.append(_blocked_entry(task, policy, tuple(dict.fromkeys(reasons))))
            continue

        drone_id = (assignments.get(task_id) if assignments else None) or task.assigned_drone_id
        if drone_id is None:
            blocked_ids.add(task_id)
            entries.append(
                _blocked_entry(task, policy, ("no drone assigned",), status=STATUS_UNSCHEDULED)
            )
            continue

        travel = _travel_time(task, travel_seconds)
        departure_floor = clock.get(drone_id, start_time)
        arrival = departure_floor + travel
        predecessor_floor = -inf
        for predecessor_id in task.predecessor_ids:
            predecessor_finish = finished.get(predecessor_id)
            if predecessor_finish is None:
                predecessor_entry = next(
                    (entry for entry in entries if entry.task_id == predecessor_id), None
                )
                if predecessor_entry is None:
                    continue
                predecessor_finish = predecessor_entry.departure_time
            predecessor_floor = max(
                predecessor_floor, predecessor_finish + task.min_lag_seconds
            )
        start, wait, wait_reasons = resolve_start(
            arrival,
            task.earliest_start,
            None if predecessor_floor == -inf else predecessor_floor,
            tolerance=tolerance,
        )
        finish = start + task.execution_duration
        departure = finish + hold
        lateness = 0.0
        deadline_ok = True
        if task.deadline is not None:
            lateness = max(0.0, finish - task.deadline)
            deadline_ok = finish <= task.deadline + tolerance
            if not deadline_ok and policy is DeadlinePolicy.HARD:
                hard_violations.append(
                    f"{task_id} finishes at {finish:.6f} s, {lateness:.6f} s after its "
                    f"hard deadline {task.deadline:.6f} s"
                )
        clock[drone_id] = departure
        finished[task_id] = finish
        entries.append(
            ScheduledTask(
                task_id=task_id,
                drone_id=drone_id,
                status=STATUS_SCHEDULED,
                travel_seconds=travel,
                arrival_time=arrival,
                start_time=start,
                wait_seconds=wait,
                wait_reasons=tuple(wait_reasons),
                finish_time=finish,
                departure_time=departure,
                deadline=task.deadline,
                deadline_policy=policy,
                lateness_seconds=lateness,
                deadline_ok=deadline_ok,
            )
        )

    blocked_tasks = tuple(
        entry.task_id for entry in entries if entry.status in {STATUS_BLOCKED, STATUS_UNSCHEDULED}
    )
    makespan = max((entry.departure_time for entry in entries if entry.scheduled), default=start_time)
    feasible = not hard_violations and not blocked_tasks and not report.has_structural_errors
    return ScheduleResult(
        entries=tuple(entries),
        feasible=feasible,
        hard_violations=tuple(hard_violations),
        blocked_tasks=blocked_tasks,
        makespan=makespan,
        issues=report.issues(),
    )


def _blocked_entry(
    task: MissionTask,
    policy: DeadlinePolicy,
    reasons: tuple[str, ...],
    *,
    status: str = STATUS_BLOCKED,
) -> ScheduledTask:
    return ScheduledTask(
        task_id=task.id,
        drone_id=task.assigned_drone_id,
        status=status,
        travel_seconds=0.0,
        arrival_time=0.0,
        start_time=0.0,
        wait_seconds=0.0,
        wait_reasons=(),
        finish_time=0.0,
        departure_time=0.0,
        deadline=task.deadline,
        deadline_policy=policy,
        lateness_seconds=0.0,
        deadline_ok=True,
        blocked_by=reasons,
    )


def _travel_time(task: MissionTask, provider: TravelProvider) -> float:
    travel = (
        float(provider(task))
        if callable(provider)
        else float(provider.get(task.id, 0.0))
    )
    if not isfinite(travel) or travel < 0:
        raise ScheduleError(f"{task.id} travel time must be finite and non-negative")
    return travel


def _topological_order(
    tasks: Sequence[MissionTask],
    edges: Mapping[str, Sequence[str]],
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Kahn order with a deterministic tie-break, plus one concrete cycle if any."""

    by_id = {task.id: task for task in tasks}
    indegree = {task.id: 0 for task in tasks}
    successors: dict[str, list[str]] = {task.id: [] for task in tasks}
    for task_id, predecessors in edges.items():
        for predecessor_id in predecessors:
            successors[predecessor_id].append(task_id)
            indegree[task_id] += 1

    def rank(task_id: str) -> tuple[int, float, str]:
        task = by_id[task_id]
        deadline = task.deadline if task.deadline is not None else inf
        return (-task.priority, deadline, task_id)

    ready = sorted((task_id for task_id, degree in indegree.items() if degree == 0), key=rank)
    order: list[str] = []
    while ready:
        task_id = ready.pop(0)
        order.append(task_id)
        for successor_id in successors[task_id]:
            indegree[successor_id] -= 1
            if indegree[successor_id] == 0:
                ready.append(successor_id)
        ready.sort(key=rank)
    if len(order) == len(tasks):
        return tuple(order), ()
    cycle = _find_cycle([task_id for task_id in indegree if task_id not in set(order)], edges)
    return tuple(order), cycle


def _find_cycle(candidates: Sequence[str], edges: Mapping[str, Sequence[str]]) -> tuple[str, ...]:
    """Return one concrete cycle path ``A → C → B → A`` over the remaining nodes."""

    remaining = set(candidates)
    for start in candidates:
        path: list[str] = []
        current = start
        while current in remaining:
            if current in path:
                loop = [*path[path.index(current) :], current]
                return tuple(loop)
            path.append(current)
            successors = [item for item in edges.get(current, ()) if item in remaining]
            if not successors:
                break
            current = successors[0]
    return (*tuple(candidates), candidates[0]) if candidates else ()
