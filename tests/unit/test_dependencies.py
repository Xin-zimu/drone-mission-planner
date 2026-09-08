"""F2 dependency validation: missing, self, duplicate and cyclic references."""

from __future__ import annotations

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MissionTask
from drone_mission_planner.planning.scheduling import (
    STATUS_BLOCKED,
    evaluate_schedule,
    validate_dependencies,
)


def _task(task_id: str, *predecessors: str, priority: int = 5) -> MissionTask:
    return MissionTask(
        task_id,
        task_id,
        Point(0.0, 0.0),
        priority=priority,
        predecessor_ids=list(predecessors),
    )


def test_missing_predecessor_is_reported() -> None:
    report = validate_dependencies([_task("T-01", "T-99")])

    assert report.missing == (("T-01", "T-99"),)
    assert report.has_structural_errors
    assert any("missing mission T-99" in issue for issue in report.issues())


def test_self_reference_is_reported() -> None:
    report = validate_dependencies([_task("T-01", "T-01")])

    assert report.self_references == ("T-01",)
    assert any("depends on itself" in issue for issue in report.issues())


def test_duplicate_dependency_is_reported() -> None:
    report = validate_dependencies([_task("T-01"), _task("T-02", "T-01", "T-01")])

    assert report.duplicates == (("T-02", "T-01"),)
    assert any("more than once" in issue for issue in report.issues())


def test_cycle_is_reported_as_a_concrete_path() -> None:
    tasks = [_task("T-A", "T-C"), _task("T-B", "T-A"), _task("T-C", "T-B")]

    report = validate_dependencies(tasks)

    assert report.cycle
    assert report.cycle[0] == report.cycle[-1]
    assert set(report.cycle) == {"T-A", "T-B", "T-C"}
    assert any("dependency cycle" in issue for issue in report.issues())
    assert " → ".join(report.cycle) in report.issues()[-1]


def test_cycle_members_are_blocked_without_hanging() -> None:
    tasks = [_task("T-A", "T-C"), _task("T-B", "T-A"), _task("T-C", "T-B")]

    result = evaluate_schedule(
        tasks,
        travel_seconds={task.id: 1.0 for task in tasks},
        assignments={task.id: "D-01" for task in tasks},
    )

    assert set(result.blocked_tasks) == {"T-A", "T-B", "T-C"}
    for entry in result.entries:
        assert entry.status == STATUS_BLOCKED
        assert any("cycle" in reason for reason in entry.blocked_by)
    assert not result.feasible


def test_cancelled_and_failed_predecessors_are_reported_as_blocking() -> None:
    cancelled = _task("T-01")
    cancelled.status = TaskStatus.CANCELLED
    failed = _task("T-02")
    failed.status = TaskStatus.FAILED

    report = validate_dependencies([cancelled, failed, _task("T-03", "T-01", "T-02")])

    assert ("T-03", "T-01") in report.blocked
    assert ("T-03", "T-02") in report.blocked


def test_topological_order_respects_dependencies_and_priority() -> None:
    tasks = [
        _task("T-03", "T-01"),
        _task("T-01", priority=9),
        _task("T-02", priority=1),
    ]

    report = validate_dependencies(tasks)

    assert report.cycle == ()
    assert report.order.index("T-01") < report.order.index("T-03")
    # No dependency between T-01 and T-02, so higher priority goes first.
    assert report.order.index("T-01") < report.order.index("T-02")


def test_order_is_deterministic_for_equal_priorities() -> None:
    tasks = [_task("T-02"), _task("T-01"), _task("T-03")]

    assert validate_dependencies(tasks).order == ("T-01", "T-02", "T-03")
    assert validate_dependencies(list(reversed(tasks))).order == ("T-01", "T-02", "T-03")


def test_successor_of_a_blocked_task_is_also_blocked() -> None:
    tasks = [_task("T-01", "T-99"), _task("T-02", "T-01")]

    result = evaluate_schedule(
        tasks,
        travel_seconds={task.id: 1.0 for task in tasks},
        assignments={task.id: "D-01" for task in tasks},
    )

    successor = result.entry("T-02")
    assert successor is not None
    assert successor.status == STATUS_BLOCKED
    assert any("T-01" in reason for reason in successor.blocked_by)
