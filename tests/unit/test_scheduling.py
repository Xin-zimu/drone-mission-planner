"""F2 time propagation against the fixed §10.4 case and §10.10 boundaries."""

from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import DeadlinePolicy, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MissionTask
from drone_mission_planner.planning.scheduling import (
    STATUS_BLOCKED,
    WAIT_PREDECESSOR,
    WAIT_TIME_WINDOW,
    ScheduleError,
    evaluate_schedule,
)

TOLERANCE = 1e-6


def _ab_tasks(policy: DeadlinePolicy = DeadlinePolicy.HARD) -> list[MissionTask]:
    """The plan §10.4 table: A then B on the same aircraft."""

    return [
        MissionTask(
            "T-A",
            "A",
            Point(0.0, 0.0),
            earliest_start=60.0,
            deadline=110.0,
            execution_duration=40.0,
        ),
        MissionTask(
            "T-B",
            "B",
            Point(0.0, 0.0),
            earliest_start=120.0,
            deadline=140.0,
            execution_duration=30.0,
            predecessor_ids=["T-A"],
            deadline_policy=policy,
        ),
    ]


TRAVEL = {"T-A": 30.0, "T-B": 20.0}
SAME_DRONE = {"T-A": "D-01", "T-B": "D-01"}


def test_plan_section_10_4_case_with_a_hard_deadline() -> None:
    result = evaluate_schedule(
        _ab_tasks(DeadlinePolicy.HARD),
        travel_seconds=TRAVEL,
        assignments=SAME_DRONE,
    )

    a = result.entry("T-A")
    b = result.entry("T-B")
    assert a is not None and b is not None
    # A: arrives 30, waits 30, starts 60, finishes 100.
    assert a.arrival_time == pytest.approx(30.0)
    assert a.wait_seconds == pytest.approx(30.0)
    assert a.start_time == pytest.approx(60.0)
    assert a.finish_time == pytest.approx(100.0)
    assert a.departure_time == pytest.approx(100.0)
    assert a.deadline_ok
    # B: the same aircraft departs A at 100 s and needs 20 s, so it arrives at 120
    # and finishes at 150 — ten seconds past its deadline.
    assert b.arrival_time == pytest.approx(120.0)
    assert b.start_time == pytest.approx(120.0)
    assert b.finish_time == pytest.approx(150.0)
    assert b.lateness_seconds == pytest.approx(10.0)
    assert not b.deadline_ok
    assert not result.feasible
    assert len(result.hard_violations) == 1
    assert "T-B" in result.hard_violations[0]
    assert result.makespan == pytest.approx(150.0)


def test_plan_section_10_4_case_with_a_soft_deadline() -> None:
    result = evaluate_schedule(
        _ab_tasks(DeadlinePolicy.SOFT),
        travel_seconds=TRAVEL,
        assignments=SAME_DRONE,
    )

    b = result.entry("T-B")
    assert b is not None
    assert b.lateness_seconds == pytest.approx(10.0)
    assert not b.deadline_ok
    assert result.feasible
    assert result.hard_violations == ()
    assert result.late_tasks() == (b,)


def test_single_leg_travel_time_alone_would_wrongly_pass() -> None:
    """Plan §10.4: B's 20 s leg is inside the 140 s deadline, the schedule is not."""

    result = evaluate_schedule(
        _ab_tasks(DeadlinePolicy.HARD),
        travel_seconds=TRAVEL,
        assignments=SAME_DRONE,
    )

    b = result.entry("T-B")
    assert b is not None
    assert b.travel_seconds == pytest.approx(20.0)
    assert b.travel_seconds <= 140.0
    assert b.finish_time > 140.0
    assert not result.feasible


def test_wait_reason_is_the_binding_bound() -> None:
    # B on its own aircraft: it arrives at 20 s, the time window opens at 120 s.
    result = evaluate_schedule(
        _ab_tasks(DeadlinePolicy.SOFT),
        travel_seconds=TRAVEL,
        assignments={"T-A": "D-01", "T-B": "D-02"},
    )

    b = result.entry("T-B")
    assert b is not None
    assert b.arrival_time == pytest.approx(20.0)
    assert b.start_time == pytest.approx(120.0)
    assert b.wait_seconds == pytest.approx(100.0)
    assert b.wait_reasons == (WAIT_TIME_WINDOW,)


def test_predecessor_floor_can_be_the_binding_bound() -> None:
    tasks = _ab_tasks(DeadlinePolicy.SOFT)
    tasks[1].earliest_start = None

    result = evaluate_schedule(
        tasks,
        travel_seconds=TRAVEL,
        assignments={"T-A": "D-01", "T-B": "D-02"},
    )

    b = result.entry("T-B")
    assert b is not None
    assert b.start_time == pytest.approx(100.0)
    assert b.wait_reasons == (WAIT_PREDECESSOR,)


def test_completed_predecessor_uses_its_actual_finish_time() -> None:
    tasks = _ab_tasks(DeadlinePolicy.SOFT)
    tasks[1].earliest_start = None

    result = evaluate_schedule(
        tasks,
        travel_seconds=TRAVEL,
        assignments={"T-A": "D-01", "T-B": "D-02"},
        completed={"T-A": 95.0},
    )

    a = result.entry("T-A")
    b = result.entry("T-B")
    assert a is not None and b is not None
    assert a.status == "completed"
    assert a.finish_time == pytest.approx(95.0)
    assert b.start_time == pytest.approx(95.0)


def test_minimum_dependency_lag_delays_the_successor() -> None:
    tasks = _ab_tasks(DeadlinePolicy.SOFT)
    tasks[1].min_lag_seconds = 25.0

    result = evaluate_schedule(
        tasks,
        travel_seconds=TRAVEL,
        assignments={"T-A": "D-01", "T-B": "D-02"},
    )

    b = result.entry("T-B")
    assert b is not None
    assert b.start_time == pytest.approx(125.0)


def test_zero_duration_task_is_legal() -> None:
    task = MissionTask("T-01", "Ping", Point(0.0, 0.0), execution_duration=0.0)

    result = evaluate_schedule([task], travel_seconds={"T-01": 12.0}, assignments={"T-01": "D-01"})

    entry = result.entry("T-01")
    assert entry is not None
    assert entry.finish_time == pytest.approx(entry.start_time)
    assert result.feasible


def test_post_service_hold_delays_departure_only() -> None:
    task = MissionTask("T-01", "Hold", Point(0.0, 0.0), execution_duration=10.0)

    result = evaluate_schedule(
        [task],
        travel_seconds={"T-01": 5.0},
        assignments={"T-01": "D-01"},
        post_service_hold={"T-01": 7.5},
    )

    entry = result.entry("T-01")
    assert entry is not None
    assert entry.finish_time == pytest.approx(15.0)
    assert entry.departure_time == pytest.approx(22.5)


def test_unassigned_task_is_reported_not_silently_dropped() -> None:
    task = MissionTask("T-01", "No drone", Point(0.0, 0.0))

    result = evaluate_schedule([task], travel_seconds={"T-01": 1.0})

    entry = result.entry("T-01")
    assert entry is not None
    assert entry.status == "unscheduled"
    assert entry.blocked_by == ("no drone assigned",)
    assert result.blocked_tasks == ("T-01",)
    assert not result.feasible


@pytest.mark.parametrize(
    ("travel", "message"),
    [(-1.0, "travel time"), (float("nan"), "travel time"), (float("inf"), "travel time")],
)
def test_non_finite_or_negative_travel_time_is_rejected(travel: float, message: str) -> None:
    task = MissionTask("T-01", "Bad leg", Point(0.0, 0.0))

    with pytest.raises(ScheduleError, match=message):
        evaluate_schedule([task], travel_seconds={"T-01": travel}, assignments={"T-01": "D-01"})


@pytest.mark.parametrize("start_time", [-1.0, float("nan")])
def test_invalid_start_time_is_rejected(start_time: float) -> None:
    task = MissionTask("T-01", "Leg", Point(0.0, 0.0))

    with pytest.raises(ScheduleError, match="start time"):
        evaluate_schedule(
            [task],
            travel_seconds={"T-01": 1.0},
            assignments={"T-01": "D-01"},
            start_time=start_time,
        )


def test_evaluation_is_deterministic() -> None:
    first = evaluate_schedule(_ab_tasks(), travel_seconds=TRAVEL, assignments=SAME_DRONE)
    second = evaluate_schedule(_ab_tasks(), travel_seconds=TRAVEL, assignments=SAME_DRONE)

    assert first == second


def test_blocked_predecessor_blocks_its_successor() -> None:
    tasks = _ab_tasks(DeadlinePolicy.SOFT)
    tasks[0].status = TaskStatus.CANCELLED

    result = evaluate_schedule(tasks, travel_seconds=TRAVEL, assignments=SAME_DRONE)

    b = result.entry("T-B")
    assert b is not None
    assert b.status == STATUS_BLOCKED
    assert any("T-A" in reason for reason in b.blocked_by)
    assert result.blocked_tasks == ("T-B",)
    assert not result.feasible
