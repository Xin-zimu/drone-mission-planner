"""F2: greedy assignment scores cumulative arrival/start/finish (plan §10.3-§10.4)."""

from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import DeadlinePolicy
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner


def _model(tasks: list[MissionTask], *, speed: float = 10.0) -> MapModel:
    model = MapModel(width=400, height=200, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10.0, 10.0),
            "B-01",
            max_speed=speed,
            air_speed=speed,
            battery_capacity=1000.0,
            remaining_battery=1000.0,
        )
    )
    model.tasks.extend(tasks)
    return model


def test_earliest_start_delays_the_aircraft_clock() -> None:
    first = MissionTask(
        "T-01", "A", Point(110.0, 10.0), earliest_start=100.0, execution_duration=5.0
    )
    second = MissionTask("T-02", "B", Point(210.0, 10.0), execution_duration=5.0)

    result = GreedyAssignmentPlanner().assign(_model([first, second]))

    by_id = {decision.task_id: decision for decision in result.decisions}
    assert by_id["T-01"].arrival_time == pytest.approx(10.0)
    assert by_id["T-01"].start_time == pytest.approx(100.0)
    assert by_id["T-01"].finish_time == pytest.approx(105.0)
    # B departs after A finishes, so its arrival carries A's whole window.
    assert by_id["T-02"].arrival_time == pytest.approx(115.0)
    assert by_id["T-02"].start_time == pytest.approx(115.0)
    assert by_id["T-02"].finish_time == pytest.approx(120.0)


def test_lateness_uses_the_cumulative_finish_not_the_single_leg() -> None:
    task = MissionTask(
        "T-01",
        "Windowed",
        Point(110.0, 10.0),
        earliest_start=30.0,
        execution_duration=5.0,
        deadline=12.0,
        deadline_policy=DeadlinePolicy.LEGACY_SOFT,
    )

    result = GreedyAssignmentPlanner().assign(_model([task]))

    decision = result.decisions[0]
    # The 10 s leg alone is inside the 12 s deadline — the pre-F2 rule saw no risk.
    assert decision.route.estimated_time <= 12.0
    # The 30 s time window pushes the finish to 35 s, ten-plus seconds late.
    assert decision.arrival_time == pytest.approx(10.0)
    assert decision.start_time == pytest.approx(30.0)
    assert decision.finish_time == pytest.approx(35.0)
    assert decision.lateness_seconds == pytest.approx(23.0)


def test_hard_deadline_rejects_every_candidate() -> None:
    task = MissionTask(
        "T-01",
        "Windowed",
        Point(110.0, 10.0),
        earliest_start=30.0,
        execution_duration=5.0,
        deadline=12.0,
        deadline_policy=DeadlinePolicy.HARD,
    )

    result = GreedyAssignmentPlanner().assign(_model([task]))

    assert result.decisions == []
    assert [failure.task_id for failure in result.failures] == ["T-01"]
    assert "hard deadline" in result.failures[0].summary()


def test_soft_deadline_assigns_and_reports_the_lateness() -> None:
    task = MissionTask(
        "T-01",
        "Windowed",
        Point(110.0, 10.0),
        earliest_start=30.0,
        execution_duration=5.0,
        deadline=12.0,
        deadline_policy=DeadlinePolicy.SOFT,
    )

    result = GreedyAssignmentPlanner().assign(_model([task]))

    assert result.assigned_count == 1
    assert result.failures == []
    assert result.decisions[0].lateness_seconds > 0.0
    assert result.schedule is not None
    assert result.schedule.feasible
    assert result.schedule.late_tasks()


def test_predecessor_is_scheduled_before_its_successor() -> None:
    first = MissionTask("T-01", "A", Point(110.0, 10.0), execution_duration=20.0)
    second = MissionTask(
        "T-02",
        "B",
        Point(210.0, 10.0),
        execution_duration=5.0,
        predecessor_ids=["T-01"],
        min_lag_seconds=7.0,
    )

    result = GreedyAssignmentPlanner().assign(_model([first, second]))

    order = [decision.task_id for decision in result.decisions]
    assert order == ["T-01", "T-02"]
    by_id = {decision.task_id: decision for decision in result.decisions}
    assert by_id["T-02"].start_time >= by_id["T-01"].finish_time + 7.0 - 1e-9


def test_blocked_predecessor_fails_the_successor_with_a_reason() -> None:
    first = MissionTask("T-01", "A", Point(110.0, 10.0))
    second = MissionTask(
        "T-02", "B", Point(210.0, 10.0), predecessor_ids=["T-99"]
    )

    result = GreedyAssignmentPlanner().assign(_model([first, second]))

    assert [failure.task_id for failure in result.failures] == ["T-02"]
    assert "missing mission T-99" in result.failures[0].summary()


def test_schedule_result_matches_the_decisions() -> None:
    first = MissionTask(
        "T-01", "A", Point(110.0, 10.0), earliest_start=25.0, execution_duration=5.0
    )
    second = MissionTask("T-02", "B", Point(210.0, 10.0), execution_duration=5.0)

    result = GreedyAssignmentPlanner().assign(_model([first, second]))

    assert result.schedule is not None
    for decision in result.decisions:
        entry = result.schedule.entry(decision.task_id)
        assert entry is not None
        assert entry.start_time == pytest.approx(decision.start_time)
        assert entry.finish_time == pytest.approx(decision.finish_time)
        assert entry.wait_seconds == pytest.approx(decision.wait_seconds)
        assert entry.drone_id == decision.drone_id
