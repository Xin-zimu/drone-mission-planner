"""SCH-06: the simulation records actual mission times and emits timeline events."""

from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import DeadlinePolicy, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.events import EventType
from drone_mission_planner.simulation.task_timeline import STATE_BLOCKED, STATE_COMPLETED


def _drone(model: MapModel, target: Point, tasks: list[str]) -> Drone:
    drone = Drone(
        "D-01",
        "Alpha",
        Point(10.0, 10.0),
        "B-01",
        max_speed=10.0,
        air_speed=10.0,
        battery_capacity=1000.0,
        remaining_battery=1000.0,
    )
    drone.assigned_tasks = tasks
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(target.x, target.y, altitude=100.0),
    ]
    model.drones.append(drone)
    return drone


def _model(task: MissionTask, *, target: Point | None = None) -> MapModel:
    model = MapModel(width=200, height=100, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.tasks.append(task)
    _drone(model, target or task.position, [task.id])
    return model


def _events(engine: SimulationEngine, event_type: EventType, target: str) -> list[float]:
    return [
        record.processed_at
        for record in engine.event_manager.history
        if record.event.event_type == event_type and record.event.target_id == target
    ]


def test_arrival_and_service_are_recorded_in_order() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = SimulationEngine(_model(task), fixed_dt=0.05)

    engine.run_until_complete()

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert timeline.arrived_at is not None
    assert timeline.service_started_at == pytest.approx(timeline.arrived_at)
    assert timeline.service_finished_at == pytest.approx(timeline.arrived_at + 4.0, abs=0.06)
    assert timeline.business_status == STATE_COMPLETED
    arrivals = _events(engine, EventType.TASK_ARRIVED, "T-01")
    starts = _events(engine, EventType.TASK_SERVICE_STARTED, "T-01")
    finishes = _events(engine, EventType.TASK_SERVICE_FINISHED, "T-01")
    assert len(arrivals) == len(starts) == len(finishes) == 1
    assert arrivals[0] <= starts[0] <= finishes[0]


def test_time_window_makes_the_aircraft_wait() -> None:
    task = MissionTask(
        "T-01", "Visit", Point(110.0, 10.0), earliest_start=30.0, execution_duration=4.0
    )
    engine = SimulationEngine(_model(task), fixed_dt=0.05)

    engine.run_until_complete()

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert timeline.wait_started_at == pytest.approx(timeline.arrived_at)
    assert timeline.wait_ended_at == pytest.approx(30.0)
    assert timeline.wait_seconds == pytest.approx(30.0 - (timeline.arrived_at or 0.0))
    assert timeline.service_started_at == pytest.approx(30.0)
    assert _events(engine, EventType.TASK_WAIT_STARTED, "T-01")
    assert _events(engine, EventType.TASK_WAIT_ENDED, "T-01")


def test_service_duration_is_recorded() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=7.0)
    engine = SimulationEngine(_model(task), fixed_dt=0.05)

    engine.run_until_complete()

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert timeline.service_seconds == pytest.approx(7.0, abs=0.06)


def test_hard_deadline_violation_is_recorded_not_hidden() -> None:
    task = MissionTask(
        "T-01",
        "Visit",
        Point(110.0, 10.0),
        execution_duration=4.0,
        deadline=5.0,
        deadline_policy=DeadlinePolicy.HARD,
    )
    engine = SimulationEngine(_model(task), fixed_dt=0.05)

    engine.run_until_complete()

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert not timeline.deadline_ok
    assert timeline.lateness_seconds > 0.0
    violations = _events(engine, EventType.TASK_DEADLINE_VIOLATED, "T-01")
    assert len(violations) == 1
    assert violations[0] == pytest.approx(timeline.service_finished_at or 0.0)


def test_failed_predecessor_blocks_the_successor() -> None:
    first = MissionTask("T-01", "A", Point(60.0, 10.0))
    first.status = TaskStatus.FAILED
    second = MissionTask(
        "T-02", "B", Point(110.0, 10.0), predecessor_ids=["T-01"], execution_duration=4.0
    )
    model = MapModel(width=200, height=100, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.tasks.extend([first, second])
    _drone(model, second.position, ["T-02"])
    engine = SimulationEngine(model, fixed_dt=0.05)

    engine.run_until_complete()

    timeline = engine.task_timeline("T-02")
    assert timeline is not None
    assert timeline.blocked_reason is not None
    assert "T-01" in timeline.blocked_reason
    assert timeline.business_status == STATE_BLOCKED
    assert timeline.service_started_at is None
    assert _events(engine, EventType.TASK_BLOCKED, "T-02")
    assert _events(engine, EventType.TASK_SERVICE_STARTED, "T-02") == []


def test_replan_preserves_completed_timelines() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = SimulationEngine(_model(task), fixed_dt=0.05)
    engine.run_until_complete()
    finished_at = engine.task_timeline("T-01").service_finished_at  # type: ignore[union-attr]

    engine.apply_replan({})

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert timeline.service_finished_at == finished_at


def test_reset_clears_the_timelines() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = SimulationEngine(_model(task), fixed_dt=0.05)
    engine.run_until_complete()

    engine.reset()

    timeline = engine.task_timeline("T-01")
    assert timeline is not None
    assert timeline.arrived_at is None
    assert timeline.service_finished_at is None
    assert engine.timelines()
