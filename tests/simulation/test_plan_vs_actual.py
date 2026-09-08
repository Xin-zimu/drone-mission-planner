"""SCH-08: the report compares planned and actual mission times (plan §10.8)."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.enums import DeadlinePolicy, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.scheduling import evaluate_schedule
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.reporting import (
    SimulationReport,
    TaskTimelineReport,
    build_simulation_report,
    export_report,
)


def _engine(
    task: MissionTask, *, planned: bool = True, travel_seconds: float = 10.0
) -> SimulationEngine:
    model = MapModel(width=200, height=100, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.tasks.append(task)
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
    drone.assigned_tasks = [task.id]
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(task.position.x, task.position.y, altitude=100.0),
    ]
    model.drones.append(drone)
    schedule = (
        evaluate_schedule(
            [task],
            travel_seconds={task.id: travel_seconds},
            assignments={task.id: drone.id},
        )
        if planned
        else None
    )
    return SimulationEngine(model, fixed_dt=0.05, planned_schedule=schedule)


def _timeline(report: SimulationReport, task_id: str) -> TaskTimelineReport:
    return next(item for item in report.task_timelines if item.task_id == task_id)


def test_planned_and_actual_times_are_reported_with_deviation() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = _engine(task, travel_seconds=5.0)
    engine.run_until_complete()

    item = _timeline(build_simulation_report(engine), "T-01")

    assert item.planned_start is not None and item.planned_finish is not None
    assert item.planned_start == pytest.approx(5.0)
    assert item.planned_finish == pytest.approx(9.0)
    assert item.actual_start is not None and item.actual_arrival is not None
    assert item.actual_finish is not None
    assert item.start_deviation is not None and item.finish_deviation is not None
    assert item.actual_start == pytest.approx(item.actual_arrival)
    assert item.start_deviation == pytest.approx(item.actual_start - item.planned_start)
    assert item.finish_deviation == pytest.approx(item.actual_finish - item.planned_finish)
    # The plan assumed a 5 s leg; the real flight takes longer, so both
    # deviations are positive and the report says so instead of hiding it.
    assert item.start_deviation > 0.0
    assert item.finish_deviation > 0.0


def test_without_a_plan_the_deviations_are_unknown_not_zero() -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = _engine(task, planned=False)
    engine.run_until_complete()

    item = _timeline(build_simulation_report(engine), "T-01")

    assert item.planned_start is None
    assert item.planned_finish is None
    assert item.start_deviation is None
    assert item.finish_deviation is None
    assert item.actual_finish is not None


def test_wait_and_service_seconds_are_reported() -> None:
    task = MissionTask(
        "T-01", "Visit", Point(110.0, 10.0), earliest_start=30.0, execution_duration=6.0
    )
    engine = _engine(task)
    engine.run_until_complete()

    item = _timeline(build_simulation_report(engine), "T-01")

    assert item.wait_seconds == pytest.approx(30.0 - (item.actual_arrival or 0.0), abs=0.06)
    assert item.service_seconds == pytest.approx(6.0, abs=0.06)


def test_deadline_violation_is_reported() -> None:
    task = MissionTask(
        "T-01",
        "Visit",
        Point(110.0, 10.0),
        execution_duration=4.0,
        deadline=5.0,
        deadline_policy=DeadlinePolicy.HARD,
    )
    engine = _engine(task)
    engine.run_until_complete()

    item = _timeline(build_simulation_report(engine), "T-01")

    assert not item.deadline_ok
    assert item.lateness_seconds > 0.0
    assert item.deadline_policy == "hard"


def test_blocked_task_reports_its_reason_and_no_actual_times() -> None:
    first = MissionTask("T-01", "A", Point(60.0, 10.0))
    first.status = TaskStatus.FAILED
    second = MissionTask(
        "T-02", "B", Point(110.0, 10.0), predecessor_ids=["T-01"], execution_duration=4.0
    )
    model = MapModel(width=200, height=100, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.tasks.extend([first, second])
    drone = Drone("D-01", "Alpha", Point(10.0, 10.0), "B-01", max_speed=10.0, air_speed=10.0,
                  battery_capacity=1000.0, remaining_battery=1000.0)
    drone.assigned_tasks = ["T-02"]
    drone.waypoints = [Waypoint(10.0, 10.0, altitude=100.0), Waypoint(110.0, 10.0, altitude=100.0)]
    model.drones.append(drone)
    engine = SimulationEngine(model, fixed_dt=0.05)
    engine.run_until_complete()

    item = _timeline(build_simulation_report(engine), "T-02")

    assert item.blocked_reason is not None
    assert "T-01" in item.blocked_reason
    assert item.actual_start is None
    assert item.actual_finish is None
    assert item.start_deviation is None


def test_all_three_export_formats_carry_the_timeline(tmp_path: Path) -> None:
    task = MissionTask("T-01", "Visit", Point(110.0, 10.0), execution_duration=4.0)
    engine = _engine(task)
    engine.run_until_complete()
    report = build_simulation_report(engine)

    json_path = export_report(report, tmp_path / "report.json")
    csv_path = export_report(report, tmp_path / "report.csv")
    html_path = export_report(report, tmp_path / "report.html")

    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["task_timelines"][0]["task_id"] == "T-01"
    assert payload["task_timelines"][0]["planned_start"] == pytest.approx(10.0)

    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.reader(stream))
    assert any(row and row[0] == "task_id" for row in rows)
    assert any(row and row[0] == "T-01" for row in rows)

    html = html_path.read_text(encoding="utf-8")
    assert "Plan versus actual" in html
    assert "T-01" in html
