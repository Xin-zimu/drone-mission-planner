"""SCH-07: the read-only Gantt renders plan and actual bars and links to the map."""

from __future__ import annotations

from PySide6.QtCore import QPoint, Qt
from PySide6.QtGui import QPixmap

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MissionTask
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.scheduling import evaluate_schedule
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.task_timeline import TaskTimeline
from drone_mission_planner.ui.main_window import MainWindow
from drone_mission_planner.ui.schedule_view import (
    LANE_PLANNED,
    LANE_SERVICE,
    LANE_WAIT,
    ScheduleView,
)


def _task() -> MissionTask:
    return MissionTask(
        "T-01", "Inspect ridge", Point(110.0, 10.0), earliest_start=30.0, execution_duration=4.0
    )


def _service() -> tuple[ProjectService, MissionTask]:
    service = ProjectService()
    service.add_base(Point(10.0, 10.0))
    drone = service.add_drone(Point(10.0, 10.0))
    task = service.add_task(Point(110.0, 10.0))
    task.name = "Inspect ridge"
    task.earliest_start = 30.0
    task.execution_duration = 4.0
    drone.assigned_tasks = [task.id]
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(110.0, 10.0, altitude=100.0),
    ]
    service.dirty = False
    return service, task


def _paint(view: ScheduleView) -> None:
    view.resize(800, 260)
    view.render(QPixmap(view.size()))


def test_view_draws_planned_and_actual_lanes(qtbot: object) -> None:
    task = _task()
    schedule = evaluate_schedule(
        [task], travel_seconds={"T-01": 10.0}, assignments={"T-01": "D-01"}
    )
    timeline = TaskTimeline(
        task_id="T-01",
        drone_id="D-01",
        arrived_at=10.0,
        wait_started_at=10.0,
        wait_ended_at=30.0,
        service_started_at=30.0,
        service_finished_at=34.0,
        deadline=task.deadline,
    )
    view = ScheduleView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]

    view.set_schedule(schedule, [timeline], {"T-01": task}, ["D-01"])
    _paint(view)

    kinds = {bar.kind for bar in view.bars()}
    assert view.row_count() == 1
    assert {LANE_PLANNED, LANE_WAIT, LANE_SERVICE} <= kinds
    assert view.bar_rect("T-01", LANE_SERVICE) is not None


def test_clicking_a_bar_emits_the_task_id(qtbot: object) -> None:
    task = _task()
    schedule = evaluate_schedule(
        [task], travel_seconds={"T-01": 10.0}, assignments={"T-01": "D-01"}
    )
    view = ScheduleView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.set_schedule(schedule, (), {"T-01": task}, ["D-01"])
    _paint(view)
    rect = view.bar_rect("T-01", LANE_PLANNED)
    assert rect is not None

    with qtbot.waitSignal(view.task_selected, timeout=1000) as blocker:  # type: ignore[attr-defined]
        qtbot.mouseClick(  # type: ignore[attr-defined]
            view, Qt.MouseButton.LeftButton, pos=QPoint(int(rect.center().x()), int(rect.center().y()))
        )

    assert blocker.args == ["T-01"]


def test_empty_view_renders_without_crashing(qtbot: object) -> None:
    view = ScheduleView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]

    view.clear("Plan or run a simulation to see the schedule")
    _paint(view)

    assert view.bars() == ()
    assert view.row_count() == 0
    assert "simulation" in view.empty_reason()


def test_main_window_tab_links_a_bar_to_the_selection(qtbot: object) -> None:
    service, task = _service()
    window = MainWindow(service)
    qtbot.addWidget(window)  # type: ignore[attr-defined]
    titles = [
        window.workspace_tabs.tabText(index)
        for index in range(window.workspace_tabs.count())
    ]
    assert "Schedule" in titles
    engine = SimulationEngine(service.project.map, fixed_dt=0.05)
    engine.run_until_complete()
    window.simulation_engine = engine
    window._render_schedule()
    _paint(window.schedule_view)
    rect = window.schedule_view.bar_rect(task.id, LANE_SERVICE)
    assert rect is not None

    qtbot.mouseClick(  # type: ignore[attr-defined]
        window.schedule_view,
        Qt.MouseButton.LeftButton,
        pos=QPoint(int(rect.center().x()), int(rect.center().y())),
    )

    assert window._selected_id == task.id
    window.service.dirty = False


def test_view_is_read_only(qtbot: object) -> None:
    service, task = _service()
    before = list(service.project.map.drones[0].waypoints)
    schedule = evaluate_schedule(
        [task], travel_seconds={"T-01": 10.0}, assignments={task.id: "D-01"}
    )
    view = ScheduleView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]

    view.set_schedule(schedule, (), {task.id: task}, ["D-01"])

    assert view.read_only is True
    assert not hasattr(view, "set_waypoint")
    assert service.project.map.drones[0].waypoints == before
    assert task.assigned_drone_id is None
