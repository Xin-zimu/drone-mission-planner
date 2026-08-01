from __future__ import annotations

from drone_mission_planner.domain.enums import DroneStatus, TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.simulation.engine import SimulationEngine


def simulation_map() -> MapModel:
    model = MapModel(width=100, height=100, grid_size=5.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.tasks.append(
        MissionTask(
            "T-01",
            "Inspect",
            Point(30, 10),
            assigned_drone_id="D-01",
            status=TaskStatus.ASSIGNED,
        )
    )
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 10), Point(30, 10), Point(10, 10)],
        )
    )
    return model


def test_manual_failure_stops_drone_and_requests_replan() -> None:
    engine = SimulationEngine(simulation_map())
    engine.start()
    engine.advance(1.5)
    before = engine.snapshot().drones[0]

    assert engine.trigger_failure("D-01", reason="Motor controller fault")
    failed = engine.snapshot().drones[0]
    assert failed.status == DroneStatus.FAILED
    assert failed.position == before.position
    assert failed.remaining_battery == before.remaining_battery
    assert failed.failure_reason == "Motor controller fault"
    assert engine.snapshot().task_statuses["T-01"] == TaskStatus.PENDING
    assert engine.drain_replan_requests() == ("D-01",)

    engine.advance(5.0)
    stopped = engine.snapshot().drones[0]
    assert stopped.position == failed.position
    assert stopped.remaining_battery == failed.remaining_battery


def test_automatic_failure_is_deterministic_and_processed() -> None:
    first = SimulationEngine(simulation_map(), random_seed=7)
    second = SimulationEngine(simulation_map(), random_seed=7)
    first_event = first.schedule_random_failure(minimum_delay=0.4, maximum_delay=0.4)
    second_event = second.schedule_random_failure(minimum_delay=0.4, maximum_delay=0.4)
    assert first_event.target_id == second_event.target_id
    assert first_event.timestamp == second_event.timestamp

    first.start()
    first.advance(0.5)
    assert first.snapshot().drones[0].status == DroneStatus.FAILED
    assert any(record.event.id == first_event.id for record in first.snapshot().events)


def test_cancelled_task_is_not_reactivated_on_reset() -> None:
    engine = SimulationEngine(simulation_map())
    assert engine.cancel_task("T-01")
    engine.reset()
    assert engine.snapshot().task_statuses["T-01"] == TaskStatus.CANCELLED
