from __future__ import annotations

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.simulation.communication import CommunicationMonitor, CommunicationNode
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.events import EventType


def test_multihop_connectivity_and_loss_transition() -> None:
    monitor = CommunicationMonitor()
    nodes = [
        CommunicationNode("B-01", Point(0, 0), 100, is_base=True),
        CommunicationNode("D-01", Point(80, 0), 100),
        CommunicationNode("D-02", Point(160, 0), 100),
    ]
    monitor.update(nodes, timestamp=0)
    assert monitor.statuses["D-01"].direct
    assert monitor.statuses["D-02"].connected
    assert monitor.statuses["D-02"].hop_count == 2

    nodes[1] = CommunicationNode("D-01", Point(80, 0), 100, available=False)
    monitor.update(nodes, timestamp=1)
    assert not monitor.statuses["D-02"].connected
    assert monitor.transitions[-1].drone_id == "D-02"


def test_auto_return_policy_releases_task_and_reaches_base() -> None:
    model = MapModel(width=120, height=80, grid_size=5)
    model.bases.append(BaseStation("B-01", "Base", Point(5, 40), communication_range=25))
    model.tasks.append(
        MissionTask(
            "T-01",
            "Remote",
            Point(100, 40),
            assigned_drone_id="D-01",
            status=TaskStatus.ASSIGNED,
        )
    )
    model.drones.append(
        Drone(
            "D-01",
            "Scout",
            Point(10, 40),
            "B-01",
            max_speed=10,
            communication_range=25,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 40), Point(100, 40), Point(5, 40)],
        )
    )
    engine = SimulationEngine(model, communication_policy="auto_return", communication_grace=0.2)
    engine.run_until_complete()

    snapshot = engine.snapshot()
    assert snapshot.drones[0].position == Point(5, 40)
    assert snapshot.task_statuses["T-01"] == TaskStatus.PENDING
    assert any(record.event.event_type == EventType.AUTO_RETURN for record in snapshot.events)
