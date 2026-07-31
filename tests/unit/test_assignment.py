from __future__ import annotations

from drone_mission_planner.domain.enums import DroneStatus
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    Obstacle,
)
from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner


def base_map() -> MapModel:
    model = MapModel(width=400, height=300, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(25, 25), 500))
    return model


def test_high_priority_tasks_are_considered_first_and_only_once() -> None:
    model = base_map()
    model.drones.extend(
        [
            Drone("D-01", "Alpha", Point(25, 25), home_base_id="B-01"),
            Drone("D-02", "Bravo", Point(35, 25), home_base_id="B-01"),
        ]
    )
    model.tasks.extend(
        [
            MissionTask("T-LOW", "Low", Point(100, 80), priority=2),
            MissionTask("T-HIGH", "High", Point(250, 180), priority=9),
            MissionTask("T-MID", "Mid", Point(180, 70), priority=5),
        ]
    )
    result = GreedyAssignmentPlanner().assign(model)
    assert [decision.task_id for decision in result.decisions] == ["T-HIGH", "T-MID", "T-LOW"]
    assert len({decision.task_id for decision in result.decisions}) == len(result.decisions)


def test_payload_constraint_selects_capable_drone() -> None:
    model = base_map()
    model.drones.extend(
        [
            Drone("D-01", "Light", Point(25, 25), "B-01", payload_capacity=1.0),
            Drone("D-02", "Cargo", Point(35, 25), "B-01", payload_capacity=8.0),
        ]
    )
    model.tasks.append(MissionTask("T-01", "Deliver", Point(150, 100), required_payload=4.0))
    result = GreedyAssignmentPlanner().assign(model)
    assert result.decisions[0].drone_id == "D-02"


def test_battery_check_includes_safe_return_and_reserve() -> None:
    model = base_map()
    model.drones.extend(
        [
            Drone(
                "D-01",
                "Weak",
                Point(25, 25),
                "B-01",
                battery_capacity=100,
                remaining_battery=35,
                energy_per_meter=0.2,
            ),
            Drone(
                "D-02",
                "Long range",
                Point(30, 25),
                "B-01",
                battery_capacity=200,
                remaining_battery=200,
                energy_per_meter=0.05,
            ),
        ]
    )
    model.tasks.append(MissionTask("T-01", "Remote", Point(300, 200)))
    result = GreedyAssignmentPlanner().assign(model)
    assert result.decisions[0].drone_id == "D-02"


def test_unassignable_task_explains_every_drone_rejection() -> None:
    model = base_map()
    model.drones.extend(
        [
            Drone("D-01", "Failed", Point(25, 25), "B-01", status=DroneStatus.FAILED),
            Drone("D-02", "Light", Point(25, 25), "B-01", payload_capacity=1.0),
        ]
    )
    model.tasks.append(MissionTask("T-01", "Heavy", Point(100, 100), required_payload=5.0))
    result = GreedyAssignmentPlanner().assign(model)
    assert not result.decisions
    assert set(result.failures[0].reasons) == {"D-01", "D-02"}
    assert "payload" in result.failures[0].summary()


def test_unreachable_routes_are_rejected() -> None:
    model = base_map()
    model.drones.append(Drone("D-01", "Alpha", Point(25, 25), "B-01"))
    model.tasks.append(MissionTask("T-01", "Blocked", Point(300, 200)))
    model.obstacles.append(Obstacle("O-01", "Wall", bounds=Rect(0, 130, 400, 25)))
    result = GreedyAssignmentPlanner().assign(model)
    assert not result.decisions
    assert "No safe path" in result.failures[0].summary()
