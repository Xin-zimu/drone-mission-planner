from __future__ import annotations

import json
from pathlib import Path

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.planning.assignment import (
    AssignmentWeights,
    GreedyAssignmentPlanner,
    build_assignment_suggestions,
    explain_assignments,
)
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.reporting import build_simulation_report, export_report


def test_explain_assignments_scores_and_sorts_candidates() -> None:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40.0, 40.0)))
    near = Drone(
        "D-01", "Alpha", Point(60.0, 60.0), "B-01", battery_capacity=500.0, remaining_battery=500.0
    )
    far = Drone(
        "D-02", "Bravo", Point(500.0, 350.0), "B-01", battery_capacity=500.0, remaining_battery=500.0
    )
    model.drones.extend([near, far])
    task = MissionTask("T-01", "Visit", Point(200.0, 200.0))
    model.tasks.append(task)

    explanations = explain_assignments(model)
    explanation = explanations[0]
    best = explanation.candidates[0]

    assert explanation.best_drone_id == "D-01"
    assert best.feasible
    assert best.distance < explanation.candidates[1].distance
    assert best.score < explanation.candidates[1].score


def test_weights_change_the_ranking() -> None:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40.0, 40.0)))
    # D-01 is close but slow, so a tight deadline makes it expensive; D-02 is
    # far but fast. Removing the deadline weight flips the ranking to distance.
    slow_near = Drone(
        "D-01",
        "Alpha",
        Point(80.0, 80.0),
        "B-01",
        battery_capacity=500.0,
        remaining_battery=500.0,
        max_speed=5.0,
        air_speed=5.0,
    )
    fast_far = Drone(
        "D-02",
        "Bravo",
        Point(400.0, 300.0),
        "B-01",
        battery_capacity=500.0,
        remaining_battery=500.0,
        max_speed=20.0,
        air_speed=20.0,
    )
    model.drones.extend([slow_near, fast_far])
    task = MissionTask("T-01", "Visit", Point(200.0, 200.0), deadline=12.0)
    model.tasks.append(task)

    default = explain_assignments(model, weights=AssignmentWeights())[0]
    distance_only = explain_assignments(
        model,
        weights=AssignmentWeights(
            battery_risk=0.0, energy=0.0, task_load=0.0, deadline=0.0, distance=1.0
        ),
    )[0]

    assert default.best_drone_id == "D-02"
    assert distance_only.best_drone_id == "D-01"


def test_planner_uses_custom_weights_for_the_actual_assignment() -> None:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40.0, 40.0)))
    slow_near = Drone(
        "D-01",
        "Alpha",
        Point(80.0, 80.0),
        "B-01",
        battery_capacity=500.0,
        remaining_battery=500.0,
        max_speed=5.0,
        air_speed=5.0,
    )
    fast_far = Drone(
        "D-02",
        "Bravo",
        Point(400.0, 300.0),
        "B-01",
        battery_capacity=500.0,
        remaining_battery=500.0,
        max_speed=20.0,
        air_speed=20.0,
    )
    model.drones.extend([slow_near, fast_far])
    task = MissionTask("T-01", "Visit", Point(200.0, 200.0), deadline=12.0)
    model.tasks.append(task)

    distance_only = GreedyAssignmentPlanner(
        weights=AssignmentWeights(
            battery_risk=0.0, energy=0.0, task_load=0.0, deadline=0.0, distance=1.0
        )
    )
    result = distance_only.assign(model)

    assert result.decisions[0].drone_id == "D-01"


def test_suggestions_map_to_real_constraints() -> None:
    model = MapModel(width=600, height=400, grid_size=10.0)
    model.bases.append(BaseStation("B-01", "Base", Point(40.0, 40.0)))
    drone = Drone(
        "D-01",
        "Alpha",
        Point(40.0, 40.0),
        "B-01",
        payload_capacity=1.0,
        remaining_battery=0.5,
    )
    model.drones.append(drone)
    heavy = MissionTask("T-HEAVY", "Heavy", Point(100.0, 100.0), required_payload=50.0)
    thirsty = MissionTask("T-FAR", "Far", Point(500.0, 350.0))
    model.tasks.extend([heavy, thirsty])

    suggestions = build_assignment_suggestions(explain_assignments(model))

    assert any("payload" in tip for tip in suggestions["T-HEAVY"])
    assert any("battery" in tip for tip in suggestions["T-FAR"])


def test_report_includes_assignment_notes(tmp_path: Path) -> None:
    engine = _plain_engine()

    report = build_simulation_report(engine, ("T-01 -> D-01 (cost 900)",))
    saved = export_report(report, tmp_path / "report.json")

    assert json.loads(saved.read_text(encoding="utf-8"))["assignment_notes"] == [
        "T-01 -> D-01 (cost 900)"
    ]


def _plain_engine() -> SimulationEngine:
    model = MapModel(width=100, height=100, grid_size=5.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10,
            energy_per_meter=0.1,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 10), Point(30, 10), Point(10, 10)],
        )
    )
    model.tasks.append(MissionTask("T-01", "Task", Point(30, 10)))
    return SimulationEngine(model, fixed_dt=0.05)
