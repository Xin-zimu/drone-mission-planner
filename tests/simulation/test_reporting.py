from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.enums import TaskStatus
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.reporting import build_simulation_report, export_report


def simulation_map() -> MapModel:
    model = MapModel(width=100, height=100, grid_size=5)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.tasks.append(
        MissionTask(
            "T-01",
            "Inspect",
            Point(30, 10),
            execution_duration=0.2,
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
            energy_per_meter=0.1,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 10), Point(30, 10), Point(10, 10)],
        )
    )
    return model


def test_report_aggregates_deterministic_mission_metrics(tmp_path: Path) -> None:
    engine = SimulationEngine(simulation_map())
    engine.run_until_complete()
    report = build_simulation_report(engine)

    assert report.completion_rate == 1.0
    assert report.completed_tasks == 1
    assert report.total_distance == 40
    assert report.total_energy_used == pytest.approx(4)
    assert report.drones[0].completed_tasks == 1

    json_path = export_report(report, tmp_path / "report.json")
    csv_path = export_report(report, tmp_path / "report.csv")
    html_path = export_report(report, tmp_path / "report.html")
    assert json.loads(json_path.read_text(encoding="utf-8"))["completed_tasks"] == 1
    assert "drone_id,status,distance_flown" in csv_path.read_text(encoding="utf-8-sig")
    assert "Task completion" in html_path.read_text(encoding="utf-8")


def test_report_includes_environment_summary() -> None:
    model = simulation_map()
    model.terrain = generate_mountain_terrain(
        width=100.0,
        height=100.0,
        resolution=10.0,
        base_altitude=20.0,
        peaks=[TerrainPeak(Point(30.0, 10.0), 12.0, 80.0)],
    )
    model.wind = WindModel(direction_to_deg=270.0, speed=4.0, gust_factor=1.3, enabled=True)
    engine = SimulationEngine(model)
    engine.run_until_complete()

    report = build_simulation_report(engine)

    env = report.environment
    assert env.terrain_type == "procedural"
    assert env.base_altitude == 20.0
    assert env.max_altitude > env.base_altitude
    assert env.peak_count == 1
    assert env.wind_enabled
    assert env.wind_speed == 4.0
    assert env.wind_direction_to_deg == 270.0
    assert env.wind_gust_factor == pytest.approx(1.3)


def test_report_includes_altitude_risks_and_per_drone_counts() -> None:
    model = simulation_map()
    model.terrain = generate_mountain_terrain(
        width=100.0,
        height=100.0,
        resolution=10.0,
        peaks=[TerrainPeak(Point(30.0, 10.0), 12.0, 120.0)],
    )
    model.drones[0].cruise_altitude = 40.0
    model.drones[0].min_clearance = 30.0
    engine = SimulationEngine(model)
    engine.run_until_complete()

    report = build_simulation_report(engine)

    assert report.altitude_risk_count >= 1
    assert report.drones[0].altitude_risk_count >= 1
    first = report.altitude_risks[0]
    assert first.drone_id == "D-01"
    assert first.kind == "terrain_clearance"
    assert first.severity == "warning"
    assert first.flight_altitude == pytest.approx(40.0)
    assert first.required_altitude > 40.0
