from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    Obstacle,
)
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.persistence.route_export import (
    RouteExportError,
    export_route_csv,
    export_route_json,
    export_route_qgc_plan,
    export_route_wpl,
)


def _export_model() -> tuple[MapModel, Drone]:
    model = MapModel(width=500, height=400, grid_size=10.0)
    model.terrain = generate_mountain_terrain(
        width=500.0,
        height=400.0,
        resolution=50.0,
        peaks=[TerrainPeak(Point(250.0, 200.0), 100.0, 40.0)],
    )
    base = BaseStation("B-01", "Base", Point(40.0, 40.0))
    base.communication_range = 1000.0
    model.bases.append(base)
    drone = Drone(
        "D-01",
        "Alpha",
        Point(40.0, 40.0),
        "B-01",
        battery_capacity=500.0,
        remaining_battery=500.0,
        communication_range=1000.0,
        planned_path=[Point(40.0, 40.0), Point(250.0, 200.0), Point(400.0, 300.0)],
        waypoints=[
            Waypoint(40.0, 40.0, altitude=60.0, speed=8.0),
            Waypoint(
                250.0,
                200.0,
                altitude=120.0,
                action=WaypointAction.TAKE_PHOTO,
                hold_seconds=2.0,
                task_id="T-01",
            ),
            Waypoint(400.0, 300.0, altitude=80.0, action=WaypointAction.RETURN_TO_LAUNCH),
        ],
    )
    model.drones.append(drone)
    task = MissionTask("T-01", "Inspect", Point(250.0, 200.0))
    task.assigned_drone_id = drone.id
    model.tasks.append(task)
    return model, drone


def test_json_export_keeps_every_waypoint_field(tmp_path: Path) -> None:
    model, drone = _export_model()

    saved = export_route_json(model, drone, tmp_path / "route.json")
    payload = json.loads(saved.read_text(encoding="utf-8"))

    assert payload["drone_id"] == "D-01"
    assert payload["coordinate_reference"] == "local_metric_ungeoreferenced"
    assert payload["flyable"] is False
    assert any("must not be flown" in note for note in payload["notes"])
    assert len(payload["waypoints"]) == 3
    mid = payload["waypoints"][1]
    assert mid["action"] == "take_photo"
    assert mid["task_id"] == "T-01"
    assert mid["speed"] is None
    assert mid["altitude_msl"] == pytest.approx(120.0)
    assert payload["risk_level"] == "low"
    assert payload["risk_score"] == pytest.approx(0.0)
    assert payload["altitude_risks"] == []


def test_csv_export_has_one_row_per_waypoint(tmp_path: Path) -> None:
    model, drone = _export_model()

    saved = export_route_csv(model, drone, tmp_path / "route.csv")
    lines = saved.read_text(encoding="utf-8").strip().splitlines()

    assert lines[0].startswith("index,x,y,altitude")
    assert len(lines) == 4
    assert lines[2].endswith("take_photo,2,T-01")


def test_qgc_plan_uses_mavlink_commands_and_marks_not_flyable(tmp_path: Path) -> None:
    model, drone = _export_model()

    saved = export_route_qgc_plan(model, drone, tmp_path / "route.plan")
    document = json.loads(saved.read_text(encoding="utf-8"))

    assert document["fileType"] == "Plan"
    assert document["flyable"] is False
    items = document["mission"]["items"]
    assert [item["command"] for item in items] == [22, 2000, 20]
    assert all(item["frame"] == 3 for item in items)
    assert items[1]["params"][4] == pytest.approx(250.0)
    assert items[1]["params"][6] == pytest.approx(120.0)


def test_wpl_export_starts_with_header_and_command_rows(tmp_path: Path) -> None:
    model, drone = _export_model()

    saved = export_route_wpl(model, drone, tmp_path / "route.waypoints")
    lines = saved.read_text(encoding="utf-8").strip().splitlines()

    assert lines[0] == "QGC WPL 110"
    assert len(lines) == 5
    assert lines[1].split("\t")[3] == "22"
    assert lines[2].split("\t")[3] == "2000"
    assert any("must not be flown" in line for line in lines)


def test_export_rejects_routes_without_waypoints(tmp_path: Path) -> None:
    model, drone = _export_model()
    drone.waypoints = []

    with pytest.raises(RouteExportError, match="no waypoints"):
        export_route_json(model, drone, tmp_path / "route.json")


def test_export_rejects_drone_without_home_base(tmp_path: Path) -> None:
    model, drone = _export_model()
    drone.home_base_id = None

    with pytest.raises(RouteExportError, match="home base"):
        export_route_json(model, drone, tmp_path / "route.json")


def test_export_rejects_critical_altitude_risks(tmp_path: Path) -> None:
    model, drone = _export_model()
    block = Obstacle("O-01", "Tower", bounds=Rect(240.0, 190.0, 20.0, 20.0))
    block.height = 300.0
    model.obstacles.append(block)

    with pytest.raises(RouteExportError, match="critical altitude risk"):
        export_route_json(model, drone, tmp_path / "route.json")


def test_export_rejects_insufficient_battery(tmp_path: Path) -> None:
    model, drone = _export_model()
    drone.remaining_battery = 0.01

    with pytest.raises(RouteExportError, match="battery"):
        export_route_json(model, drone, tmp_path / "route.json")
