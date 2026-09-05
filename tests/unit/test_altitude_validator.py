from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
)
from drone_mission_planner.domain.terrain import TerrainPeak, flat_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.altitude_validator import (
    AltitudeRisk,
    AltitudeRiskKind,
    AltitudeRiskSeverity,
    validate_altitude_path,
    validate_model_altitudes,
)


def altitude_map(*, cruise_altitude: float = 100.0) -> MapModel:
    model = MapModel(width=200, height=100, grid_size=10.0)
    model.terrain = flat_terrain(altitude=0.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10.0, 10.0)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10.0, 10.0),
            "B-01",
            cruise_altitude=cruise_altitude,
            min_clearance=30.0,
        )
    )
    return model


def test_flat_route_at_cruise_altitude_has_no_risks() -> None:
    model = altitude_map()
    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])
    assert risks == ()


def test_low_cruise_over_terrain_peak_reports_clearance_warning() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))

    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])

    terrain_risks = [
        risk for risk in risks if risk.kind == AltitudeRiskKind.TERRAIN_CLEARANCE
    ]
    assert len(terrain_risks) == 1
    assert terrain_risks[0].severity == AltitudeRiskSeverity.WARNING
    assert terrain_risks[0].flight_altitude == pytest.approx(100.0)
    # The risk fires at the first sampled point below clearance, so the required
    # altitude lies between the flat cruise value and the peak-top value.
    assert 100.0 < terrain_risks[0].required_altitude <= 150.0


def test_cruise_high_enough_clears_the_peak() -> None:
    model = altitude_map(cruise_altitude=250.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))

    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])

    assert all(
        risk.kind != AltitudeRiskKind.TERRAIN_CLEARANCE
        and risk.kind != AltitudeRiskKind.OBSTACLE_HEIGHT
        for risk in risks
    )


def test_obstacle_top_plus_clearance_is_critical_when_below() -> None:
    model = altitude_map(cruise_altitude=60.0)
    model.obstacles.append(Obstacle("O-01", "Tower", bounds=Rect(80.0, 0.0, 40.0, 20.0), height=45.0))

    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])

    obstacle_risks = [risk for risk in risks if risk.kind == AltitudeRiskKind.OBSTACLE_HEIGHT]
    assert len(obstacle_risks) == 1
    assert obstacle_risks[0].severity == AltitudeRiskSeverity.CRITICAL
    assert obstacle_risks[0].object_id == "O-01"
    assert obstacle_risks[0].required_altitude == pytest.approx(75.0)


def test_cruise_above_obstacle_and_clearance_is_safe() -> None:
    model = altitude_map(cruise_altitude=120.0)
    model.obstacles.append(Obstacle("O-01", "Tower", bounds=Rect(80.0, 0.0, 40.0, 20.0), height=45.0))

    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])

    assert all(risk.kind != AltitudeRiskKind.OBSTACLE_HEIGHT for risk in risks)


def test_no_fly_ceiling_is_critical_below_ceiling_and_allows_overflight_above() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.no_fly_zones.append(NoFlyZone("N-01", "Zone", bounds=Rect(80.0, 0.0, 40.0, 20.0), ceiling_altitude=120.0))

    below = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])
    assert [risk.kind for risk in below] == [AltitudeRiskKind.NO_FLY_CEILING]

    model.drones[0].cruise_altitude = 150.0
    above = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])
    assert above == ()


def test_no_fly_policy_rejects_any_overflight_when_disallowed() -> None:
    model = altitude_map(cruise_altitude=150.0)
    model.no_fly_zones.append(NoFlyZone("N-01", "Zone", bounds=Rect(80.0, 0.0, 40.0, 20.0), ceiling_altitude=120.0))

    risks = validate_altitude_path(
        model,
        model.drones[0],
        [Point(10, 10), Point(190, 10)],
        allow_no_fly_overflight=False,
    )

    assert [risk.kind for risk in risks] == [AltitudeRiskKind.NO_FLY_POLICY]
    assert risks[0].severity == AltitudeRiskSeverity.CRITICAL


def test_task_target_altitude_below_clearance_is_reported() -> None:
    model = altitude_map(cruise_altitude=100.0)
    task = MissionTask(
        "T-01",
        "Low inspect",
        Point(190.0, 10.0),
        target_altitude=10.0,
        assigned_drone_id="D-01",
    )
    model.tasks.append(task)
    model.drones[0].assigned_tasks.append("T-01")

    risks = validate_altitude_path(model, model.drones[0], [Point(10, 10), Point(190, 10)])

    task_risks = [risk for risk in risks if risk.kind == AltitudeRiskKind.TASK_ALTITUDE]
    assert len(task_risks) == 1
    assert task_risks[0].severity == AltitudeRiskSeverity.WARNING
    assert task_risks[0].object_id == "T-01"
    assert task_risks[0].required_altitude == pytest.approx(30.0)


def test_validate_model_altitudes_aggregates_all_drones() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))
    model.drones[0].planned_path = [Point(10, 10), Point(190, 10)]
    model.drones.append(
        Drone(
            "D-02",
            "Bravo",
            Point(10.0, 50.0),
            "B-01",
            cruise_altitude=100.0,
            min_clearance=30.0,
            planned_path=[Point(10, 50), Point(190, 50)],
        )
    )

    risks = validate_model_altitudes(model)

    assert {risk.drone_id for risk in risks} == {"D-01"}
    assert risks[0].kind == AltitudeRiskKind.TERRAIN_CLEARANCE


def test_risk_summary_lists_drone_leg_and_altitudes() -> None:
    risk = AltitudeRisk(
        "D-01",
        2,
        AltitudeRiskKind.OBSTACLE_HEIGHT,
        AltitudeRiskSeverity.CRITICAL,
        Point(100.0, 10.0),
        75.0,
        60.0,
        object_id="O-01",
        message="obstacle top plus clearance is above commanded altitude",
    )
    text = risk.summary()

    # Summary omits the drone id because callers render it inside a per-drone
    # tooltip or table row; it must still identify the leg, object, and values.
    assert "leg 2" in text
    assert "O-01" in text
    assert "obstacle top plus clearance is above commanded altitude" in text
    assert "60.0/75.0" in text


def test_waypoint_altitudes_high_enough_clear_the_peak() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))
    drone = model.drones[0]
    path = [Point(10, 10), Point(190, 10)]
    drone.planned_path = list(path)
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(190.0, 10.0, altitude=220.0),
    ]

    risks = validate_altitude_path(model, drone, path)

    assert risks == ()


def test_waypoint_altitudes_below_clearance_report_the_edited_altitude() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))
    drone = model.drones[0]
    path = [Point(10, 10), Point(190, 10)]
    drone.planned_path = list(path)
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=40.0),
        Waypoint(190.0, 10.0, altitude=40.0),
    ]

    risks = validate_altitude_path(model, drone, path)

    terrain_risks = [
        risk for risk in risks if risk.kind == AltitudeRiskKind.TERRAIN_CLEARANCE
    ]
    assert terrain_risks
    assert terrain_risks[0].flight_altitude == pytest.approx(40.0)


def test_mismatched_waypoint_count_falls_back_to_commanded_altitude() -> None:
    model = altitude_map(cruise_altitude=100.0)
    model.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 30.0, 120.0))
    drone = model.drones[0]
    path = [Point(10, 10), Point(190, 10)]
    drone.planned_path = list(path)
    drone.waypoints = [Waypoint(10.0, 10.0, altitude=40.0)]

    risks = validate_altitude_path(model, drone, path)

    terrain_risks = [
        risk for risk in risks if risk.kind == AltitudeRiskKind.TERRAIN_CLEARANCE
    ]
    assert terrain_risks
    assert terrain_risks[0].flight_altitude == pytest.approx(100.0)
