from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    Obstacle,
)
from drone_mission_planner.domain.terrain import flat_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.planning.risk_assessment import (
    assess_mission_risk,
    assess_route_risk,
    build_risk_matrix,
    risk_level,
)


def _model() -> tuple[MapModel, Drone]:
    model = MapModel(width=500, height=400, grid_size=10.0)
    model.terrain = flat_terrain(altitude=0.0)
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
        planned_path=[Point(40.0, 40.0), Point(200.0, 120.0)],
        waypoints=[
            Waypoint(40.0, 40.0, altitude=80.0),
            Waypoint(200.0, 120.0, altitude=80.0, action=WaypointAction.RETURN_TO_LAUNCH),
        ],
    )
    model.drones.append(drone)
    return model, drone


def test_low_risk_route_scores_low() -> None:
    model, drone = _model()

    assessment = assess_route_risk(model, drone)

    assert assessment.level == "low"
    assert assessment.score == pytest.approx(0.0)
    assert assessment.factors == ()


def test_obstacle_crossing_produces_critical_airspace_factor() -> None:
    model, drone = _model()
    obstacle = Obstacle("O-01", "Tower", bounds=Rect(110.0, 70.0, 20.0, 20.0))
    obstacle.height = 200.0
    model.obstacles.append(obstacle)

    assessment = assess_route_risk(model, drone)
    kinds = {(factor.kind, factor.severity) for factor in assessment.factors}

    assert ("airspace", "critical") in kinds
    assert assessment.level == "critical"
    assert assessment.factors[0].segment_index is not None
    assert assessment.factors[0].object_id == "O-01"


def test_low_battery_produces_critical_battery_factor() -> None:
    model, drone = _model()
    drone.remaining_battery = 0.01

    assessment = assess_route_risk(model, drone)

    battery = [factor for factor in assessment.factors if factor.kind == "battery"]
    assert battery and battery[0].severity == "critical"
    assert assessment.level == "critical"


def test_small_battery_reserve_warns() -> None:
    model, drone = _model()
    # Enough for the ~14 energy route, but below 15% of the 500 capacity.
    drone.remaining_battery = 80.0

    assessment = assess_route_risk(model, drone)
    battery = [factor for factor in assessment.factors if factor.kind == "battery"]

    assert battery and battery[0].severity == "warning"


def test_communication_range_exceeded_is_flagged() -> None:
    model, drone = _model()
    model.bases[0].communication_range = 120.0
    drone.communication_range = 120.0

    assessment = assess_route_risk(model, drone)
    communication = [factor for factor in assessment.factors if factor.kind == "communication"]

    assert communication
    assert assessment.level in {"elevated", "high", "critical"}


def test_long_hover_and_missing_return_are_action_risks() -> None:
    model, drone = _model()
    drone.waypoints = [
        Waypoint(40.0, 40.0, altitude=80.0, hold_seconds=120.0),
        Waypoint(200.0, 120.0, altitude=80.0),
    ]

    assessment = assess_route_risk(model, drone)
    actions = [factor.message for factor in assessment.factors if factor.kind == "action"]

    assert len(actions) == 2
    assert any("hold" in message for message in actions)
    assert any("return-to-launch" in message for message in actions)


def test_mission_assessment_and_matrix_are_deterministic() -> None:
    model, _drone = _model()
    second = Drone(
        "D-02",
        "Bravo",
        Point(60.0, 60.0),
        "B-01",
        planned_path=[Point(60.0, 60.0), Point(160.0, 160.0)],
        waypoints=[
            Waypoint(60.0, 60.0, altitude=80.0),
            Waypoint(160.0, 160.0, altitude=80.0, action=WaypointAction.RETURN_TO_LAUNCH),
        ],
    )
    model.drones.append(second)

    first = assess_mission_risk(model)
    again = assess_mission_risk(model)
    assert first == again

    matrix = build_risk_matrix(first)
    assert [row.drone_id for row in matrix] == ["D-01", "D-02"]
    assert all(row.level == "low" for row in matrix)


def test_risk_level_thresholds_are_stable() -> None:
    assert risk_level(0.0) == "low"
    assert risk_level(12.0) == "moderate"
    assert risk_level(30.0) == "elevated"
    assert risk_level(50.0) == "high"
    assert risk_level(95.0) == "critical"
