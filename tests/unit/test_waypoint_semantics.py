"""F1 semantic preservation (plan 9.7 acceptance cases 1 and 3).

Waypoints that share a position but differ in action must survive a save/load
cycle, and editing altitude must move the route vertically only.
"""

from __future__ import annotations

from pathlib import Path

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.terrain import TerrainPeak
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.persistence.project_repository import ProjectRepository
from drone_mission_planner.planning.altitude_validator import validate_altitude_path


def test_same_position_waypoints_keep_their_actions(tmp_path: Path) -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(60.0, 30.0, altitude=100.0, action=WaypointAction.TAKE_PHOTO),
        Waypoint(60.0, 30.0, altitude=100.0, action=WaypointAction.HOVER, hold_seconds=5.0),
        Waypoint(10.0, 10.0, altitude=100.0, action=WaypointAction.RETURN_TO_LAUNCH),
    ]
    path = tmp_path / "semantics.dmproj"

    service.save(path)
    loaded = ProjectRepository().load(path)

    actions = [waypoint.action for waypoint in loaded.map.drones[0].waypoints]
    assert actions == [
        WaypointAction.FLY_TO,
        WaypointAction.TAKE_PHOTO,
        WaypointAction.HOVER,
        WaypointAction.RETURN_TO_LAUNCH,
    ]
    assert loaded.map.drones[0].waypoints[2].hold_seconds == 5.0
    assert len(loaded.map.drones[0].planned_path) == 4


def test_altitude_edit_keeps_the_two_dimensional_position() -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=100.0),
        Waypoint(60.0, 30.0, altitude=100.0),
    ]
    before = list(drone.planned_path)

    service.edit_waypoint(drone.id, 1, "altitude", 250.0)

    assert drone.waypoints[1].altitude == 250.0
    assert drone.planned_path == before


def test_altitude_edit_updates_the_altitude_risk() -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    drone = service.add_drone(Point(10.0, 10.0))
    drone.min_clearance = 30.0
    service.project.map.terrain.peaks.append(TerrainPeak(Point(100.0, 10.0), 40.0, 120.0))
    drone.waypoints = [
        Waypoint(10.0, 10.0, altitude=60.0),
        Waypoint(190.0, 10.0, altitude=60.0),
    ]

    low_risks = validate_altitude_path(service.project.map, drone, drone.planned_path)
    service.edit_waypoint(drone.id, 0, "altitude", 220.0)
    service.edit_waypoint(drone.id, 1, "altitude", 220.0)
    high_risks = validate_altitude_path(service.project.map, drone, drone.planned_path)

    assert low_risks
    assert high_risks == ()
    assert drone.planned_path == [Point(10.0, 10.0), Point(190.0, 10.0)]


def test_clearing_one_route_leaves_the_other_route_intact() -> None:
    service = ProjectService()
    service.add_base(Point(0.0, 0.0))
    first = service.add_drone(Point(10.0, 10.0))
    second = service.add_drone(Point(20.0, 20.0))
    first.waypoints = [Waypoint(10.0, 10.0, altitude=100.0), Waypoint(60.0, 30.0, altitude=100.0)]
    second.waypoints = [Waypoint(20.0, 20.0, altitude=100.0), Waypoint(80.0, 40.0, altitude=100.0)]

    service.clear_route(first.id)

    assert first.waypoints == []
    assert len(second.waypoints) == 2
    assert second.planned_path == [Point(20.0, 20.0), Point(80.0, 40.0)]
