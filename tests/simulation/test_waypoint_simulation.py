from __future__ import annotations

import pytest

from drone_mission_planner.domain.enums import (
    AltitudeMode,
    DroneStatus,
    WaypointAction,
)
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    SearchArea,
)
from drone_mission_planner.domain.terrain import TerrainModel, TerrainPeak, flat_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.events import EventType


def _map_with_route(
    waypoints: list[Waypoint],
    *,
    terrain: TerrainModel | None = None,
) -> tuple[MapModel, Drone]:
    model = MapModel(width=300, height=300, grid_size=10.0)
    model.terrain = terrain if terrain is not None else flat_terrain(altitude=0.0)
    model.bases.append(BaseStation("B-01", "Base", Point(20.0, 20.0)))
    drone = Drone(
        "D-01",
        "Alpha",
        Point(20.0, 20.0),
        "B-01",
        max_speed=10.0,
        cruise_altitude=40.0,
        min_clearance=30.0,
        planned_path=[waypoint.point for waypoint in waypoints],
        waypoints=waypoints,
    )
    model.drones.append(drone)
    return model, drone


def _run(engine: SimulationEngine, seconds: float) -> list[DroneStatus]:
    statuses: list[DroneStatus] = []
    steps = int(seconds / engine.fixed_dt)
    for _ in range(steps):
        engine.step_once()
        statuses.append(engine.runtimes["D-01"].status)
    return statuses


def test_altitude_follows_waypoint_profile_within_climb_rate() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=55.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    statuses = _run(engine, 30.0)
    runtime = engine.runtimes["D-01"]

    assert DroneStatus.CLIMBING in statuses
    assert runtime.altitude_gain == pytest.approx(15.0, abs=0.5)
    assert runtime.max_altitude == pytest.approx(55.0, abs=0.5)
    assert runtime.status == DroneStatus.COMPLETED


def test_hover_action_holds_position_and_consumes_energy() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=40.0, action=WaypointAction.HOVER, hold_seconds=4.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    statuses = _run(engine, 30.0)
    runtime = engine.runtimes["D-01"]

    assert DroneStatus.HOVERING in statuses
    assert runtime.waiting_time >= 4.0
    assert runtime.energy_used > 0.0
    assert runtime.status == DroneStatus.COMPLETED


def test_take_photo_records_event_and_pause() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=40.0, action=WaypointAction.TAKE_PHOTO),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    _run(engine, 30.0)
    runtime = engine.runtimes["D-01"]

    assert runtime.photos_taken == 1
    assert runtime.waiting_time >= 2.0
    photo_events = [
        record
        for record in engine.event_manager.history
        if record.event.event_type == EventType.WAYPOINT_PHOTO
    ]
    assert len(photo_events) == 1
    assert photo_events[0].event.target_id == "D-01"


def test_waypoint_speed_caps_the_leg() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=40.0, speed=4.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    _run(engine, 40.0)
    runtime = engine.runtimes["D-01"]

    # 100 m at the 4 m/s waypoint speed cap needs at least 25 s of flight.
    assert runtime.flight_time >= 25.0
    assert runtime.status == DroneStatus.COMPLETED


def test_landing_descends_to_terrain_and_records_event() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(220.0, 20.0, altitude=40.0, action=WaypointAction.LAND),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    statuses = _run(engine, 60.0)
    runtime = engine.runtimes["D-01"]

    assert DroneStatus.LANDING in statuses
    assert runtime.status == DroneStatus.COMPLETED
    assert runtime.current_altitude == pytest.approx(0.0, abs=0.1)
    assert runtime.altitude_loss >= 40.0
    landing_events = [
        record
        for record in engine.event_manager.history
        if record.event.event_type == EventType.WAYPOINT_LANDING
    ]
    assert len(landing_events) == 1


def test_coverage_updates_only_on_scan_legs() -> None:
    covered_model, _ = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=40.0, action=WaypointAction.SCAN),
            Waypoint(220.0, 20.0, altitude=40.0, action=WaypointAction.SCAN),
        ]
    )
    covered_model.search_areas.append(SearchArea("S-01", "Area", Rect(20.0, 20.0, 200.0, 40.0)))
    covered_engine = SimulationEngine(covered_model, fixed_dt=0.05)
    _run(covered_engine, 25.0)
    covered = covered_engine.coverage_monitor.snapshot()[0].covered_cells
    assert covered > 0

    plain_model, _ = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=40.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    plain_model.search_areas.append(SearchArea("S-01", "Area", Rect(20.0, 20.0, 200.0, 40.0)))
    plain_engine = SimulationEngine(plain_model, fixed_dt=0.05)
    _run(plain_engine, 25.0)
    plain = plain_engine.coverage_monitor.snapshot()[0].covered_cells
    assert plain == 0


def test_agl_waypoints_resolve_to_msl_over_terrain() -> None:
    terrain = flat_terrain(altitude=0.0)
    terrain.peaks.append(TerrainPeak(Point(120.0, 20.0), 60.0, 80.0))
    model, drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=50.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ],
        terrain=terrain,
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    runtime = engine.runtimes[drone.id]

    peak_altitude = model.terrain.altitude_at(120.0, 20.0)
    assert runtime.waypoint_altitudes[1] == pytest.approx(50.0)
    drone.waypoints[1].altitude_mode = AltitudeMode.AGL
    runtime.apply_waypoints(drone, runtime.path, model.terrain)
    assert runtime.waypoint_altitudes[1] == pytest.approx(peak_altitude + 50.0)


def test_altitude_extremes_are_tracked() -> None:
    model, _drone = _map_with_route(
        [
            Waypoint(20.0, 20.0, altitude=40.0),
            Waypoint(120.0, 20.0, altitude=60.0),
            Waypoint(220.0, 20.0, altitude=40.0),
        ]
    )
    engine = SimulationEngine(model, fixed_dt=0.05)
    _run(engine, 30.0)
    runtime = engine.runtimes["D-01"]

    assert runtime.max_altitude == pytest.approx(60.0, abs=0.5)
    assert runtime.min_clearance_seen is not None
    assert runtime.min_clearance_seen >= 40.0 - 0.5
