from __future__ import annotations

import json

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import AltitudeMode, ObstacleShape
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.terrain import TerrainPeak, generate_mountain_terrain
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.ui.scene3d_export import (
    RISK_COLOR_CRITICAL,
    build_scene3d,
    terrain_color,
)

HEX_DIGITS = set("0123456789abcdef")


def _project_with_terrain() -> ProjectService:
    service = ProjectService()
    service.add_base(Point(80.0, 90.0))
    service.add_drone(Point(100.0, 110.0))
    service.add_task(Point(420.0, 260.0))
    service.project.map.terrain = generate_mountain_terrain(
        width=float(service.project.map.width),
        height=float(service.project.map.height),
        resolution=50.0,
        peaks=[TerrainPeak(Point(250.0, 200.0), 120.0, 140.0)],
    )
    return service


def test_terrain_mesh_covers_map_and_colors_match_gradient() -> None:
    service = _project_with_terrain()
    scene = build_scene3d(service.project.map)

    terrain = scene.terrain
    assert terrain.xs[0] == 0.0
    assert terrain.xs[-1] == service.project.map.width
    assert terrain.ys[0] == 0.0
    assert terrain.ys[-1] == service.project.map.height
    assert len(terrain.altitudes) == len(terrain.xs) * len(terrain.ys)
    assert len(terrain.cell_colors) == (len(terrain.xs) - 1) * (len(terrain.ys) - 1)
    assert max(terrain.altitudes) > 100.0
    for color in terrain.cell_colors:
        assert color.startswith("#") and len(color) == 7
        assert set(color[1:]) <= HEX_DIGITS


def test_terrain_color_matches_shared_gradient() -> None:
    assert terrain_color(0.0, 0.0, 100.0) == "#234b3a"
    assert terrain_color(100.0, 0.0, 100.0) == "#eef2f5"
    assert terrain_color(50.0, 0.0, 100.0) == terrain_color(50.0, 0.0, 100.0)


def test_routes_prefer_waypoint_altitudes_with_msl_and_agl_modes() -> None:
    service = _project_with_terrain()
    drone = service.project.map.drones[0]
    drone.waypoints = [
        Waypoint(100.0, 110.0, altitude=150.0),
        Waypoint(250.0, 200.0, altitude=20.0, altitude_mode=AltitudeMode.AGL),
    ]
    scene = build_scene3d(service.project.map)

    route = scene.routes[0]
    assert route.object_id == drone.id
    peak_altitude = service.project.map.terrain.altitude_at(250.0, 200.0)
    assert route.points[0][2] == 150.0
    assert route.points[1][2] == peak_altitude + 20.0
    assert scene.markers[0].object_id == drone.id


def test_route_falls_back_to_planned_path_with_clearance() -> None:
    service = _project_with_terrain()
    drone = service.project.map.drones[0]
    drone.cruise_altitude = 90.0
    drone.min_clearance = 25.0
    drone.planned_path = [Point(100.0, 110.0), Point(250.0, 200.0)]
    scene = build_scene3d(service.project.map)

    route = scene.routes[0]
    peak_altitude = service.project.map.terrain.altitude_at(250.0, 200.0)
    assert route.points[0][2] == 90.0
    assert route.points[1][2] == max(90.0, peak_altitude + 25.0)


def test_obstacle_and_no_fly_volumes_stand_on_terrain() -> None:
    service = _project_with_terrain()
    obstacle = service.add_obstacle(Rect(400.0, 100.0, 80.0, 60.0))
    obstacle.height = 75.0
    cylinder = service.add_obstacle(Rect(600.0, 100.0, 80.0, 80.0))
    cylinder.shape = ObstacleShape.CIRCLE
    cylinder.radius = 40.0
    cylinder.height = 50.0
    zone = service.add_no_fly_zone(Rect(300.0, 400.0, 90.0, 70.0), temporary=True)
    zone.ceiling_altitude = 210.0
    scene = build_scene3d(service.project.map)

    volumes = {volume.object_id: volume for volume in scene.volumes}
    assert len(volumes) == 3
    box = volumes[obstacle.id]
    assert len(box.footprint) == 4
    assert box.kind == "obstacle"
    assert box.top == box.bottom + 75.0
    prism = volumes[cylinder.id]
    assert len(prism.footprint) == 16
    zone_volume = volumes[zone.id]
    assert zone_volume.kind == "no_fly"
    terrain = service.project.map.terrain
    expected_top = (
        max(terrain.altitude_at(x, y) for x, y in zone_volume.footprint) + 210.0
    )
    assert zone_volume.top == expected_top


def test_crossing_a_tall_obstacle_marks_risk_segments() -> None:
    service = ProjectService()
    service.project.map.terrain = generate_mountain_terrain(
        width=float(service.project.map.width),
        height=float(service.project.map.height),
        resolution=50.0,
        peaks=[],
    )
    drone = service.add_drone(Point(50.0, 250.0))
    obstacle = service.add_obstacle(Rect(200.0, 220.0, 100.0, 60.0))
    obstacle.height = 200.0
    drone.planned_path = [Point(50.0, 250.0), Point(450.0, 250.0)]
    scene = build_scene3d(service.project.map)

    route = scene.routes[0]
    assert route.risk_colors
    assert RISK_COLOR_CRITICAL in route.risk_colors.values()


def test_live_positions_override_markers() -> None:
    service = _project_with_terrain()
    drone = service.project.map.drones[0]
    scene = build_scene3d(
        service.project.map,
        live_positions={drone.id: (Point(300.0, 180.0), 132.5)},
    )

    marker = scene.markers[0]
    assert (marker.x, marker.y, marker.z) == (300.0, 180.0, 132.5)


def test_coverage_cells_are_sampled_on_terrain() -> None:
    service = _project_with_terrain()
    area = service.add_search_area(Rect(200.0, 150.0, 220.0, 180.0))
    terrain = service.project.map.terrain
    covered = {area.id: (Point(210.0, 160.0), Point(300.0, 200.0))}
    uncovered = {area.id: (Point(400.0, 300.0),)}
    scene = build_scene3d(
        service.project.map,
        covered_cells=covered,
        uncovered_cells=uncovered,
        coverage_cell_size=18.0,
    )

    coverage = scene.coverage
    assert coverage is not None
    assert coverage.cell_size == 18.0
    assert coverage.covered[0][2] == terrain.altitude_at(210.0, 160.0) + 0.4
    assert coverage.uncovered[0][2] == terrain.altitude_at(400.0, 300.0) + 0.5
    assert build_scene3d(service.project.map).coverage is None


def test_wind_arrow_follows_wind_vector() -> None:
    service = _project_with_terrain()
    service.project.map.wind = WindModel(direction_to_deg=90.0, speed=6.0, enabled=True)
    scene = build_scene3d(service.project.map)

    assert scene.wind is not None
    assert scene.wind.dx > 0.0
    assert abs(scene.wind.dy) < 1e-9
    assert scene.wind.speed == 6.0


def test_scene_is_deterministic_and_json_serializable() -> None:
    service = _project_with_terrain()
    service.add_obstacle(Rect(400.0, 100.0, 80.0, 60.0))
    service.add_drone(Point(500.0, 120.0))
    first = build_scene3d(service.project.map)
    second = build_scene3d(service.project.map)

    assert first == second
    assert [route.object_id for route in first.routes] == sorted(
        route.object_id for route in first.routes
    )
    encoded = first.to_json_dict()
    assert json.dumps(encoded, sort_keys=True)
    assert encoded["width"] == service.project.map.width
    assert len(encoded["markers"]) == len(service.project.map.drones)
