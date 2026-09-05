from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.waypoint import Waypoint, path_from_waypoints
from drone_mission_planner.persistence.mission_import import (
    MissionImportError,
    apply_import,
    load_geojson,
    load_kml,
    load_waypoint_csv,
)

GEOJSON = """{
  "type": "FeatureCollection",
  "features": [
    {"type": "Feature", "properties": {"name": "Ridge", "kind": "search_area"},
     "geometry": {"type": "Polygon", "coordinates": [[[100, 100], [300, 100], [300, 250], [100, 250], [100, 100]]]}},
    {"type": "Feature", "properties": {"name": "Tower zone", "kind": "no_fly_zone"},
     "geometry": {"type": "Polygon", "coordinates": [[[320, 120], [400, 120], [400, 200], [320, 120]]]}},
    {"type": "Feature", "properties": {"name": "Checkpoint", "kind": "task"},
     "geometry": {"type": "Point", "coordinates": [420, 300]}},
    {"type": "Feature", "properties": {"name": "Route A", "kind": "waypoints"},
     "geometry": {"type": "LineString", "coordinates": [[50, 50], [150, 80], [260, 120]]}}
  ]
}"""

KML = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark><name>Kml task</name><Point><coordinates>210.0,320.0,0.0</coordinates></Point></Placemark>
    <Placemark><name>Kml area</name>
      <Polygon><outerBoundaryIs><LinearRing><coordinates>
        120.0,300.0 260.0,300.0 260.0,380.0 120.0,380.0 120.0,300.0
      </coordinates></LinearRing></outerBoundaryIs></Polygon>
    </Placemark>
  </Document>
</kml>"""


def _service() -> ProjectService:
    service = ProjectService()
    service.add_base(Point(40.0, 40.0))
    return service


def test_geojson_imports_areas_zone_task_and_route(tmp_path: Path) -> None:
    path = tmp_path / "mission.geojson"
    path.write_text(GEOJSON, encoding="utf-8")
    service = _service()
    drone = service.add_drone(Point(50.0, 50.0))

    preview = load_geojson(service.project.map, path)

    assert len(preview.items) == 3
    assert preview.waypoint_route_name == "Route A"
    assert len(preview.waypoint_route) == 3
    assert preview.warnings == []

    created = apply_import(service, preview, drone_id=drone.id)

    assert len(created) == 4
    area = service.project.map.search_areas[0]
    assert area.name == "Ridge"
    assert len(area.points) == 5
    assert service.project.map.no_fly_zones[0].points[0] == Point(320.0, 120.0)
    assert service.project.map.drones[0].waypoints[2].point == Point(260.0, 120.0)
    assert service.project.map.drones[0].planned_path == path_from_waypoints(
        service.project.map.drones[0].waypoints
    )
    assert service.dirty


def test_kml_imports_point_and_polygon(tmp_path: Path) -> None:
    path = tmp_path / "mission.kml"
    path.write_text(KML, encoding="utf-8")
    service = _service()

    preview = load_kml(service.project.map, path)

    assert [item.kind for item in preview.items] == ["task", "search_area"]
    created = apply_import(service, preview)

    assert len(created) == 2
    assert service.project.map.tasks[0].position == Point(210.0, 320.0)
    assert len(service.project.map.search_areas[0].points) == 5


def test_waypoint_csv_imports_full_waypoint_columns(tmp_path: Path) -> None:
    path = tmp_path / "route.csv"
    path.write_text(
        "x,y,altitude,altitude_mode,speed,action,hold_seconds,task_id\n"
        "40,40,100,msl,8,fly_to,0,\n"
        "250,200,50,agl,,take_photo,2.5,T-01\n"
        "420,300,120,msl,,return_to_launch,0,\n",
        encoding="utf-8",
    )
    service = _service()
    drone = service.add_drone(Point(40.0, 40.0))

    preview = load_waypoint_csv(service.project.map, path)
    apply_import(service, preview, drone_id=drone.id)

    waypoints = service.project.map.drones[0].waypoints
    assert len(waypoints) == 3
    assert waypoints[1].altitude_mode == AltitudeMode.AGL
    assert waypoints[1].action == WaypointAction.TAKE_PHOTO
    assert waypoints[1].hold_seconds == 2.5
    assert waypoints[1].task_id == "T-01"
    assert waypoints[2].speed is None


def test_geojson_out_of_bounds_and_duplicates_warn(tmp_path: Path) -> None:
    path = tmp_path / "wide.geojson"
    path.write_text(
        """{"features": [
          {"properties": {"name": "Far"}, "geometry": {"type": "Point", "coordinates": [5000, 5000]}},
          {"properties": {"name": "Far"}, "geometry": {"type": "Point", "coordinates": [220, 210]}}
        ]}""",
        encoding="utf-8",
    )
    service = _service()

    preview = load_geojson(service.project.map, path)

    assert any("outside" in warning for warning in preview.warnings)
    assert any("Duplicate" in warning for warning in preview.warnings)
    assert len(preview.items) == 2


def test_waypoint_csv_errors_carry_line_numbers(tmp_path: Path) -> None:
    path = tmp_path / "broken.csv"
    path.write_text("40,40,100\nzz,60,70\n", encoding="utf-8")
    service = _service()

    with pytest.raises(MissionImportError, match="line 2"):
        load_waypoint_csv(service.project.map, path)


def test_waypoint_route_requires_target_drone(tmp_path: Path) -> None:
    path = tmp_path / "route.geojson"
    path.write_text(
        """{"features": [{"properties": {"name": "R", "kind": "waypoints"},
          "geometry": {"type": "LineString", "coordinates": [[10, 10], [80, 60]]}}]}""",
        encoding="utf-8",
    )
    service = _service()
    preview = load_geojson(service.project.map, path)

    with pytest.raises(MissionImportError, match="target drone"):
        apply_import(service, preview)


def test_replace_waypoints_rejects_out_of_bounds_and_reverts(tmp_path: Path) -> None:
    service = _service()
    drone = service.add_drone(Point(40.0, 40.0))
    service.dirty = False

    outside = [Waypoint(60.0, 60.0, altitude=80.0), Waypoint(2000.0, 2000.0, altitude=80.0)]
    with pytest.raises(ValueError):  # second waypoint is outside the map
        service.replace_waypoints(drone.id, outside)

    assert drone.waypoints == []
    assert not service.dirty
