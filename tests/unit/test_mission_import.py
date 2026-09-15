from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.georeference import (
    EnuCoordinate,
    GeoCoordinate,
    HeightDatum,
    HeightReference,
    ProjectGeoreference,
)
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.persistence.mission_import import (
    MissionImportError,
    apply_import,
    load_geojson,
    load_kml,
    load_waypoint_csv,
)
from drone_mission_planner.persistence.project_repository import ProjectRepository

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
    assert preview.source_metadata is not None
    assert service.project.data_sources == []
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
    assert service.project.map.drones[0].planned_path == [
        waypoint.point for waypoint in service.project.map.drones[0].waypoints
    ]
    assert service.project.data_sources[0].source_path == str(path)
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


def test_kml_import_converts_lon_lat_in_georeferenced_projects(tmp_path: Path) -> None:
    origin = GeoCoordinate(31.2304, 121.4737, 10.0)
    path = tmp_path / "mission.kml"
    path.write_text(
        """<kml><Placemark><Point><coordinates>121.4747496041,31.2303999957,10</coordinates></Point></Placemark></kml>""",
        encoding="utf-8",
    )
    service = _service()
    service.project.georeference = ProjectGeoreference.georeferenced(
        origin=origin,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
    )

    preview = load_kml(service.project.map, path, georeference=service.project.georeference)

    assert preview.items[0].points[0].x == pytest.approx(100.0, abs=1e-3)
    assert preview.items[0].points[0].y == pytest.approx(0.0, abs=1e-3)
    assert preview.source_metadata is not None
    assert preview.source_metadata.object_count == 1
    assert preview.source_metadata.source_crs == "EPSG:4326"
    assert preview.source_metadata.local_bounds is not None


def test_explicit_local_only_project_does_not_treat_kml_lon_lat_as_metres(tmp_path: Path) -> None:
    path = tmp_path / "mission.kml"
    path.write_text(
        "<kml><Placemark><Point><coordinates>121.4747,31.2304,10</coordinates></Point></Placemark></kml>",
        encoding="utf-8",
    )

    with pytest.raises(MissionImportError, match="georeferenced project"):
        load_kml(
            _service().project.map,
            path,
            georeference=ProjectGeoreference.local_only(),
        )


def test_geojson_preview_rejects_unsupported_crs(tmp_path: Path) -> None:
    path = tmp_path / "mercator.geojson"
    path.write_text(
        """{"type": "FeatureCollection",
          "crs": {"type": "name", "properties": {"name": "EPSG:3857"}},
          "features": [{"properties": {"name": "P", "kind": "task"},
          "geometry": {"type": "Point", "coordinates": [0, 0]}}]}""",
        encoding="utf-8",
    )

    with pytest.raises(MissionImportError, match="expected EPSG:4326"):
        load_geojson(_service().project.map, path)


def test_geojson_preview_rejects_illegal_geographic_coordinates(tmp_path: Path) -> None:
    path = tmp_path / "bad-lat.geojson"
    path.write_text(
        """{"type": "FeatureCollection",
          "features": [{"properties": {"name": "Bad", "kind": "task"},
          "geometry": {"type": "Point", "coordinates": [121.47, 95.0]}}]}""",
        encoding="utf-8",
    )
    service = _service()
    service.project.georeference = ProjectGeoreference.georeferenced(
        origin=GeoCoordinate(31.2304, 121.4737, 10.0),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
    )

    with pytest.raises(MissionImportError, match="Invalid geographic coordinate"):
        load_geojson(service.project.map, path, georeference=service.project.georeference)


def test_geojson_preview_reports_georeference_range_violations(tmp_path: Path) -> None:
    origin = GeoCoordinate(31.2304, 121.4737, 10.0)
    far = ProjectGeoreference.georeferenced(
        origin=origin,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        valid_radius_m=20.0,
    )
    point = far.local_adapter().enu_to_geodetic(
        EnuCoordinate(80.0, 0.0, 0.0)
    )
    path = tmp_path / "far.geojson"
    path.write_text(
        f"""{{"type": "FeatureCollection",
          "features": [{{"properties": {{"name": "Far", "kind": "task"}},
          "geometry": {{"type": "Point", "coordinates": [{point.longitude_deg}, {point.latitude_deg}]}}}}]}}""",
        encoding="utf-8",
    )
    service = _service()

    preview = load_geojson(service.project.map, path, georeference=far)

    assert any("outside_valid_radius" in warning for warning in preview.warnings)
    assert preview.source_metadata is not None
    assert preview.source_metadata.validation_status == "unverifiable"


def test_kml_non_numeric_coordinates_raise_import_error(tmp_path: Path) -> None:
    path = tmp_path / "broken.kml"
    path.write_text(
        "<kml><Placemark><Point><coordinates>bad,20</coordinates></Point></Placemark></kml>",
        encoding="utf-8",
    )

    with pytest.raises(MissionImportError, match="numeric"):
        load_kml(_service().project.map, path)


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
    assert service.project.data_sources == []


def test_replace_route_rejects_out_of_bounds_and_reverts(tmp_path: Path) -> None:
    service = _service()
    drone = service.add_drone(Point(40.0, 40.0))
    service.dirty = False

    outside = [Waypoint(60.0, 60.0, altitude=80.0), Waypoint(2000.0, 2000.0, altitude=80.0)]
    with pytest.raises(ValueError):  # second waypoint is outside the map
        service.replace_route(drone.id, outside)

    assert drone.waypoints == []
    assert not service.dirty


def test_geojson_polygon_holes_apply_and_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "holes.geojson"
    path.write_text(
        """{"type": "FeatureCollection", "features": [{
          "id": "parcel-7",
          "properties": {"name": "Parcel", "kind": "search_area", "owner": "survey"},
          "geometry": {"type": "Polygon", "coordinates": [
            [[100,100], [400,100], [400,400], [100,400], [100,100]],
            [[150,150], [200,150], [200,200], [150,200], [150,150]],
            [[300,250], [350,250], [350,300], [300,300], [300,250]]
          ]}
        }]}""",
        encoding="utf-8",
    )
    service = _service()

    preview = load_geojson(service.project.map, path)
    created = apply_import(service, preview)

    assert created == ["S-01"]
    area = service.project.map.search_areas[0]
    assert len(area.points) == 5
    assert [[point for point in hole] for hole in area.holes] == [
        [Point(150.0, 150.0), Point(200.0, 150.0), Point(200.0, 200.0), Point(150.0, 200.0), Point(150.0, 150.0)],
        [Point(300.0, 250.0), Point(350.0, 250.0), Point(350.0, 300.0), Point(300.0, 300.0), Point(300.0, 250.0)],
    ]
    assert preview.items[0].source_feature_id == "parcel-7"
    assert preview.items[0].properties["owner"] == "survey"

    saved = tmp_path / "holes.dmproj"
    ProjectRepository().save(service.project, saved)
    loaded = ProjectRepository().load(saved)

    assert len(loaded.map.search_areas[0].holes) == 2
    assert loaded.data_sources[0].source_path == str(path)


def test_geojson_multipolygon_preserves_source_id_and_part_indexes(tmp_path: Path) -> None:
    path = tmp_path / "multi.geojson"
    path.write_text(
        """{"features": [{
          "id": "mp-1",
          "properties": {"name": "Split area", "kind": "search_area"},
          "geometry": {"type": "MultiPolygon", "coordinates": [
            [[[10,10], [80,10], [80,80], [10,80], [10,10]]],
            [[[200,200], [260,200], [260,260], [200,260], [200,200]]]
          ]}
        }]}""",
        encoding="utf-8",
    )
    service = _service()

    preview = load_geojson(service.project.map, path)
    apply_import(service, preview)

    assert [(item.source_feature_id, item.part_index) for item in preview.items] == [
        ("mp-1", 0),
        ("mp-1", 1),
    ]
    assert [area.name for area in service.project.map.search_areas] == [
        "Split area part 1",
        "Split area part 2",
    ]


def test_geojson_multiline_keeps_independent_route_parts(tmp_path: Path) -> None:
    path = tmp_path / "routes.geojson"
    path.write_text(
        """{"features": [{
          "id": "route-source",
          "properties": {"name": "Survey legs", "kind": "waypoints"},
          "geometry": {"type": "MultiLineString", "coordinates": [
            [[40,40], [80,40]],
            [[300,300], [340,300]]
          ]}
        }]}""",
        encoding="utf-8",
    )
    service = _service()
    drone = service.add_drone(Point(40.0, 40.0))
    service.dirty = False

    preview = load_geojson(service.project.map, path)

    assert len(preview.route_parts) == 2
    assert preview.route_parts[0].waypoints[-1].point == Point(80.0, 40.0)
    assert preview.route_parts[1].waypoints[0].point == Point(300.0, 300.0)
    with pytest.raises(MissionImportError, match="multiple independent route parts"):
        apply_import(service, preview, drone_id=drone.id)
    assert service.project.map.drones[0].waypoints == []
    assert service.project.data_sources == []
    assert not service.dirty

    apply_import(service, preview, drone_id=drone.id, route_part_index=1)

    assert [waypoint.point for waypoint in service.project.map.drones[0].waypoints] == [
        Point(300.0, 300.0),
        Point(340.0, 300.0),
    ]


def test_kml_polygon_preserves_inner_boundaries(tmp_path: Path) -> None:
    path = tmp_path / "holes.kml"
    path.write_text(
        """<kml><Placemark><name>KML holes</name><Polygon>
          <outerBoundaryIs><LinearRing><coordinates>
            100,100 300,100 300,300 100,300 100,100
          </coordinates></LinearRing></outerBoundaryIs>
          <innerBoundaryIs><LinearRing><coordinates>
            140,140 180,140 180,180 140,180 140,140
          </coordinates></LinearRing></innerBoundaryIs>
        </Polygon></Placemark></kml>""",
        encoding="utf-8",
    )
    service = _service()

    preview = load_kml(service.project.map, path)
    apply_import(service, preview)

    assert len(service.project.map.search_areas[0].holes) == 1
    assert service.project.map.search_areas[0].holes[0][0] == Point(140.0, 140.0)


def test_geojson_topology_issues_block_apply_transactionally(tmp_path: Path) -> None:
    path = tmp_path / "bad-topology.geojson"
    path.write_text(
        """{"features": [
          {"id": "open", "properties": {"name": "Open"}, "geometry": {"type": "Polygon",
            "coordinates": [[[10,10], [80,10], [80,80], [10,80]]]}},
          {"id": "zero", "properties": {"name": "Zero"}, "geometry": {"type": "Polygon",
            "coordinates": [[[100,100], [150,100], [200,100], [100,100]]]}},
          {"id": "self", "properties": {"name": "Self"}, "geometry": {"type": "Polygon",
            "coordinates": [[[300,100], [360,160], [300,160], [360,100], [300,100]]]}},
          {"id": "hole-out", "properties": {"name": "Hole out"}, "geometry": {"type": "Polygon",
            "coordinates": [
              [[100,300], [240,300], [240,440], [100,440], [100,300]],
              [[220,420], [280,420], [280,480], [220,480], [220,420]]
            ]}},
          {"id": "holes-cross", "properties": {"name": "Holes cross"}, "geometry": {"type": "Polygon",
            "coordinates": [
              [[400,300], [650,300], [650,550], [400,550], [400,300]],
              [[430,330], [540,330], [540,440], [430,440], [430,330]],
              [[500,390], [610,390], [610,500], [500,500], [500,390]]
            ]}},
          {"id": "dupe", "properties": {"name": "Duplicate vertex"}, "geometry": {"type": "LineString",
            "coordinates": [[40,40], [40,40], [90,40]]}}
        ]}""",
        encoding="utf-8",
    )
    service = ProjectService()
    service.dirty = False

    preview = load_geojson(service.project.map, path)
    codes = {issue.code for issue in preview.issues}

    assert {
        "ring_not_closed",
        "zero_area",
        "self_intersection",
        "hole_outside_outer",
        "holes_overlap",
        "consecutive_duplicate_vertex",
    } <= codes
    assert preview.source_metadata is not None
    assert preview.source_metadata.validation_status == "unverifiable"
    with pytest.raises(MissionImportError, match="blocking topology issue"):
        apply_import(service, preview)

    assert service.project.map.search_areas == []
    assert service.project.data_sources == []
    assert service.undo_label is None
    assert not service.dirty
