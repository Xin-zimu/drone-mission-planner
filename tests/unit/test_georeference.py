from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.data_source import (
    DataSourceKind,
    DataSourceMetadata,
    SourceBounds,
)
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.georeference import (
    WGS84_GEOGRAPHIC_2D_CRS,
    CalibrationControlPoint,
    CrsCoordinateAdapter,
    EnuCoordinate,
    GeoCoordinate,
    Geofence,
    GeofenceKind,
    GeoreferenceError,
    GeoreferenceValidationStatus,
    HeightDatum,
    HeightReference,
    LocalTangentPlaneAdapter,
    ProjectGeoreference,
    ProjectGeoreferenceMode,
    SpatialBounds,
)
from drone_mission_planner.domain.models import ProjectModel
from drone_mission_planner.persistence.migrations import CURRENT_PROJECT_VERSION, migrate_project
from drone_mission_planner.persistence.project_repository import ProjectRepository


def _shanghai_origin() -> GeoCoordinate:
    return GeoCoordinate(latitude_deg=31.2304, longitude_deg=121.4737, height_m=10.0)


def test_origin_maps_to_zero_enu() -> None:
    adapter = LocalTangentPlaneAdapter(_shanghai_origin())

    enu = adapter.geodetic_to_enu(_shanghai_origin())

    assert enu.east_m == pytest.approx(0.0, abs=1e-6)
    assert enu.north_m == pytest.approx(0.0, abs=1e-6)
    assert enu.up_m == pytest.approx(0.0, abs=1e-6)


def test_enu_axes_are_east_north_up() -> None:
    adapter = LocalTangentPlaneAdapter(_shanghai_origin())

    east_geo = adapter.enu_to_geodetic(EnuCoordinate(east_m=100.0, north_m=0.0, up_m=0.0))
    north_geo = adapter.enu_to_geodetic(EnuCoordinate(east_m=0.0, north_m=100.0, up_m=0.0))
    east_back = adapter.geodetic_to_enu(east_geo)
    north_back = adapter.geodetic_to_enu(north_geo)

    assert east_geo.longitude_deg > _shanghai_origin().longitude_deg
    assert east_back.east_m == pytest.approx(100.0, abs=1e-4)
    assert east_back.north_m == pytest.approx(0.0, abs=1e-4)
    assert north_geo.latitude_deg > _shanghai_origin().latitude_deg
    assert north_back.east_m == pytest.approx(0.0, abs=1e-4)
    assert north_back.north_m == pytest.approx(100.0, abs=1e-4)


def test_geodetic_enu_round_trip_is_stable_for_fixed_reference_data() -> None:
    adapter = LocalTangentPlaneAdapter(_shanghai_origin())
    coordinate = GeoCoordinate(latitude_deg=31.2311, longitude_deg=121.4752, height_m=45.0)

    enu = adapter.geodetic_to_enu(coordinate)
    restored = adapter.enu_to_geodetic(enu)

    assert restored.latitude_deg == pytest.approx(coordinate.latitude_deg, abs=1e-9)
    assert restored.longitude_deg == pytest.approx(coordinate.longitude_deg, abs=1e-9)
    assert restored.height_m == pytest.approx(coordinate.height_m, abs=1e-4)


def test_ecef_round_trip_is_stable_for_fixed_reference_data() -> None:
    adapter = LocalTangentPlaneAdapter(_shanghai_origin())
    coordinate = GeoCoordinate(latitude_deg=31.2311, longitude_deg=121.4752, height_m=45.0)

    ecef = adapter.geodetic_to_ecef(coordinate)
    restored = adapter.ecef_to_geodetic(ecef)

    assert restored.latitude_deg == pytest.approx(coordinate.latitude_deg, abs=1e-9)
    assert restored.longitude_deg == pytest.approx(coordinate.longitude_deg, abs=1e-9)
    assert restored.height_m == pytest.approx(coordinate.height_m, abs=1e-4)


def test_projection_adapter_uses_explicit_crs_order() -> None:
    forward = CrsCoordinateAdapter(WGS84_GEOGRAPHIC_2D_CRS, "EPSG:3857")
    inverse = CrsCoordinateAdapter("EPSG:3857", WGS84_GEOGRAPHIC_2D_CRS)

    web_mercator = forward.transform_xy(121.4737, 31.2304)
    restored = inverse.transform_xy(web_mercator.x, web_mercator.y)

    assert restored.x == pytest.approx(121.4737, abs=1e-9)
    assert restored.y == pytest.approx(31.2304, abs=1e-9)


def test_invalid_coordinates_and_crs_are_rejected() -> None:
    with pytest.raises(GeoreferenceError, match="latitude"):
        GeoCoordinate(latitude_deg=91.0, longitude_deg=0.0)
    with pytest.raises(GeoreferenceError, match="longitude"):
        GeoCoordinate(latitude_deg=0.0, longitude_deg=181.0)
    with pytest.raises(GeoreferenceError, match="invalid CRS"):
        CrsCoordinateAdapter("not-a-crs", WGS84_GEOGRAPHIC_2D_CRS)


def test_project_georeference_blocks_real_export_until_height_is_known() -> None:
    local = ProjectGeoreference.local_only()
    unknown_height = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(),
    )
    known_height = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        validation_status=GeoreferenceValidationStatus.VALIDATED,
    )
    unvalidated = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
    )

    assert not local.can_export_real_coordinates
    assert not unknown_height.can_export_real_coordinates
    assert not unvalidated.can_export_real_coordinates
    assert known_height.can_export_real_coordinates
    with pytest.raises(GeoreferenceError, match="local-only"):
        local.require_ready_for_real_export()
    with pytest.raises(GeoreferenceError, match="height datum is unknown"):
        unknown_height.require_ready_for_real_export()
    with pytest.raises(GeoreferenceError, match="not validated"):
        unvalidated.require_ready_for_real_export()
    known_height.require_ready_for_real_export()


def test_project_georeference_converts_local_points_through_enu() -> None:
    georeference = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
    )

    coordinate = georeference.local_to_geodetic(Point(25.0, 40.0), up_m=5.0)
    restored = georeference.geodetic_to_local(coordinate)

    assert restored.x == pytest.approx(25.0, abs=1e-4)
    assert restored.y == pytest.approx(40.0, abs=1e-4)


def test_control_point_residuals_are_queryable() -> None:
    origin = _shanghai_origin()
    adapter = LocalTangentPlaneAdapter(origin)
    observed = adapter.enu_to_geodetic(EnuCoordinate(100.5, 40.25, 12.0))
    georeference = ProjectGeoreference.georeferenced(
        origin=origin,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        control_points=(
            CalibrationControlPoint(
                "CP-01",
                "control",
                EnuCoordinate(100.0, 40.0, 10.0),
                observed,
            ),
        ),
    )

    report = georeference.calibration_report(horizontal_tolerance_m=1.0, vertical_tolerance_m=3.0)

    assert report.within_tolerance
    assert len(report.residuals) == 1
    assert report.residuals[0].east_error_m == pytest.approx(0.5, abs=1e-4)
    assert report.residuals[0].north_error_m == pytest.approx(0.25, abs=1e-4)
    assert report.residuals[0].up_error_m == pytest.approx(2.0, abs=1e-4)


def test_control_point_report_warns_when_residual_exceeds_tolerance() -> None:
    origin = _shanghai_origin()
    observed = LocalTangentPlaneAdapter(origin).enu_to_geodetic(EnuCoordinate(8.0, 0.0, 0.0))
    georeference = ProjectGeoreference.georeferenced(
        origin=origin,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        control_points=(
            CalibrationControlPoint("CP-01", "control", EnuCoordinate(0.0, 0.0), observed),
        ),
    )

    report = georeference.calibration_report(horizontal_tolerance_m=1.0)

    assert not report.within_tolerance
    assert "horizontal residual" in report.warnings[0]


def test_spatial_constraints_report_radius_bounds_and_geofence_violations() -> None:
    georeference = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        valid_radius_m=50.0,
        spatial_bounds=SpatialBounds(-20.0, -20.0, 80.0, 80.0),
        geofences=(
            Geofence(
                "GF-01",
                "allowed",
                polygon=(Point(-10.0, -10.0), Point(60.0, -10.0), Point(60.0, 60.0)),
            ),
            Geofence(
                "GF-02",
                "blocked",
                kind=GeofenceKind.EXCLUSION,
                center=Point(5.0, 5.0),
                radius_m=10.0,
            ),
        ),
    )

    violations = georeference.spatial_violations(
        (EnuCoordinate(5.0, 5.0), EnuCoordinate(100.0, 100.0))
    )

    assert {violation.code for violation in violations} >= {
        "inside_exclusion_geofence",
        "outside_valid_radius",
        "outside_spatial_bounds",
        "outside_inclusion_geofence",
    }


def test_reanchoring_changes_local_coordinates_but_preserves_real_positions() -> None:
    origin = _shanghai_origin()
    georeference = ProjectGeoreference.georeferenced(
        origin=origin,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        control_points=(
            CalibrationControlPoint(
                "CP-01",
                "control",
                EnuCoordinate(40.0, 20.0, 0.0),
                LocalTangentPlaneAdapter(origin).enu_to_geodetic(EnuCoordinate(40.0, 20.0)),
            ),
        ),
        validation_status=GeoreferenceValidationStatus.VALIDATED,
    )
    true_coordinate = georeference.local_adapter().enu_to_geodetic(
        EnuCoordinate(100.0, 50.0, 0.0)
    )
    new_origin = georeference.local_adapter().enu_to_geodetic(EnuCoordinate(20.0, 10.0, 0.0))

    reanchored = georeference.with_origin(new_origin)
    new_local = reanchored.geodetic_to_local(true_coordinate)

    assert new_local.x != pytest.approx(100.0)
    assert new_local.y != pytest.approx(50.0)
    assert reanchored.local_to_geodetic(new_local).latitude_deg == pytest.approx(
        true_coordinate.latitude_deg,
        abs=1e-9,
    )
    assert reanchored.validation_status == GeoreferenceValidationStatus.STALE
    assert reanchored.control_points[0].local.east_m != pytest.approx(40.0)


def test_georeference_round_trip_is_persisted(tmp_path: Path) -> None:
    project = ProjectModel(name="Referenced")
    project.georeference = ProjectGeoreference.georeferenced(
        origin=_shanghai_origin(),
        height_reference=HeightReference(
            HeightDatum.ORTHOMETRIC,
            source="survey",
            geoid_model="EGM2008",
        ),
        valid_radius_m=2500.0,
        control_points=(
            CalibrationControlPoint(
                "CP-01",
                "control",
                EnuCoordinate(0.0, 0.0, 0.0),
                _shanghai_origin(),
            ),
        ),
        spatial_bounds=SpatialBounds(-100.0, -100.0, 100.0, 100.0),
        geofences=(Geofence("GF-01", "allowed", center=Point(0.0, 0.0), radius_m=100.0),),
        validation_status=GeoreferenceValidationStatus.VALIDATED,
    )
    project.data_sources.append(
        DataSourceMetadata(
            id="geojson:demo",
            kind=DataSourceKind.GEOJSON,
            source_path="fixtures/demo.geojson",
            source_crs="EPSG:4326",
            source_bounds=SourceBounds(121.4737, 31.2304, 121.4747, 31.2314),
            local_bounds=SpatialBounds(0.0, 0.0, 100.0, 110.0),
            horizontal_units="degree",
            vertical_units="unknown",
            object_count=2,
            sha256="abc123",
            revision="rev1",
        )
    )
    path = tmp_path / "referenced.dmproj"

    ProjectRepository().save(project, path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    loaded = ProjectRepository().load(path)

    assert raw["version"] == CURRENT_PROJECT_VERSION
    assert raw["data_sources"][0]["source_path"] == "fixtures/demo.geojson"
    assert raw["georeference"]["mode"] == ProjectGeoreferenceMode.GEOREFERENCED
    assert raw["georeference"]["origin"]["latitude_deg"] == pytest.approx(31.2304)
    assert loaded.georeference.mode == ProjectGeoreferenceMode.GEOREFERENCED
    assert loaded.georeference.origin == _shanghai_origin()
    assert loaded.georeference.height_reference.datum == HeightDatum.ORTHOMETRIC
    assert loaded.georeference.height_reference.geoid_model == "EGM2008"
    assert loaded.georeference.valid_radius_m == pytest.approx(2500.0)
    assert loaded.georeference.control_points[0].id == "CP-01"
    assert loaded.georeference.spatial_bounds is not None
    assert loaded.georeference.geofences[0].id == "GF-01"
    assert loaded.georeference.validation_status == GeoreferenceValidationStatus.VALIDATED
    assert loaded.data_sources[0].kind == DataSourceKind.GEOJSON
    assert loaded.data_sources[0].source_crs == "EPSG:4326"
    assert loaded.data_sources[0].local_bounds is not None


def test_v19_migration_keeps_legacy_coordinates_local_only() -> None:
    raw = {
        "version": "1.9",
        "name": "Legacy local map",
        "equipment": None,
        "planning_settings": {},
        "simulation_settings": {"fixed_dt": 0.05},
        "map": {},
    }

    migrated = migrate_project(raw)

    assert migrated["version"] == CURRENT_PROJECT_VERSION
    assert migrated["georeference"]["mode"] == ProjectGeoreferenceMode.LOCAL_ONLY
    assert migrated["georeference"]["origin"] is None
    assert migrated["georeference"]["height_reference"]["datum"] == HeightDatum.UNKNOWN
    assert migrated["georeference"]["control_points"] == []
    assert migrated["georeference"]["geofences"] == []
    assert migrated["data_sources"] == []
