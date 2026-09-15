from __future__ import annotations

import struct
from dataclasses import replace
from pathlib import Path

import pytest
from pyproj import Transformer

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.georeference import (
    GeoCoordinate,
    GeoreferenceValidationStatus,
    HeightDatum,
    HeightReference,
    ProjectGeoreference,
)
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.persistence.project_repository import ProjectRepository
from drone_mission_planner.persistence.route_export import RouteExportError, export_route_json
from drone_mission_planner.persistence.terrain_import import (
    TerrainImportError,
    apply_geotiff_dem_import,
    load_geotiff_dem,
    load_geotiff_dem_preview,
    load_terrain_csv,
)


def test_load_terrain_csv_builds_grid_preview_and_model(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "10,20,100\n"
        "20,20,110\n"
        "10,30,120\n"
        "20,30,140\n",
        encoding="utf-8",
    )

    result = load_terrain_csv(path)

    assert result.preview.sample_count == 4
    assert result.preview.grid_width == 2
    assert result.preview.grid_height == 2
    assert result.preview.min_elevation == 100.0
    assert result.preview.max_elevation == 140.0
    assert result.preview.resolution == 10.0
    assert result.terrain.terrain_type == "grid"
    assert result.terrain.altitude_at(15.0, 25.0) == pytest.approx(117.5)
    assert "4 samples" in result.preview.summary()


def test_load_terrain_csv_rejects_missing_columns(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text("x,z\n0,10\n", encoding="utf-8")

    with pytest.raises(TerrainImportError, match="missing required column"):
        load_terrain_csv(path)


def test_load_terrain_csv_reports_non_numeric_line_and_column(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,not-a-number\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="line 3 column elevation"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_duplicate_samples(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "0,0,20\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="duplicates elevation sample"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_incomplete_grid(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,20\n"
        "0,10,30\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="complete regular grid"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_irregular_spacing(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,20\n"
        "25,0,30\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="x values must form a regular grid"):
        load_terrain_csv(path)


def test_load_geotiff_dem_preview_reports_metadata_and_unknown_height(
    tmp_path: Path,
) -> None:
    path = tmp_path / "dem.tif"
    _write_minimal_geotiff(path)
    georeference = ProjectGeoreference.georeferenced(
        origin=GeoCoordinate(31.2304, 121.4737, 10.0),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        valid_radius_m=1000.0,
        validation_status=GeoreferenceValidationStatus.VALIDATED,
    )

    preview = load_geotiff_dem_preview(path, georeference=georeference)

    assert preview.raster_width == 3
    assert preview.raster_height == 2
    assert preview.band_count == 1
    assert preview.dtype == "float32"
    assert preview.source_crs == "EPSG:4326"
    assert preview.source_bounds is not None
    assert preview.local_bounds is not None
    assert preview.resolution_x_m is not None
    assert preview.resolution_x_m > 0.5
    assert preview.min_elevation_m == pytest.approx(10.0)
    assert preview.max_elevation_m == pytest.approx(16.0)
    assert preview.height_reference.datum == HeightDatum.UNKNOWN
    assert any("height datum is unknown" in warning for warning in preview.warnings)
    assert preview.source_metadata.sha256
    assert preview.source_metadata.elevation_min_m == pytest.approx(10.0)


def test_load_geotiff_dem_preview_reports_georeference_range_violation(
    tmp_path: Path,
) -> None:
    path = tmp_path / "dem.tif"
    _write_minimal_geotiff(path)
    georeference = ProjectGeoreference.georeferenced(
        origin=GeoCoordinate(31.2304, 121.4737, 10.0),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        valid_radius_m=0.25,
    )

    preview = load_geotiff_dem_preview(path, georeference=georeference)

    assert any("outside_valid_radius" in warning for warning in preview.warnings)


def test_load_geotiff_dem_applies_grid_with_nodata_mask(tmp_path: Path) -> None:
    path = tmp_path / "projected-dem.tif"
    georeference = _validated_georeference()
    _write_projected_geotiff(path)

    result = load_geotiff_dem(
        path,
        georeference=georeference,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="survey"),
    )

    terrain = result.terrain
    assert terrain.terrain_type == "grid"
    assert terrain.source_data_id == result.preview.source_metadata.id
    assert terrain.grid_width == 3
    assert terrain.grid_height == 2
    assert any(not valid for row in terrain.grid_valid_mask for valid in row)
    assert terrain.grid_origin is not None
    assert terrain.sample_at(terrain.grid_origin.x, terrain.grid_origin.y).valid

    nodata_row, nodata_column = next(
        (row_index, column_index)
        for row_index, row in enumerate(terrain.grid_valid_mask)
        for column_index, valid in enumerate(row)
        if not valid
    )
    nodata = terrain.sample_at(
        terrain.grid_origin.x + nodata_column * terrain.resolution,
        terrain.grid_origin.y + nodata_row * terrain.resolution,
    )
    assert not nodata.valid
    assert "NoData" in nodata.reason

    outside = terrain.sample_at(terrain.grid_origin.x - terrain.resolution, terrain.grid_origin.y)
    assert not outside.valid
    assert "outside" in outside.reason


def test_apply_geotiff_dem_records_metadata_and_round_trips(tmp_path: Path) -> None:
    path = tmp_path / "projected-dem.tif"
    _write_projected_geotiff(path)
    service = ProjectService()
    service.project.georeference = _validated_georeference()
    result = load_geotiff_dem(
        path,
        georeference=service.project.georeference,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="survey"),
    )

    metadata = apply_geotiff_dem_import(service, result)

    assert metadata.validation_status == "validated"
    assert service.project.map.terrain.source_data_id == metadata.id
    assert service.project.data_sources[0].source_path == str(path)
    assert service.dirty

    saved = tmp_path / "dem.dmproj"
    ProjectRepository().save(service.project, saved)
    loaded = ProjectRepository().load(saved)

    assert loaded.map.terrain.source_data_id == metadata.id
    assert any(not valid for row in loaded.map.terrain.grid_valid_mask for valid in row)
    assert loaded.data_sources[0].validation_status == "validated"


def test_apply_geotiff_dem_failure_is_transactional(tmp_path: Path) -> None:
    path = tmp_path / "projected-dem.tif"
    _write_projected_geotiff(path)
    service = ProjectService()
    service.project.georeference = _validated_georeference()
    service.dirty = False
    result = load_geotiff_dem(
        path,
        georeference=service.project.georeference,
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="survey"),
    )
    broken = replace(result, terrain=TerrainModel(resolution=0.0))

    with pytest.raises(ValueError, match="terrain resolution"):
        apply_geotiff_dem_import(service, broken)

    assert service.project.map.terrain.terrain_type == "flat"
    assert service.project.data_sources == []
    assert service.undo_label is None
    assert not service.dirty


def test_load_geotiff_dem_rejects_unknown_height_datum(tmp_path: Path) -> None:
    path = tmp_path / "projected-dem.tif"
    _write_projected_geotiff(path)

    with pytest.raises(TerrainImportError, match="known height datum"):
        load_geotiff_dem(
            path,
            georeference=_validated_georeference(),
            height_reference=HeightReference(),
        )


def test_dem_nodata_blocks_route_export(tmp_path: Path) -> None:
    path = tmp_path / "projected-dem.tif"
    _write_projected_geotiff(path)
    result = load_geotiff_dem(
        path,
        georeference=_validated_georeference(),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="survey"),
    )
    terrain = result.terrain
    assert terrain.grid_origin is not None
    nodata_row, nodata_column = next(
        (row_index, column_index)
        for row_index, row in enumerate(terrain.grid_valid_mask)
        for column_index, valid in enumerate(row)
        if not valid
    )
    nodata_point = (
        terrain.grid_origin.x + nodata_column * terrain.resolution,
        terrain.grid_origin.y + nodata_row * terrain.resolution,
    )
    model = MapModel(width=1000, height=1000)
    model.terrain = terrain
    model.bases.append(BaseStation("B-01", "Base", terrain.grid_origin, communication_range=1000.0))
    drone = Drone(
        "D-01",
        "Drone",
        terrain.grid_origin,
        home_base_id="B-01",
        battery_capacity=1000.0,
        remaining_battery=1000.0,
        communication_range=1000.0,
        waypoints=[
            Waypoint(terrain.grid_origin.x, terrain.grid_origin.y, altitude=100.0),
            Waypoint(nodata_point[0], nodata_point[1], altitude=100.0),
        ],
    )
    model.drones.append(drone)

    with pytest.raises(RouteExportError, match="terrain elevation is unknown"):
        export_route_json(model, drone, tmp_path / "route.json")


def _write_minimal_geotiff(path: Path) -> None:
    width = 3
    height = 2
    samples = [10.0, 12.0, -9999.0, 14.0, 16.0, 15.0]
    sample_data = struct.pack("<" + "f" * len(samples), *samples)
    external_values = {
        33550: struct.pack("<ddd", 0.00001, 0.00001, 0.0),
        33922: struct.pack("<dddddd", 0.0, 0.0, 0.0, 121.4737, 31.2304, 0.0),
        34735: struct.pack(
            "<" + "H" * 16,
            1,
            1,
            0,
            3,
            1024,
            0,
            1,
            2,
            1025,
            0,
            1,
            1,
            2048,
            0,
            1,
            4326,
        ),
        42113: b"-9999\0",
    }
    tag_count = 14
    external_offset = 8 + 2 + tag_count * 12 + 4
    value_offsets: dict[int, int] = {}
    cursor = external_offset
    for tag, payload in external_values.items():
        value_offsets[tag] = cursor
        cursor += len(payload)
        if cursor % 2:
            cursor += 1
    strip_offset = cursor

    def entry(tag: int, tag_type: int, count: int, value: int) -> bytes:
        type_size = {2: 1, 3: 2, 4: 4, 12: 8}[tag_type]
        if count * type_size <= 4:
            if tag_type == 3:
                raw = struct.pack("<H", value).ljust(4, b"\0")
            elif tag_type == 4:
                raw = struct.pack("<I", value)
            else:
                raw = bytes(value).ljust(4, b"\0")
        else:
            raw = struct.pack("<I", value)
        return struct.pack("<HHI", tag, tag_type, count) + raw

    entries = [
        entry(256, 4, 1, width),
        entry(257, 4, 1, height),
        entry(258, 3, 1, 32),
        entry(259, 3, 1, 1),
        entry(262, 3, 1, 1),
        entry(273, 4, 1, strip_offset),
        entry(277, 3, 1, 1),
        entry(278, 4, 1, height),
        entry(279, 4, 1, len(sample_data)),
        entry(339, 3, 1, 3),
        entry(33550, 12, 3, value_offsets[33550]),
        entry(33922, 12, 6, value_offsets[33922]),
        entry(34735, 3, 16, value_offsets[34735]),
        entry(42113, 2, 6, value_offsets[42113]),
    ]
    tag_count = len(entries)
    header = b"II" + struct.pack("<HI", 42, 8)
    ifd = struct.pack("<H", tag_count) + b"".join(entries) + struct.pack("<I", 0)
    external = bytearray()
    for payload in external_values.values():
        external.extend(payload)
        if len(external) % 2:
            external.extend(b"\0")
    path.write_bytes(header + ifd + bytes(external) + sample_data)


def _validated_georeference() -> ProjectGeoreference:
    return ProjectGeoreference.georeferenced(
        origin=GeoCoordinate(31.2304, 121.4737, 10.0),
        height_reference=HeightReference(HeightDatum.ELLIPSOID, source="rtk"),
        valid_radius_m=1000.0,
        validation_status=GeoreferenceValidationStatus.VALIDATED,
    )


def _write_projected_geotiff(path: Path) -> None:
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    x, y = transformer.transform(121.4737, 31.2304)
    _write_minimal_geotiff_with_georeferencing(
        path,
        pixel_scale=(5.0, 5.0, 0.0),
        tiepoint=(0.0, 0.0, 0.0, x, y, 0.0),
        geo_keys=(
            1,
            1,
            0,
            3,
            1024,
            0,
            1,
            1,
            1025,
            0,
            1,
            1,
            3072,
            0,
            1,
            3857,
        ),
    )


def _write_minimal_geotiff_with_georeferencing(
    path: Path,
    *,
    pixel_scale: tuple[float, float, float],
    tiepoint: tuple[float, float, float, float, float, float],
    geo_keys: tuple[int, ...],
) -> None:
    width = 3
    height = 2
    samples = [10.0, 12.0, -9999.0, 14.0, 16.0, 15.0]
    sample_data = struct.pack("<" + "f" * len(samples), *samples)
    external_values = {
        33550: struct.pack("<ddd", *pixel_scale),
        33922: struct.pack("<dddddd", *tiepoint),
        34735: struct.pack("<" + "H" * len(geo_keys), *geo_keys),
        42113: b"-9999\0",
    }
    tag_count = 14
    external_offset = 8 + 2 + tag_count * 12 + 4
    value_offsets: dict[int, int] = {}
    cursor = external_offset
    for tag, payload in external_values.items():
        value_offsets[tag] = cursor
        cursor += len(payload)
        if cursor % 2:
            cursor += 1
    strip_offset = cursor

    def entry(tag: int, tag_type: int, count: int, value: int) -> bytes:
        type_size = {2: 1, 3: 2, 4: 4, 12: 8}[tag_type]
        if count * type_size <= 4:
            if tag_type == 3:
                raw = struct.pack("<H", value).ljust(4, b"\0")
            elif tag_type == 4:
                raw = struct.pack("<I", value)
            else:
                raw = bytes(value).ljust(4, b"\0")
        else:
            raw = struct.pack("<I", value)
        return struct.pack("<HHI", tag, tag_type, count) + raw

    entries = [
        entry(256, 4, 1, width),
        entry(257, 4, 1, height),
        entry(258, 3, 1, 32),
        entry(259, 3, 1, 1),
        entry(262, 3, 1, 1),
        entry(273, 4, 1, strip_offset),
        entry(277, 3, 1, 1),
        entry(278, 4, 1, height),
        entry(279, 4, 1, len(sample_data)),
        entry(339, 3, 1, 3),
        entry(33550, 12, 3, value_offsets[33550]),
        entry(33922, 12, 6, value_offsets[33922]),
        entry(34735, 3, len(geo_keys), value_offsets[34735]),
        entry(42113, 2, 6, value_offsets[42113]),
    ]
    header = b"II" + struct.pack("<HI", 42, 8)
    ifd = struct.pack("<H", len(entries)) + b"".join(entries) + struct.pack("<I", 0)
    external = bytearray()
    for payload in external_values.values():
        external.extend(payload)
        if len(external) % 2:
            external.extend(b"\0")
    path.write_bytes(header + ifd + bytes(external) + sample_data)
