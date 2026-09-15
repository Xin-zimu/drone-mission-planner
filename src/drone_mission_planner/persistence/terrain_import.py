from __future__ import annotations

import csv
import hashlib
import struct
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot, isfinite
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pyproj import CRS

from drone_mission_planner.domain.data_source import (
    DataSourceKind,
    DataSourceMetadata,
    DataSourceValidationStatus,
    SourceBounds,
)
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.georeference import (
    WGS84_GEOGRAPHIC_2D_CRS,
    CrsCoordinateAdapter,
    EnuCoordinate,
    GeoCoordinate,
    GeoreferenceError,
    HeightReference,
    ProjectGeoreference,
    SpatialBounds,
)
from drone_mission_planner.domain.terrain import TerrainModel, grid_terrain

if TYPE_CHECKING:
    from drone_mission_planner.app.project_service import ProjectService

REQUIRED_COLUMNS = ("x", "y", "elevation")
_NODATA_FILL = 0.0


class TerrainImportError(ValueError):
    """Raised when an elevation import file is malformed."""


@dataclass(frozen=True, slots=True)
class TerrainImportPreview:
    source: str
    sample_count: int
    grid_width: int
    grid_height: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_elevation: float
    max_elevation: float
    resolution: float

    def summary(self) -> str:
        return (
            f"{self.sample_count} samples, {self.grid_width} x {self.grid_height} grid, "
            f"x {self.min_x:.1f}-{self.max_x:.1f} m, "
            f"y {self.min_y:.1f}-{self.max_y:.1f} m, "
            f"elevation {self.min_elevation:.1f}-{self.max_elevation:.1f} m, "
            f"{self.resolution:.1f} m resolution"
        )


@dataclass(frozen=True, slots=True)
class GeoTiffDemPreview:
    source: str
    raster_width: int
    raster_height: int
    band_count: int
    dtype: str
    source_crs: str
    source_bounds: SourceBounds | None
    local_bounds: SpatialBounds | None
    native_resolution_x: float
    native_resolution_y: float
    resolution_x_m: float | None
    resolution_y_m: float | None
    min_elevation_m: float | None
    max_elevation_m: float | None
    nodata: float | None
    has_rotation: bool
    height_reference: HeightReference
    warnings: tuple[str, ...]
    source_metadata: DataSourceMetadata

    def summary(self) -> str:
        elevation = (
            "elevation unknown"
            if self.min_elevation_m is None or self.max_elevation_m is None
            else f"elevation {self.min_elevation_m:.1f}-{self.max_elevation_m:.1f} m"
        )
        resolution = (
            f"{self.resolution_x_m:.1f} x {self.resolution_y_m:.1f} m"
            if self.resolution_x_m is not None and self.resolution_y_m is not None
            else f"{self.native_resolution_x:g} x {self.native_resolution_y:g} source units"
        )
        return (
            f"GeoTIFF DEM preview: {self.raster_width} x {self.raster_height}, "
            f"{self.band_count} band(s), {self.source_crs}, {resolution} resolution, {elevation}"
        )


@dataclass(frozen=True, slots=True)
class TerrainImportResult:
    preview: TerrainImportPreview
    terrain: TerrainModel


@dataclass(frozen=True, slots=True)
class GeoTiffDemImportResult:
    preview: GeoTiffDemPreview
    terrain: TerrainModel


@dataclass(frozen=True, slots=True)
class _TerrainSample:
    x: float
    y: float
    elevation: float


@dataclass(frozen=True, slots=True)
class _GeoTiffImage:
    width: int
    height: int
    samples_per_pixel: int
    bits_per_sample: int
    sample_format: int
    compression: int
    dtype: str
    strip_offsets: tuple[int, ...]
    strip_byte_counts: tuple[int, ...]
    geo_keys: dict[int, int]
    pixel_scale: tuple[float, ...] | None
    tiepoint: tuple[float, ...] | None
    transformation: tuple[float, ...] | None
    nodata: float | None
    native_resolution_x: float
    native_resolution_y: float
    has_rotation: bool
    endian: str


def load_terrain_csv(path: str | Path) -> TerrainImportResult:
    source = Path(path)
    try:
        with source.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            columns = _required_columns(reader.fieldnames)
            samples = _read_samples(reader, columns)
    except OSError as exc:
        raise TerrainImportError(f"Cannot read elevation CSV: {exc}") from exc

    if len(samples) < 2:
        raise TerrainImportError("CSV must contain at least two elevation samples")
    return _build_grid(source, samples)


def load_geotiff_dem_preview(
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
    height_reference: HeightReference | None = None,
) -> GeoTiffDemPreview:
    """Read GeoTIFF DEM metadata and elevation range without mutating a project."""

    source = Path(path)
    try:
        payload = source.read_bytes()
    except OSError as exc:
        raise TerrainImportError(f"Cannot read GeoTIFF DEM: {exc}") from exc
    image = _read_tiff(payload)
    warnings: list[str] = []
    source_crs = _geotiff_crs(image.geo_keys)
    if source_crs == "unknown":
        warnings.append("GeoTIFF CRS is missing; preview cannot be spatially verified")
    source_bounds, affine = _geotiff_bounds(image)
    if source_bounds is None:
        warnings.append("GeoTIFF geotransform is missing; preview cannot compute bounds")
    local_bounds: SpatialBounds | None = None
    resolution_x_m: float | None = None
    resolution_y_m: float | None = None
    if georeference is not None:
        if not georeference.is_georeferenced:
            warnings.append(
                "project is local-only; GeoTIFF source coordinates were not interpreted as local metres"
            )
        elif source_bounds is not None and source_crs != "unknown":
            try:
                corner_points = [
                    _source_xy_to_local(source_crs, x, y, georeference)
                    for x, y in _bounds_corners(source_bounds)
                ]
                local_bounds = _local_bounds(corner_points)
                if affine is not None:
                    origin = _source_xy_to_local(source_crs, affine[2], affine[5], georeference)
                    x_neighbor = _source_xy_to_local(
                        source_crs,
                        affine[2] + affine[0],
                        affine[5] + affine[3],
                        georeference,
                    )
                    y_neighbor = _source_xy_to_local(
                        source_crs,
                        affine[2] + affine[1],
                        affine[5] + affine[4],
                        georeference,
                    )
                    resolution_x_m = origin.distance_to(x_neighbor)
                    resolution_y_m = origin.distance_to(y_neighbor)
                warnings.extend(_spatial_warnings(georeference, corner_points))
            except (GeoreferenceError, ValueError) as exc:
                warnings.append(f"GeoTIFF spatial verification failed: {exc}")
    elevations = _read_dem_values(payload, image)
    if image.nodata is not None:
        elevations = [value for value in elevations if value != image.nodata]
    if not elevations:
        warnings.append("GeoTIFF DEM contains no valid elevation samples")
    min_elevation = min(elevations) if elevations else None
    max_elevation = max(elevations) if elevations else None
    datum = height_reference or HeightReference()
    if not datum.is_known:
        warnings.append("DEM height datum is unknown; elevations remain unverifiable")
    sha256 = _file_sha256(source)
    revision = _metadata_revision(
        kind=DataSourceKind.GEOTIFF_DEM,
        source_crs=source_crs,
        source_hash=sha256,
        width=image.width,
        height=image.height,
    )
    metadata = DataSourceMetadata(
        id=f"{DataSourceKind.GEOTIFF_DEM.value}:{revision}",
        kind=DataSourceKind.GEOTIFF_DEM,
        source_path=str(source),
        source_crs=source_crs,
        source_bounds=source_bounds,
        local_bounds=local_bounds,
        horizontal_units=_horizontal_units(source_crs),
        vertical_units="metre",
        native_resolution_x=image.native_resolution_x if image.native_resolution_x > 0.0 else None,
        native_resolution_y=image.native_resolution_y if image.native_resolution_y > 0.0 else None,
        resolution_x_m=resolution_x_m,
        resolution_y_m=resolution_y_m,
        height_reference=datum,
        object_count=1,
        elevation_min_m=min_elevation,
        elevation_max_m=max_elevation,
        sha256=sha256,
        revision=revision,
        validation_status=(
            DataSourceValidationStatus.UNVERIFIABLE
            if warnings
            else DataSourceValidationStatus.PREVIEWED
        ),
        warnings=tuple(warnings),
    )
    return GeoTiffDemPreview(
        source=str(source),
        raster_width=image.width,
        raster_height=image.height,
        band_count=image.samples_per_pixel,
        dtype=image.dtype,
        source_crs=source_crs,
        source_bounds=source_bounds,
        local_bounds=local_bounds,
        native_resolution_x=image.native_resolution_x,
        native_resolution_y=image.native_resolution_y,
        resolution_x_m=resolution_x_m,
        resolution_y_m=resolution_y_m,
        min_elevation_m=min_elevation,
        max_elevation_m=max_elevation,
        nodata=image.nodata,
        has_rotation=image.has_rotation,
        height_reference=datum,
        warnings=tuple(warnings),
        source_metadata=metadata,
    )


def load_geotiff_dem(
    path: str | Path,
    *,
    georeference: ProjectGeoreference,
    height_reference: HeightReference,
) -> GeoTiffDemImportResult:
    """Read and convert a GeoTIFF DEM into the project's local terrain grid."""

    preview = load_geotiff_dem_preview(
        path,
        georeference=georeference,
        height_reference=height_reference,
    )
    source = Path(path)
    try:
        payload = source.read_bytes()
    except OSError as exc:
        raise TerrainImportError(f"Cannot read GeoTIFF DEM: {exc}") from exc
    image = _read_tiff(payload)
    terrain = _geotiff_terrain(payload, image, preview, georeference)
    return GeoTiffDemImportResult(preview=preview, terrain=terrain)


def apply_geotiff_dem_import(
    service: ProjectService,
    result: GeoTiffDemImportResult,
) -> DataSourceMetadata:
    """Apply a GeoTIFF DEM terrain and record its resource metadata atomically."""

    metadata = replace(
        result.preview.source_metadata,
        validation_status=DataSourceValidationStatus.VALIDATED,
    )
    with service.change("Import DEM"):
        service.update_environment(result.terrain, service.project.map.wind)
        service.add_data_source(metadata)
    return metadata


def _read_tiff(payload: bytes) -> _GeoTiffImage:
    if len(payload) < 8:
        raise TerrainImportError("GeoTIFF file is too small")
    if payload[:2] == b"II":
        endian = "<"
    elif payload[:2] == b"MM":
        endian = ">"
    else:
        raise TerrainImportError("GeoTIFF must be a classic TIFF file")
    magic = struct.unpack_from(endian + "H", payload, 2)[0]
    if magic != 42:
        raise TerrainImportError("BigTIFF is not supported in the F5-a preview reader")
    ifd_offset = struct.unpack_from(endian + "I", payload, 4)[0]
    tags = _read_ifd(payload, endian, ifd_offset)
    width = _tag_int(tags, 256, "ImageWidth")
    height = _tag_int(tags, 257, "ImageLength")
    samples_per_pixel = _tag_int(tags, 277, "SamplesPerPixel", default=1)
    if samples_per_pixel != 1:
        raise TerrainImportError("GeoTIFF DEM preview supports single-band rasters only")
    compression = _tag_int(tags, 259, "Compression", default=1)
    if compression != 1:
        raise TerrainImportError("GeoTIFF DEM preview supports uncompressed rasters only")
    bits_per_sample = _tag_int(tags, 258, "BitsPerSample", default=32)
    sample_format = _tag_int(tags, 339, "SampleFormat", default=1)
    strip_offsets = _tag_int_tuple(tags, 273, "StripOffsets")
    strip_byte_counts = _tag_int_tuple(tags, 279, "StripByteCounts")
    pixel_scale = _tag_float_tuple(tags.get(33550), expected_min=2)
    tiepoint = _tag_float_tuple(tags.get(33922), expected_min=6)
    transformation = _tag_float_tuple(tags.get(34264), expected_min=16)
    affine = _affine(pixel_scale, tiepoint, transformation)
    native_resolution_x, native_resolution_y, has_rotation = _native_resolution(affine)
    return _GeoTiffImage(
        width=width,
        height=height,
        samples_per_pixel=samples_per_pixel,
        bits_per_sample=bits_per_sample,
        sample_format=sample_format,
        compression=compression,
        dtype=_dtype_name(bits_per_sample, sample_format),
        strip_offsets=strip_offsets,
        strip_byte_counts=strip_byte_counts,
        geo_keys=_geo_keys(tags.get(34735)),
        pixel_scale=pixel_scale,
        tiepoint=tiepoint,
        transformation=transformation,
        nodata=_nodata(tags.get(42113)),
        native_resolution_x=native_resolution_x,
        native_resolution_y=native_resolution_y,
        has_rotation=has_rotation,
        endian=endian,
    )


def _geotiff_terrain(
    payload: bytes,
    image: _GeoTiffImage,
    preview: GeoTiffDemPreview,
    georeference: ProjectGeoreference,
) -> TerrainModel:
    if not georeference.is_georeferenced:
        raise TerrainImportError("DEM application requires a georeferenced project")
    if preview.source_crs == "unknown":
        raise TerrainImportError("DEM application requires a known GeoTIFF CRS")
    if preview.source_bounds is None:
        raise TerrainImportError("DEM application requires a GeoTIFF geotransform")
    if image.has_rotation:
        raise TerrainImportError("DEM application does not yet support rotated GeoTIFF grids")
    if not preview.height_reference.is_known:
        raise TerrainImportError("DEM application requires a known height datum")
    if preview.resolution_x_m is None or preview.resolution_y_m is None:
        raise TerrainImportError("DEM application requires verifiable local resolution")
    if not _close_resolution(preview.resolution_x_m, preview.resolution_y_m):
        raise TerrainImportError(
            "DEM application requires near-square pixels because TerrainModel stores one resolution"
        )
    affine = _affine(image.pixel_scale, image.tiepoint, image.transformation)
    if affine is None:
        raise TerrainImportError("DEM application requires a GeoTIFF geotransform")

    values = _read_dem_values(payload, image)
    rows: list[list[float]] = []
    masks: list[list[bool]] = []
    for row_index in range(image.height):
        row_values: list[float] = []
        row_mask: list[bool] = []
        for column_index in range(image.width):
            value = values[row_index * image.width + column_index]
            valid = image.nodata is None or value != image.nodata
            row_values.append(value if valid else _NODATA_FILL)
            row_mask.append(valid)
        rows.append(row_values)
        masks.append(row_mask)

    if not any(valid for row in masks for valid in row):
        raise TerrainImportError("DEM application requires at least one valid elevation sample")

    local_centres = [
        [
            _source_xy_to_local(
                preview.source_crs,
                *_apply_affine(affine, column + 0.5, row + 0.5),
                georeference,
            )
            for column in range(image.width)
        ]
        for row in range(image.height)
    ]
    first_row = local_centres[0]
    if first_row[-1].x < first_row[0].x:
        rows = [list(reversed(row)) for row in rows]
        masks = [list(reversed(row)) for row in masks]
        local_centres = [list(reversed(row)) for row in local_centres]
    if local_centres[-1][0].y < local_centres[0][0].y:
        rows = list(reversed(rows))
        masks = list(reversed(masks))
        local_centres = list(reversed(local_centres))

    origin = local_centres[0][0]
    resolution = (preview.resolution_x_m + preview.resolution_y_m) / 2.0
    return grid_terrain(
        origin=origin,
        resolution=resolution,
        altitudes=rows,
        valid_mask=masks,
        source_data_id=preview.source_metadata.id,
    )


def _read_ifd(payload: bytes, endian: str, offset: int) -> dict[int, Any]:
    if offset <= 0 or offset + 2 > len(payload):
        raise TerrainImportError("GeoTIFF IFD offset is invalid")
    entry_count = struct.unpack_from(endian + "H", payload, offset)[0]
    entries_offset = offset + 2
    tags: dict[int, Any] = {}
    for index in range(entry_count):
        entry_offset = entries_offset + index * 12
        if entry_offset + 12 > len(payload):
            raise TerrainImportError("GeoTIFF IFD entry exceeds file size")
        tag, tag_type, count = struct.unpack_from(endian + "HHI", payload, entry_offset)
        raw_value = payload[entry_offset + 8 : entry_offset + 12]
        value = _read_tag_value(payload, endian, tag_type, count, raw_value)
        tags[tag] = value
    return tags


def _read_tag_value(
    payload: bytes,
    endian: str,
    tag_type: int,
    count: int,
    raw_value: bytes,
) -> Any:
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 11: 4, 12: 8}
    formats = {1: "B", 3: "H", 4: "I", 11: "f", 12: "d"}
    size = sizes.get(tag_type)
    if size is None:
        return None
    byte_count = size * count
    if byte_count <= 4:
        data = raw_value[:byte_count]
    else:
        value_offset = struct.unpack(endian + "I", raw_value)[0]
        if value_offset + byte_count > len(payload):
            raise TerrainImportError("GeoTIFF tag value exceeds file size")
        data = payload[value_offset : value_offset + byte_count]
    if tag_type == 2:
        return data.split(b"\0", 1)[0].decode("ascii", errors="replace")
    if tag_type == 5:
        rational_values: list[float] = []
        for item_offset in range(0, byte_count, 8):
            numerator, denominator = struct.unpack_from(endian + "II", data, item_offset)
            rational_values.append(float(numerator) / float(denominator or 1))
        return tuple(rational_values)
    fmt = endian + formats[tag_type] * count
    decoded = struct.unpack(fmt, data)
    return decoded[0] if count == 1 else decoded


def _tag_int(
    tags: dict[int, Any],
    tag: int,
    name: str,
    *,
    default: int | None = None,
) -> int:
    value = tags.get(tag)
    if value is None:
        if default is not None:
            return default
        raise TerrainImportError(f"GeoTIFF is missing {name}")
    if isinstance(value, tuple):
        value = value[0]
    return int(value)


def _tag_int_tuple(tags: dict[int, Any], tag: int, name: str) -> tuple[int, ...]:
    value = tags.get(tag)
    if value is None:
        raise TerrainImportError(f"GeoTIFF is missing {name}")
    if isinstance(value, tuple):
        return tuple(int(item) for item in value)
    return (int(value),)


def _tag_float_tuple(value: Any, *, expected_min: int) -> tuple[float, ...] | None:
    if value is None:
        return None
    values = value if isinstance(value, tuple) else (value,)
    if len(values) < expected_min:
        return None
    return tuple(float(item) for item in values)


def _geo_keys(value: Any) -> dict[int, int]:
    if value is None:
        return {}
    values = tuple(int(item) for item in (value if isinstance(value, tuple) else (value,)))
    if len(values) < 4:
        return {}
    key_count = values[3]
    keys: dict[int, int] = {}
    for offset in range(4, min(len(values), 4 + key_count * 4), 4):
        key_id, _tiff_tag_location, _count, value_offset = values[offset : offset + 4]
        keys[key_id] = value_offset
    return keys


def _geotiff_crs(geo_keys: dict[int, int]) -> str:
    projected = geo_keys.get(3072)
    if projected is not None and projected not in {0, 32767}:
        return f"EPSG:{projected}"
    geographic = geo_keys.get(2048)
    if geographic is not None and geographic not in {0, 32767}:
        return f"EPSG:{geographic}"
    return "unknown"


def _nodata(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(str(value).strip())
    except ValueError:
        return None


def _dtype_name(bits_per_sample: int, sample_format: int) -> str:
    sample_kind = {1: "uint", 2: "int", 3: "float"}.get(sample_format, "unknown")
    return f"{sample_kind}{bits_per_sample}"


def _affine(
    pixel_scale: tuple[float, ...] | None,
    tiepoint: tuple[float, ...] | None,
    transformation: tuple[float, ...] | None,
) -> tuple[float, float, float, float, float, float] | None:
    if transformation is not None:
        return (
            transformation[0],
            transformation[1],
            transformation[3],
            transformation[4],
            transformation[5],
            transformation[7],
        )
    if pixel_scale is None or tiepoint is None:
        return None
    i, j, _k, x, y, _z = tiepoint[:6]
    scale_x, scale_y, _scale_z = pixel_scale[:3]
    return (scale_x, 0.0, x - i * scale_x, 0.0, -scale_y, y + j * scale_y)


def _native_resolution(
    affine: tuple[float, float, float, float, float, float] | None,
) -> tuple[float, float, bool]:
    if affine is None:
        return 0.0, 0.0, False
    a, b, _c, d, e, _f = affine
    return hypot(a, d), hypot(b, e), abs(b) > 1e-12 or abs(d) > 1e-12


def _geotiff_bounds(
    image: _GeoTiffImage,
) -> tuple[SourceBounds | None, tuple[float, float, float, float, float, float] | None]:
    affine = _affine(image.pixel_scale, image.tiepoint, image.transformation)
    if affine is None:
        return None, None
    corners = [_apply_affine(affine, x, y) for x, y in _raster_corners(image.width, image.height)]
    xs = [x for x, _y in corners]
    ys = [y for _x, y in corners]
    return SourceBounds(min(xs), min(ys), max(xs), max(ys)), affine


def _apply_affine(
    affine: tuple[float, float, float, float, float, float],
    column: float,
    row: float,
) -> tuple[float, float]:
    a, b, c, d, e, f = affine
    return a * column + b * row + c, d * column + e * row + f


def _raster_corners(width: int, height: int) -> tuple[tuple[float, float], ...]:
    return ((0.0, 0.0), (float(width), 0.0), (float(width), float(height)), (0.0, float(height)))


def _read_dem_values(payload: bytes, image: _GeoTiffImage) -> list[float]:
    fmt = _sample_format(image.bits_per_sample, image.sample_format, image.endian)
    sample_size = struct.calcsize(fmt)
    expected_samples = image.width * image.height
    values: list[float] = []
    for offset, byte_count in zip(image.strip_offsets, image.strip_byte_counts, strict=False):
        if offset < 0 or byte_count < 0 or offset + byte_count > len(payload):
            raise TerrainImportError("GeoTIFF strip exceeds file size")
        strip = payload[offset : offset + byte_count]
        sample_count = byte_count // sample_size
        values.extend(float(value[0]) for value in struct.iter_unpack(fmt, strip[: sample_count * sample_size]))
    if len(values) < expected_samples:
        raise TerrainImportError("GeoTIFF strip data has fewer samples than ImageWidth x ImageLength")
    return values[:expected_samples]


def _sample_format(bits_per_sample: int, sample_format: int, endian: str) -> str:
    formats = {
        (8, 1): "B",
        (16, 1): "H",
        (32, 1): "I",
        (8, 2): "b",
        (16, 2): "h",
        (32, 2): "i",
        (32, 3): "f",
        (64, 3): "d",
    }
    fmt = formats.get((bits_per_sample, sample_format))
    if fmt is None:
        raise TerrainImportError(
            f"GeoTIFF sample type {sample_format}/{bits_per_sample} is not supported"
        )
    return endian + fmt


def _bounds_corners(bounds: SourceBounds) -> tuple[tuple[float, float], ...]:
    return (
        (bounds.min_x, bounds.min_y),
        (bounds.max_x, bounds.min_y),
        (bounds.max_x, bounds.max_y),
        (bounds.min_x, bounds.max_y),
    )


def _source_xy_to_local(
    source_crs: str,
    x: float,
    y: float,
    georeference: ProjectGeoreference,
) -> Point:
    crs = CRS.from_user_input(source_crs)
    if crs.is_geographic:
        longitude, latitude = x, y
    else:
        transformed = CrsCoordinateAdapter(source_crs, WGS84_GEOGRAPHIC_2D_CRS).transform_xy(x, y)
        longitude, latitude = transformed.x, transformed.y
    return georeference.geodetic_to_local(GeoCoordinate(latitude_deg=latitude, longitude_deg=longitude))


def _local_bounds(points: list[Point]) -> SpatialBounds | None:
    if not points:
        return None
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    return SpatialBounds(min(xs), min(ys), max(xs), max(ys))


def _spatial_warnings(
    georeference: ProjectGeoreference,
    points: list[Point],
) -> list[str]:
    violations = georeference.spatial_violations(
        tuple(EnuCoordinate(point.x, point.y) for point in points)
    )
    return [f"georeference {violation.code}: {violation.message}" for violation in violations[:8]]


def _horizontal_units(source_crs: str) -> str:
    if source_crs == "unknown":
        return "unknown"
    crs = CRS.from_user_input(source_crs)
    if crs.is_geographic:
        return "degree"
    axis = crs.axis_info[0] if crs.axis_info else None
    return str(axis.unit_name) if axis is not None and axis.unit_name else "unknown"


def _file_sha256(source: Path) -> str:
    digest = hashlib.sha256()
    try:
        with source.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise TerrainImportError(f"Cannot hash import source: {exc}") from exc
    return digest.hexdigest()


def _metadata_revision(
    *,
    kind: DataSourceKind,
    source_crs: str,
    source_hash: str,
    width: int,
    height: int,
) -> str:
    payload = f"{kind.value}:{source_crs}:{source_hash}:{width}x{height}".encode()
    return hashlib.sha256(payload).hexdigest()[:16]


def _required_columns(fieldnames: Sequence[str] | None) -> dict[str, str]:
    if fieldnames is None:
        raise TerrainImportError("CSV must include a header row with x, y, elevation columns")
    normalized = {name.strip().lower(): name for name in fieldnames if name is not None}
    missing = [column for column in REQUIRED_COLUMNS if column not in normalized]
    if missing:
        raise TerrainImportError(
            "CSV is missing required column(s): " + ", ".join(sorted(missing))
        )
    return {column: normalized[column] for column in REQUIRED_COLUMNS}


def _read_samples(
    reader: csv.DictReader[str], columns: dict[str, str]
) -> list[_TerrainSample]:
    samples: list[_TerrainSample] = []
    seen: set[tuple[float, float]] = set()
    for row in reader:
        if _is_empty_row(row):
            continue
        line_number = reader.line_num
        x = _parse_float(row.get(columns["x"]), "x", line_number)
        y = _parse_float(row.get(columns["y"]), "y", line_number)
        elevation = _parse_float(row.get(columns["elevation"]), "elevation", line_number)
        key = (x, y)
        if key in seen:
            raise TerrainImportError(
                f"CSV line {line_number} duplicates elevation sample at x={x:g}, y={y:g}"
            )
        seen.add(key)
        samples.append(_TerrainSample(x, y, elevation))
    return samples


def _is_empty_row(row: Mapping[str, str | None]) -> bool:
    return all(value is None or value.strip() == "" for value in row.values())


def _parse_float(value: str | None, column: str, line_number: int) -> float:
    if value is None or value.strip() == "":
        raise TerrainImportError(f"CSV line {line_number} column {column} is empty")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise TerrainImportError(
            f"CSV line {line_number} column {column} must be numeric"
        ) from exc
    if not isfinite(parsed):
        raise TerrainImportError(f"CSV line {line_number} column {column} must be finite")
    return parsed


def _build_grid(source: Path, samples: list[_TerrainSample]) -> TerrainImportResult:
    x_values = sorted({sample.x for sample in samples})
    y_values = sorted({sample.y for sample in samples})
    expected_samples = len(x_values) * len(y_values)
    if expected_samples != len(samples):
        raise TerrainImportError(
            "CSV samples must fill a complete regular grid; "
            f"expected {expected_samples} samples from unique x/y values, got {len(samples)}"
        )

    resolution = _grid_resolution(x_values, y_values)
    sample_map = {(sample.x, sample.y): sample.elevation for sample in samples}
    altitudes = [[sample_map[(x, y)] for x in x_values] for y in y_values]
    terrain = grid_terrain(
        origin=Point(x_values[0], y_values[0]),
        resolution=resolution,
        altitudes=altitudes,
    )
    preview = TerrainImportPreview(
        source=str(source),
        sample_count=len(samples),
        grid_width=len(x_values),
        grid_height=len(y_values),
        min_x=x_values[0],
        max_x=x_values[-1],
        min_y=y_values[0],
        max_y=y_values[-1],
        min_elevation=terrain.min_altitude,
        max_elevation=terrain.max_altitude,
        resolution=terrain.resolution,
    )
    return TerrainImportResult(preview=preview, terrain=terrain)


def _grid_resolution(x_values: list[float], y_values: list[float]) -> float:
    x_step = _regular_step(x_values, "x")
    y_step = _regular_step(y_values, "y")
    steps = [step for step in (x_step, y_step) if step is not None]
    if not steps:
        raise TerrainImportError("CSV must contain at least two distinct x or y values")
    resolution = steps[0]
    if any(not _close(step, resolution) for step in steps[1:]):
        raise TerrainImportError(
            "CSV x and y spacing must match because terrain grids use one resolution"
        )
    return resolution


def _regular_step(values: list[float], axis: str) -> float | None:
    if len(values) < 2:
        return None
    deltas = [right - left for left, right in pairwise(values)]
    step = deltas[0]
    if step <= 0 or not isfinite(step):
        raise TerrainImportError(f"CSV {axis} values must be strictly increasing")
    for delta in deltas[1:]:
        if delta <= 0 or not isfinite(delta) or not _close(delta, step):
            raise TerrainImportError(f"CSV {axis} values must form a regular grid")
    return step


def _close(left: float, right: float) -> bool:
    return abs(left - right) <= max(1e-6, abs(right) * 1e-6)


def _close_resolution(left: float, right: float) -> bool:
    return abs(left - right) <= max(1e-6, max(abs(left), abs(right)) * 0.05)
