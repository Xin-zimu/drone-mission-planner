from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from math import isfinite

from .georeference import HeightReference, SpatialBounds


class DataSourceError(ValueError):
    """Raised when imported GIS/DEM source metadata is incomplete or invalid."""


class DataSourceKind(StrEnum):
    GEOJSON = "geojson"
    KML = "kml"
    GEOTIFF_DEM = "geotiff_dem"
    ELEVATION_CSV = "elevation_csv"


class DataSourceValidationStatus(StrEnum):
    PREVIEWED = "previewed"
    VALIDATED = "validated"
    UNVERIFIABLE = "unverifiable"


@dataclass(frozen=True, slots=True)
class SourceBounds:
    """Axis-aligned bounds in the source CRS and native coordinate units."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float
    min_z: float | None = None
    max_z: float | None = None

    def __post_init__(self) -> None:
        values = (self.min_x, self.min_y, self.max_x, self.max_y)
        vertical = tuple(value for value in (self.min_z, self.max_z) if value is not None)
        if not all(isfinite(value) for value in values + vertical):
            raise DataSourceError("source bounds must be finite")
        if self.min_x > self.max_x or self.min_y > self.max_y:
            raise DataSourceError("source bounds minimums cannot exceed maximums")
        if self.min_z is not None and self.max_z is not None and self.min_z > self.max_z:
            raise DataSourceError("source vertical bounds minimum cannot exceed maximum")


@dataclass(frozen=True, slots=True)
class DataSourceMetadata:
    """Traceable metadata for an imported or previewed GIS/DEM resource."""

    id: str
    kind: DataSourceKind
    source_path: str
    source_crs: str = "unknown"
    source_bounds: SourceBounds | None = None
    local_bounds: SpatialBounds | None = None
    horizontal_units: str = "unknown"
    vertical_units: str = "unknown"
    horizontal_accuracy_m: float | None = None
    vertical_accuracy_m: float | None = None
    native_resolution_x: float | None = None
    native_resolution_y: float | None = None
    resolution_x_m: float | None = None
    resolution_y_m: float | None = None
    height_reference: HeightReference = field(default_factory=HeightReference)
    object_count: int = 0
    elevation_min_m: float | None = None
    elevation_max_m: float | None = None
    sha256: str = ""
    revision: str = ""
    validation_status: DataSourceValidationStatus = DataSourceValidationStatus.PREVIEWED
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", DataSourceKind(self.kind))
        object.__setattr__(
            self,
            "validation_status",
            DataSourceValidationStatus(self.validation_status),
        )
        if not self.id.strip():
            raise DataSourceError("data source id is required")
        if not self.source_path.strip():
            raise DataSourceError("data source path is required")
        if self.object_count < 0:
            raise DataSourceError("data source object count cannot be negative")
        for name in (
            "horizontal_accuracy_m",
            "vertical_accuracy_m",
            "native_resolution_x",
            "native_resolution_y",
            "resolution_x_m",
            "resolution_y_m",
        ):
            value = getattr(self, name)
            if value is not None and (not isfinite(value) or value <= 0.0):
                raise DataSourceError(f"{name} must be positive when provided")
        for name in ("elevation_min_m", "elevation_max_m"):
            value = getattr(self, name)
            if value is not None and not isfinite(value):
                raise DataSourceError(f"{name} must be finite when provided")
        if (
            self.elevation_min_m is not None
            and self.elevation_max_m is not None
            and self.elevation_min_m > self.elevation_max_m
        ):
            raise DataSourceError("elevation minimum cannot exceed maximum")
