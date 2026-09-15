from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from enum import StrEnum
from math import hypot, isfinite, sqrt
from typing import cast

from pyproj import CRS, Transformer
from pyproj.enums import TransformDirection

from .geometry import Point

WGS84_GEOGRAPHIC_2D_CRS = "EPSG:4326"
WGS84_GEOGRAPHIC_3D_CRS = "EPSG:4979"
WGS84_ECEF_CRS = "EPSG:4978"


class GeoreferenceError(ValueError):
    """Raised when georeferencing data is incomplete or internally invalid."""


class ProjectGeoreferenceMode(StrEnum):
    LOCAL_ONLY = "local_only"
    GEOREFERENCED = "georeferenced"


class HeightDatum(StrEnum):
    UNKNOWN = "unknown"
    ELLIPSOID = "ellipsoid"
    ORTHOMETRIC = "orthometric"
    AGL = "agl"
    RELATIVE_HOME = "relative_home"


class GeoreferenceValidationStatus(StrEnum):
    UNKNOWN = "unknown"
    VALIDATED = "validated"
    STALE = "stale"


class GeofenceKind(StrEnum):
    INCLUSION = "inclusion"
    EXCLUSION = "exclusion"


@dataclass(frozen=True, slots=True)
class GeoCoordinate:
    """Geodetic coordinate in WGS84 axis names, with height in metres."""

    latitude_deg: float
    longitude_deg: float
    height_m: float = 0.0

    def __post_init__(self) -> None:
        if not isfinite(self.latitude_deg) or not -90.0 <= self.latitude_deg <= 90.0:
            raise GeoreferenceError("latitude must be finite and within [-90, 90] degrees")
        if not isfinite(self.longitude_deg) or not -180.0 <= self.longitude_deg <= 180.0:
            raise GeoreferenceError("longitude must be finite and within [-180, 180] degrees")
        if not isfinite(self.height_m):
            raise GeoreferenceError("height must be finite")


@dataclass(frozen=True, slots=True)
class EcefCoordinate:
    """Earth-centred, Earth-fixed Cartesian coordinate in metres."""

    x_m: float
    y_m: float
    z_m: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x_m, self.y_m, self.z_m)):
            raise GeoreferenceError("ECEF coordinates must be finite")


@dataclass(frozen=True, slots=True)
class EnuCoordinate:
    """Local tangent-plane coordinate: east, north, up in metres."""

    east_m: float
    north_m: float
    up_m: float = 0.0

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.east_m, self.north_m, self.up_m)):
            raise GeoreferenceError("ENU coordinates must be finite")

    @property
    def point(self) -> Point:
        return Point(self.east_m, self.north_m)


@dataclass(frozen=True, slots=True)
class SpatialBounds:
    """Axis-aligned valid local ENU extent, in metres."""

    min_east_m: float
    min_north_m: float
    max_east_m: float
    max_north_m: float
    min_up_m: float | None = None
    max_up_m: float | None = None

    def __post_init__(self) -> None:
        values = (
            self.min_east_m,
            self.min_north_m,
            self.max_east_m,
            self.max_north_m,
        )
        vertical = tuple(
            value for value in (self.min_up_m, self.max_up_m) if value is not None
        )
        if not all(isfinite(value) for value in values + vertical):
            raise GeoreferenceError("spatial bounds must be finite")
        if self.min_east_m > self.max_east_m or self.min_north_m > self.max_north_m:
            raise GeoreferenceError("spatial bounds minimums cannot exceed maximums")
        if (
            self.min_up_m is not None
            and self.max_up_m is not None
            and self.min_up_m > self.max_up_m
        ):
            raise GeoreferenceError("vertical bounds minimum cannot exceed maximum")

    def contains(self, coordinate: EnuCoordinate) -> bool:
        if not (
            self.min_east_m <= coordinate.east_m <= self.max_east_m
            and self.min_north_m <= coordinate.north_m <= self.max_north_m
        ):
            return False
        if self.min_up_m is not None and coordinate.up_m < self.min_up_m:
            return False
        return not (self.max_up_m is not None and coordinate.up_m > self.max_up_m)


@dataclass(frozen=True, slots=True)
class Geofence:
    """Local ENU circular or polygonal inclusion/exclusion fence."""

    id: str
    name: str
    kind: GeofenceKind = GeofenceKind.INCLUSION
    polygon: tuple[Point, ...] = ()
    center: Point | None = None
    radius_m: float | None = None
    floor_m: float | None = None
    ceiling_m: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", GeofenceKind(self.kind))
        if not self.id.strip():
            raise GeoreferenceError("geofence id is required")
        if len(self.polygon) < 3 and (self.center is None or self.radius_m is None):
            raise GeoreferenceError("geofence requires a polygon or a circle")
        if self.radius_m is not None and (not isfinite(self.radius_m) or self.radius_m <= 0.0):
            raise GeoreferenceError("geofence radius must be positive")
        for point in self.polygon:
            if not isfinite(point.x) or not isfinite(point.y):
                raise GeoreferenceError("geofence polygon coordinates must be finite")
        if self.center is not None and (not isfinite(self.center.x) or not isfinite(self.center.y)):
            raise GeoreferenceError("geofence center must be finite")
        for value in (self.floor_m, self.ceiling_m):
            if value is not None and not isfinite(value):
                raise GeoreferenceError("geofence vertical limits must be finite")
        if (
            self.floor_m is not None
            and self.ceiling_m is not None
            and self.floor_m > self.ceiling_m
        ):
            raise GeoreferenceError("geofence floor cannot exceed ceiling")

    def contains(self, coordinate: EnuCoordinate) -> bool:
        if self.floor_m is not None and coordinate.up_m < self.floor_m:
            return False
        if self.ceiling_m is not None and coordinate.up_m > self.ceiling_m:
            return False
        if self.center is not None and self.radius_m is not None:
            return coordinate.point.distance_to(self.center) <= self.radius_m
        return _point_in_polygon(coordinate.point, self.polygon)


@dataclass(frozen=True, slots=True)
class SpatialConstraintViolation:
    code: str
    message: str
    point: EnuCoordinate
    fence_id: str | None = None


@dataclass(frozen=True, slots=True)
class ProjectedCoordinate:
    """Coordinate in an explicit projected CRS, with optional vertical metres."""

    x: float
    y: float
    z: float | None = None

    def __post_init__(self) -> None:
        values = (self.x, self.y) if self.z is None else (self.x, self.y, self.z)
        if not all(isfinite(value) for value in values):
            raise GeoreferenceError("projected coordinates must be finite")


@dataclass(frozen=True, slots=True)
class HeightReference:
    """Project-level statement of what waypoint heights mean."""

    datum: HeightDatum = HeightDatum.UNKNOWN
    source: str = ""
    geoid_model: str | None = None
    home_altitude_m: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "datum", HeightDatum(self.datum))
        if self.home_altitude_m is not None and not isfinite(self.home_altitude_m):
            raise GeoreferenceError("home altitude must be finite when provided")

    @property
    def is_known(self) -> bool:
        return self.datum != HeightDatum.UNKNOWN

    def require_known_for_real_export(self) -> None:
        if not self.is_known:
            raise GeoreferenceError("height datum is unknown; real-coordinate export is not verified")


@dataclass(frozen=True, slots=True)
class CalibrationControlPoint:
    """Observed real coordinate paired with an expected local ENU coordinate."""

    id: str
    name: str
    local: EnuCoordinate
    observed: GeoCoordinate

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise GeoreferenceError("control point id is required")


@dataclass(frozen=True, slots=True)
class ControlPointResidual:
    id: str
    name: str
    expected: EnuCoordinate
    actual: EnuCoordinate

    @property
    def east_error_m(self) -> float:
        return self.actual.east_m - self.expected.east_m

    @property
    def north_error_m(self) -> float:
        return self.actual.north_m - self.expected.north_m

    @property
    def up_error_m(self) -> float:
        return self.actual.up_m - self.expected.up_m

    @property
    def horizontal_error_m(self) -> float:
        return hypot(self.east_error_m, self.north_error_m)

    @property
    def absolute_vertical_error_m(self) -> float:
        return abs(self.up_error_m)


@dataclass(frozen=True, slots=True)
class CalibrationReport:
    residuals: tuple[ControlPointResidual, ...]
    horizontal_rmse_m: float
    vertical_rmse_m: float
    max_horizontal_error_m: float
    max_vertical_error_m: float
    horizontal_tolerance_m: float
    vertical_tolerance_m: float
    warnings: tuple[str, ...] = ()

    @property
    def within_tolerance(self) -> bool:
        return not self.warnings


@dataclass(frozen=True, slots=True)
class CrsCoordinateAdapter:
    """Thin wrapper around pyproj for explicit CRS-to-CRS coordinate transforms."""

    source_crs: str
    target_crs: str
    always_xy: bool = True

    def __post_init__(self) -> None:
        _crs(self.source_crs)
        _crs(self.target_crs)

    def transform_xy(self, x: float, y: float) -> Point:
        if not isfinite(x) or not isfinite(y):
            raise GeoreferenceError("coordinates must be finite")
        result = cast(
            tuple[float, float],
            self._transformer().transform(x, y),
        )
        return Point(float(result[0]), float(result[1]))

    def transform_xyz(self, x: float, y: float, z: float) -> ProjectedCoordinate:
        if not all(isfinite(value) for value in (x, y, z)):
            raise GeoreferenceError("coordinates must be finite")
        result = cast(
            tuple[float, float, float],
            self._transformer().transform(x, y, z),
        )
        return ProjectedCoordinate(float(result[0]), float(result[1]), float(result[2]))

    def _transformer(self) -> Transformer:
        return Transformer.from_crs(self.source_crs, self.target_crs, always_xy=self.always_xy)


@dataclass(frozen=True, slots=True)
class LocalTangentPlaneAdapter:
    """Convert WGS84 geodetic, ECEF, and local ENU metres around one origin."""

    origin: GeoCoordinate
    geodetic_crs: str = WGS84_GEOGRAPHIC_3D_CRS
    ecef_crs: str = WGS84_ECEF_CRS

    def __post_init__(self) -> None:
        _crs(self.geodetic_crs)
        _crs(self.ecef_crs)

    def geodetic_to_ecef(self, coordinate: GeoCoordinate) -> EcefCoordinate:
        result = cast(
            tuple[float, float, float],
            self._geodetic_to_ecef().transform(
                coordinate.longitude_deg,
                coordinate.latitude_deg,
                coordinate.height_m,
            ),
        )
        return EcefCoordinate(float(result[0]), float(result[1]), float(result[2]))

    def ecef_to_geodetic(self, coordinate: EcefCoordinate) -> GeoCoordinate:
        result = cast(
            tuple[float, float, float],
            self._ecef_to_geodetic().transform(
                coordinate.x_m,
                coordinate.y_m,
                coordinate.z_m,
            ),
        )
        return GeoCoordinate(
            latitude_deg=float(result[1]),
            longitude_deg=float(result[0]),
            height_m=float(result[2]),
        )

    def geodetic_to_enu(self, coordinate: GeoCoordinate) -> EnuCoordinate:
        return self.ecef_to_enu(self.geodetic_to_ecef(coordinate))

    def enu_to_geodetic(self, coordinate: EnuCoordinate) -> GeoCoordinate:
        return self.ecef_to_geodetic(self.enu_to_ecef(coordinate))

    def ecef_to_enu(self, coordinate: EcefCoordinate) -> EnuCoordinate:
        result = cast(
            tuple[float, float, float],
            self._ecef_to_enu().transform(coordinate.x_m, coordinate.y_m, coordinate.z_m),
        )
        return EnuCoordinate(float(result[0]), float(result[1]), float(result[2]))

    def enu_to_ecef(self, coordinate: EnuCoordinate) -> EcefCoordinate:
        result = cast(
            tuple[float, float, float],
            self._ecef_to_enu().transform(
                coordinate.east_m,
                coordinate.north_m,
                coordinate.up_m,
                direction=TransformDirection.INVERSE,
            ),
        )
        return EcefCoordinate(float(result[0]), float(result[1]), float(result[2]))

    def local_point_to_geodetic(self, point: Point, up_m: float = 0.0) -> GeoCoordinate:
        return self.enu_to_geodetic(EnuCoordinate(point.x, point.y, up_m))

    def geodetic_to_local_point(self, coordinate: GeoCoordinate) -> Point:
        return self.geodetic_to_enu(coordinate).point

    def _geodetic_to_ecef(self) -> Transformer:
        return Transformer.from_crs(self.geodetic_crs, self.ecef_crs, always_xy=True)

    def _ecef_to_geodetic(self) -> Transformer:
        return Transformer.from_crs(self.ecef_crs, self.geodetic_crs, always_xy=True)

    def _ecef_to_enu(self) -> Transformer:
        return Transformer.from_pipeline(
            "+proj=pipeline "
            f"+step +proj=topocentric +ellps=WGS84 +lat_0={self.origin.latitude_deg} "
            f"+lon_0={self.origin.longitude_deg} +h_0={self.origin.height_m}"
        )


@dataclass(frozen=True, slots=True)
class ProjectGeoreference:
    """Project-level georeferencing configuration for local ENU planning."""

    mode: ProjectGeoreferenceMode = ProjectGeoreferenceMode.LOCAL_ONLY
    origin: GeoCoordinate | None = None
    horizontal_crs: str = WGS84_GEOGRAPHIC_2D_CRS
    height_reference: HeightReference = field(default_factory=HeightReference)
    valid_radius_m: float | None = None
    control_points: tuple[CalibrationControlPoint, ...] = ()
    spatial_bounds: SpatialBounds | None = None
    geofences: tuple[Geofence, ...] = ()
    validation_status: GeoreferenceValidationStatus = GeoreferenceValidationStatus.UNKNOWN
    revision: str = "0"

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", ProjectGeoreferenceMode(self.mode))
        object.__setattr__(
            self,
            "validation_status",
            GeoreferenceValidationStatus(self.validation_status),
        )
        _crs(self.horizontal_crs)
        if self.mode == ProjectGeoreferenceMode.GEOREFERENCED and self.origin is None:
            raise GeoreferenceError("georeferenced projects require an origin")
        if self.valid_radius_m is not None and (
            not isfinite(self.valid_radius_m) or self.valid_radius_m <= 0.0
        ):
            raise GeoreferenceError("valid radius must be positive when provided")
        if len({point.id for point in self.control_points}) != len(self.control_points):
            raise GeoreferenceError("control point ids must be unique")
        if len({fence.id for fence in self.geofences}) != len(self.geofences):
            raise GeoreferenceError("geofence ids must be unique")

    @classmethod
    def local_only(cls) -> ProjectGeoreference:
        return cls()

    @classmethod
    def georeferenced(
        cls,
        *,
        origin: GeoCoordinate,
        height_reference: HeightReference,
        horizontal_crs: str = WGS84_GEOGRAPHIC_2D_CRS,
        valid_radius_m: float | None = None,
        control_points: tuple[CalibrationControlPoint, ...] = (),
        spatial_bounds: SpatialBounds | None = None,
        geofences: tuple[Geofence, ...] = (),
        validation_status: GeoreferenceValidationStatus = GeoreferenceValidationStatus.UNKNOWN,
        revision: str = "0",
    ) -> ProjectGeoreference:
        return cls(
            mode=ProjectGeoreferenceMode.GEOREFERENCED,
            origin=origin,
            horizontal_crs=horizontal_crs,
            height_reference=height_reference,
            valid_radius_m=valid_radius_m,
            control_points=control_points,
            spatial_bounds=spatial_bounds,
            geofences=geofences,
            validation_status=validation_status,
            revision=revision,
        )

    @property
    def is_georeferenced(self) -> bool:
        return self.mode == ProjectGeoreferenceMode.GEOREFERENCED

    @property
    def can_export_real_coordinates(self) -> bool:
        return (
            self.mode == ProjectGeoreferenceMode.GEOREFERENCED
            and self.origin is not None
            and self.height_reference.is_known
            and self.validation_status == GeoreferenceValidationStatus.VALIDATED
        )

    def require_ready_for_real_export(self) -> None:
        if self.mode == ProjectGeoreferenceMode.LOCAL_ONLY:
            raise GeoreferenceError("project is local-only; real-coordinate export is unavailable")
        if self.origin is None:
            raise GeoreferenceError("georeferenced project has no origin")
        self.height_reference.require_known_for_real_export()
        if self.validation_status != GeoreferenceValidationStatus.VALIDATED:
            raise GeoreferenceError("georeference is not validated")

    def local_adapter(self) -> LocalTangentPlaneAdapter:
        if self.origin is None:
            raise GeoreferenceError("local ENU conversion requires a georeferenced origin")
        return LocalTangentPlaneAdapter(self.origin)

    def local_to_geodetic(self, point: Point, up_m: float = 0.0) -> GeoCoordinate:
        return self.local_adapter().local_point_to_geodetic(point, up_m)

    def geodetic_to_local(self, coordinate: GeoCoordinate) -> Point:
        return self.local_adapter().geodetic_to_local_point(coordinate)

    def control_point_residuals(self) -> tuple[ControlPointResidual, ...]:
        adapter = self.local_adapter()
        return tuple(
            ControlPointResidual(
                id=point.id,
                name=point.name,
                expected=point.local,
                actual=adapter.geodetic_to_enu(point.observed),
            )
            for point in self.control_points
        )

    def calibration_report(
        self,
        *,
        horizontal_tolerance_m: float = 1.0,
        vertical_tolerance_m: float = 2.0,
    ) -> CalibrationReport:
        if horizontal_tolerance_m <= 0.0 or vertical_tolerance_m <= 0.0:
            raise GeoreferenceError("calibration tolerances must be positive")
        residuals = self.control_point_residuals()
        if not residuals:
            return CalibrationReport(
                residuals=(),
                horizontal_rmse_m=0.0,
                vertical_rmse_m=0.0,
                max_horizontal_error_m=0.0,
                max_vertical_error_m=0.0,
                horizontal_tolerance_m=horizontal_tolerance_m,
                vertical_tolerance_m=vertical_tolerance_m,
                warnings=("no control points configured",),
            )
        horizontal_errors = [residual.horizontal_error_m for residual in residuals]
        vertical_errors = [residual.absolute_vertical_error_m for residual in residuals]
        warnings: list[str] = []
        for residual in residuals:
            if residual.horizontal_error_m > horizontal_tolerance_m:
                warnings.append(
                    f"{residual.id}: horizontal residual {residual.horizontal_error_m:.2f} m "
                    f"exceeds {horizontal_tolerance_m:.2f} m"
                )
            if residual.absolute_vertical_error_m > vertical_tolerance_m:
                warnings.append(
                    f"{residual.id}: vertical residual {residual.absolute_vertical_error_m:.2f} m "
                    f"exceeds {vertical_tolerance_m:.2f} m"
                )
        return CalibrationReport(
            residuals=residuals,
            horizontal_rmse_m=_rmse(horizontal_errors),
            vertical_rmse_m=_rmse(vertical_errors),
            max_horizontal_error_m=max(horizontal_errors),
            max_vertical_error_m=max(vertical_errors),
            horizontal_tolerance_m=horizontal_tolerance_m,
            vertical_tolerance_m=vertical_tolerance_m,
            warnings=tuple(warnings),
        )

    def spatial_violations(
        self,
        coordinates: tuple[EnuCoordinate, ...],
    ) -> tuple[SpatialConstraintViolation, ...]:
        violations: list[SpatialConstraintViolation] = []
        for coordinate in coordinates:
            if self.valid_radius_m is not None:
                radius = hypot(coordinate.east_m, coordinate.north_m)
                if radius > self.valid_radius_m:
                    violations.append(
                        SpatialConstraintViolation(
                            code="outside_valid_radius",
                            message=(
                                f"point radius {radius:.1f} m exceeds "
                                f"valid radius {self.valid_radius_m:.1f} m"
                            ),
                            point=coordinate,
                        )
                    )
            if self.spatial_bounds is not None and not self.spatial_bounds.contains(coordinate):
                violations.append(
                    SpatialConstraintViolation(
                        code="outside_spatial_bounds",
                        message="point is outside the configured spatial bounds",
                        point=coordinate,
                    )
                )
            inclusions = [fence for fence in self.geofences if fence.kind == GeofenceKind.INCLUSION]
            if inclusions and not any(fence.contains(coordinate) for fence in inclusions):
                violations.append(
                    SpatialConstraintViolation(
                        code="outside_inclusion_geofence",
                        message="point is outside every inclusion geofence",
                        point=coordinate,
                    )
                )
            for fence in self.geofences:
                if fence.kind == GeofenceKind.EXCLUSION and fence.contains(coordinate):
                    violations.append(
                        SpatialConstraintViolation(
                            code="inside_exclusion_geofence",
                            message=f"point is inside exclusion geofence {fence.id}",
                            point=coordinate,
                            fence_id=fence.id,
                        )
                    )
        return tuple(violations)

    def with_validation_status(
        self,
        status: GeoreferenceValidationStatus,
    ) -> ProjectGeoreference:
        return ProjectGeoreference(
            mode=self.mode,
            origin=self.origin,
            horizontal_crs=self.horizontal_crs,
            height_reference=self.height_reference,
            valid_radius_m=self.valid_radius_m,
            control_points=self.control_points,
            spatial_bounds=self.spatial_bounds,
            geofences=self.geofences,
            validation_status=status,
            revision=_revision_for(self, extra=status.value),
        )

    def with_origin(self, origin: GeoCoordinate) -> ProjectGeoreference:
        def reanchor_point(point: Point) -> Point:
            return self.reanchor_coordinate(EnuCoordinate(point.x, point.y), origin).point

        def reanchor_bounds(bounds: SpatialBounds | None) -> SpatialBounds | None:
            if bounds is None:
                return None
            corners = [
                reanchor_point(Point(bounds.min_east_m, bounds.min_north_m)),
                reanchor_point(Point(bounds.max_east_m, bounds.min_north_m)),
                reanchor_point(Point(bounds.max_east_m, bounds.max_north_m)),
                reanchor_point(Point(bounds.min_east_m, bounds.max_north_m)),
            ]
            xs = [point.x for point in corners]
            ys = [point.y for point in corners]
            return SpatialBounds(
                min_east_m=min(xs),
                min_north_m=min(ys),
                max_east_m=max(xs),
                max_north_m=max(ys),
                min_up_m=bounds.min_up_m,
                max_up_m=bounds.max_up_m,
            )

        def reanchor_fence(fence: Geofence) -> Geofence:
            center = reanchor_point(fence.center) if fence.center is not None else None
            return Geofence(
                id=fence.id,
                name=fence.name,
                kind=fence.kind,
                polygon=tuple(reanchor_point(point) for point in fence.polygon),
                center=center,
                radius_m=fence.radius_m,
                floor_m=fence.floor_m,
                ceiling_m=fence.ceiling_m,
            )

        return ProjectGeoreference(
            mode=ProjectGeoreferenceMode.GEOREFERENCED,
            origin=origin,
            horizontal_crs=self.horizontal_crs,
            height_reference=self.height_reference,
            valid_radius_m=self.valid_radius_m,
            control_points=tuple(
                CalibrationControlPoint(
                    id=point.id,
                    name=point.name,
                    local=self.reanchor_coordinate(point.local, origin),
                    observed=point.observed,
                )
                for point in self.control_points
            ),
            spatial_bounds=reanchor_bounds(self.spatial_bounds),
            geofences=tuple(reanchor_fence(fence) for fence in self.geofences),
            validation_status=GeoreferenceValidationStatus.STALE,
            revision=_revision_for(self, extra=f"{origin!r}:stale"),
        )

    def reanchor_coordinate(
        self,
        coordinate: EnuCoordinate,
        new_origin: GeoCoordinate,
    ) -> EnuCoordinate:
        geodetic = self.local_adapter().enu_to_geodetic(coordinate)
        return LocalTangentPlaneAdapter(new_origin).geodetic_to_enu(geodetic)


def _crs(value: str) -> CRS:
    try:
        return CRS.from_user_input(value)
    except Exception as exc:
        raise GeoreferenceError(f"invalid CRS {value!r}") from exc


def _point_in_polygon(point: Point, polygon: tuple[Point, ...]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, current in enumerate(polygon):
        previous = polygon[j]
        crosses = (current.y > point.y) != (previous.y > point.y)
        if crosses:
            x_intersect = (previous.x - current.x) * (point.y - current.y) / (
                previous.y - current.y
            ) + current.x
            if point.x <= x_intersect:
                inside = not inside
        j = i
    return inside


def _rmse(values: list[float]) -> float:
    if not values:
        return 0.0
    return sqrt(sum(value * value for value in values) / len(values))


def _revision_for(georeference: ProjectGeoreference, *, extra: str) -> str:
    digest = hashlib.sha256(
        (
            georeference.mode.value
            + repr(georeference.origin)
            + georeference.horizontal_crs
            + repr(georeference.height_reference)
            + repr(georeference.valid_radius_m)
            + repr(georeference.control_points)
            + repr(georeference.spatial_bounds)
            + repr(georeference.geofences)
            + extra
        ).encode("utf-8")
    ).hexdigest()
    return digest[:16]
