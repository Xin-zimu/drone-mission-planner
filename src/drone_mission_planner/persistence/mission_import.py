"""Mission data import: GeoJSON, basic KML, and waypoint CSV files.

Parsers are pure Python and produce a preview first; applying the preview
goes through :class:`~drone_mission_planner.app.project_service.ProjectService`
so validation, dirty marking, and ID generation stay in one place. Legacy
callers may still preview local metric GeoJSON/KML by omitting georeference,
but explicit project georeferences keep external lon/lat data separate from
local ENU metres.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass, field
from math import isclose
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from pyproj import CRS

from drone_mission_planner.domain.data_source import (
    DataSourceKind,
    DataSourceMetadata,
    DataSourceValidationStatus,
    SourceBounds,
)
from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.georeference import (
    WGS84_GEOGRAPHIC_2D_CRS,
    EnuCoordinate,
    GeoCoordinate,
    GeoreferenceError,
    HeightReference,
    ProjectGeoreference,
    SpatialBounds,
)
from drone_mission_planner.domain.models import MapModel
from drone_mission_planner.domain.waypoint import Waypoint

if TYPE_CHECKING:
    from drone_mission_planner.app.project_service import ProjectService

OBJECT_KINDS = ("search_area", "no_fly_zone", "task", "base")
_GEOJSON_KINDS = ("search_area", "no_fly_zone", "task", "base", "waypoints")
_CSV_HEADER = "x,y,altitude[,altitude_mode,speed,action,hold_seconds,task_id]"
_EPSILON = 1e-9


class MissionImportError(ValueError):
    """Raised when an import file cannot be parsed or is invalid."""


@dataclass(frozen=True, slots=True)
class ImportIssue:
    """One preview issue tied to the source feature and geometry part."""

    severity: Literal["warning", "blocking"]
    code: str
    message: str
    source_feature_id: str
    part_index: int


@dataclass(slots=True)
class ImportItem:
    """One object to create, with its polygon or position points."""

    kind: str
    name: str
    points: list[Point]
    source_line: int | None = None
    holes: list[list[Point]] = field(default_factory=list)
    source_feature_id: str = ""
    part_index: int = 0
    properties: dict[str, Any] = field(default_factory=dict)
    source_bounds: SourceBounds | None = None
    local_bounds: SpatialBounds | None = None
    conversion_config: str = ""
    issues: list[ImportIssue] = field(default_factory=list)


@dataclass(slots=True)
class ImportRoutePart:
    """One independent imported route geometry."""

    name: str
    waypoints: list[Waypoint]
    source_feature_id: str
    part_index: int
    properties: dict[str, Any] = field(default_factory=dict)
    source_bounds: SourceBounds | None = None
    local_bounds: SpatialBounds | None = None
    conversion_config: str = ""
    issues: list[ImportIssue] = field(default_factory=list)


@dataclass(slots=True)
class MissionImportPreview:
    """Validated import content shown to the user before applying."""

    source_format: str
    source_path: str = ""
    source_metadata: DataSourceMetadata | None = None
    items: list[ImportItem] = field(default_factory=list)
    route_parts: list[ImportRoutePart] = field(default_factory=list)
    waypoint_route: list[Waypoint] = field(default_factory=list)
    route_from_geometry: bool = False
    waypoint_route_name: str = ""
    warnings: list[str] = field(default_factory=list)
    issues: list[ImportIssue] = field(default_factory=list)

    def summary(self) -> str:
        counts: dict[str, int] = {}
        for item in self.items:
            counts[item.kind] = counts.get(item.kind, 0) + 1
        parts = [f"{count} {kind}(s)" for kind, count in sorted(counts.items())]
        if self.waypoint_route:
            parts.append(f"{len(self.waypoint_route)} waypoint(s) for {self.waypoint_route_name or 'drone'}")
        elif self.route_parts:
            parts.append(f"{len(self.route_parts)} independent route part(s)")
        header = f"{self.source_format} import: " + ", ".join(parts) if parts else "no objects"
        if self.warnings:
            header += f"; {len(self.warnings)} warning(s)"
        if self.blocking_issues:
            header += f"; {len(self.blocking_issues)} blocking issue(s)"
        return header

    @property
    def object_count(self) -> int:
        return len(self.items) + len(self.route_parts)

    @property
    def blocking_issues(self) -> list[ImportIssue]:
        return [issue for issue in self.issues if issue.severity == "blocking"]


def load_geojson(
    map_model: MapModel,
    path: str | Path,
    *,
    default_kind: str = "search_area",
    georeference: ProjectGeoreference | None = None,
) -> MissionImportPreview:
    """Parse a GeoJSON FeatureCollection.

    Omitting georeference keeps the legacy local-metre import path. Passing a
    georeferenced project interprets coordinates as GeoJSON lon/lat positions
    and converts them to local ENU metres; passing local-only rejects the import
    so lon/lat is not mistaken for local metres.
    """

    source = Path(path)
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MissionImportError(f"Cannot read GeoJSON: {exc}") from exc
    features = document.get("features")
    if not isinstance(features, list):
        raise MissionImportError("GeoJSON root must contain a 'features' array")
    source_crs = _geojson_crs(document)

    preview = MissionImportPreview(source_format="GeoJSON", source_path=str(source))
    seen_names: set[str] = set()
    source_positions: list[tuple[float, float, float | None]] = []
    for index, feature in enumerate(features, start=1):
        if not isinstance(feature, dict):
            raise MissionImportError(f"Feature {index} is not an object")
        properties = feature.get("properties") or {}
        if not isinstance(properties, dict):
            raise MissionImportError(f"Feature {index}: properties must be an object")
        name = str(properties.get("name") or f"Imported-{index:02d}")
        geometry = feature.get("geometry") or {}
        if not isinstance(geometry, dict):
            raise MissionImportError(f"Feature {index} ({name}): geometry must be an object")
        kind = str(properties.get("kind") or default_kind)
        if kind not in _GEOJSON_KINDS:
            raise MissionImportError(
                f"Feature {index} ({name}): unknown kind {kind!r}; expected one of {_GEOJSON_KINDS}"
            )
        if name in seen_names:
            preview.warnings.append(f"Duplicate object name {name!r}; both will be imported")
        seen_names.add(name)
        source_feature_id = str(feature.get("id") or properties.get("id") or index)
        source_positions.extend(
            _parse_geojson_geometry(
                preview,
                map_model,
                geometry,
                name=name,
                kind=kind,
                source_feature_id=source_feature_id,
                properties=dict(properties),
                source_line=index,
                georeference=georeference,
            )
        )
    metadata_crs = source_crs if georeference is not None else "local_metric_legacy"
    _finalize_vector_preview(
        preview,
        DataSourceKind.GEOJSON,
        source,
        metadata_crs,
        source_positions,
        georeference,
    )
    return preview


def load_kml(
    map_model: MapModel,
    path: str | Path,
    *,
    georeference: ProjectGeoreference | None = None,
) -> MissionImportPreview:
    """Parse basic KML Placemarks.

    Omitting georeference keeps the legacy local-metre inspection path.
    Passing a georeferenced project converts KML lon/lat[,alt] into local ENU
    metres; passing local-only rejects the import so lon/lat is not mistaken
    for local metres.
    """

    source = Path(path)
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as exc:
        raise MissionImportError(f"Cannot read KML: {exc}") from exc

    preview = MissionImportPreview(source_format="KML", source_path=str(source))
    source_positions: list[tuple[float, float, float | None]] = []
    namespace = ""
    if root.tag.startswith("{"):
        namespace = "{" + root.tag.split("}")[0].strip("{") + "}"
    placemarks = root.findall(f".//{namespace}Placemark") if namespace else root.findall(".//Placemark")
    for index, placemark in enumerate(placemarks, start=1):
        name_node = placemark.find(f"{namespace}name") if namespace else placemark.find("name")
        name = (name_node.text or f"Imported-{index:02d}").strip() if name_node is not None else f"Imported-{index:02d}"
        parts = _parse_kml_placemark(
            preview,
            map_model,
            placemark,
            namespace,
            name=name,
            source_feature_id=str(index),
            source_line=index,
            georeference=georeference,
        )
        source_positions.extend(parts)
        if not parts:
            preview.warnings.append(f"Placemark {index} ({name}): no Point or Polygon geometry; skipped")
    metadata_crs = WGS84_GEOGRAPHIC_2D_CRS if georeference is not None else "local_metric_legacy"
    _finalize_vector_preview(
        preview,
        DataSourceKind.KML,
        source,
        metadata_crs,
        source_positions,
        georeference,
    )
    return preview


def _parse_geojson_geometry(
    preview: MissionImportPreview,
    map_model: MapModel,
    geometry: dict[str, Any],
    *,
    name: str,
    kind: str,
    source_feature_id: str,
    properties: dict[str, Any],
    source_line: int,
    georeference: ProjectGeoreference | None,
) -> list[tuple[float, float, float | None]]:
    geometry_type = str(geometry.get("type", ""))
    if geometry_type == "GeometryCollection":
        geometries = geometry.get("geometries")
        if not isinstance(geometries, list):
            raise MissionImportError(f"Feature {source_line} ({name}): GeometryCollection needs geometries")
        positions: list[tuple[float, float, float | None]] = []
        for part_index, child in enumerate(geometries):
            if not isinstance(child, dict):
                _record_issue(
                    preview,
                    "blocking",
                    "unsupported_geometry",
                    "GeometryCollection member is not an object",
                    source_feature_id,
                    part_index,
                )
                continue
            positions.extend(
                _parse_geojson_geometry_part(
                    preview,
                    map_model,
                    child,
                    name=name,
                    kind=kind,
                    source_feature_id=source_feature_id,
                    part_index=part_index,
                    properties=properties,
                    source_line=source_line,
                    georeference=georeference,
                )
            )
        return positions
    return _parse_geojson_geometry_part(
        preview,
        map_model,
        geometry,
        name=name,
        kind=kind,
        source_feature_id=source_feature_id,
        part_index=0,
        properties=properties,
        source_line=source_line,
        georeference=georeference,
    )


def _parse_geojson_geometry_part(
    preview: MissionImportPreview,
    map_model: MapModel,
    geometry: dict[str, Any],
    *,
    name: str,
    kind: str,
    source_feature_id: str,
    part_index: int,
    properties: dict[str, Any],
    source_line: int,
    georeference: ProjectGeoreference | None,
) -> list[tuple[float, float, float | None]]:
    geometry_type = str(geometry.get("type", ""))
    coordinates = geometry.get("coordinates")
    positions: list[tuple[float, float, float | None]] = []
    if coordinates is None:
        _record_issue(
            preview,
            "blocking",
            "missing_coordinates",
            f"{geometry_type or 'geometry'} is missing coordinates",
            source_feature_id,
            part_index,
        )
        return positions

    if geometry_type == "Point":
        point = _position_3d(coordinates)
        positions.append(point)
        _add_point_item(
            preview,
            map_model,
            name=name,
            kind=kind,
            source_feature_id=source_feature_id,
            part_index=part_index,
            properties=properties,
            source_line=source_line,
            values=[point],
            georeference=georeference,
        )
    elif geometry_type == "MultiPoint":
        points = _list(coordinates, "MultiPoint coordinates")
        for offset, raw in enumerate(points):
            point = _position_3d(raw)
            positions.append(point)
            _add_point_item(
                preview,
                map_model,
                name=_part_name(name, offset, len(points)),
                kind=kind,
                source_feature_id=source_feature_id,
                part_index=part_index + offset,
                properties=properties,
                source_line=source_line,
                values=[point],
                georeference=georeference,
            )
    elif geometry_type == "LineString":
        line = _line_positions(coordinates, "LineString")
        positions.extend(line)
        _add_route_part(
            preview,
            name=name,
            source_feature_id=source_feature_id,
            part_index=part_index,
            properties=properties,
            values=line,
            georeference=georeference,
        )
    elif geometry_type == "MultiLineString":
        lines = _list(coordinates, "MultiLineString coordinates")
        for offset, raw in enumerate(lines):
            line = _line_positions(raw, "MultiLineString")
            positions.extend(line)
            _add_route_part(
                preview,
                name=_part_name(name, offset, len(lines)),
                source_feature_id=source_feature_id,
                part_index=part_index + offset,
                properties=properties,
                values=line,
                georeference=georeference,
            )
    elif geometry_type == "Polygon":
        rings = _polygon_rings(coordinates, "Polygon")
        positions.extend(point for ring in rings for point in ring)
        _add_polygon_item(
            preview,
            map_model,
            name=name,
            kind=kind,
            source_feature_id=source_feature_id,
            part_index=part_index,
            properties=properties,
            source_line=source_line,
            rings=rings,
            georeference=georeference,
        )
    elif geometry_type == "MultiPolygon":
        polygons = _list(coordinates, "MultiPolygon coordinates")
        for offset, raw in enumerate(polygons):
            rings = _polygon_rings(raw, "MultiPolygon")
            positions.extend(point for ring in rings for point in ring)
            _add_polygon_item(
                preview,
                map_model,
                name=_part_name(name, offset, len(polygons)),
                kind=kind,
                source_feature_id=source_feature_id,
                part_index=part_index + offset,
                properties=properties,
                source_line=source_line,
                rings=rings,
                georeference=georeference,
            )
    else:
        _record_issue(
            preview,
            "blocking",
            "unsupported_geometry",
            f"unsupported GeoJSON geometry {geometry_type!r}",
            source_feature_id,
            part_index,
        )
    return positions


def _parse_kml_placemark(
    preview: MissionImportPreview,
    map_model: MapModel,
    placemark: ET.Element,
    namespace: str,
    *,
    name: str,
    source_feature_id: str,
    source_line: int,
    georeference: ProjectGeoreference | None,
) -> list[tuple[float, float, float | None]]:
    positions: list[tuple[float, float, float | None]] = []
    part_index = 0
    for point in _kml_geometry_nodes(placemark, namespace, "Point"):
        coordinates = _kml_coordinates_text(point, namespace)
        values = _parse_coordinate_text(coordinates)
        positions.extend(values)
        if len(values) != 1:
            _record_issue(
                preview,
                "blocking",
                "invalid_point",
                "KML Point needs exactly one coordinate",
                source_feature_id,
                part_index,
            )
        else:
            _add_point_item(
                preview,
                map_model,
                name=_part_name(name, part_index, 1),
                kind="task",
                source_feature_id=source_feature_id,
                part_index=part_index,
                properties={},
                source_line=source_line,
                values=values,
                georeference=georeference,
            )
        part_index += 1
    for line in _kml_geometry_nodes(placemark, namespace, "LineString"):
        values = _parse_coordinate_text(_kml_coordinates_text(line, namespace))
        positions.extend(values)
        _add_route_part(
            preview,
            name=_part_name(name, part_index, 1),
            source_feature_id=source_feature_id,
            part_index=part_index,
            properties={},
            values=values,
            georeference=georeference,
        )
        part_index += 1
    for polygon in _kml_geometry_nodes(placemark, namespace, "Polygon"):
        rings = _kml_polygon_rings(polygon, namespace)
        positions.extend(point for ring in rings for point in ring)
        _add_polygon_item(
            preview,
            map_model,
            name=_part_name(name, part_index, 1),
            kind="search_area",
            source_feature_id=source_feature_id,
            part_index=part_index,
            properties={},
            source_line=source_line,
            rings=rings,
            georeference=georeference,
        )
        part_index += 1
    return positions


def _add_point_item(
    preview: MissionImportPreview,
    map_model: MapModel,
    *,
    name: str,
    kind: str,
    source_feature_id: str,
    part_index: int,
    properties: dict[str, Any],
    source_line: int,
    values: list[tuple[float, float, float | None]],
    georeference: ProjectGeoreference | None,
) -> None:
    local_points = _import_points_3d(values, georeference)
    item_kind = "task" if kind == "search_area" else kind
    item = ImportItem(
        item_kind,
        name,
        local_points,
        source_line,
        source_feature_id=source_feature_id,
        part_index=part_index,
        properties=properties,
        source_bounds=_source_bounds(values),
        local_bounds=_local_bounds(local_points),
        conversion_config=_conversion_config(georeference),
    )
    _check_item(preview, map_model, item)


def _add_route_part(
    preview: MissionImportPreview,
    *,
    name: str,
    source_feature_id: str,
    part_index: int,
    properties: dict[str, Any],
    values: list[tuple[float, float, float | None]],
    georeference: ProjectGeoreference | None,
) -> None:
    local_points = _import_points_3d(values, georeference)
    issues = _line_issues(local_points, source_feature_id, part_index)
    preview.issues.extend(issues)
    preview.warnings.extend(_format_issue(issue) for issue in issues if issue.severity == "warning")
    route = ImportRoutePart(
        name=name,
        waypoints=[Waypoint(point.x, point.y, altitude=0.0) for point in local_points],
        source_feature_id=source_feature_id,
        part_index=part_index,
        properties=properties,
        source_bounds=_source_bounds(values),
        local_bounds=_local_bounds(local_points),
        conversion_config=_conversion_config(georeference),
        issues=issues,
    )
    preview.route_parts.append(route)
    if len(preview.route_parts) == 1:
        preview.waypoint_route = list(route.waypoints)
        preview.waypoint_route_name = route.name
    else:
        preview.waypoint_route = []
        preview.waypoint_route_name = ""
    preview.route_from_geometry = True


def _add_polygon_item(
    preview: MissionImportPreview,
    map_model: MapModel,
    *,
    name: str,
    kind: str,
    source_feature_id: str,
    part_index: int,
    properties: dict[str, Any],
    source_line: int,
    rings: list[list[tuple[float, float, float | None]]],
    georeference: ProjectGeoreference | None,
) -> None:
    local_rings = [_import_points_3d(ring, georeference) for ring in rings]
    outer = local_rings[0]
    holes = local_rings[1:]
    issues = _polygon_issues(outer, holes, source_feature_id, part_index)
    preview.issues.extend(issues)
    preview.warnings.extend(_format_issue(issue) for issue in issues if issue.severity == "warning")
    item = ImportItem(
        "task" if kind == "waypoints" else kind,
        name,
        outer,
        source_line,
        holes=holes,
        source_feature_id=source_feature_id,
        part_index=part_index,
        properties=properties,
        source_bounds=_source_bounds([point for ring in rings for point in ring]),
        local_bounds=_local_bounds([point for ring in local_rings for point in ring]),
        conversion_config=_conversion_config(georeference),
        issues=issues,
    )
    _check_item(preview, map_model, item)


def load_waypoint_csv(map_model: MapModel, path: str | Path) -> MissionImportPreview:
    """Parse a waypoint CSV (`x,y,altitude` plus optional columns) with line numbers."""

    source = Path(path)
    try:
        lines = source.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise MissionImportError(f"Cannot read waypoint CSV: {exc}") from exc

    preview = MissionImportPreview(source_format="Waypoint CSV")
    waypoints: list[Waypoint] = []
    for line_number, raw in enumerate(lines, start=1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        columns = [column.strip() for column in text.split(",")]
        if columns[0].lower() == "x":
            continue
        if len(columns) < 3:
            raise MissionImportError(
                f"Waypoint CSV line {line_number}: expected {_CSV_HEADER}, got {len(columns)} column(s)"
            )
        try:
            x = float(columns[0])
            y = float(columns[1])
            altitude = float(columns[2])
        except ValueError as exc:
            raise MissionImportError(f"Waypoint CSV line {line_number}: numeric field invalid ({exc})") from exc
        mode = AltitudeMode.MSL
        speed: float | None = None
        action = WaypointAction.FLY_TO
        hold = 0.0
        task_id: str | None = None
        if len(columns) >= 4 and columns[3]:
            try:
                mode = AltitudeMode(columns[3].lower())
            except ValueError as exc:
                raise MissionImportError(
                    f"Waypoint CSV line {line_number}: altitude_mode must be msl or agl"
                ) from exc
        if len(columns) >= 5 and columns[4]:
            try:
                speed = float(columns[4])
            except ValueError as exc:
                raise MissionImportError(f"Waypoint CSV line {line_number}: speed must be numeric") from exc
        if len(columns) >= 6 and columns[5]:
            try:
                action = WaypointAction(columns[5].lower())
            except ValueError as exc:
                raise MissionImportError(f"Waypoint CSV line {line_number}: unknown action {columns[5]!r}") from exc
        if len(columns) >= 7 and columns[6]:
            try:
                hold = float(columns[6])
            except ValueError as exc:
                raise MissionImportError(f"Waypoint CSV line {line_number}: hold_seconds must be numeric") from exc
        if len(columns) >= 8 and columns[7]:
            task_id = columns[7]
        waypoints.append(
            Waypoint(
                x,
                y,
                altitude=altitude,
                altitude_mode=mode,
                speed=speed,
                action=action,
                hold_seconds=hold,
                task_id=task_id,
            )
        )
    if not waypoints:
        raise MissionImportError("Waypoint CSV contains no waypoints")
    for index, waypoint in enumerate(waypoints, start=1):
        _check_bounds(preview, map_model, f"waypoint {index}", waypoint.point, None)
    preview.waypoint_route = waypoints
    preview.waypoint_route_name = source.stem
    return preview


def apply_import(
    service: ProjectService,
    preview: MissionImportPreview,
    *,
    drone_id: str | None = None,
    route_part_index: int | None = None,
) -> list[str]:
    """Create the previewed objects and return their ids (route: drone id)."""

    if preview.blocking_issues:
        first = preview.blocking_issues[0]
        raise MissionImportError(
            f"Import has blocking topology issue {first.code} in source "
            f"{first.source_feature_id} part {first.part_index}: {first.message}"
        )
    route_parts = list(preview.route_parts)
    if preview.waypoint_route and not route_parts:
        route_parts = [
            ImportRoutePart(
                name=preview.waypoint_route_name,
                waypoints=list(preview.waypoint_route),
                source_feature_id="legacy-route",
                part_index=0,
            )
        ]
    selected_route: ImportRoutePart | None = None
    if route_parts:
        if drone_id is None:
            raise MissionImportError("Waypoint routes need a target drone; select one and re-import")
        if route_part_index is None:
            if len(route_parts) != 1:
                raise MissionImportError(
                    "Import contains multiple independent route parts; choose one route_part_index "
                    "instead of connecting them into one route"
                )
            selected_route = route_parts[0]
        else:
            selected_route = next(
                (part for part in route_parts if part.part_index == route_part_index),
                None,
            )
            if selected_route is None:
                raise MissionImportError(f"Route part {route_part_index} does not exist")

    created: list[str] = []
    with service.change("Import mission data"):
        for item in preview.items:
            if not item.points:
                continue
            if item.kind == "base":
                base = service.add_base(item.points[0])
                base.name = item.name
                created.append(base.id)
            elif item.kind == "task":
                task = service.add_task(item.points[0])
                task.name = item.name
                created.append(task.id)
            elif item.kind in {"search_area", "no_fly_zone"}:
                xs = [point.x for point in item.points]
                ys = [point.y for point in item.points]
                from drone_mission_planner.domain.geometry import Rect

                bounds = Rect(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
                if item.kind == "no_fly_zone":
                    zone = service.add_no_fly_zone(bounds)
                    zone.name = item.name
                    zone.points = list(item.points)
                    created.append(zone.id)
                else:
                    area = service.add_search_area(bounds)
                    area.name = item.name
                    area.points = list(item.points)
                    area.holes = [list(hole) for hole in item.holes]
                    created.append(area.id)
        if selected_route is not None and drone_id is not None:
            waypoints = list(selected_route.waypoints)
            if preview.route_from_geometry:
                waypoints = _with_route_altitudes(service, drone_id, waypoints)
            service.replace_route(drone_id, waypoints)
            created.append(drone_id)
        if preview.source_metadata is not None:
            service.add_data_source(preview.source_metadata)
    return created


def _position_3d(value: Any) -> tuple[float, float, float | None]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise MissionImportError("GeoJSON position needs at least x and y")
    if not isinstance(value[0], (int, float)) or not isinstance(value[1], (int, float)):
        raise MissionImportError("GeoJSON position values must be numeric")
    z = value[2] if len(value) >= 3 else None
    if z is not None and not isinstance(z, (int, float)):
        raise MissionImportError("GeoJSON position height must be numeric")
    return (float(value[0]), float(value[1]), float(z) if z is not None else None)


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list) or not value:
        raise MissionImportError(f"{label} are empty")
    return value


def _line_positions(value: Any, label: str) -> list[tuple[float, float, float | None]]:
    return [_position_3d(point) for point in _list(value, f"{label} coordinates")]


def _polygon_rings(value: Any, label: str) -> list[list[tuple[float, float, float | None]]]:
    rings = [_line_positions(ring, f"{label} ring") for ring in _list(value, f"{label} coordinates")]
    if not rings:
        raise MissionImportError(f"{label} geometry coordinates are empty")
    return rings


def _part_name(name: str, part_index: int, total_parts: int) -> str:
    if total_parts <= 1:
        return name
    return f"{name} part {part_index + 1}"


def _conversion_config(georeference: ProjectGeoreference | None) -> str:
    if georeference is None:
        return "legacy_local_metric"
    if georeference.is_georeferenced:
        return f"{WGS84_GEOGRAPHIC_2D_CRS}->project_enu:{georeference.revision}"
    return "local_only_rejected"


def _record_issue(
    preview: MissionImportPreview,
    severity: Literal["warning", "blocking"],
    code: str,
    message: str,
    source_feature_id: str,
    part_index: int,
) -> ImportIssue:
    issue = ImportIssue(severity, code, message, source_feature_id, part_index)
    preview.issues.append(issue)
    if severity == "warning":
        preview.warnings.append(_format_issue(issue))
    return issue


def _format_issue(issue: ImportIssue) -> str:
    return (
        f"{issue.source_feature_id} part {issue.part_index}: "
        f"{issue.code}: {issue.message}"
    )


def _line_issues(
    points: list[Point],
    source_feature_id: str,
    part_index: int,
) -> list[ImportIssue]:
    issues: list[ImportIssue] = []
    if len(_distinct_consecutive(points)) < 2:
        issues.append(
            ImportIssue(
                "blocking",
                "line_point_count",
                "LineString needs at least two distinct positions",
                source_feature_id,
                part_index,
            )
        )
    issues.extend(
        ImportIssue(
            "warning",
            "consecutive_duplicate_vertex",
            f"vertex {index} repeats the previous coordinate",
            source_feature_id,
            part_index,
        )
        for index in _consecutive_duplicate_indexes(points)
    )
    return issues


def _polygon_issues(
    outer: list[Point],
    holes: list[list[Point]],
    source_feature_id: str,
    part_index: int,
) -> list[ImportIssue]:
    issues: list[ImportIssue] = []
    issues.extend(_ring_issues(outer, source_feature_id, part_index, "outer ring"))
    for hole_index, hole in enumerate(holes, start=1):
        issues.extend(
            _ring_issues(hole, source_feature_id, part_index, f"hole {hole_index}")
        )
    if any(issue.severity == "blocking" for issue in issues):
        return issues

    outer_open = _without_closure(outer)
    for hole_index, hole in enumerate(holes, start=1):
        hole_open = _without_closure(hole)
        if any(not _point_in_ring(point, outer_open) for point in hole_open):
            issues.append(
                ImportIssue(
                    "blocking",
                    "hole_outside_outer",
                    f"hole {hole_index} has vertices outside the outer ring",
                    source_feature_id,
                    part_index,
                )
            )
        elif _rings_cross(outer_open, hole_open):
            issues.append(
                ImportIssue(
                    "blocking",
                    "hole_crosses_outer",
                    f"hole {hole_index} crosses the outer ring",
                    source_feature_id,
                    part_index,
                )
            )
    for left_index, left in enumerate(holes, start=1):
        left_open = _without_closure(left)
        for right_index, right in enumerate(holes[left_index:], start=left_index + 1):
            right_open = _without_closure(right)
            if _rings_cross(left_open, right_open) or _ring_contains_any(left_open, right_open):
                issues.append(
                    ImportIssue(
                        "blocking",
                        "holes_overlap",
                        f"hole {left_index} overlaps or crosses hole {right_index}",
                        source_feature_id,
                        part_index,
                    )
                )
    return issues


def _ring_issues(
    ring: list[Point],
    source_feature_id: str,
    part_index: int,
    label: str,
) -> list[ImportIssue]:
    issues: list[ImportIssue] = []
    distinct = _distinct_consecutive(ring)
    if len(ring) < 4 or len(_without_closure(distinct)) < 3:
        issues.append(
            ImportIssue(
                "blocking",
                "ring_point_count",
                f"{label} needs at least three distinct vertices and a closing coordinate",
                source_feature_id,
                part_index,
            )
        )
        return issues
    if not _same_point(ring[0], ring[-1]):
        issues.append(
            ImportIssue(
                "blocking",
                "ring_not_closed",
                f"{label} is not closed",
                source_feature_id,
                part_index,
            )
        )
    area = abs(_signed_area(ring))
    if area <= _EPSILON:
        issues.append(
            ImportIssue(
                "blocking",
                "zero_area",
                f"{label} has zero area",
                source_feature_id,
                part_index,
            )
        )
    if _ring_self_intersects(ring):
        issues.append(
            ImportIssue(
                "blocking",
                "self_intersection",
                f"{label} self-intersects",
                source_feature_id,
                part_index,
            )
        )
    issues.extend(
        ImportIssue(
            "warning",
            "consecutive_duplicate_vertex",
            f"{label} vertex {index} repeats the previous coordinate",
            source_feature_id,
            part_index,
        )
        for index in _consecutive_duplicate_indexes(ring)
    )
    return issues


def _distinct_consecutive(points: list[Point]) -> list[Point]:
    distinct: list[Point] = []
    for point in points:
        if not distinct or not _same_point(point, distinct[-1]):
            distinct.append(point)
    return distinct


def _consecutive_duplicate_indexes(points: list[Point]) -> list[int]:
    return [
        index
        for index in range(1, len(points))
        if _same_point(points[index - 1], points[index])
    ]


def _without_closure(ring: list[Point]) -> list[Point]:
    if len(ring) > 1 and _same_point(ring[0], ring[-1]):
        return ring[:-1]
    return list(ring)


def _same_point(left: Point, right: Point) -> bool:
    return isclose(left.x, right.x, abs_tol=_EPSILON) and isclose(
        left.y, right.y, abs_tol=_EPSILON
    )


def _signed_area(ring: list[Point]) -> float:
    if len(ring) < 3:
        return 0.0
    area = 0.0
    for left, right in zip(ring, [*ring[1:], ring[0]], strict=False):
        area += left.x * right.y - right.x * left.y
    return area / 2.0


def _ring_self_intersects(ring: list[Point]) -> bool:
    points = _without_closure(ring)
    segments = list(zip(points, [*points[1:], points[0]], strict=False))
    for left_index, left in enumerate(segments):
        for right_index, right in enumerate(segments[left_index + 1:], start=left_index + 1):
            if abs(left_index - right_index) <= 1:
                continue
            if left_index == 0 and right_index == len(segments) - 1:
                continue
            if _segments_intersect(left[0], left[1], right[0], right[1]):
                return True
    return False


def _rings_cross(left: list[Point], right: list[Point]) -> bool:
    left_segments = list(zip(left, [*left[1:], left[0]], strict=False))
    right_segments = list(zip(right, [*right[1:], right[0]], strict=False))
    return any(
        _segments_intersect(left_start, left_end, right_start, right_end)
        for left_start, left_end in left_segments
        for right_start, right_end in right_segments
    )


def _ring_contains_any(left: list[Point], right: list[Point]) -> bool:
    return any(_point_in_ring(point, left) for point in right) or any(
        _point_in_ring(point, right) for point in left
    )


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if o1 == 0 and _on_segment(a, c, b):
        return True
    if o2 == 0 and _on_segment(a, d, b):
        return True
    if o3 == 0 and _on_segment(c, a, d):
        return True
    if o4 == 0 and _on_segment(c, b, d):
        return True
    return o1 != o2 and o3 != o4


def _orientation(a: Point, b: Point, c: Point) -> int:
    value = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    if abs(value) <= _EPSILON:
        return 0
    return 1 if value > 0 else 2


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    return (
        min(a.x, c.x) - _EPSILON <= b.x <= max(a.x, c.x) + _EPSILON
        and min(a.y, c.y) - _EPSILON <= b.y <= max(a.y, c.y) + _EPSILON
    )


def _point_in_ring(point: Point, ring: list[Point]) -> bool:
    inside = False
    j = len(ring) - 1
    for i, current in enumerate(ring):
        previous = ring[j]
        if _on_segment(previous, point, current):
            return True
        crosses = (current.y > point.y) != (previous.y > point.y)
        if crosses:
            x_intersect = (previous.x - current.x) * (point.y - current.y) / (
                previous.y - current.y
            ) + current.x
            if point.x <= x_intersect:
                inside = not inside
        j = i
    return inside


def _kml_geometry_nodes(
    placemark: ET.Element,
    namespace: str,
    tag: str,
) -> list[ET.Element]:
    qualified = f"{namespace}{tag}" if namespace else tag
    return placemark.findall(f".//{qualified}")


def _kml_coordinates_text(node: ET.Element, namespace: str) -> str:
    coordinates = node.find(f"{namespace}coordinates") if namespace else node.find("coordinates")
    return (coordinates.text or "").strip() if coordinates is not None else ""


def _kml_polygon_rings(
    polygon: ET.Element,
    namespace: str,
) -> list[list[tuple[float, float, float | None]]]:
    outer = (
        polygon.find(f"{namespace}outerBoundaryIs/{namespace}LinearRing/{namespace}coordinates")
        if namespace
        else polygon.find("outerBoundaryIs/LinearRing/coordinates")
    )
    if outer is None:
        outer = polygon.find(f".//{namespace}coordinates") if namespace else polygon.find(".//coordinates")
    outer_text = (outer.text or "").strip() if outer is not None else ""
    rings = [_parse_coordinate_text(outer_text)]
    inner_nodes = (
        polygon.findall(f"{namespace}innerBoundaryIs/{namespace}LinearRing/{namespace}coordinates")
        if namespace
        else polygon.findall("innerBoundaryIs/LinearRing/coordinates")
    )
    rings.extend(_parse_coordinate_text((node.text or "").strip()) for node in inner_nodes)
    return rings


def _with_route_altitudes(
    service: ProjectService,
    drone_id: str,
    waypoints: list[Waypoint],
) -> list[Waypoint]:
    """Give geometry-derived waypoints the drone's cruise/clearance altitude."""

    from drone_mission_planner.domain.models import Drone

    drone = service.project.map.find(drone_id)
    if not isinstance(drone, Drone):
        raise MissionImportError(f"Target drone {drone_id} does not exist")
    terrain = service.project.map.terrain
    return [
        Waypoint(
            waypoint.x,
            waypoint.y,
            altitude=max(
                drone.cruise_altitude,
                terrain.altitude_at(waypoint.x, waypoint.y) + drone.min_clearance,
            ),
        )
        for waypoint in waypoints
    ]


def _flatten_geojson_positions(
    coordinates: Any,
    require_pairs: bool = False,
) -> list[tuple[float, float]]:
    """Flatten GeoJSON coordinates into (x, y) pairs, handling nesting."""

    flat = [(x, y) for x, y, _z in _flatten_geojson_positions_3d(coordinates)]
    if require_pairs and len(flat) < 2:
        raise MissionImportError("Waypoint geometry needs at least two positions")
    return flat


def _flatten_geojson_positions_3d(coordinates: Any) -> list[tuple[float, float, float | None]]:
    """Flatten GeoJSON coordinates into lon/lat[/height] source positions."""

    flat: list[tuple[float, float, float | None]] = []

    def walk(node: Any) -> None:
        if isinstance(node, (list, tuple)) and node:
            first = node[0]
            if isinstance(first, (int, float)):
                if len(node) < 2:
                    raise MissionImportError("GeoJSON position needs at least x and y")
                if not isinstance(node[1], (int, float)):
                    raise MissionImportError("GeoJSON position values must be numeric")
                z = node[2] if len(node) >= 3 else None
                if z is not None and not isinstance(z, (int, float)):
                    raise MissionImportError("GeoJSON position height must be numeric")
                flat.append((float(first), float(node[1]), float(z) if z is not None else None))
            elif any(not isinstance(child, (list, tuple)) for child in node):
                raise MissionImportError("GeoJSON position values must be numeric")
            else:
                for child in node:
                    walk(child)

    if not isinstance(coordinates, (list, tuple)) or not coordinates:
        raise MissionImportError("GeoJSON geometry coordinates are empty")
    walk(coordinates)
    if not flat:
        raise MissionImportError("GeoJSON geometry coordinates are empty")
    return flat


def _import_points_3d(
    values: Sequence[tuple[float, float, float | None]],
    georeference: ProjectGeoreference | None,
) -> list[Point]:
    if georeference is None:
        return [Point(x, y) for x, y, _z in values]
    if not georeference.is_georeferenced:
        raise MissionImportError(
            "GeoJSON/KML coordinates are external lon/lat data; configure a georeferenced "
            "project before importing them"
        )
    try:
        return [
            georeference.geodetic_to_local(
                GeoCoordinate(
                    latitude_deg=latitude,
                    longitude_deg=longitude,
                    height_m=0.0 if altitude is None else altitude,
                )
            )
            for longitude, latitude, altitude in values
        ]
    except GeoreferenceError as exc:
        raise MissionImportError(f"Invalid geographic coordinate: {exc}") from exc


def _parse_coordinate_text(text: str) -> list[tuple[float, float, float | None]]:
    values: list[tuple[float, float, float | None]] = []
    for token in text.split():
        parts = token.split(",")
        if len(parts) < 2:
            raise MissionImportError(f"KML coordinate {token!r} must be lon,lat[,alt]")
        try:
            longitude = float(parts[0])
            latitude = float(parts[1])
            altitude = float(parts[2]) if len(parts) >= 3 else 0.0
        except ValueError as exc:
            raise MissionImportError(
                f"KML coordinate {token!r} must contain numeric lon,lat[,alt] values"
            ) from exc
        values.append((longitude, latitude, altitude))
    return values


def _geojson_crs(document: dict[str, Any]) -> str:
    crs_data = document.get("crs")
    if crs_data is None:
        return WGS84_GEOGRAPHIC_2D_CRS
    if not isinstance(crs_data, dict):
        raise MissionImportError("GeoJSON crs member must be an object")
    properties = crs_data.get("properties")
    name = properties.get("name") if isinstance(properties, dict) else None
    if not isinstance(name, str) or not name.strip():
        raise MissionImportError("GeoJSON crs member must include properties.name")
    try:
        source_crs = CRS.from_user_input(name)
        expected = CRS.from_user_input(WGS84_GEOGRAPHIC_2D_CRS)
    except Exception as exc:
        raise MissionImportError(f"Invalid GeoJSON CRS {name!r}") from exc
    if not source_crs.equals(expected, ignore_axis_order=True):
        raise MissionImportError(
            f"GeoJSON CRS {name!r} is not supported for F5-a preview; expected EPSG:4326"
        )
    return name


def _finalize_vector_preview(
    preview: MissionImportPreview,
    kind: DataSourceKind,
    source: Path,
    source_crs: str,
    source_positions: list[tuple[float, float, float | None]],
    georeference: ProjectGeoreference | None,
) -> None:
    local_points = _preview_points(preview)
    if georeference is not None and georeference.is_georeferenced:
        _append_spatial_warnings(preview, georeference, local_points)
    metadata_warnings = [*preview.warnings]
    metadata_warnings.extend(_format_issue(issue) for issue in preview.blocking_issues)
    sha256 = _file_sha256(source)
    revision = _metadata_revision(
        kind=kind,
        source_crs=source_crs,
        source_hash=sha256,
        object_count=preview.object_count,
    )
    preview.source_metadata = DataSourceMetadata(
        id=f"{kind.value}:{revision}",
        kind=kind,
        source_path=str(source),
        source_crs=source_crs,
        source_bounds=_source_bounds(source_positions),
        local_bounds=_local_bounds(local_points),
        horizontal_units=_horizontal_units(source_crs),
        vertical_units="metre" if any(z is not None for _x, _y, z in source_positions) else "unknown",
        height_reference=HeightReference(),
        object_count=preview.object_count,
        sha256=sha256,
        revision=revision,
        validation_status=(
            DataSourceValidationStatus.UNVERIFIABLE
            if metadata_warnings or preview.blocking_issues
            else DataSourceValidationStatus.PREVIEWED
        ),
        warnings=tuple(metadata_warnings),
    )


def _append_spatial_warnings(
    preview: MissionImportPreview,
    georeference: ProjectGeoreference,
    points: list[Point],
) -> None:
    violations = georeference.spatial_violations(
        tuple(EnuCoordinate(point.x, point.y) for point in points)
    )
    for violation in violations[:8]:
        preview.warnings.append(f"georeference {violation.code}: {violation.message}")
    if len(violations) > 8:
        preview.warnings.append(
            f"georeference constraints have {len(violations) - 8} more violation(s)"
        )


def _preview_points(preview: MissionImportPreview) -> list[Point]:
    points: list[Point] = []
    for item in preview.items:
        points.extend(item.points)
        for hole in item.holes:
            points.extend(hole)
    points.extend(waypoint.point for route in preview.route_parts for waypoint in route.waypoints)
    if not preview.route_parts:
        points.extend(waypoint.point for waypoint in preview.waypoint_route)
    return points


def _source_bounds(
    values: list[tuple[float, float, float | None]],
) -> SourceBounds | None:
    if not values:
        return None
    xs = [x for x, _y, _z in values]
    ys = [y for _x, y, _z in values]
    zs = [z for _x, _y, z in values if z is not None]
    return SourceBounds(
        min_x=min(xs),
        min_y=min(ys),
        max_x=max(xs),
        max_y=max(ys),
        min_z=min(zs) if zs else None,
        max_z=max(zs) if zs else None,
    )


def _local_bounds(points: list[Point]) -> SpatialBounds | None:
    if not points:
        return None
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    return SpatialBounds(
        min_east_m=min(xs),
        min_north_m=min(ys),
        max_east_m=max(xs),
        max_north_m=max(ys),
    )


def _horizontal_units(source_crs: str) -> str:
    if source_crs == "local_metric_legacy":
        return "metre"
    try:
        crs = CRS.from_user_input(source_crs)
    except Exception:
        return "unknown"
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
        raise MissionImportError(f"Cannot hash import source: {exc}") from exc
    return digest.hexdigest()


def _metadata_revision(
    *,
    kind: DataSourceKind,
    source_crs: str,
    source_hash: str,
    object_count: int,
) -> str:
    payload = f"{kind.value}:{source_crs}:{source_hash}:{object_count}".encode()
    return hashlib.sha256(payload).hexdigest()[:16]


def _check_item(
    preview: MissionImportPreview,
    map_model: MapModel,
    item: ImportItem,
) -> None:
    if not item.points:
        preview.warnings.append(f"{item.name}: empty geometry; skipped")
        return
    if item.kind in {"search_area", "no_fly_zone"} and len(item.points) < 3:
        preview.warnings.append(f"{item.name}: polygon has fewer than 3 points; skipped")
        return
    for index, point in enumerate(item.points, start=1):
        _check_bounds(preview, map_model, f"{item.name} point {index}", point, item.source_line)
    preview.items.append(item)


def _check_bounds(
    preview: MissionImportPreview,
    map_model: MapModel,
    label: str,
    point: Point,
    source_line: int | None,
) -> None:
    if 0.0 <= point.x <= map_model.width and 0.0 <= point.y <= map_model.height:
        return
    location = f" (source line {source_line})" if source_line else ""
    preview.warnings.append(
        f"{label} at ({point.x:.1f}, {point.y:.1f}) is outside the {map_model.width}x"
        f"{map_model.height} m map{location}"
    )
