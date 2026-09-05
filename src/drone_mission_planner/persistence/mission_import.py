"""Mission data import: GeoJSON, basic KML, and waypoint CSV files.

Parsers are pure Python and produce a preview first; applying the preview
goes through :class:`~drone_mission_planner.app.project_service.ProjectService`
so validation, dirty marking, and ID generation stay in one place. All
coordinates are local metric task coordinates, matching `.dmproj`.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MapModel
from drone_mission_planner.domain.waypoint import Waypoint

if TYPE_CHECKING:
    from drone_mission_planner.app.project_service import ProjectService

OBJECT_KINDS = ("search_area", "no_fly_zone", "task", "base")
_GEOJSON_KINDS = ("search_area", "no_fly_zone", "task", "base", "waypoints")
_CSV_HEADER = "x,y,altitude[,altitude_mode,speed,action,hold_seconds,task_id]"


class MissionImportError(ValueError):
    """Raised when an import file cannot be parsed or is invalid."""


@dataclass(slots=True)
class ImportItem:
    """One object to create, with its polygon or position points."""

    kind: str
    name: str
    points: list[Point]
    source_line: int | None = None


@dataclass(slots=True)
class MissionImportPreview:
    """Validated import content shown to the user before applying."""

    source_format: str
    items: list[ImportItem] = field(default_factory=list)
    waypoint_route: list[Waypoint] = field(default_factory=list)
    route_from_geometry: bool = False
    waypoint_route_name: str = ""
    warnings: list[str] = field(default_factory=list)

    def summary(self) -> str:
        counts: dict[str, int] = {}
        for item in self.items:
            counts[item.kind] = counts.get(item.kind, 0) + 1
        parts = [f"{count} {kind}(s)" for kind, count in sorted(counts.items())]
        if self.waypoint_route:
            parts.append(f"{len(self.waypoint_route)} waypoint(s) for {self.waypoint_route_name or 'drone'}")
        header = f"{self.source_format} import: " + ", ".join(parts) if parts else "no objects"
        if self.warnings:
            header += f"; {len(self.warnings)} warning(s)"
        return header


def load_geojson(
    map_model: MapModel,
    path: str | Path,
    *,
    default_kind: str = "search_area",
) -> MissionImportPreview:
    """Parse a GeoJSON FeatureCollection with local metric coordinates."""

    source = Path(path)
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MissionImportError(f"Cannot read GeoJSON: {exc}") from exc
    features = document.get("features")
    if not isinstance(features, list):
        raise MissionImportError("GeoJSON root must contain a 'features' array")

    preview = MissionImportPreview(source_format="GeoJSON")
    seen_names: set[str] = set()
    for index, feature in enumerate(features, start=1):
        if not isinstance(feature, dict):
            raise MissionImportError(f"Feature {index} is not an object")
        properties = feature.get("properties") or {}
        name = str(properties.get("name") or f"Imported-{index:02d}")
        geometry = feature.get("geometry") or {}
        geometry_type = str(geometry.get("type", ""))
        kind = str(properties.get("kind") or default_kind)
        if kind not in _GEOJSON_KINDS:
            raise MissionImportError(
                f"Feature {index} ({name}): unknown kind {kind!r}; expected one of {_GEOJSON_KINDS}"
            )
        if name in seen_names:
            preview.warnings.append(f"Duplicate object name {name!r}; both will be imported")
        seen_names.add(name)
        try:
            coordinates = geometry["coordinates"]
        except KeyError as exc:
            raise MissionImportError(f"Feature {index} ({name}): missing geometry coordinates") from exc

        if kind == "waypoints" or geometry_type == "LineString":
            if geometry_type not in {"LineString", "MultiPoint"}:
                preview.warnings.append(
                    f"Feature {index} ({name}): waypoints kind expects LineString geometry"
                )
            flat = _flatten_geojson_positions(coordinates, require_pairs=True)
            preview.waypoint_route = [Waypoint(x, y, altitude=0.0) for x, y in flat]
            preview.route_from_geometry = True
            preview.waypoint_route_name = name
            continue

        points = [Point(x, y) for x, y in _flatten_geojson_positions(coordinates)]
        if geometry_type == "Point":
            if len(points) != 1:
                raise MissionImportError(f"Feature {index} ({name}): Point needs exactly one position")
            points = points[:1]
            if kind == "search_area":
                kind = "task"
        _check_item(preview, map_model, ImportItem(kind, name, points, index))
    return preview


def load_kml(map_model: MapModel, path: str | Path) -> MissionImportPreview:
    """Parse basic KML Placemarks (Point and Polygon) in local metric coordinates."""

    source = Path(path)
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as exc:
        raise MissionImportError(f"Cannot read KML: {exc}") from exc

    preview = MissionImportPreview(source_format="KML")
    namespace = ""
    if root.tag.startswith("{"):
        namespace = "{" + root.tag.split("}")[0].strip("{") + "}"
    placemarks = root.findall(f".//{namespace}Placemark") if namespace else root.findall(".//Placemark")
    for index, placemark in enumerate(placemarks, start=1):
        name_node = placemark.find(f"{namespace}name") if namespace else placemark.find("name")
        name = (name_node.text or f"Imported-{index:02d}").strip() if name_node is not None else f"Imported-{index:02d}"
        point = placemark.find(f".//{namespace}Point") if namespace else placemark.find(".//Point")
        polygon = placemark.find(f".//{namespace}Polygon") if namespace else placemark.find(".//Polygon")
        if point is not None:
            coordinates = point.find(f"{namespace}coordinates") if namespace else point.find("coordinates")
            text = (coordinates.text or "").strip() if coordinates is not None else ""
            values = _parse_coordinate_text(text)
            if len(values) != 1:
                raise MissionImportError(f"Placemark {index} ({name}): Point needs exactly one coordinate")
            _check_item(preview, map_model, ImportItem("task", name, [Point(values[0][0], values[0][1])], index))
        elif polygon is not None:
            ring = polygon.find(f".//{namespace}coordinates") if namespace else polygon.find(".//coordinates")
            text = (ring.text or "").strip() if ring is not None else ""
            values = _parse_coordinate_text(text)
            points = [Point(x, y) for x, y, *_rest in values]
            _check_item(preview, map_model, ImportItem("search_area", name, points, index))
        else:
            preview.warnings.append(f"Placemark {index} ({name}): no Point or Polygon geometry; skipped")
    return preview


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
) -> list[str]:
    """Create the previewed objects and return their ids (route: drone id)."""

    created: list[str] = []
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
                created.append(area.id)
    if preview.waypoint_route:
        if drone_id is None:
            raise MissionImportError("Waypoint routes need a target drone; select one and re-import")
        waypoints = list(preview.waypoint_route)
        if preview.route_from_geometry:
            waypoints = _with_route_altitudes(service, drone_id, waypoints)
        service.replace_waypoints(drone_id, waypoints)
        created.append(drone_id)
    return created


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

    flat: list[tuple[float, float]] = []

    def walk(node: Any) -> None:
        if isinstance(node, (list, tuple)) and node and isinstance(node[0], (int, float)):
            if len(node) < 2:
                raise MissionImportError("GeoJSON position needs at least x and y")
            flat.append((float(node[0]), float(node[1])))
        elif isinstance(node, (list, tuple)):
            for child in node:
                walk(child)

    if not isinstance(coordinates, (list, tuple)) or not coordinates:
        raise MissionImportError("GeoJSON geometry coordinates are empty")
    walk(coordinates)
    if require_pairs and len(flat) < 2:
        raise MissionImportError("Waypoint geometry needs at least two positions")
    return flat


def _parse_coordinate_text(text: str) -> list[tuple[float, float, float]]:
    values: list[tuple[float, float, float]] = []
    for token in text.split():
        parts = token.split(",")
        if len(parts) < 2:
            raise MissionImportError(f"KML coordinate {token!r} must be lon,lat[,alt]")
        longitude = float(parts[0])
        latitude = float(parts[1])
        altitude = float(parts[2]) if len(parts) >= 3 else 0.0
        values.append((longitude, latitude, altitude))
    return values


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
