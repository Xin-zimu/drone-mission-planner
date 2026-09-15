from __future__ import annotations

import json
import os
from dataclasses import asdict
from enum import Enum
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.basemap import BasemapModel
from drone_mission_planner.domain.data_source import (
    DataSourceKind,
    DataSourceMetadata,
    DataSourceValidationStatus,
    SourceBounds,
)
from drone_mission_planner.domain.enums import (
    AltitudeMode,
    DeadlinePolicy,
    DroneStatus,
    ObstacleShape,
    TaskStatus,
    TaskType,
    WaypointAction,
)
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.georeference import (
    CalibrationControlPoint,
    EnuCoordinate,
    GeoCoordinate,
    Geofence,
    GeofenceKind,
    GeoreferenceValidationStatus,
    HeightDatum,
    HeightReference,
    ProjectGeoreference,
    ProjectGeoreferenceMode,
    SpatialBounds,
)
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
    ProjectModel,
    SearchArea,
)
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.domain.wind import WindModel

from .equipment_codec import decode_equipment
from .migrations import (
    CURRENT_PROJECT_VERSION,
    MigrationError,
    MigrationReport,
    migrate_project,
)
from .terrain_codec import terrain_from_data

CURRENT_VERSION = CURRENT_PROJECT_VERSION


class ProjectFormatError(ValueError):
    """Raised when a project file cannot be parsed or migrated."""


def _json_ready(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    return value


def _point(data: dict[str, Any]) -> Point:
    return Point(float(data["x"]), float(data["y"]))


def _rect(data: dict[str, Any]) -> Rect:
    return Rect(float(data["x"]), float(data["y"]), float(data["width"]), float(data["height"]))


def _wind(data: Any) -> WindModel:
    if not isinstance(data, dict):
        return WindModel()
    speed = float(data.get("speed", 0.0))
    direction_raw = data.get("direction_to_deg", data.get("direction_deg", 0.0))
    return WindModel(
        direction_to_deg=float(0.0 if direction_raw is None else direction_raw),
        speed=speed,
        gust_factor=float(data.get("gust_factor", 0.0)),
        enabled=bool(data.get("enabled", speed > 0.0)),
    )


def _georeference(data: Any) -> ProjectGeoreference:
    if not isinstance(data, dict):
        return ProjectGeoreference.local_only()

    origin_data = data.get("origin")
    origin = (
        GeoCoordinate(
            latitude_deg=float(origin_data["latitude_deg"]),
            longitude_deg=float(origin_data["longitude_deg"]),
            height_m=float(origin_data.get("height_m", 0.0)),
        )
        if isinstance(origin_data, dict)
        else None
    )
    height_data = data.get("height_reference")
    height_reference = (
        HeightReference(
            datum=HeightDatum(height_data.get("datum", HeightDatum.UNKNOWN)),
            source=str(height_data.get("source", "")),
            geoid_model=(
                str(height_data["geoid_model"])
                if height_data.get("geoid_model") is not None
                else None
            ),
            home_altitude_m=(
                float(height_data["home_altitude_m"])
                if height_data.get("home_altitude_m") is not None
                else None
            ),
        )
        if isinstance(height_data, dict)
        else HeightReference()
    )
    bounds_data = data.get("spatial_bounds")
    spatial_bounds = (
        SpatialBounds(
            min_east_m=float(bounds_data["min_east_m"]),
            min_north_m=float(bounds_data["min_north_m"]),
            max_east_m=float(bounds_data["max_east_m"]),
            max_north_m=float(bounds_data["max_north_m"]),
            min_up_m=(
                float(bounds_data["min_up_m"])
                if bounds_data.get("min_up_m") is not None
                else None
            ),
            max_up_m=(
                float(bounds_data["max_up_m"])
                if bounds_data.get("max_up_m") is not None
                else None
            ),
        )
        if isinstance(bounds_data, dict)
        else None
    )
    return ProjectGeoreference(
        mode=ProjectGeoreferenceMode(data.get("mode", ProjectGeoreferenceMode.LOCAL_ONLY)),
        origin=origin,
        horizontal_crs=str(data.get("horizontal_crs", "EPSG:4326")),
        height_reference=height_reference,
        valid_radius_m=(
            float(data["valid_radius_m"]) if data.get("valid_radius_m") is not None else None
        ),
        control_points=tuple(
            _control_point(item)
            for item in data.get("control_points", [])
            if isinstance(item, dict)
        ),
        spatial_bounds=spatial_bounds,
        geofences=tuple(
            _geofence(item) for item in data.get("geofences", []) if isinstance(item, dict)
        ),
        validation_status=GeoreferenceValidationStatus(
            data.get("validation_status", GeoreferenceValidationStatus.UNKNOWN)
        ),
        revision=str(data.get("revision", "0")),
    )


def _data_source(data: dict[str, Any]) -> DataSourceMetadata:
    bounds_data = data.get("source_bounds")
    local_bounds_data = data.get("local_bounds")
    height_data = data.get("height_reference")
    height_reference = (
        HeightReference(
            datum=HeightDatum(height_data.get("datum", HeightDatum.UNKNOWN)),
            source=str(height_data.get("source", "")),
            geoid_model=(
                str(height_data["geoid_model"])
                if height_data.get("geoid_model") is not None
                else None
            ),
            home_altitude_m=(
                float(height_data["home_altitude_m"])
                if height_data.get("home_altitude_m") is not None
                else None
            ),
        )
        if isinstance(height_data, dict)
        else HeightReference()
    )
    return DataSourceMetadata(
        id=str(data["id"]),
        kind=DataSourceKind(data["kind"]),
        source_path=str(data["source_path"]),
        source_crs=str(data.get("source_crs", "unknown")),
        source_bounds=(
            SourceBounds(
                min_x=float(bounds_data["min_x"]),
                min_y=float(bounds_data["min_y"]),
                max_x=float(bounds_data["max_x"]),
                max_y=float(bounds_data["max_y"]),
                min_z=float(bounds_data["min_z"]) if bounds_data.get("min_z") is not None else None,
                max_z=float(bounds_data["max_z"]) if bounds_data.get("max_z") is not None else None,
            )
            if isinstance(bounds_data, dict)
            else None
        ),
        local_bounds=(
            SpatialBounds(
                min_east_m=float(local_bounds_data["min_east_m"]),
                min_north_m=float(local_bounds_data["min_north_m"]),
                max_east_m=float(local_bounds_data["max_east_m"]),
                max_north_m=float(local_bounds_data["max_north_m"]),
                min_up_m=(
                    float(local_bounds_data["min_up_m"])
                    if local_bounds_data.get("min_up_m") is not None
                    else None
                ),
                max_up_m=(
                    float(local_bounds_data["max_up_m"])
                    if local_bounds_data.get("max_up_m") is not None
                    else None
                ),
            )
            if isinstance(local_bounds_data, dict)
            else None
        ),
        horizontal_units=str(data.get("horizontal_units", "unknown")),
        vertical_units=str(data.get("vertical_units", "unknown")),
        horizontal_accuracy_m=_optional_float(data.get("horizontal_accuracy_m")),
        vertical_accuracy_m=_optional_float(data.get("vertical_accuracy_m")),
        native_resolution_x=_optional_float(data.get("native_resolution_x")),
        native_resolution_y=_optional_float(data.get("native_resolution_y")),
        resolution_x_m=_optional_float(data.get("resolution_x_m")),
        resolution_y_m=_optional_float(data.get("resolution_y_m")),
        height_reference=height_reference,
        object_count=int(data.get("object_count", 0)),
        elevation_min_m=_optional_float(data.get("elevation_min_m")),
        elevation_max_m=_optional_float(data.get("elevation_max_m")),
        sha256=str(data.get("sha256", "")),
        revision=str(data.get("revision", "")),
        validation_status=DataSourceValidationStatus(
            data.get("validation_status", DataSourceValidationStatus.PREVIEWED)
        ),
        warnings=tuple(str(warning) for warning in data.get("warnings", [])),
    )


def _optional_float(value: Any) -> float | None:
    return float(value) if value is not None else None


def _enu(data: dict[str, Any]) -> EnuCoordinate:
    return EnuCoordinate(
        east_m=float(data["east_m"]),
        north_m=float(data["north_m"]),
        up_m=float(data.get("up_m", 0.0)),
    )


def _control_point(data: dict[str, Any]) -> CalibrationControlPoint:
    observed_data = data["observed"]
    return CalibrationControlPoint(
        id=str(data["id"]),
        name=str(data.get("name", data["id"])),
        local=_enu(data["local"]),
        observed=GeoCoordinate(
            latitude_deg=float(observed_data["latitude_deg"]),
            longitude_deg=float(observed_data["longitude_deg"]),
            height_m=float(observed_data.get("height_m", 0.0)),
        ),
    )


def _geofence(data: dict[str, Any]) -> Geofence:
    center_data = data.get("center")
    return Geofence(
        id=str(data["id"]),
        name=str(data.get("name", data["id"])),
        kind=GeofenceKind(data.get("kind", GeofenceKind.INCLUSION)),
        polygon=tuple(_point(point) for point in data.get("polygon", []) if isinstance(point, dict)),
        center=_point(center_data) if isinstance(center_data, dict) else None,
        radius_m=float(data["radius_m"]) if data.get("radius_m") is not None else None,
        floor_m=float(data["floor_m"]) if data.get("floor_m") is not None else None,
        ceiling_m=float(data["ceiling_m"]) if data.get("ceiling_m") is not None else None,
    )


def _waypoint(data: Any) -> Waypoint:
    if isinstance(data, dict):
        speed_raw = data.get("speed")
        task_id_raw = data.get("task_id")
        return Waypoint(
            x=float(data["x"]),
            y=float(data["y"]),
            altitude=float(data.get("altitude", 0.0)),
            altitude_mode=AltitudeMode(data.get("altitude_mode", AltitudeMode.MSL)),
            speed=float(speed_raw) if speed_raw is not None else None,
            action=WaypointAction(data.get("action", WaypointAction.FLY_TO)),
            hold_seconds=float(data.get("hold_seconds", 0.0)),
            task_id=str(task_id_raw) if task_id_raw is not None else None,
        )
    if isinstance(data, (list, tuple)) and len(data) >= 3:
        return Waypoint(float(data[0]), float(data[1]), float(data[2]))
    raise ValueError("waypoint must be an object or [x, y, altitude]")


def _drone(data: dict[str, Any]) -> Drone:
    """Decode one drone. Routes live only in ``waypoints`` (format 1.7+)."""

    waypoints = [_waypoint(item) for item in data.get("waypoints", [])]
    return Drone(
        id=data["id"],
        name=data["name"],
        position=_point(data["position"]),
        home_base_id=data.get("home_base_id"),
        role=str(data.get("role", "mission")),
        status=DroneStatus(data.get("status", DroneStatus.IDLE)),
        max_speed=float(data.get("max_speed", 15.0)),
        battery_capacity=float(data.get("battery_capacity", 100.0)),
        remaining_battery=float(data.get("remaining_battery", 100.0)),
        energy_per_meter=float(data.get("energy_per_meter", 0.08)),
        payload_capacity=float(data.get("payload_capacity", 3.0)),
        current_payload=float(data.get("current_payload", 0.0)),
        communication_range=float(data.get("communication_range", 180.0)),
        safety_radius=float(data.get("safety_radius", 6.0)),
        assigned_tasks=list(data.get("assigned_tasks", [])),
        waypoints=waypoints,
        cruise_altitude=float(data.get("cruise_altitude", 100.0)),
        min_clearance=float(data.get("min_clearance", 30.0)),
        climb_rate=float(data.get("climb_rate", 3.0)),
        descent_rate=float(data.get("descent_rate", 2.5)),
        hover_power=float(data.get("hover_power", 90.0)),
        ground_idle_power=float(data.get("ground_idle_power", 5.0)),
        climb_power=float(data.get("climb_power", 140.0)),
        descent_power=float(data.get("descent_power", 35.0)),
        horizontal_power=float(data.get("horizontal_power", 110.0)),
        air_speed=float(data.get("air_speed", data.get("max_speed", 15.0))),
    )


class ProjectRepository:
    """Read and write deterministic, human-readable `.dmproj` JSON files."""

    def __init__(self) -> None:
        self.last_migration_report = MigrationReport()

    def save(self, project: ProjectModel, path: str | Path) -> Path:
        validate_project(project)
        target = Path(path)
        if target.suffix.lower() != ".dmproj":
            target = target.with_suffix(".dmproj")
        target.parent.mkdir(parents=True, exist_ok=True)
        data = _json_ready(asdict(project))
        data["version"] = CURRENT_VERSION
        temporary = target.with_name(f".{target.name}.tmp")
        try:
            temporary.write_text(
                json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, target)
        except OSError:
            temporary.unlink(missing_ok=True)
            raise
        return target

    def load(self, path: str | Path) -> ProjectModel:
        """Load a project and remember the migration report of the last load."""

        project, _report = self.load_with_report(path)
        return project

    def load_with_report(self, path: str | Path) -> tuple[ProjectModel, MigrationReport]:
        """Load a project plus what the format migration changed or could not decide."""

        source = Path(path)
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ProjectFormatError(f"Cannot read project: {exc}") from exc
        if not isinstance(raw, dict):
            raise ProjectFormatError("Project root must be a JSON object")
        report = MigrationReport()
        try:
            raw = migrate_project(raw, report)
        except MigrationError as exc:
            raise ProjectFormatError(str(exc)) from exc
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise ProjectFormatError(f"Invalid project data during migration: {exc}") from exc
        version = str(raw.get("version", ""))
        if version != CURRENT_VERSION:
            raise ProjectFormatError(
                f"Unsupported project version {version or 'missing'}; expected {CURRENT_VERSION}"
            )
        try:
            project = self._decode(raw)
            validate_project(project)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProjectFormatError(f"Invalid project data: {exc}") from exc
        self.last_migration_report = report
        return project, report

    def _decode(self, raw: dict[str, Any]) -> ProjectModel:
        map_data = raw.get("map", {})
        grid_size = float(map_data.get("grid_size", 25.0))
        terrain = terrain_from_data(map_data.get("terrain"), grid_size)
        basemap_data = map_data.get("basemap")
        basemap = (
            BasemapModel(
                file=str(basemap_data.get("file", "")),
                opacity=float(basemap_data.get("opacity", 0.5)),
                visible=bool(basemap_data.get("visible", True)),
                locked=bool(basemap_data.get("locked", True)),
                meters_per_pixel=float(basemap_data.get("meters_per_pixel", 1.0)),
                origin_x=float(basemap_data.get("origin_x", 0.0)),
                origin_y=float(basemap_data.get("origin_y", 0.0)),
                flip_y=bool(basemap_data.get("flip_y", False)),
                rotation_deg=float(basemap_data.get("rotation_deg", 0.0)),
            )
            if isinstance(basemap_data, dict) and basemap_data.get("file")
            else None
        )
        map_model = MapModel(
            width=int(map_data.get("width", 1000)),
            height=int(map_data.get("height", 700)),
            grid_size=grid_size,
            terrain=terrain,
            basemap=basemap,
            wind=_wind(map_data.get("wind")),
            bases=[
                BaseStation(
                    id=item["id"],
                    name=item["name"],
                    position=_point(item["position"]),
                    communication_range=float(item.get("communication_range", 180.0)),
                )
                for item in map_data.get("bases", [])
            ],
            drones=[_drone(item) for item in map_data.get("drones", [])],
            obstacles=[
                Obstacle(
                    id=item["id"],
                    name=item["name"],
                    shape=ObstacleShape(item.get("shape", ObstacleShape.RECTANGLE)),
                    bounds=_rect(item["bounds"]),
                    points=[_point(point) for point in item.get("points", [])],
                    radius=float(item.get("radius", 0.0)),
                    height=float(item.get("height", 45.0)),
                )
                for item in map_data.get("obstacles", [])
            ],
            no_fly_zones=[
                NoFlyZone(
                    id=item["id"],
                    name=item["name"],
                    shape=ObstacleShape(item.get("shape", ObstacleShape.RECTANGLE)),
                    bounds=_rect(item["bounds"]),
                    points=[_point(point) for point in item.get("points", [])],
                    temporary=bool(item.get("temporary", False)),
                    ceiling_altitude=float(item.get("ceiling_altitude", 120.0)),
                )
                for item in map_data.get("no_fly_zones", [])
            ],
            tasks=[
                MissionTask(
                    id=item["id"],
                    name=item["name"],
                    position=_point(item["position"]),
                    task_type=TaskType(item.get("task_type", TaskType.INSPECTION)),
                    priority=int(item.get("priority", 5)),
                    status=TaskStatus(item.get("status", TaskStatus.PENDING)),
                    required_payload=float(item.get("required_payload", 0.0)),
                    earliest_start=item.get("earliest_start"),
                    deadline=item.get("deadline"),
                    deadline_policy=DeadlinePolicy(
                        item.get("deadline_policy", DeadlinePolicy.HARD)
                    ),
                    predecessor_ids=[str(value) for value in item.get("predecessor_ids", [])],
                    min_lag_seconds=float(item.get("min_lag_seconds", 0.0)),
                    execution_duration=float(item.get("execution_duration", 4.0)),
                    assigned_drone_id=item.get("assigned_drone_id"),
                    target_altitude=float(item.get("target_altitude", 100.0)),
                )
                for item in map_data.get("tasks", [])
            ],
            search_areas=[
                SearchArea(
                    id=item["id"],
                    name=item["name"],
                    bounds=_rect(item["bounds"]),
                    points=[_point(point) for point in item.get("points", [])],
                    scan_spacing=float(item.get("scan_spacing", 45.0)),
                    boundary_margin=float(item.get("boundary_margin", 8.0)),
                    target_coverage=float(item.get("target_coverage", 0.95)),
                    holes=[
                        [_point(point) for point in hole]
                        for hole in item.get("holes", [])
                        if isinstance(hole, list)
                    ],
                    priority=int(item.get("priority", 0)),
                    scan_direction=str(item.get("scan_direction", "horizontal")),
                )
                for item in map_data.get("search_areas", [])
            ],
        )
        return ProjectModel(
            name=str(raw.get("name", "Untitled mission")),
            version=CURRENT_VERSION,
            equipment=decode_equipment(raw.get("equipment")),
            georeference=_georeference(raw.get("georeference")),
            data_sources=[
                _data_source(item)
                for item in raw.get("data_sources", [])
                if isinstance(item, dict)
            ],
            map=map_model,
            planning_settings=dict(raw.get("planning_settings", {})),
            simulation_settings=dict(raw.get("simulation_settings", {})),
        )
