from __future__ import annotations

import json
from dataclasses import asdict
from enum import Enum
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.enums import DroneStatus, ObstacleShape, TaskStatus, TaskType
from drone_mission_planner.domain.geometry import Point, Rect
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
from drone_mission_planner.domain.terrain import TerrainModel, TerrainPeak
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.domain.wind import WindModel

from .migrations import MigrationError, migrate_project

CURRENT_VERSION = "1.3"


class ProjectFormatError(ValueError):
    """Raised when a project file cannot be parsed or migrated."""


def _json_ready(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    return value


def _point(data: dict[str, Any]) -> Point:
    return Point(float(data["x"]), float(data["y"]))


def _rect(data: dict[str, Any]) -> Rect:
    return Rect(float(data["x"]), float(data["y"]), float(data["width"]), float(data["height"]))


def _terrain_peak(data: Any) -> TerrainPeak:
    if isinstance(data, dict):
        if "center" in data:
            center = _point(data["center"])
        else:
            center = Point(float(data["center_x"]), float(data["center_y"]))
        height_raw = data.get("height", data.get("peak_height", 0.0))
        height = float(0.0 if height_raw is None else height_raw)
        return TerrainPeak(center=center, radius=float(data["radius"]), height=height)
    if isinstance(data, (list, tuple)) and len(data) == 4:
        return TerrainPeak(
            center=Point(float(data[0]), float(data[1])),
            radius=float(data[2]),
            height=float(data[3]),
        )
    raise ValueError("terrain peak must be an object or [x, y, radius, height]")


def _terrain_grid_altitudes(data: Any) -> list[list[float]]:
    if data is None:
        return []
    if not isinstance(data, list):
        raise ValueError("terrain grid altitudes must be a list")
    rows: list[list[float]] = []
    for row in data:
        if not isinstance(row, list):
            raise ValueError("terrain grid altitude rows must be lists")
        rows.append([float(value) for value in row])
    return rows


def _terrain(data: Any, fallback_resolution: float) -> TerrainModel:
    if not isinstance(data, dict):
        return TerrainModel(resolution=fallback_resolution)
    peaks = [_terrain_peak(item) for item in data.get("peaks", [])]
    grid_altitudes = _terrain_grid_altitudes(data.get("grid_altitudes", []))
    grid_origin_data = data.get("grid_origin")
    grid_origin = _point(grid_origin_data) if isinstance(grid_origin_data, dict) else None
    base_altitude_raw = data.get("base_altitude", data.get("altitude", 0.0))
    base_altitude = float(0.0 if base_altitude_raw is None else base_altitude_raw)
    grid_values = [value for row in grid_altitudes for value in row]
    if grid_values:
        default_min = min(grid_values)
        default_max = max(grid_values)
    else:
        default_min = base_altitude + sum(min(0.0, peak.height) for peak in peaks)
        default_max = base_altitude + sum(max(0.0, peak.height) for peak in peaks)
    return TerrainModel(
        terrain_type=str(data.get("terrain_type", data.get("type", "flat"))),
        resolution=float(data.get("resolution", fallback_resolution)),
        base_altitude=base_altitude,
        min_altitude=float(data.get("min_altitude", default_min)),
        max_altitude=float(data.get("max_altitude", default_max)),
        peaks=peaks,
        grid_origin=grid_origin,
        grid_width=int(data.get("grid_width", len(grid_altitudes[0]) if grid_altitudes else 0)),
        grid_height=int(data.get("grid_height", len(grid_altitudes))),
        grid_altitudes=grid_altitudes,
    )


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


class ProjectRepository:
    """Read and write deterministic, human-readable `.dmproj` JSON files."""

    def save(self, project: ProjectModel, path: str | Path) -> Path:
        validate_project(project)
        target = Path(path)
        if target.suffix.lower() != ".dmproj":
            target = target.with_suffix(".dmproj")
        target.parent.mkdir(parents=True, exist_ok=True)
        data = _json_ready(asdict(project))
        data["version"] = CURRENT_VERSION
        target.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return target

    def load(self, path: str | Path) -> ProjectModel:
        source = Path(path)
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ProjectFormatError(f"Cannot read project: {exc}") from exc
        if not isinstance(raw, dict):
            raise ProjectFormatError("Project root must be a JSON object")
        try:
            raw = migrate_project(raw)
        except MigrationError as exc:
            raise ProjectFormatError(str(exc)) from exc
        version = str(raw.get("version", ""))
        if version != CURRENT_VERSION:
            raise ProjectFormatError(
                f"Unsupported project version {version or 'missing'}; expected {CURRENT_VERSION}"
            )
        try:
            project = self._decode(raw)
            validate_project(project)
            return project
        except (KeyError, TypeError, ValueError) as exc:
            raise ProjectFormatError(f"Invalid project data: {exc}") from exc

    def _decode(self, raw: dict[str, Any]) -> ProjectModel:
        map_data = raw.get("map", {})
        grid_size = float(map_data.get("grid_size", 25.0))
        map_model = MapModel(
            width=int(map_data.get("width", 1000)),
            height=int(map_data.get("height", 700)),
            grid_size=grid_size,
            terrain=_terrain(map_data.get("terrain"), grid_size),
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
            drones=[
                Drone(
                    id=item["id"],
                    name=item["name"],
                    position=_point(item["position"]),
                    home_base_id=item.get("home_base_id"),
                    status=DroneStatus(item.get("status", DroneStatus.IDLE)),
                    max_speed=float(item.get("max_speed", 15.0)),
                    battery_capacity=float(item.get("battery_capacity", 100.0)),
                    remaining_battery=float(item.get("remaining_battery", 100.0)),
                    energy_per_meter=float(item.get("energy_per_meter", 0.08)),
                    payload_capacity=float(item.get("payload_capacity", 3.0)),
                    current_payload=float(item.get("current_payload", 0.0)),
                    communication_range=float(item.get("communication_range", 180.0)),
                    safety_radius=float(item.get("safety_radius", 6.0)),
                    assigned_tasks=list(item.get("assigned_tasks", [])),
                    planned_path=[_point(point) for point in item.get("planned_path", [])],
                    cruise_altitude=float(item.get("cruise_altitude", 100.0)),
                    min_clearance=float(item.get("min_clearance", 30.0)),
                    climb_rate=float(item.get("climb_rate", 3.0)),
                    descent_rate=float(item.get("descent_rate", 2.5)),
                    hover_power=float(item.get("hover_power", 90.0)),
                    climb_power=float(item.get("climb_power", 140.0)),
                    descent_power=float(item.get("descent_power", 35.0)),
                    horizontal_power=float(item.get("horizontal_power", 110.0)),
                    air_speed=float(item.get("air_speed", item.get("max_speed", 15.0))),
                )
                for item in map_data.get("drones", [])
            ],
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
                )
                for item in map_data.get("search_areas", [])
            ],
        )
        return ProjectModel(
            name=str(raw.get("name", "Untitled mission")),
            version=CURRENT_VERSION,
            map=map_model,
            planning_settings=dict(raw.get("planning_settings", {})),
            simulation_settings=dict(raw.get("simulation_settings", {})),
        )
