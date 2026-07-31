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
)

CURRENT_VERSION = "1.0"


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


class ProjectRepository:
    """Read and write deterministic, human-readable `.dmproj` JSON files."""

    def save(self, project: ProjectModel, path: str | Path) -> Path:
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
        version = str(raw.get("version", ""))
        if version != CURRENT_VERSION:
            raise ProjectFormatError(
                f"Unsupported project version {version or 'missing'}; expected {CURRENT_VERSION}"
            )
        try:
            return self._decode(raw)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProjectFormatError(f"Invalid project data: {exc}") from exc

    def _decode(self, raw: dict[str, Any]) -> ProjectModel:
        map_data = raw.get("map", {})
        map_model = MapModel(
            width=int(map_data.get("width", 1000)),
            height=int(map_data.get("height", 700)),
            grid_size=float(map_data.get("grid_size", 25.0)),
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
                )
                for item in map_data.get("tasks", [])
            ],
        )
        return ProjectModel(
            name=str(raw.get("name", "Untitled mission")),
            version=CURRENT_VERSION,
            map=map_model,
            planning_settings=dict(raw.get("planning_settings", {})),
            simulation_settings=dict(raw.get("simulation_settings", {})),
        )
