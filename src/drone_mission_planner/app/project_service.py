from __future__ import annotations

from dataclasses import fields
from math import isfinite
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.basemap import BasemapModel
from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MapObject,
    MissionTask,
    NoFlyZone,
    Obstacle,
    ProjectModel,
    SearchArea,
)
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.domain.waypoint import Waypoint, path_from_waypoints
from drone_mission_planner.domain.wind import WindModel
from drone_mission_planner.persistence.project_repository import ProjectRepository

EDITABLE_WAYPOINT_FIELDS = ("altitude", "altitude_mode", "speed", "action", "hold_seconds")


class ProjectService:
    """Owns the active project and its file lifecycle without depending on Qt."""

    def __init__(self, repository: ProjectRepository | None = None) -> None:
        self.repository = repository or ProjectRepository()
        self.project = ProjectModel()
        self.path: Path | None = None
        self.dirty = False
        self._counters: dict[str, int] = {
            "base": 0,
            "drone": 0,
            "obstacle": 0,
            "no_fly": 0,
            "task": 0,
            "search": 0,
        }

    def new_project(self, name: str = "Untitled mission") -> ProjectModel:
        self.project = ProjectModel(name=name)
        self.path = None
        self.dirty = False
        self._counters = {
            "base": 0,
            "drone": 0,
            "obstacle": 0,
            "no_fly": 0,
            "task": 0,
            "search": 0,
        }
        return self.project

    def load(self, path: str | Path) -> ProjectModel:
        self.project = self.repository.load(path)
        self.path = Path(path)
        self.dirty = False
        self._recount()
        return self.project

    def save(self, path: str | Path | None = None) -> Path:
        target = Path(path) if path is not None else self.path
        if target is None:
            raise ValueError("A target path is required for a new project")
        self.path = self.repository.save(self.project, target)
        self.dirty = False
        return self.path

    def add_base(self, position: Point) -> BaseStation:
        index = self._next("base")
        item = BaseStation(f"B-{index:02d}", f"Base {index}", position)
        self.project.map.bases.append(item)
        self.dirty = True
        return item

    def add_drone(self, position: Point) -> Drone:
        index = self._next("drone")
        base_id = self.project.map.bases[0].id if self.project.map.bases else None
        item = Drone(f"D-{index:02d}", f"Drone {index}", position, home_base_id=base_id)
        self.project.map.drones.append(item)
        self.dirty = True
        return item

    def add_task(self, position: Point) -> MissionTask:
        index = self._next("task")
        item = MissionTask(f"T-{index:02d}", f"Inspection {index}", position)
        self.project.map.tasks.append(item)
        self.dirty = True
        return item

    def add_obstacle(self, bounds: Rect) -> Obstacle:
        index = self._next("obstacle")
        item = Obstacle(f"O-{index:02d}", f"Obstacle {index}", bounds=bounds.normalized)
        self.project.map.obstacles.append(item)
        self.dirty = True
        return item

    def add_no_fly_zone(self, bounds: Rect, *, temporary: bool = False) -> NoFlyZone:
        index = self._next("no_fly")
        item = NoFlyZone(
            f"N-{index:02d}",
            f"No-fly zone {index}",
            bounds=bounds.normalized,
            temporary=temporary,
        )
        self.project.map.no_fly_zones.append(item)
        self.dirty = True
        return item

    def add_search_area(self, bounds: Rect) -> SearchArea:
        index = self._next("search")
        item = SearchArea(f"S-{index:02d}", f"Search area {index}", bounds.normalized)
        self.project.map.search_areas.append(item)
        self.dirty = True
        return item

    def remove(self, object_id: str) -> MapObject | None:
        removed = self.project.map.remove(object_id)
        if removed is not None:
            self.dirty = True
        return removed

    def update_environment(self, terrain: TerrainModel, wind: WindModel) -> None:
        previous_terrain = self.project.map.terrain
        previous_wind = self.project.map.wind
        self.project.map.terrain = terrain
        self.project.map.wind = wind
        try:
            validate_project(self.project)
        except ValueError:
            self.project.map.terrain = previous_terrain
            self.project.map.wind = previous_wind
            raise
        self.dirty = True

    def update_property(self, object_id: str, name: str, value: Any) -> MapObject:
        item = self.project.map.find(object_id)
        if item is None:
            raise KeyError(object_id)
        allowed = {field.name for field in fields(item)} - {"id"}
        if name not in allowed:
            raise ValueError(f"Property {name!r} is not editable")
        previous = getattr(item, name)
        setattr(item, name, value)
        try:
            validate_project(self.project)
        except ValueError:
            setattr(item, name, previous)
            raise
        self.dirty = True
        return item

    def set_basemap_file(self, file: str) -> BasemapModel:
        """Attach a local image basemap (or clear it with an empty string)."""

        self.project.map.basemap = BasemapModel(file=file)
        self.dirty = True
        return self.project.map.basemap

    def update_basemap(self, **fields: Any) -> BasemapModel:
        """Update calibration/display fields of the current basemap."""

        basemap = self.project.map.basemap
        if basemap is None:
            raise ValueError("no basemap imported")
        allowed = {
            "opacity", "visible", "locked", "meters_per_pixel",
            "origin_x", "origin_y", "flip_y", "rotation_deg",
        }
        for name, value in fields.items():
            if name not in allowed:
                raise ValueError(f"Basemap field {name!r} is not editable")
            setattr(basemap, name, value)
        self.dirty = True
        return basemap

    def create_drone_from_model(self, model_name: str, position: Point) -> Drone:
        """Add a drone whose parameters come from an equipment model."""

        model = self.project.equipment.drone_model(model_name)
        if model is None:
            raise KeyError(f"Unknown drone model {model_name!r}")
        drone = self.add_drone(position)
        for field_name in (
            "max_speed",
            "air_speed",
            "payload_capacity",
            "communication_range",
            "energy_per_meter",
            "cruise_altitude",
            "min_clearance",
            "climb_rate",
            "descent_rate",
            "hover_power",
            "climb_power",
            "descent_power",
            "horizontal_power",
        ):
            setattr(drone, field_name, getattr(model, field_name))
        return drone

    def set_drone_battery(self, drone_id: str, battery_name: str) -> Drone:
        """Fit a battery pack and recompute the usable energy budget."""

        pack = self.project.equipment.battery(battery_name)
        if pack is None:
            raise KeyError(f"Unknown battery pack {battery_name!r}")
        drone = self._drone(drone_id)
        drone.battery_capacity = pack.capacity
        drone.remaining_battery = pack.effective_capacity()
        self.dirty = True
        return drone

    def attach_payload(self, drone_id: str, payload_name: str) -> Drone:
        """Mount a payload, increasing the drone's carried weight."""

        payload = self.project.equipment.payload(payload_name)
        if payload is None:
            raise KeyError(f"Unknown payload {payload_name!r}")
        drone = self._drone(drone_id)
        drone.current_payload += payload.weight
        self.dirty = True
        return drone

    def apply_mission_template(self, template_name: str) -> dict[str, float | int | bool | str]:
        """Merge a mission template's settings into planning settings."""

        template = self.project.equipment.mission_template(template_name)
        if template is None:
            raise KeyError(f"Unknown mission template {template_name!r}")
        self.project.planning_settings.update(template.settings)
        self.dirty = True
        return dict(template.settings)

    def update_waypoint(self, drone_id: str, index: int, name: str, value: Any) -> Waypoint:
        """Edit one editable field of a drone waypoint and keep the path in sync."""

        drone = self._drone(drone_id)
        if name not in EDITABLE_WAYPOINT_FIELDS:
            raise ValueError(f"Waypoint field {name!r} is not editable")
        if not 0 <= index < len(drone.waypoints):
            raise IndexError(f"Waypoint index {index} is out of range")
        waypoint = drone.waypoints[index]
        previous = getattr(waypoint, name)
        _apply_waypoint_field(waypoint, name, value)
        self._sync_waypoint_path(drone)
        try:
            validate_project(self.project)
        except ValueError:
            setattr(waypoint, name, previous)
            self._sync_waypoint_path(drone)
            raise
        self.dirty = True
        return waypoint

    def replace_waypoints(self, drone_id: str, waypoints: list[Waypoint]) -> Drone:
        """Replace a drone's whole waypoint list (imports, presets, planning)."""

        drone = self._drone(drone_id)
        previous = list(drone.waypoints)
        previous_path = list(drone.planned_path)
        drone.waypoints = list(waypoints)
        self._sync_waypoint_path(drone)
        try:
            validate_project(self.project)
        except ValueError:
            drone.waypoints = previous
            drone.planned_path = previous_path
            raise
        self.dirty = True
        return drone

    def remove_waypoint(self, drone_id: str, index: int) -> Waypoint:
        """Delete a waypoint when it is safe to remove, keeping the path in sync."""

        drone = self._drone(drone_id)
        if not 0 <= index < len(drone.waypoints):
            raise IndexError(f"Waypoint index {index} is out of range")
        waypoint = drone.waypoints[index]
        if index == 0:
            raise ValueError("The departure waypoint cannot be deleted")
        if waypoint.task_id is not None:
            raise ValueError(
                f"Waypoint {index + 1} is linked to {waypoint.task_id}; cancel or reassign "
                "the mission instead of deleting it"
            )
        if waypoint.action == WaypointAction.RETURN_TO_LAUNCH:
            raise ValueError("The return-to-launch waypoint cannot be deleted")
        if any(item.action == WaypointAction.SCAN for item in drone.waypoints):
            raise ValueError(
                "Coverage scan routes only allow altitude and speed adjustments"
            )
        removed = drone.waypoints.pop(index)
        self._sync_waypoint_path(drone)
        self.dirty = True
        return removed

    def _drone(self, drone_id: str) -> Drone:
        item = self.project.map.find(drone_id)
        if not isinstance(item, Drone):
            raise KeyError(drone_id)
        return item

    @staticmethod
    def _sync_waypoint_path(drone: Drone) -> None:
        drone.planned_path = path_from_waypoints(drone.waypoints)

    def _next(self, kind: str) -> int:
        self._counters[kind] += 1
        return self._counters[kind]

    def _recount(self) -> None:
        self._counters = {
            "base": len(self.project.map.bases),
            "drone": len(self.project.map.drones),
            "obstacle": len(self.project.map.obstacles),
            "no_fly": len(self.project.map.no_fly_zones),
            "task": len(self.project.map.tasks),
            "search": len(self.project.map.search_areas),
        }


def _apply_waypoint_field(waypoint: Waypoint, name: str, value: Any) -> None:
    """Coerce and validate one waypoint field, raising ValueError on bad input."""

    if name == "altitude":
        altitude = float(value)
        if not isfinite(altitude) or altitude < 0.0:
            raise ValueError("Waypoint altitude must be a non-negative number")
        waypoint.altitude = altitude
    elif name == "altitude_mode":
        waypoint.altitude_mode = AltitudeMode(str(value).lower())
    elif name == "speed":
        if value is None or str(value).strip().lower() in {"", "none", "auto"}:
            waypoint.speed = None
            return
        speed = float(value)
        if not isfinite(speed) or speed <= 0.0:
            raise ValueError("Waypoint speed must be positive or empty for the cruise speed")
        waypoint.speed = speed
    elif name == "action":
        waypoint.action = WaypointAction(str(value).lower())
    elif name == "hold_seconds":
        hold = float(value)
        if not isfinite(hold) or hold < 0.0:
            raise ValueError("Waypoint hold time must be a non-negative number")
        waypoint.hold_seconds = hold
    else:
        raise ValueError(f"Waypoint field {name!r} is not editable")
