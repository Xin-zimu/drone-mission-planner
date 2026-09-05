from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, fields
from math import isfinite
from pathlib import Path
from typing import Any

from drone_mission_planner.domain.basemap import BasemapModel
from drone_mission_planner.domain.enums import AltitudeMode, TaskStatus, WaypointAction
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
MAX_HISTORY_ENTRIES = 50


@dataclass(frozen=True, slots=True)
class HistoryEntry:
    """One restorable project snapshot and the operation that replaced it."""

    label: str
    project: ProjectModel


class ProjectService:
    """Owns the active project and its file lifecycle without depending on Qt."""

    def __init__(self, repository: ProjectRepository | None = None) -> None:
        self.repository = repository or ProjectRepository()
        self.project = ProjectModel()
        self.path: Path | None = None
        self.dirty = False
        self._saved_project: ProjectModel | None = deepcopy(self.project)
        self._undo_stack: list[HistoryEntry] = []
        self._redo_stack: list[HistoryEntry] = []
        self._change_depth = 0
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
        self._saved_project = deepcopy(self.project)
        self.clear_history()
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
        self._saved_project = deepcopy(self.project)
        self.clear_history()
        self._recount()
        return self.project

    def save(self, path: str | Path | None = None) -> Path:
        target = Path(path) if path is not None else self.path
        if target is None:
            raise ValueError("A target path is required for a new project")
        self.path = self.repository.save(self.project, target)
        self.dirty = False
        self._saved_project = deepcopy(self.project)
        return self.path

    def restore_recovery(self, project: ProjectModel, source_path: str | Path | None) -> None:
        """Adopt an autosaved project while keeping it explicitly unsaved."""

        self.project = deepcopy(project)
        self.path = Path(source_path) if source_path else None
        self._saved_project = None
        self.dirty = True
        self.clear_history()
        self._recount()

    @contextmanager
    def change(self, label: str) -> Iterator[None]:
        """Group model mutations into one undo step and roll back failed changes."""

        if self._change_depth:
            self._change_depth += 1
            try:
                yield
            finally:
                self._change_depth -= 1
            return

        before = deepcopy(self.project)
        before_dirty = self.dirty
        self._change_depth = 1
        try:
            yield
        except Exception:
            if self.project != before:
                self.project = before
                self._recount()
            self.dirty = before_dirty
            raise
        else:
            if self.project != before:
                self._undo_stack.append(HistoryEntry(label, before))
                del self._undo_stack[:-MAX_HISTORY_ENTRIES]
                self._redo_stack.clear()
                self._update_dirty()
        finally:
            self._change_depth = 0

    def undo(self) -> str | None:
        """Restore the previous model snapshot and return the reverted operation label."""

        if not self._undo_stack:
            return None
        entry = self._undo_stack.pop()
        self._redo_stack.append(HistoryEntry(entry.label, deepcopy(self.project)))
        self.project = entry.project
        self._recount()
        self._update_dirty()
        return entry.label

    def redo(self) -> str | None:
        """Reapply the most recently undone model snapshot."""

        if not self._redo_stack:
            return None
        entry = self._redo_stack.pop()
        self._undo_stack.append(HistoryEntry(entry.label, deepcopy(self.project)))
        self.project = entry.project
        self._recount()
        self._update_dirty()
        return entry.label

    @property
    def undo_label(self) -> str | None:
        return self._undo_stack[-1].label if self._undo_stack else None

    @property
    def redo_label(self) -> str | None:
        return self._redo_stack[-1].label if self._redo_stack else None

    def clear_history(self) -> None:
        self._undo_stack.clear()
        self._redo_stack.clear()

    def _update_dirty(self) -> None:
        self.dirty = self._saved_project is None or self.project != self._saved_project

    def add_base(self, position: Point) -> BaseStation:
        with self.change("Add base"):
            index = self._next("base")
            item = BaseStation(f"B-{index:02d}", f"Base {index}", position)
            self.project.map.bases.append(item)
        return item

    def add_drone(self, position: Point) -> Drone:
        with self.change("Add drone"):
            index = self._next("drone")
            base_id = self.project.map.bases[0].id if self.project.map.bases else None
            item = Drone(f"D-{index:02d}", f"Drone {index}", position, home_base_id=base_id)
            self.project.map.drones.append(item)
        return item

    def add_task(self, position: Point) -> MissionTask:
        with self.change("Add mission"):
            index = self._next("task")
            item = MissionTask(f"T-{index:02d}", f"Inspection {index}", position)
            self.project.map.tasks.append(item)
        return item

    def add_obstacle(self, bounds: Rect) -> Obstacle:
        with self.change("Add obstacle"):
            index = self._next("obstacle")
            item = Obstacle(f"O-{index:02d}", f"Obstacle {index}", bounds=bounds.normalized)
            self.project.map.obstacles.append(item)
        return item

    def add_no_fly_zone(self, bounds: Rect, *, temporary: bool = False) -> NoFlyZone:
        with self.change("Add no-fly zone"):
            index = self._next("no_fly")
            item = NoFlyZone(
                f"N-{index:02d}",
                f"No-fly zone {index}",
                bounds=bounds.normalized,
                temporary=temporary,
            )
            self.project.map.no_fly_zones.append(item)
        return item

    def add_search_area(self, bounds: Rect) -> SearchArea:
        with self.change("Add search area"):
            index = self._next("search")
            item = SearchArea(f"S-{index:02d}", f"Search area {index}", bounds.normalized)
            self.project.map.search_areas.append(item)
        return item

    def remove(self, object_id: str) -> MapObject | None:
        with self.change("Delete object"):
            removed = self.project.map.remove(object_id)
            if isinstance(removed, BaseStation):
                replacement_base_id = (
                    self.project.map.bases[0].id if self.project.map.bases else None
                )
                for drone in self.project.map.drones:
                    if drone.home_base_id == removed.id:
                        drone.home_base_id = replacement_base_id
            elif isinstance(removed, Drone):
                for task in self.project.map.tasks:
                    if task.assigned_drone_id == removed.id:
                        task.assigned_drone_id = None
                        if task.status == TaskStatus.ASSIGNED:
                            task.status = TaskStatus.PENDING
            elif isinstance(removed, MissionTask):
                for drone in self.project.map.drones:
                    drone.assigned_tasks = [
                        task_id for task_id in drone.assigned_tasks if task_id != removed.id
                    ]
                    for waypoint in drone.waypoints:
                        if waypoint.task_id == removed.id:
                            waypoint.task_id = None
            if removed is not None:
                validate_project(self.project)
        return removed

    def update_environment(self, terrain: TerrainModel, wind: WindModel) -> None:
        with self.change("Update environment"):
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

    def update_property(self, object_id: str, name: str, value: Any) -> MapObject:
        item = self.project.map.find(object_id)
        if item is None:
            raise KeyError(object_id)
        allowed = {field.name for field in fields(item)} - {"id"}
        if name not in allowed:
            raise ValueError(f"Property {name!r} is not editable")
        with self.change(f"Edit {item.id}"):
            previous = getattr(item, name)
            setattr(item, name, value)
            try:
                validate_project(self.project)
            except ValueError:
                setattr(item, name, previous)
                raise
        return item

    def set_basemap_file(self, file: str) -> BasemapModel:
        """Attach a local image basemap (or clear it with an empty string)."""

        with self.change("Import basemap"):
            self.project.map.basemap = BasemapModel(file=file)
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
        with self.change("Edit basemap"):
            for name, value in fields.items():
                if name not in allowed:
                    raise ValueError(f"Basemap field {name!r} is not editable")
                setattr(basemap, name, value)
        return basemap

    def create_drone_from_model(self, model_name: str, position: Point) -> Drone:
        """Add a drone whose parameters come from an equipment model."""

        model = self.project.equipment.drone_model(model_name)
        if model is None:
            raise KeyError(f"Unknown drone model {model_name!r}")
        with self.change("Add drone from equipment model"):
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
        with self.change("Change drone battery"):
            drone = self._drone(drone_id)
            drone.battery_capacity = pack.capacity
            drone.remaining_battery = pack.effective_capacity()
        return drone

    def attach_payload(self, drone_id: str, payload_name: str) -> Drone:
        """Mount a payload, increasing the drone's carried weight."""

        payload = self.project.equipment.payload(payload_name)
        if payload is None:
            raise KeyError(f"Unknown payload {payload_name!r}")
        with self.change("Attach payload"):
            drone = self._drone(drone_id)
            drone.current_payload += payload.weight
        return drone

    def apply_mission_template(self, template_name: str) -> dict[str, float | int | bool | str]:
        """Merge a mission template's settings into planning settings."""

        template = self.project.equipment.mission_template(template_name)
        if template is None:
            raise KeyError(f"Unknown mission template {template_name!r}")
        with self.change("Apply mission template"):
            self.project.planning_settings.update(template.settings)
        return dict(template.settings)

    def update_waypoint(self, drone_id: str, index: int, name: str, value: Any) -> Waypoint:
        """Edit one editable field of a drone waypoint and keep the path in sync."""

        drone = self._drone(drone_id)
        if name not in EDITABLE_WAYPOINT_FIELDS:
            raise ValueError(f"Waypoint field {name!r} is not editable")
        if not 0 <= index < len(drone.waypoints):
            raise IndexError(f"Waypoint index {index} is out of range")
        with self.change("Edit waypoint"):
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
        return waypoint

    def replace_waypoints(self, drone_id: str, waypoints: list[Waypoint]) -> Drone:
        """Replace a drone's whole waypoint list (imports, presets, planning)."""

        drone = self._drone(drone_id)
        with self.change("Replace route waypoints"):
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
        with self.change("Delete waypoint"):
            removed = drone.waypoints.pop(index)
            self._sync_waypoint_path(drone)
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
        def highest(items: list[MapObject], prefix: str) -> int:
            values = []
            for item in items:
                head, separator, suffix = item.id.rpartition("-")
                if separator and head == prefix and suffix.isdigit():
                    values.append(int(suffix))
            return max(values, default=0)

        self._counters = {
            "base": highest(list(self.project.map.bases), "B"),
            "drone": highest(list(self.project.map.drones), "D"),
            "obstacle": highest(list(self.project.map.obstacles), "O"),
            "no_fly": highest(list(self.project.map.no_fly_zones), "N"),
            "task": highest(list(self.project.map.tasks), "T"),
            "search": highest(list(self.project.map.search_areas), "S"),
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
