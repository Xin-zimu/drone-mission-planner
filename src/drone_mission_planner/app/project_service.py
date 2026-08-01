from __future__ import annotations

from dataclasses import fields
from pathlib import Path
from typing import Any

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
from drone_mission_planner.domain.validation import validate_project
from drone_mission_planner.persistence.project_repository import ProjectRepository


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
