from __future__ import annotations

from dataclasses import dataclass, field

from .enums import DroneStatus, ObstacleShape, TaskStatus, TaskType
from .geometry import Point, Rect


@dataclass(slots=True)
class BaseStation:
    id: str
    name: str
    position: Point
    communication_range: float = 180.0


@dataclass(slots=True)
class Drone:
    id: str
    name: str
    position: Point
    home_base_id: str | None = None
    status: DroneStatus = DroneStatus.IDLE
    max_speed: float = 15.0
    battery_capacity: float = 100.0
    remaining_battery: float = 100.0
    energy_per_meter: float = 0.08
    payload_capacity: float = 3.0
    current_payload: float = 0.0
    communication_range: float = 180.0
    safety_radius: float = 6.0
    assigned_tasks: list[str] = field(default_factory=list)
    planned_path: list[Point] = field(default_factory=list)


@dataclass(slots=True)
class Obstacle:
    id: str
    name: str
    shape: ObstacleShape = ObstacleShape.RECTANGLE
    bounds: Rect = field(default_factory=lambda: Rect(0.0, 0.0, 30.0, 30.0))
    points: list[Point] = field(default_factory=list)
    radius: float = 0.0


@dataclass(slots=True)
class NoFlyZone:
    id: str
    name: str
    shape: ObstacleShape = ObstacleShape.RECTANGLE
    bounds: Rect = field(default_factory=lambda: Rect(0.0, 0.0, 30.0, 30.0))
    points: list[Point] = field(default_factory=list)
    temporary: bool = False


@dataclass(slots=True)
class MissionTask:
    id: str
    name: str
    position: Point
    task_type: TaskType = TaskType.INSPECTION
    priority: int = 5
    status: TaskStatus = TaskStatus.PENDING
    required_payload: float = 0.0
    earliest_start: float | None = None
    deadline: float | None = None
    execution_duration: float = 4.0
    assigned_drone_id: str | None = None


@dataclass(slots=True)
class SearchArea:
    id: str
    name: str
    bounds: Rect
    points: list[Point] = field(default_factory=list)
    scan_spacing: float = 45.0
    boundary_margin: float = 8.0
    target_coverage: float = 0.95

    def polygon(self) -> list[Point]:
        if self.points:
            return list(self.points)
        rect = self.bounds.normalized
        return [
            Point(rect.x, rect.y),
            Point(rect.x + rect.width, rect.y),
            Point(rect.x + rect.width, rect.y + rect.height),
            Point(rect.x, rect.y + rect.height),
        ]


type MapObject = BaseStation | Drone | Obstacle | NoFlyZone | MissionTask | SearchArea


@dataclass(slots=True)
class MapModel:
    width: int = 1000
    height: int = 700
    grid_size: float = 25.0
    obstacles: list[Obstacle] = field(default_factory=list)
    no_fly_zones: list[NoFlyZone] = field(default_factory=list)
    bases: list[BaseStation] = field(default_factory=list)
    drones: list[Drone] = field(default_factory=list)
    tasks: list[MissionTask] = field(default_factory=list)
    search_areas: list[SearchArea] = field(default_factory=list)

    def objects(self) -> list[MapObject]:
        return [
            *self.bases,
            *self.drones,
            *self.obstacles,
            *self.no_fly_zones,
            *self.tasks,
            *self.search_areas,
        ]

    def find(self, object_id: str) -> MapObject | None:
        return next((item for item in self.objects() if item.id == object_id), None)

    def remove(self, object_id: str) -> MapObject | None:
        item = self.find(object_id)
        if isinstance(item, BaseStation):
            self.bases.remove(item)
        elif isinstance(item, Drone):
            self.drones.remove(item)
        elif isinstance(item, Obstacle):
            self.obstacles.remove(item)
        elif isinstance(item, NoFlyZone):
            self.no_fly_zones.remove(item)
        elif isinstance(item, MissionTask):
            self.tasks.remove(item)
        elif isinstance(item, SearchArea):
            self.search_areas.remove(item)
        else:
            return None
        return item

    def validate_position(self, point: Point) -> bool:
        return 0.0 <= point.x <= self.width and 0.0 <= point.y <= self.height


@dataclass(slots=True)
class ProjectModel:
    name: str = "Untitled mission"
    version: str = "1.0"
    map: MapModel = field(default_factory=MapModel)
    planning_settings: dict[str, float | int | bool | str] = field(default_factory=dict)
    simulation_settings: dict[str, float | int | bool | str] = field(
        default_factory=lambda: {
            "fixed_dt": 0.05,
            "random_seed": 42,
            "communication_policy": "log_only",
            "communication_grace": 5.0,
        }
    )
