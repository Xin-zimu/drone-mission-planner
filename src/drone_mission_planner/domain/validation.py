from __future__ import annotations

from math import ceil, isfinite

from .geometry import Point, Rect
from .models import ProjectModel


class ProjectValidationError(ValueError):
    def __init__(self, issues: list[str]) -> None:
        self.issues = tuple(issues)
        super().__init__("; ".join(issues))


def validate_project(project: ProjectModel) -> None:
    model = project.map
    issues: list[str] = []
    if model.width <= 0 or model.height <= 0:
        issues.append("map dimensions must be positive")
    if not isfinite(model.grid_size) or model.grid_size <= 0:
        issues.append("map grid size must be positive")
    elif ceil(model.width / model.grid_size) > 500 or ceil(model.height / model.grid_size) > 500:
        issues.append("planning grid may not exceed 500 x 500 cells")

    ids = [item.id for item in model.objects()]
    if len(ids) != len(set(ids)):
        issues.append("object IDs must be unique")
    base_ids = {item.id for item in model.bases}
    drone_ids = {item.id for item in model.drones}

    for base in model.bases:
        _position(base.id, base.position, model.width, model.height, issues)
        if base.communication_range <= 0:
            issues.append(f"{base.id} communication range must be positive")
    for drone in model.drones:
        _position(drone.id, drone.position, model.width, model.height, issues)
        if drone.home_base_id is not None and drone.home_base_id not in base_ids:
            issues.append(f"{drone.id} references missing home base {drone.home_base_id}")
        if drone.max_speed <= 0:
            issues.append(f"{drone.id} max speed must be positive")
        if drone.battery_capacity <= 0:
            issues.append(f"{drone.id} battery capacity must be positive")
        if not 0 <= drone.remaining_battery <= drone.battery_capacity:
            issues.append(f"{drone.id} remaining battery must be within capacity")
        if drone.energy_per_meter < 0:
            issues.append(f"{drone.id} energy per metre cannot be negative")
        if drone.payload_capacity < 0 or not 0 <= drone.current_payload <= drone.payload_capacity:
            issues.append(f"{drone.id} payload values are inconsistent")
        if drone.communication_range <= 0:
            issues.append(f"{drone.id} communication range must be positive")
        if drone.safety_radius < 0:
            issues.append(f"{drone.id} safety radius cannot be negative")
    for task in model.tasks:
        _position(task.id, task.position, model.width, model.height, issues)
        if not 0 <= task.priority <= 10:
            issues.append(f"{task.id} priority must be between 0 and 10")
        if task.required_payload < 0 or task.execution_duration < 0:
            issues.append(f"{task.id} payload and duration cannot be negative")
        if task.assigned_drone_id is not None and task.assigned_drone_id not in drone_ids:
            issues.append(f"{task.id} references missing drone {task.assigned_drone_id}")
    for item in model.obstacles:
        _rect(item.id, item.bounds, model.width, model.height, issues)
    for zone in model.no_fly_zones:
        _rect(zone.id, zone.bounds, model.width, model.height, issues)
    for area in model.search_areas:
        _rect(area.id, area.bounds, model.width, model.height, issues)
        if area.scan_spacing <= 0 or area.boundary_margin < 0:
            issues.append(f"{area.id} scan spacing/margin is invalid")
        if not 0 < area.target_coverage <= 1:
            issues.append(f"{area.id} target coverage must be in (0, 1]")
        for point in area.points:
            _position(area.id, point, model.width, model.height, issues)
    if issues:
        raise ProjectValidationError(issues)


def _position(object_id: str, point: Point, width: float, height: float, issues: list[str]) -> None:
    if not (isfinite(point.x) and isfinite(point.y)):
        issues.append(f"{object_id} position must be finite")
    elif not (0 <= point.x <= width and 0 <= point.y <= height):
        issues.append(f"{object_id} position is outside the map")


def _rect(object_id: str, rect: Rect, width: float, height: float, issues: list[str]) -> None:
    bounds = rect.normalized
    if bounds.width <= 0 or bounds.height <= 0:
        issues.append(f"{object_id} bounds must have positive size")
    if (
        bounds.x < 0
        or bounds.y < 0
        or bounds.x + bounds.width > width
        or bounds.y + bounds.height > height
    ):
        issues.append(f"{object_id} bounds extend outside the map")
