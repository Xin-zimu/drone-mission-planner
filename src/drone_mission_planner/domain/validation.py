from __future__ import annotations

from math import ceil, isfinite

from .geometry import Point, Rect
from .models import ProjectModel
from .terrain import TerrainModel


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
    if model.terrain.terrain_type not in {"flat", "procedural", "grid"}:
        issues.append("terrain type must be flat, procedural, or grid")
    if not isfinite(model.terrain.resolution) or model.terrain.resolution <= 0:
        issues.append("terrain resolution must be positive")
    if not all(
        isfinite(value)
        for value in (
            model.terrain.base_altitude,
            model.terrain.min_altitude,
            model.terrain.max_altitude,
        )
    ):
        issues.append("terrain altitude values must be finite")
    elif model.terrain.min_altitude > model.terrain.max_altitude:
        issues.append("terrain minimum altitude cannot exceed maximum altitude")
    for index, peak in enumerate(model.terrain.peaks, start=1):
        if not isfinite(peak.center.x) or not isfinite(peak.center.y):
            issues.append(f"terrain peak {index} center must be finite")
        if not isfinite(peak.radius) or peak.radius <= 0:
            issues.append(f"terrain peak {index} radius must be positive")
        if not isfinite(peak.height):
            issues.append(f"terrain peak {index} height must be finite")
    if model.terrain.terrain_type == "grid":
        _terrain_grid(model.terrain, issues)
    if not isfinite(model.wind.direction_to_deg):
        issues.append("wind direction must be finite")
    if not isfinite(model.wind.speed) or model.wind.speed < 0:
        issues.append("wind speed cannot be negative")
    if not isfinite(model.wind.gust_factor) or not 0 <= model.wind.gust_factor <= 1:
        issues.append("wind gust factor must be between 0 and 1")

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
        if drone.cruise_altitude < 0 or drone.min_clearance < 0:
            issues.append(f"{drone.id} altitude settings cannot be negative")
        if drone.climb_rate <= 0 or drone.descent_rate <= 0 or drone.air_speed < 0:
            issues.append(
                f"{drone.id} climb/descent speeds must be positive and air speed cannot be negative"
            )
        if not all(
            isfinite(value)
            for value in (
                drone.cruise_altitude,
                drone.min_clearance,
                drone.climb_rate,
                drone.descent_rate,
                drone.hover_power,
                drone.climb_power,
                drone.descent_power,
                drone.horizontal_power,
                drone.air_speed,
            )
        ):
            issues.append(f"{drone.id} flight energy parameters must be finite")
        if (
            drone.hover_power < 0
            or drone.climb_power < 0
            or drone.descent_power < 0
            or drone.horizontal_power < 0
        ):
            issues.append(f"{drone.id} flight power settings cannot be negative")
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
        if drone.role not in {"mission", "relay"}:
            issues.append(f"{drone.id} role must be mission or relay")
        for waypoint_index, waypoint in enumerate(drone.waypoints, start=1):
            _position(
                f"{drone.id} waypoint {waypoint_index}",
                waypoint.point,
                model.width,
                model.height,
                issues,
            )
            if not isfinite(waypoint.altitude) or waypoint.altitude < 0:
                issues.append(f"{drone.id} waypoint {waypoint_index} altitude must be non-negative")
            if waypoint.speed is not None and (
                not isfinite(waypoint.speed) or waypoint.speed <= 0
            ):
                issues.append(f"{drone.id} waypoint {waypoint_index} speed must be positive")
            if not isfinite(waypoint.hold_seconds) or waypoint.hold_seconds < 0:
                issues.append(
                    f"{drone.id} waypoint {waypoint_index} hold seconds must be non-negative"
                )
    for task in model.tasks:
        _position(task.id, task.position, model.width, model.height, issues)
        if not 0 <= task.priority <= 10:
            issues.append(f"{task.id} priority must be between 0 and 10")
        if task.required_payload < 0 or task.execution_duration < 0:
            issues.append(f"{task.id} payload and duration cannot be negative")
        if not isfinite(task.target_altitude) or task.target_altitude < 0:
            issues.append(f"{task.id} target altitude must be non-negative")
        if task.assigned_drone_id is not None and task.assigned_drone_id not in drone_ids:
            issues.append(f"{task.id} references missing drone {task.assigned_drone_id}")
    for item in model.obstacles:
        _rect(item.id, item.bounds, model.width, model.height, issues)
        if not isfinite(item.height) or item.height < 0:
            issues.append(f"{item.id} height must be non-negative")
    for zone in model.no_fly_zones:
        _rect(zone.id, zone.bounds, model.width, model.height, issues)
        if not isfinite(zone.ceiling_altitude) or zone.ceiling_altitude < 0:
            issues.append(f"{zone.id} ceiling altitude must be non-negative")
    for area in model.search_areas:
        _rect(area.id, area.bounds, model.width, model.height, issues)
        if area.scan_spacing <= 0 or area.boundary_margin < 0:
            issues.append(f"{area.id} scan spacing/margin is invalid")
        if not 0 < area.target_coverage <= 1:
            issues.append(f"{area.id} target coverage must be in (0, 1]")
        if area.scan_direction not in {"horizontal", "vertical"}:
            issues.append(f"{area.id} scan direction must be horizontal or vertical")
        for point in area.points:
            _position(area.id, point, model.width, model.height, issues)
        for hole_index, hole in enumerate(area.holes, start=1):
            if len(hole) < 3:
                issues.append(f"{area.id} hole {hole_index} must contain at least three points")
            for point in hole:
                _position(f"{area.id} hole {hole_index}", point, model.width, model.height, issues)
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


def _terrain_grid(terrain: TerrainModel, issues: list[str]) -> None:
    origin = terrain.grid_origin
    if origin is None or not isfinite(origin.x) or not isfinite(origin.y):
        issues.append("terrain grid origin must be finite")
    if terrain.grid_width <= 0 or terrain.grid_height <= 0:
        issues.append("terrain grid dimensions must be positive")
    if len(terrain.grid_altitudes) != terrain.grid_height:
        issues.append("terrain grid height must match altitude rows")
        return
    values: list[float] = []
    for row_index, row in enumerate(terrain.grid_altitudes, start=1):
        if len(row) != terrain.grid_width:
            issues.append(f"terrain grid row {row_index} width must match grid width")
        for value in row:
            if not isfinite(value):
                issues.append(f"terrain grid row {row_index} altitude values must be finite")
                continue
            values.append(value)
    if not values:
        return
    actual_min = min(values)
    actual_max = max(values)
    tolerance = 1e-6
    if (
        abs(actual_min - terrain.min_altitude) > tolerance
        or abs(actual_max - terrain.max_altitude) > tolerance
    ):
        issues.append("terrain grid altitude range must match grid samples")
