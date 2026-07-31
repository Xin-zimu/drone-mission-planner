from __future__ import annotations

from itertools import pairwise

from drone_mission_planner.domain.geometry import Point

from .grid import GridMap


def validate_path(points: list[Point], grid: GridMap) -> tuple[bool, str | None]:
    if not points:
        return False, "Path has no waypoints"
    for point in points:
        if grid.is_blocked(grid.world_to_cell(point)):
            return False, f"Waypoint ({point.x:.1f}, {point.y:.1f}) is unsafe"
    for start, end in pairwise(points):
        if not grid.segment_is_clear(start, end):
            return False, "A path segment crosses an obstacle or no-fly zone"
    return True, None
