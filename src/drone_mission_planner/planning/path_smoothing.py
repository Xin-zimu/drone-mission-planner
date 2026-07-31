from __future__ import annotations

from drone_mission_planner.domain.geometry import Point

from .grid import GridMap


def remove_collinear(points: list[Point], *, epsilon: float = 1e-9) -> list[Point]:
    if len(points) < 3:
        return list(points)
    result = [points[0]]
    for index in range(1, len(points) - 1):
        previous = result[-1]
        current = points[index]
        following = points[index + 1]
        cross = (current.x - previous.x) * (following.y - current.y) - (current.y - previous.y) * (
            following.x - current.x
        )
        if abs(cross) > epsilon:
            result.append(current)
    result.append(points[-1])
    return result


def smooth_path(points: list[Point], grid: GridMap) -> list[Point]:
    compact = remove_collinear(points)
    if len(compact) < 3:
        return compact
    result = [compact[0]]
    anchor = 0
    while anchor < len(compact) - 1:
        candidate = len(compact) - 1
        while candidate > anchor + 1 and not grid.segment_is_clear(
            compact[anchor], compact[candidate]
        ):
            candidate -= 1
        result.append(compact[candidate])
        anchor = candidate
    return result
