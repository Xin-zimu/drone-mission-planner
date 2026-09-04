from __future__ import annotations

from dataclasses import dataclass, field

from drone_mission_planner.domain.geometry import Point


@dataclass(slots=True)
class PathResult:
    success: bool
    waypoints: list[Point] = field(default_factory=list)
    total_distance: float = 0.0
    estimated_time: float = 0.0
    estimated_energy: float = 0.0
    expanded_nodes: int = 0
    failure_reason: str | None = None
    raw_waypoint_count: int = 0

    @classmethod
    def failure(cls, reason: str, *, expanded_nodes: int = 0) -> PathResult:
        return cls(False, expanded_nodes=expanded_nodes, failure_reason=reason)
