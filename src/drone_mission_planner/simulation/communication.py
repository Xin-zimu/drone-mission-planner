from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from drone_mission_planner.domain.geometry import Point


@dataclass(frozen=True, slots=True)
class CommunicationNode:
    id: str
    position: Point
    communication_range: float
    is_base: bool = False
    available: bool = True


@dataclass(frozen=True, slots=True)
class CommunicationStatus:
    drone_id: str
    connected: bool
    direct: bool
    hop_count: int | None
    nearest_base_distance: float
    disconnected_for: float
    policy: str


@dataclass(frozen=True, slots=True)
class CommunicationTransition:
    drone_id: str
    connected: bool
    timestamp: float
    message: str


class CommunicationMonitor:
    """Build a bidirectional range graph and track base reachability transitions."""

    def __init__(self, *, policy: str = "log_only", grace_period: float = 5.0) -> None:
        if policy not in {"log_only", "auto_return"}:
            raise ValueError("communication policy must be log_only or auto_return")
        if grace_period < 0:
            raise ValueError("communication grace period cannot be negative")
        self.policy = policy
        self.grace_period = grace_period
        self.statuses: dict[str, CommunicationStatus] = {}
        self.transitions: list[CommunicationTransition] = []
        self._disconnected_since: dict[str, float] = {}
        self._policy_applied: set[str] = set()
        self._links: tuple[tuple[str, str], ...] = ()

    @property
    def links(self) -> tuple[tuple[str, str], ...]:
        return self._links

    def update(self, nodes: list[CommunicationNode], *, timestamp: float) -> tuple[str, ...]:
        available = {node.id: node for node in nodes if node.available}
        bases = sorted(node.id for node in available.values() if node.is_base)
        drones = sorted(node.id for node in nodes if not node.is_base)
        adjacency = {node_id: set[str]() for node_id in available}
        node_list = sorted(available.values(), key=lambda item: item.id)
        links: list[tuple[str, str]] = []
        for index, first in enumerate(node_list):
            for second in node_list[index + 1 :]:
                if first.position.distance_to(second.position) <= min(
                    first.communication_range, second.communication_range
                ):
                    adjacency[first.id].add(second.id)
                    adjacency[second.id].add(first.id)
                    links.append((first.id, second.id))
        self._links = tuple(links)

        requests: list[str] = []
        for drone_id in drones:
            node = next(item for item in nodes if item.id == drone_id)
            hop_count = self._shortest_base_hops(drone_id, bases, adjacency)
            connected = node.available and hop_count is not None
            previous = self.statuses.get(drone_id)
            if connected:
                self._disconnected_since.pop(drone_id, None)
                disconnected_for = 0.0
                self._policy_applied.discard(drone_id)
            else:
                since = self._disconnected_since.setdefault(drone_id, timestamp)
                disconnected_for = max(0.0, timestamp - since)
            nearest = min(
                (node.position.distance_to(available[base_id].position) for base_id in bases),
                default=float("inf"),
            )
            status = CommunicationStatus(
                drone_id,
                connected,
                connected and hop_count == 1,
                hop_count,
                nearest,
                disconnected_for,
                self.policy,
            )
            self.statuses[drone_id] = status
            if previous is not None and previous.connected != connected:
                message = (
                    f"base link restored in {hop_count} hop(s)" if connected else "base link lost"
                )
                self.transitions.append(
                    CommunicationTransition(drone_id, connected, timestamp, message)
                )
            if (
                not connected
                and self.policy == "auto_return"
                and disconnected_for >= self.grace_period
                and drone_id not in self._policy_applied
                and node.available
            ):
                self._policy_applied.add(drone_id)
                requests.append(drone_id)
        return tuple(requests)

    def reset(self) -> None:
        self.statuses.clear()
        self.transitions.clear()
        self._disconnected_since.clear()
        self._policy_applied.clear()
        self._links = ()

    @staticmethod
    def _shortest_base_hops(
        start: str, bases: list[str], adjacency: dict[str, set[str]]
    ) -> int | None:
        if start not in adjacency:
            return None
        queue = deque([(start, 0)])
        visited = {start}
        base_set = set(bases)
        while queue:
            current, distance = queue.popleft()
            if current in base_set:
                return distance
            for neighbor in sorted(adjacency[current]):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, distance + 1))
        return None
