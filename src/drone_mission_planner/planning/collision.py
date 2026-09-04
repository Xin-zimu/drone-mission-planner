from __future__ import annotations

from dataclasses import dataclass

from drone_mission_planner.domain.geometry import Point


@dataclass(frozen=True, slots=True)
class MotionState:
    drone_id: str
    position: Point
    path: tuple[Point, ...]
    segment_index: int
    max_speed: float
    safety_radius: float
    priority: int


@dataclass(frozen=True, slots=True)
class PredictedConflict:
    drone_a: str
    drone_b: str
    predicted_distance: float
    time_to_conflict: float
    yielding_drone_id: str

    @property
    def pair(self) -> tuple[str, str]:
        return self.drone_a, self.drone_b


class ConflictDetector:
    """Sample deterministic future trajectories and assign one yielding aircraft."""

    def __init__(self, *, horizon: float = 2.0, sample_interval: float = 0.2) -> None:
        if horizon <= 0 or sample_interval <= 0:
            raise ValueError("conflict horizon and sample interval must be positive")
        self.horizon = horizon
        self.sample_interval = sample_interval

    def detect(self, states: list[MotionState]) -> tuple[PredictedConflict, ...]:
        conflicts: list[PredictedConflict] = []
        ordered = sorted(states, key=lambda item: item.drone_id)
        for index, first in enumerate(ordered):
            for second in ordered[index + 1 :]:
                safety_distance = first.safety_radius + second.safety_radius
                closest = float("inf")
                closest_time = 0.0
                sample_count = int(self.horizon / self.sample_interval)
                for sample in range(sample_count + 1):
                    seconds = sample * self.sample_interval
                    distance = predict_position(first, seconds).distance_to(
                        predict_position(second, seconds)
                    )
                    if distance < closest:
                        closest = distance
                        closest_time = seconds
                if closest >= safety_distance:
                    continue
                yielding = self._yielding_drone(first, second)
                conflicts.append(
                    PredictedConflict(
                        first.drone_id,
                        second.drone_id,
                        closest,
                        closest_time,
                        yielding,
                    )
                )
        return tuple(conflicts)

    @staticmethod
    def _yielding_drone(first: MotionState, second: MotionState) -> str:
        if first.priority != second.priority:
            return first.drone_id if first.priority < second.priority else second.drone_id
        return max(first.drone_id, second.drone_id)


def predict_position(state: MotionState, seconds: float) -> Point:
    remaining = max(0.0, seconds) * state.max_speed
    position = state.position
    index = state.segment_index
    while remaining > 1e-9 and index < len(state.path):
        target = state.path[index]
        distance = position.distance_to(target)
        if distance <= remaining:
            position = target
            remaining -= distance
            index += 1
        else:
            position = position.lerp(target, remaining / max(distance, 1e-9))
            remaining = 0.0
    return position
