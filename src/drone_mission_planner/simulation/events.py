from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


class EventType(StrEnum):
    DRONE_FAILURE = "drone_failure"
    NEW_TASK = "new_task"
    TASK_CANCELLED = "task_cancelled"
    TEMP_NO_FLY_ZONE = "temporary_no_fly_zone"
    COLLISION_HOLD = "collision_hold"
    COMMUNICATION_LOSS = "communication_loss"
    COMMUNICATION_RESTORED = "communication_restored"
    AUTO_RETURN = "auto_return"


@dataclass(frozen=True, slots=True)
class SimulationEvent:
    id: str
    timestamp: float
    event_type: EventType
    target_id: str
    parameters: dict[str, str | float | int | bool] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class EventRecord:
    event: SimulationEvent
    processed_at: float
    message: str


class EventManager:
    def __init__(self) -> None:
        self._pending: list[SimulationEvent] = []
        self._history: list[EventRecord] = []
        self._counter = 0

    @property
    def history(self) -> tuple[EventRecord, ...]:
        return tuple(self._history)

    @property
    def pending(self) -> tuple[SimulationEvent, ...]:
        return tuple(self._pending)

    def create(
        self,
        timestamp: float,
        event_type: EventType,
        target_id: str,
        parameters: dict[str, str | float | int | bool] | None = None,
    ) -> SimulationEvent:
        if timestamp < 0:
            raise ValueError("event timestamp cannot be negative")
        self._counter += 1
        return SimulationEvent(
            f"E-{self._counter:04d}",
            timestamp,
            event_type,
            target_id,
            dict(parameters or {}),
        )

    def schedule(self, event: SimulationEvent) -> None:
        self._pending.append(event)
        self._pending.sort(key=lambda item: (item.timestamp, item.id))

    def pop_due(self, current_time: float) -> tuple[SimulationEvent, ...]:
        split = 0
        while split < len(self._pending) and self._pending[split].timestamp <= current_time + 1e-9:
            split += 1
        due = tuple(self._pending[:split])
        del self._pending[:split]
        return due

    def record(self, event: SimulationEvent, processed_at: float, message: str) -> None:
        self._history.append(EventRecord(event, processed_at, message))

    def clear(self) -> None:
        self._pending.clear()
        self._history.clear()
