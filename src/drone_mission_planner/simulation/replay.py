"""Replay recording: fixed-interval 3D snapshots of a finished simulation.

The recorder only reads engine state; frames never feed back into the
simulation, so replaying cannot change the deterministic result.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from drone_mission_planner.domain.enums import DroneStatus

if TYPE_CHECKING:
    from drone_mission_planner.simulation.engine import SimulationEngine


@dataclass(frozen=True, slots=True)
class ReplayDroneState:
    """One drone's 3D state at a replay frame."""

    drone_id: str
    x: float
    y: float
    z: float
    status: str
    remaining_battery: float


@dataclass(frozen=True, slots=True)
class ReplayFrame:
    """A complete, immutable snapshot of the simulation at one sample time."""

    time: float
    drones: tuple[ReplayDroneState, ...]
    coverage: tuple[tuple[str, float], ...]
    processed_events: int


class ReplayRecorder:
    """Collects replay frames at a fixed simulation-time interval."""

    def __init__(self, interval: float = 0.5) -> None:
        if interval <= 0:
            raise ValueError("replay interval must be positive")
        self.interval = interval
        self._frames: list[ReplayFrame] = []
        self._next_sample = 0.0

    @property
    def frames(self) -> tuple[ReplayFrame, ...]:
        return tuple(self._frames)

    def reset(self) -> None:
        self._frames.clear()
        self._next_sample = 0.0

    def record(self, engine: SimulationEngine) -> None:
        """Capture a frame when simulation time reaches the next sample."""

        while engine.time + 1e-9 >= self._next_sample:
            self._frames.append(self._capture(engine))
            self._next_sample += self.interval

    def _capture(self, engine: SimulationEngine) -> ReplayFrame:
        snapshot = engine.snapshot()
        drones = tuple(
            ReplayDroneState(
                state.id,
                state.position.x,
                state.position.y,
                state.current_altitude,
                state.status.value
                if isinstance(state.status, DroneStatus)
                else str(state.status),
                state.remaining_battery,
            )
            for state in snapshot.drones
        )
        coverage = tuple(
            (item.area_id, item.coverage) for item in snapshot.coverage
        )
        return ReplayFrame(
            engine.time,
            drones,
            coverage,
            len(engine.event_manager.history),
        )

    def frame_index_for_time(self, time: float) -> int:
        """Index of the first frame at or after ``time`` (clamped)."""

        for index, frame in enumerate(self._frames):
            if frame.time + 1e-9 >= time:
                return index
        return max(0, len(self._frames) - 1)


def export_replay(engine: SimulationEngine, path: str | Path) -> Path:
    """Write the recorded replay frames as JSON for external inspection."""

    document: dict[str, Any] = {
        "interval": engine.replay.interval,
        "frames": [
            {
                "time": frame.time,
                "drones": [
                    {
                        "drone_id": state.drone_id,
                        "x": state.x,
                        "y": state.y,
                        "z": state.z,
                        "status": state.status,
                        "remaining_battery": state.remaining_battery,
                    }
                    for state in frame.drones
                ],
                "coverage": [[area_id, value] for area_id, value in frame.coverage],
                "processed_events": frame.processed_events,
            }
            for frame in engine.replay.frames
        ],
    }
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    return target
