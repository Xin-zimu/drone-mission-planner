from __future__ import annotations

from dataclasses import dataclass
from statistics import fmean


@dataclass(frozen=True, slots=True)
class ExecutionSample:
    monotonic_s: float
    wall_time_utc: str
    x_m: float
    y_m: float
    z_m: float
    battery_voltage: float | None
    state: str
    waypoint_index: int | None
    tracking_error_m: float | None = None


@dataclass(frozen=True, slots=True)
class ExecutionEvent:
    monotonic_s: float
    wall_time_utc: str
    event_type: str
    message: str
    waypoint_index: int | None = None


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    mission_id: str
    source_route_hash: str
    started_at_utc: str
    ended_at_utc: str | None
    result: str
    planned_duration_s: float
    actual_duration_s: float | None
    completed_waypoints: int
    samples: tuple[ExecutionSample, ...]
    events: tuple[ExecutionEvent, ...]
    abort_reason: str | None = None

    @property
    def mean_position_error_m(self) -> float | None:
        errors = [sample.tracking_error_m for sample in self.samples if sample.tracking_error_m is not None]
        return fmean(errors) if errors else None

    @property
    def max_position_error_m(self) -> float | None:
        errors = [sample.tracking_error_m for sample in self.samples if sample.tracking_error_m is not None]
        return max(errors, default=None)

    @property
    def telemetry_gaps(self) -> int:
        if len(self.samples) < 2:
            return 0
        gaps = 0
        previous = self.samples[0]
        for sample in self.samples[1:]:
            if sample.monotonic_s - previous.monotonic_s > 0.5:
                gaps += 1
            previous = sample
        return gaps

