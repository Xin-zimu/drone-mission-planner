from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RobotState:
    robot_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float = 0.0
    battery_voltage: float | None = None
    pose_age_s: float = 0.0


class TelemetryAdapter:
    """Placeholder telemetry adapter for CF5/CF6."""
