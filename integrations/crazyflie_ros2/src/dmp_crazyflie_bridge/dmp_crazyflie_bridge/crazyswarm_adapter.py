from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import Protocol

from .protocol_models import BridgeRobot
from .telemetry import RobotState


class CrazyflieAdapter(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def capabilities(self) -> BridgeRobot: ...
    def latest_state(self) -> RobotState: ...
    async def takeoff(self, height_m: float, duration_s: float) -> None: ...
    async def go_to(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        duration_s: float,
    ) -> None: ...
    async def land(self, height_m: float, duration_s: float) -> None: ...
    def emergency(self) -> None: ...


@dataclass(slots=True)
class SimCrazyflieAdapter:
    robot_id: str = "cf1"
    time_scale: float = 0.0
    connected: bool = False
    state: RobotState = field(default_factory=lambda: RobotState("cf1", 0.0, 0.0, 0.0))
    command_log: list[str] = field(default_factory=list)
    fail_on_command: str | None = None
    disconnect_on_command: str | None = None
    pose_age_s: float = 0.0
    tracking_error_offset_m: float = 0.0
    _command_active: bool = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def capabilities(self) -> BridgeRobot:
        return BridgeRobot(
            robot_id=self.robot_id,
            connected=self.connected,
            positioning_mode="sim",
            xy_positioning_available=True,
            pose_stream_available=True,
            pose_rate_hz=50.0,
        )

    def latest_state(self) -> RobotState:
        return RobotState(
            self.state.robot_id,
            self.state.x_m + self.tracking_error_offset_m,
            self.state.y_m,
            self.state.z_m,
            self.state.yaw_rad,
            self.state.battery_voltage,
            self.pose_age_s,
        )

    async def takeoff(self, height_m: float, duration_s: float) -> None:
        await self._run_command("takeoff", duration_s)
        self.state = RobotState(self.robot_id, self.state.x_m, self.state.y_m, height_m)

    async def go_to(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        duration_s: float,
    ) -> None:
        await self._run_command("go_to", duration_s)
        self.state = RobotState(self.robot_id, x_m, y_m, z_m, yaw_rad)

    async def land(self, height_m: float, duration_s: float) -> None:
        await self._run_command("land", duration_s)
        self.state = RobotState(self.robot_id, self.state.x_m, self.state.y_m, height_m)

    def emergency(self) -> None:
        self.command_log.append("emergency")
        self.state = RobotState(self.robot_id, self.state.x_m, self.state.y_m, 0.0)

    async def _run_command(self, name: str, duration_s: float) -> None:
        if self._command_active:
            raise RuntimeError("overlapping Crazyflie command")
        if not self.connected:
            raise RuntimeError("Crazyflie adapter is not connected")
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("command duration must be positive")
        self._command_active = True
        self.command_log.append(name)
        try:
            if self.disconnect_on_command == name:
                self.connected = False
                raise RuntimeError("Crazyflie adapter disconnected")
            if self.fail_on_command == name:
                raise TimeoutError(f"{name} timeout")
            await asyncio.sleep(duration_s * self.time_scale)
        finally:
            self._command_active = False


class Crazyswarm2Adapter:
    """Adapter placeholder for the real Crazyswarm2 backend."""

    def connect(self) -> None:
        raise NotImplementedError("Crazyswarm2 integration requires a ROS 2 environment")
