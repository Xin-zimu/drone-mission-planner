from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Protocol

from .protocol_models import BridgeRobot
from .telemetry import (
    DEFAULT_BATTERY_CRITICAL_V,
    DEFAULT_POSE_STALE_TIMEOUT_S,
    DEFAULT_STATUS_STALE_TIMEOUT_S,
    UNKNOWN_POSITIONING,
    PositioningCapabilities,
    RobotState,
    TelemetryCollector,
    positioning_from_deck_params,
)

DECK_PARAMETER_NAMES = (
    "deck.bcFlow2",
    "deck.bcZRanger2",
    "deck.bcLighthouse4",
    "deck.bcLoco",
    "deck.bcDWM1000",
)


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
            z_positioning_available=True,
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


@dataclass(slots=True)
class Crazyswarm2Adapter:
    """Read-only Crazyswarm2 hardware telemetry adapter.

    This adapter intentionally does not implement flight commands. It only subscribes
    to Crazyswarm2 telemetry topics and reads deck capability parameters.
    """

    robot_id: str = "cf231"
    status_stale_timeout_s: float = DEFAULT_STATUS_STALE_TIMEOUT_S
    pose_stale_timeout_s: float = DEFAULT_POSE_STALE_TIMEOUT_S
    battery_critical_voltage: float = DEFAULT_BATTERY_CRITICAL_V
    crazyflie_server_node: str = "/crazyflie_server"
    _collector: TelemetryCollector = field(init=False)
    _diagnostics: list[str] = field(default_factory=list)
    _connected: bool = False
    _rclpy: Any = None
    _context: Any = None
    _node: Any = None
    _executor: Any = None
    _spin_thread: threading.Thread | None = None
    _parameter_client: Any = None
    _last_capability_read_s: float = 0.0
    _capability_read_interval_s: float = 2.0

    def __post_init__(self) -> None:
        self._collector = TelemetryCollector(
            self.robot_id,
            status_stale_timeout_s=self.status_stale_timeout_s,
            pose_stale_timeout_s=self.pose_stale_timeout_s,
            battery_critical_voltage=self.battery_critical_voltage,
        )

    def connect(self) -> None:
        if self._connected:
            return
        try:
            import rclpy
            from crazyflie_interfaces.msg import Status
            from geometry_msgs.msg import PoseStamped
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.parameter_client import AsyncParameterClient
        except Exception as exc:  # pragma: no cover - exercised on non-ROS hosts
            self._record_diagnostic(f"ROS 2 telemetry unavailable: {type(exc).__name__}: {exc}")
            return

        self._rclpy = rclpy
        self._context = rclpy.context.Context()
        rclpy.init(context=self._context)
        node_name = f"dmp_{self.robot_id}_telemetry"
        self._node = Node(node_name, context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._node.create_subscription(Status, f"/{self.robot_id}/status", self._collector.record_status, 10)
        self._node.create_subscription(PoseStamped, f"/{self.robot_id}/pose", self._collector.record_pose, 10)
        self._parameter_client = AsyncParameterClient(self._node, self.crazyflie_server_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f"dmp-{self.robot_id}-ros-spin",
            daemon=True,
        )
        self._spin_thread.start()
        self._connected = True
        self._record_diagnostic(f"subscribed to /{self.robot_id}/status and /{self.robot_id}/pose")

    def disconnect(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy is not None and self._context is not None:
            try:
                self._rclpy.shutdown(context=self._context)
            except Exception as exc:  # pragma: no cover - shutdown best effort
                self._record_diagnostic(f"ROS 2 shutdown warning: {exc}")
        self._connected = False

    def capabilities(self) -> BridgeRobot:
        self._refresh_positioning_capabilities()
        snapshot = self._collector.snapshot()
        status = snapshot.status
        pose = snapshot.pose
        return BridgeRobot(
            robot_id=self.robot_id,
            connected=snapshot.connected,
            positioning_mode=snapshot.positioning.mode,
            xy_positioning_available=snapshot.positioning.xy_available,
            z_positioning_available=snapshot.positioning.z_available,
            pose_stream_available=snapshot.pose_stream_available,
            pose_rate_hz=snapshot.pose_rate_hz,
            status_rate_hz=snapshot.status_rate_hz,
            battery_voltage=None if status is None else status.battery_voltage,
            battery_critical=snapshot.battery_critical,
            battery_critical_voltage=snapshot.battery_critical_voltage,
            rssi=None if status is None else status.rssi,
            latency_unicast=None if status is None else status.latency_unicast,
            num_rx_unicast=None if status is None else status.num_rx_unicast,
            num_tx_unicast=None if status is None else status.num_tx_unicast,
            status_age_s=snapshot.status_age_s,
            pose_age_s=snapshot.pose_age_s,
            x_m=None if pose is None else pose.x_m,
            y_m=None if pose is None else pose.y_m,
            z_m=None if pose is None else pose.z_m,
            positioning_evidence=snapshot.positioning.evidence,
            diagnostics=snapshot.diagnostics + snapshot.positioning.diagnostics,
        )

    def latest_state(self) -> RobotState:
        return self._collector.snapshot().latest_state()

    def telemetry_payload(
        self,
        *,
        mission_id: str | None = None,
        active_waypoint_index: int | None = None,
        flight_state: str = "bridge_ready",
    ) -> dict[str, Any]:
        self._refresh_positioning_capabilities()
        return self._collector.snapshot().to_telemetry_payload(
            mission_id=mission_id,
            active_waypoint_index=active_waypoint_index,
            flight_state=flight_state,
        )

    async def takeoff(self, height_m: float, duration_s: float) -> None:
        raise RuntimeError("hardware takeoff is disabled in CF7 telemetry gate")

    async def go_to(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        duration_s: float,
    ) -> None:
        raise RuntimeError("hardware go_to is disabled in CF7 telemetry gate")

    async def land(self, height_m: float, duration_s: float) -> None:
        raise RuntimeError("hardware land is disabled in CF7 telemetry gate")

    def emergency(self) -> None:
        raise RuntimeError("hardware emergency is disabled in CF7 telemetry gate")

    @property
    def pose_stability_payload(self) -> dict[str, Any]:
        return self._collector.snapshot().pose_stability.to_payload()

    @property
    def battery_critical(self) -> bool:
        return self._collector.snapshot().battery_critical

    def set_positioning_for_test(self, capabilities: PositioningCapabilities) -> None:
        self._collector.set_positioning(capabilities)

    def _refresh_positioning_capabilities(self) -> None:
        if self._parameter_client is None:
            self._collector.set_positioning(UNKNOWN_POSITIONING)
            self._collector.set_diagnostics(tuple(self._diagnostics))
            return
        now = time.monotonic()
        if now - self._last_capability_read_s < self._capability_read_interval_s:
            return
        self._last_capability_read_s = now
        try:
            if not self._parameter_client.wait_for_services(timeout_sec=0.05):
                self._record_diagnostic(f"{self.crazyflie_server_node} parameter service unavailable")
                self._collector.set_positioning(UNKNOWN_POSITIONING)
                self._collector.set_diagnostics(tuple(self._diagnostics))
                return
            names = tuple(f"{self.robot_id}.params.{name}" for name in DECK_PARAMETER_NAMES)
            future = self._parameter_client.get_parameters(names)
            deadline = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                self._record_diagnostic("deck parameter read timed out")
                self._collector.set_diagnostics(tuple(self._diagnostics))
                return
            parameter_values = tuple(future.result().values)
            if len(parameter_values) != len(DECK_PARAMETER_NAMES):
                self._record_diagnostic(
                    "deck parameter values unavailable from "
                    f"{self.crazyflie_server_node}: expected {len(DECK_PARAMETER_NAMES)}, "
                    f"received {len(parameter_values)}"
                )
                self._collector.set_positioning(UNKNOWN_POSITIONING)
                self._collector.set_diagnostics(tuple(self._diagnostics))
                return
            values = {
                name: _parameter_value(value)
                for name, value in zip(DECK_PARAMETER_NAMES, parameter_values, strict=True)
            }
            self._collector.set_positioning(positioning_from_deck_params(values))
        except Exception as exc:
            self._record_diagnostic(f"deck parameter read failed: {type(exc).__name__}: {exc}")
        self._collector.set_diagnostics(tuple(self._diagnostics))

    def _record_diagnostic(self, message: str) -> None:
        if message not in self._diagnostics:
            self._diagnostics.append(message)
        self._collector.set_diagnostics(tuple(self._diagnostics))


def _parameter_value(parameter_value: Any) -> Any:
    for attr in (
        "bool_value",
        "integer_value",
        "double_value",
        "string_value",
    ):
        value = getattr(parameter_value, attr, None)
        if value not in (None, "", 0, 0.0, False):
            return value
    return getattr(parameter_value, "integer_value", 0)
