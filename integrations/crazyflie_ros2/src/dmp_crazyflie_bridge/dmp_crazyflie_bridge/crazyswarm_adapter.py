from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol
from uuid import uuid4

from .capability_probe import (
    CAPABILITY_SOURCE_CFLIB,
    CAPABILITY_SOURCE_ROS,
    CapabilitySnapshotValidation,
    default_snapshot_path,
    load_capability_snapshot,
)
from .protocol_models import BridgeRobot
from .telemetry import (
    DEFAULT_BATTERY_CRITICAL_V,
    DEFAULT_POSE_STALE_TIMEOUT_S,
    DEFAULT_STATUS_STALE_TIMEOUT_S,
    UNKNOWN_POSITIONING,
    PositioningCapabilities,
    ReadinessPolicy,
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
    robot_uri: str | None = None
    capability_snapshot_path: Path | None = None
    capability_snapshot_max_age_s: float = 3600.0
    readiness_policy: ReadinessPolicy = field(default_factory=ReadinessPolicy)
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
    _capability_diagnostics: tuple[str, ...] = ()
    _capability_source: str | None = None
    _capability_captured_at: str | None = None
    _session_id: str | None = None

    def __post_init__(self) -> None:
        self._collector = TelemetryCollector(
            self.robot_id,
            status_stale_timeout_s=self.status_stale_timeout_s,
            pose_stale_timeout_s=self.pose_stale_timeout_s,
            battery_critical_voltage=self.battery_critical_voltage,
            readiness_policy=self.readiness_policy,
        )

    def connect(self) -> None:
        if self._connected:
            return
        try:
            import rclpy
            from crazyflie_interfaces.msg import Status
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.parameter_client import AsyncParameterClient
            from sensor_msgs.msg import Range
            from std_msgs.msg import Float32
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
        self._node.create_subscription(Odometry, f"/{self.robot_id}/odom", self._collector.record_odom, 10)
        self._node.create_subscription(Range, f"/{self.robot_id}/range/zrange", self._collector.record_range, 10)
        self._node.create_subscription(Float32, f"/{self.robot_id}/zrange", self._collector.record_range, 10)
        self._parameter_client = AsyncParameterClient(self._node, self.crazyflie_server_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f"dmp-{self.robot_id}-ros-spin",
            daemon=True,
        )
        self._spin_thread.start()
        self._connected = True
        self._session_id = str(uuid4())
        self._record_diagnostic(
            f"subscribed to /{self.robot_id}/status, /{self.robot_id}/pose, /{self.robot_id}/odom and zrange topics"
        )

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
        self._session_id = None

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
            capability_source=self._capability_source,
            capability_captured_at=self._capability_captured_at,
            diagnostics=_dedupe(snapshot.diagnostics + snapshot.positioning.diagnostics),
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
    def readiness_payload(self) -> dict[str, Any]:
        return self._collector.readiness_payload()

    @property
    def session_id(self) -> str | None:
        return self._session_id

    @property
    def battery_critical(self) -> bool:
        return self._collector.snapshot().battery_critical

    def set_positioning_for_test(self, capabilities: PositioningCapabilities) -> None:
        self._collector.set_positioning(capabilities)

    def _refresh_positioning_capabilities(self) -> None:
        if self._parameter_client is None:
            self._set_unknown_positioning("crazyflie_server parameter client is unavailable")
            return
        now = time.monotonic()
        if now - self._last_capability_read_s < self._capability_read_interval_s:
            return
        self._last_capability_read_s = now
        try:
            if not self._parameter_client.wait_for_services(timeout_sec=0.05):
                self._set_unknown_positioning(f"{self.crazyflie_server_node} parameter service unavailable")
                return
            names_by_deck = self._discover_deck_parameter_names()
            missing = tuple(name for name in DECK_PARAMETER_NAMES if name not in names_by_deck)
            if missing:
                self._set_unknown_positioning(
                    "deck parameters not listed by "
                    f"{self.crazyflie_server_node}: missing {', '.join(missing)}"
                )
                return
            names = tuple(names_by_deck[name] for name in DECK_PARAMETER_NAMES)
            future = self._parameter_client.get_parameters(names)
            if not self._wait_for_future(future, timeout_s=0.5):
                self._set_unknown_positioning("deck parameter read timed out")
                return
            result = future.result()
            parameter_values = tuple(getattr(result, "values", ()))
            if len(parameter_values) != len(DECK_PARAMETER_NAMES):
                self._set_unknown_positioning(
                    "deck parameter values unavailable from "
                    f"{self.crazyflie_server_node}: expected {len(DECK_PARAMETER_NAMES)}, "
                    f"received {len(parameter_values)}"
                )
                return
            unset = tuple(
                name for name, value in zip(names, parameter_values, strict=True)
                if not _parameter_is_set(value)
            )
            if unset:
                self._set_unknown_positioning(
                    "deck parameters are listed but unset by "
                    f"{self.crazyflie_server_node}: {', '.join(unset)}"
                )
                return
            values = {
                full_name: _parameter_value(value)
                for full_name, value in zip(names, parameter_values, strict=True)
            }
            self._collector.set_positioning(positioning_from_deck_params(values))
            self._capability_source = CAPABILITY_SOURCE_ROS
            self._capability_captured_at = None
            self._capability_diagnostics = ()
        except Exception as exc:
            self._set_unknown_positioning(f"deck parameter read failed: {type(exc).__name__}: {exc}")
        self._collector.set_diagnostics(self._combined_diagnostics())

    def _discover_deck_parameter_names(self) -> dict[str, str]:
        future = self._parameter_client.list_parameters(
            prefixes=[f"{self.robot_id}.params.deck"],
            depth=0,
        )
        if not self._wait_for_future(future, timeout_s=0.5):
            raise TimeoutError("deck parameter list timed out")
        result = future.result()
        listed = tuple(getattr(getattr(result, "result", result), "names", ()))
        names_by_deck: dict[str, str] = {}
        for listed_name in listed:
            if not isinstance(listed_name, str):
                continue
            for deck_name in DECK_PARAMETER_NAMES:
                expected = f"{self.robot_id}.params.{deck_name}"
                if listed_name == expected or listed_name.endswith(f".{expected}"):
                    names_by_deck[deck_name] = listed_name
        return names_by_deck

    @staticmethod
    def _wait_for_future(future: Any, *, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return bool(future.done())

    def _set_unknown_positioning(self, diagnostic: str) -> None:
        snapshot = self._load_snapshot_capabilities()
        if snapshot.valid and snapshot.snapshot is not None:
            self._collector.set_positioning(positioning_from_deck_params(snapshot.snapshot.deck_values))
            self._capability_source = CAPABILITY_SOURCE_CFLIB
            self._capability_captured_at = snapshot.snapshot.captured_at_utc
            self._capability_diagnostics = (diagnostic,)
            self._collector.set_diagnostics(self._combined_diagnostics())
            return
        diagnostics: tuple[str, ...] = (diagnostic,)
        if snapshot.diagnostic is not None:
            diagnostics += (snapshot.diagnostic,)
        self._capability_source = None
        self._capability_captured_at = None
        self._capability_diagnostics = diagnostics
        self._collector.set_positioning(
            PositioningCapabilities(
                mode=UNKNOWN_POSITIONING.mode,
                xy_available=UNKNOWN_POSITIONING.xy_available,
                z_available=UNKNOWN_POSITIONING.z_available,
                pose_available=UNKNOWN_POSITIONING.pose_available,
                relative=UNKNOWN_POSITIONING.relative,
                diagnostics=UNKNOWN_POSITIONING.diagnostics + diagnostics,
            )
        )
        self._collector.set_diagnostics(self._combined_diagnostics())

    def _load_snapshot_capabilities(self) -> CapabilitySnapshotValidation:
        path = self.capability_snapshot_path or default_snapshot_path(self.robot_id)
        return load_capability_snapshot(
            path,
            robot_id=self.robot_id,
            uri=self.robot_uri,
            max_age_s=self.capability_snapshot_max_age_s,
        )

    def _combined_diagnostics(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys((*self._diagnostics, *self._capability_diagnostics)))

    def _record_diagnostic(self, message: str) -> None:
        if message not in self._diagnostics:
            self._diagnostics.append(message)
        self._collector.set_diagnostics(self._combined_diagnostics())


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


def _parameter_is_set(parameter_value: Any) -> bool:
    value_type = getattr(parameter_value, "type", None)
    return value_type is None or int(value_type) != 0


def _dedupe(values: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(values))
