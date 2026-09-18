from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

BRIDGE_SRC = (
    Path(__file__).resolve().parents[2]
    / "integrations"
    / "crazyflie_ros2"
    / "src"
    / "dmp_crazyflie_bridge"
)
sys.path.insert(0, str(BRIDGE_SRC))

telemetry = importlib.import_module("dmp_crazyflie_bridge.telemetry")
protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")
crazyswarm_adapter = importlib.import_module("dmp_crazyflie_bridge.crazyswarm_adapter")


def test_status_pose_mapping_and_stale_reconnect() -> None:
    clock = FakeClock()
    collector = telemetry.TelemetryCollector("cf231", clock=clock)
    collector.set_positioning(
        telemetry.positioning_from_deck_params(
            {
                "deck.bcFlow2": 1,
                "deck.bcZRanger2": 1,
                "deck.bcLighthouse4": 0,
                "deck.bcLoco": 0,
                "deck.bcDWM1000": 0,
            }
        )
    )

    collector.record_status(FakeStatus(3.91, 1, 37, 6, 10, 10))
    collector.record_pose(FakePose(1.0, 2.0, 0.1))

    snapshot = collector.snapshot()
    assert snapshot.connected is True
    assert snapshot.pose_stream_available is True
    assert snapshot.status.battery_voltage == 3.91
    assert snapshot.pose.x_m == 1.0
    assert snapshot.positioning.mode == "flow"
    assert snapshot.positioning.xy_available is True
    assert snapshot.positioning.z_available is True

    clock.advance(3.0)
    stale = collector.snapshot()
    assert stale.connected is False
    assert stale.pose_stream_available is False

    collector.record_status(FakeStatus(3.89, 1, 36, 7, 11, 11))
    collector.record_pose(FakePose(1.1, 2.0, 0.1))
    assert collector.snapshot().connected is True


def test_positioning_capability_flow_missing_and_z_only() -> None:
    flow = telemetry.positioning_from_deck_params(
        {
            "deck.bcFlow2": "1",
            "deck.bcZRanger2": "1",
            "deck.bcLighthouse4": "0",
            "deck.bcLoco": "0",
            "deck.bcDWM1000": "0",
        }
    )
    assert flow.mode == "flow"
    assert flow.xy_available is True
    assert flow.z_available is True
    assert "deck.bcFlow2=1" in flow.evidence

    z_only = telemetry.positioning_from_deck_params(
        {
            "deck.bcFlow2": 0,
            "deck.bcZRanger2": 1,
            "deck.bcLighthouse4": 0,
            "deck.bcLoco": 0,
            "deck.bcDWM1000": 0,
        }
    )
    assert z_only.mode == "z_ranger"
    assert z_only.xy_available is False
    assert z_only.z_available is True

    none = telemetry.positioning_from_deck_params(
        {
            "deck.bcFlow2": 0,
            "deck.bcZRanger2": 0,
            "deck.bcLighthouse4": 0,
            "deck.bcLoco": 0,
            "deck.bcDWM1000": 0,
        }
    )
    assert none.mode == "none"
    assert none.xy_available is False
    assert none.z_available is False


def test_deck_parameter_reader_recovers_when_service_becomes_available() -> None:
    adapter = _adapter_with_client(FakeParameterClient(service_ready=False))

    adapter._refresh_positioning_capabilities()
    assert adapter._collector.snapshot().positioning.mode == "unknown"

    adapter._parameter_client = FakeParameterClient(values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1})
    adapter._refresh_positioning_capabilities()

    snapshot = adapter._collector.snapshot()
    assert snapshot.positioning.mode == "flow"
    assert snapshot.positioning.xy_available is True
    assert snapshot.positioning.z_available is True
    assert "deck.bcFlow2=1" in snapshot.positioning.evidence


def test_deck_parameter_reader_preserves_z_only_and_all_zero() -> None:
    z_only = _adapter_with_client(FakeParameterClient(values={"deck.bcFlow2": 0, "deck.bcZRanger2": 1}))
    z_only._refresh_positioning_capabilities()
    assert z_only._collector.snapshot().positioning.mode == "z_ranger"

    none = _adapter_with_client(FakeParameterClient(values={}))
    none._refresh_positioning_capabilities()
    snapshot = none._collector.snapshot()
    assert snapshot.positioning.mode == "none"
    assert snapshot.positioning.xy_available is False
    assert snapshot.positioning.z_available is False


def test_deck_parameter_reader_blocks_partial_unset_and_timeout() -> None:
    partial = _adapter_with_client(FakeParameterClient(value_count=3))
    partial._refresh_positioning_capabilities()
    partial_snapshot = partial._collector.snapshot()
    assert partial_snapshot.positioning.mode == "unknown"
    assert any("expected 5, received 3" in item for item in partial_snapshot.diagnostics)

    unset = _adapter_with_client(FakeParameterClient(unset_names={"deck.bcFlow2"}))
    unset._refresh_positioning_capabilities()
    unset_snapshot = unset._collector.snapshot()
    assert unset_snapshot.positioning.mode == "unknown"
    assert any("listed but unset" in item for item in unset_snapshot.diagnostics)

    timeout = _adapter_with_client(FakeParameterClient(get_timeout=True))
    timeout._refresh_positioning_capabilities()
    timeout_snapshot = timeout._collector.snapshot()
    assert timeout_snapshot.positioning.mode == "unknown"
    assert any("timed out" in item for item in timeout_snapshot.diagnostics)


def test_deck_parameter_reader_uses_cf231_runtime_names_with_namespace() -> None:
    client = FakeParameterClient(
        namespace_prefix="swarm.",
        values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1},
    )
    adapter = _adapter_with_client(client)

    adapter._refresh_positioning_capabilities()

    assert adapter._collector.snapshot().positioning.mode == "flow"
    assert client.requested_names == (
        "swarm.cf231.params.deck.bcFlow2",
        "swarm.cf231.params.deck.bcZRanger2",
        "swarm.cf231.params.deck.bcLighthouse4",
        "swarm.cf231.params.deck.bcLoco",
        "swarm.cf231.params.deck.bcDWM1000",
    )


def test_battery_critical_and_pose_stability_gate() -> None:
    clock = FakeClock()
    collector = telemetry.TelemetryCollector("cf231", clock=clock)
    collector.record_status(FakeStatus(3.61, 0, 37, 6, 10, 10))
    for index in range(60):
        collector.record_pose(FakePose(float(index) * 0.001, 0.0, 0.01))
        clock.advance(0.1)

    snapshot = collector.snapshot()
    assert snapshot.battery_critical is True
    assert snapshot.pose_stability.observed is True
    assert snapshot.pose_stability.stable is False
    assert snapshot.pose_stability.xy_displacement_m > 0.03


def test_hardware_protocol_uses_cf231_and_realistic_blockers(monkeypatch: Any) -> None:
    fake = FakeHardwareAdapter("cf231")
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda robot_id, **_: fake)
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231")

    caps = server.handle_message({"type": "get_capabilities", "request_id": "cap-1"})
    robot = caps["robots"][0]
    assert robot["robot_id"] == "cf231"
    assert robot["connected"] is True
    assert robot["positioning_mode"] == "flow"
    assert robot["xy_positioning_available"] is True
    assert robot["z_positioning_available"] is True
    assert robot["pose_stream_available"] is True
    assert robot["battery_critical"] is True

    telemetry_payload = server.handle_message({"type": "get_telemetry", "request_id": "tel-1"})
    assert telemetry_payload["type"] == "telemetry"
    assert telemetry_payload["robot_id"] == "cf231"
    assert telemetry_payload["battery_voltage"] == 3.61
    assert telemetry_payload["positioning_mode"] == "flow"

    server.handle_message({"type": "select_robot", "request_id": "select-1", "robot_id": "cf231"})
    preflight = server.handle_message({"type": "run_preflight", "request_id": "preflight-1"})
    codes = {issue["code"] for issue in preflight["issues"]}
    assert "robot_disconnected" not in codes
    assert "xy_positioning_missing" not in codes
    assert "pose_stream_missing" not in codes
    assert "battery_critical" in codes
    assert "pose_stability_failed" in codes
    assert preflight["passed"] is False


def test_hardware_preflight_blocks_flow_without_current_pose_origin(monkeypatch: Any) -> None:
    fake = FakeHardwareAdapter("cf231")
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda robot_id, **_: fake)
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231")
    server.state.selected_robot_id = "cf231"
    server.state.loaded_mission_id = "mission-1"
    server.state.loaded_route_hash = "route-hash"
    server.state.loaded_mission = {
        "mission_id": "mission-1",
        "source_route_hash": "route-hash",
        "frame": {
            "mode": "relative_current_pose",
            "origin_source": "manual",
            "origin_captured_at_utc": None,
        },
        "waypoints": [
            {
                "index": 1,
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.4,
                "yaw_rad": 0.0,
                "duration_s": 1.0,
                "action": "fly_to",
            }
        ],
    }

    preflight = server.handle_message({"type": "run_preflight", "request_id": "preflight-frame"})

    codes = {issue["code"] for issue in preflight["issues"]}
    assert "frame_origin_missing" in codes
    assert "xy_positioning_missing" not in codes


@dataclass(slots=True)
class FakeClock:
    now: float = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@dataclass(frozen=True, slots=True)
class FakeStatus:
    battery_voltage: float
    pm_state: int
    rssi: int
    latency_unicast: int
    num_rx_unicast: int
    num_tx_unicast: int


class FakePose:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.pose = _Pose(_Point(x, y, z), _Orientation())


@dataclass(frozen=True, slots=True)
class _Point:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class _Orientation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass(frozen=True, slots=True)
class _Pose:
    position: _Point
    orientation: _Orientation


class FakeHardwareAdapter:
    def __init__(self, robot_id: str) -> None:
        self.robot_id = robot_id

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    def capabilities(self) -> Any:
        return protocol_server.BridgeRobot(
            robot_id=self.robot_id,
            connected=True,
            positioning_mode="flow",
            xy_positioning_available=True,
            z_positioning_available=True,
            pose_stream_available=True,
            pose_rate_hz=9.57,
            status_rate_hz=0.95,
            battery_voltage=3.61,
            battery_critical=True,
            battery_critical_voltage=3.7,
            rssi=37,
            latency_unicast=6,
            num_rx_unicast=425,
            num_tx_unicast=425,
            status_age_s=0.2,
            pose_age_s=0.1,
            x_m=1.0,
            y_m=2.0,
            z_m=0.1,
            positioning_evidence=("deck.bcFlow2=1", "deck.bcZRanger2=1"),
        )

    @property
    def pose_stability_payload(self) -> dict[str, Any]:
        return {
            "observed": True,
            "stable": False,
            "sample_count": 60,
            "duration_s": 5.9,
            "xy_displacement_m": 0.049,
            "z_displacement_m": 0.001,
            "max_sample_gap_s": 0.1,
            "reason": "pose_drift_or_gap_exceeds_limit",
        }

    def telemetry_payload(self, **_: Any) -> dict[str, Any]:
        return {
            "type": "telemetry",
            "robot_id": self.robot_id,
            "connected": True,
            "x_m": 1.0,
            "y_m": 2.0,
            "z_m": 0.1,
            "battery_voltage": 3.61,
            "rssi": 37,
            "latency_unicast": 6,
            "status_age_s": 0.2,
            "pose_age_s": 0.1,
            "positioning_mode": "flow",
            "xy_positioning_available": True,
            "z_positioning_available": True,
            "pose_stream_available": True,
        }


def _adapter_with_client(client: FakeParameterClient) -> Any:
    adapter = crazyswarm_adapter.Crazyswarm2Adapter(robot_id="cf231")
    adapter._parameter_client = client
    adapter._capability_read_interval_s = 0.0
    return adapter


class FakeParameterClient:
    def __init__(
        self,
        *,
        service_ready: bool = True,
        namespace_prefix: str = "",
        values: dict[str, int] | None = None,
        value_count: int | None = None,
        unset_names: set[str] | None = None,
        get_timeout: bool = False,
    ) -> None:
        self.service_ready = service_ready
        self.namespace_prefix = namespace_prefix
        self.values = values or {}
        self.value_count = value_count
        self.unset_names = unset_names or set()
        self.get_timeout = get_timeout
        self.requested_names: tuple[str, ...] = ()

    def wait_for_services(self, *, timeout_sec: float) -> bool:
        return self.service_ready

    def list_parameters(self, *, prefixes: list[str], depth: int) -> Any:
        names = tuple(
            f"{self.namespace_prefix}cf231.params.{name}"
            for name in crazyswarm_adapter.DECK_PARAMETER_NAMES
        )
        return FakeFuture(FakeListResponse(names))

    def get_parameters(self, names: tuple[str, ...]) -> Any:
        self.requested_names = names
        if self.get_timeout:
            return FakeFuture(None, done=False)
        selected_names = names if self.value_count is None else names[: self.value_count]
        values = tuple(
            FakeParameterValue(
                integer_value=self.values.get(_short_deck_name(name), 0),
                parameter_type=0 if _short_deck_name(name) in self.unset_names else 2,
            )
            for name in selected_names
        )
        return FakeFuture(FakeGetResponse(values))


class FakeFuture:
    def __init__(self, result: Any, *, done: bool = True) -> None:
        self._result = result
        self._done = done

    def done(self) -> bool:
        return self._done

    def result(self) -> Any:
        return self._result


@dataclass(frozen=True, slots=True)
class FakeListResult:
    names: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class FakeListResponse:
    names: tuple[str, ...]

    @property
    def result(self) -> FakeListResult:
        return FakeListResult(self.names)


@dataclass(frozen=True, slots=True)
class FakeGetResponse:
    values: tuple[Any, ...]


@dataclass(frozen=True, slots=True)
class FakeParameterValue:
    integer_value: int
    parameter_type: int
    bool_value: bool = False
    double_value: float = 0.0
    string_value: str = ""

    @property
    def type(self) -> int:
        return self.parameter_type


def _short_deck_name(parameter_name: str) -> str:
    for deck_name in crazyswarm_adapter.DECK_PARAMETER_NAMES:
        if parameter_name.endswith(deck_name):
            return str(deck_name)
    return parameter_name
