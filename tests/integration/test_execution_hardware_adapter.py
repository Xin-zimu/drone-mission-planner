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
    assert z_only.mode == "unknown"
    assert z_only.xy_available is False
    assert z_only.z_available is True


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
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda robot_id: fake)
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
