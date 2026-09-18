from __future__ import annotations

import asyncio
import importlib
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

BRIDGE_SRC = (
    Path(__file__).resolve().parents[2]
    / "integrations"
    / "crazyflie_ros2"
    / "src"
    / "dmp_crazyflie_bridge"
)
sys.path.insert(0, str(BRIDGE_SRC))

cf8_acceptance = importlib.import_module("dmp_crazyflie_bridge.cf8_acceptance")
crazyswarm_adapter = importlib.import_module("dmp_crazyflie_bridge.crazyswarm_adapter")
protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")


def test_cf8_runner_success_uses_absolute_origin_targets_and_never_go_to() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(z_m=0.034)
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
            adapter,
            policy=_fast_policy(),
        )

        result = await runner.run()

        assert result.passed is True
        assert result.target_z_m == pytest.approx(0.284)
        assert result.landing_target_z_m == pytest.approx(0.034)
        assert adapter.takeoff_calls == [(pytest.approx(0.284), pytest.approx(0.2))]
        assert adapter.land_calls == [(pytest.approx(0.034), pytest.approx(0.2))]
        assert adapter.go_to_calls == []
        assert "takeoff_service_accepted" in _event_names(result)
        assert "landed_confirmed" in _event_names(result)

    asyncio.run(scenario())


def test_cf8_runner_preflight_failure_sends_no_flight_services() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(readiness_issues=[{"code": "battery_critical", "blocking": True}])
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "preflight_failed"
        assert adapter.takeoff_calls == []
        assert adapter.land_calls == []
        assert adapter.arm_calls == []

    asyncio.run(scenario())


def test_cf8_runner_safety_fault_during_hover_attempts_controlled_land() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(fail_after_takeoff="battery_critical")
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy(hover_duration_s=0.05))

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "battery_critical"
        assert len(adapter.takeoff_calls) == 1
        assert len(adapter.land_calls) == 1
        assert "controlled_land_started" in _event_names(result)

    asyncio.run(scenario())


def test_cf8_runner_abort_before_takeoff_sends_no_flight_services() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter()
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())
        runner.request_abort()

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "operator_abort_before_takeoff"
        assert adapter.takeoff_calls == []
        assert adapter.land_calls == []

    asyncio.run(scenario())


def test_cf8_runner_takeoff_service_timeout_is_distinct_from_motion_timeout() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(takeoff_error=TimeoutError("takeoff service timeout"))
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "takeoff_service_timeout"
        assert adapter.takeoff_calls == []
        assert adapter.land_calls == [pytest.approx((0.02, 0.2))]

    asyncio.run(scenario())


def test_cf8_protocol_interlocks_block_before_runner_or_services(monkeypatch: Any, tmp_path: Path) -> None:
    fake = FakeAcceptanceAdapter()
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda *_, **__: fake)
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231")

    missing_confirm = asyncio.run(
        server.handle_message_async({"type": "run_cf8_acceptance", "request_id": "cf8-1"})
    )
    disabled = asyncio.run(
        server.handle_message_async(
            {"type": "run_cf8_acceptance", "request_id": "cf8-2", "confirm_real_flight": True}
        )
    )

    assert missing_confirm["code"] == "operator_confirmation_required"
    assert disabled["code"] == "hardware_flight_disabled"
    assert fake.takeoff_calls == []
    assert fake.land_calls == []

    enabled_config = tmp_path / "bridge.yaml"
    enabled_config.write_text("hardware_flight_enabled: true\n", encoding="utf-8")
    failing = FakeAcceptanceAdapter(readiness_issues=[{"code": "range_unavailable", "blocking": True}])
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda *_, **__: failing)
    enabled_server = protocol_server.BridgeProtocolServer(
        backend="hardware",
        robot_id="cf231",
        config_path=str(enabled_config),
    )
    started = asyncio.run(
        _start_and_poll(
            enabled_server,
            {"type": "run_cf8_acceptance", "request_id": "cf8-3", "confirm_real_flight": True},
        )
    )

    assert started["type"] == "cf8_acceptance_result"
    assert started["passed"] is False
    assert started["failure_code"] == "preflight_failed"
    assert failing.takeoff_calls == []
    assert failing.land_calls == []


def test_cf8_protocol_execute_mission_remains_disabled_and_go_to_remains_disabled(monkeypatch: Any, tmp_path: Path) -> None:
    fake = FakeAcceptanceAdapter()
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda *_, **__: fake)
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text("hardware_flight_enabled: true\n", encoding="utf-8")
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231", config_path=str(config_path))

    execute = server.handle_message({"type": "execute_mission", "request_id": "exec-1"})
    with pytest.raises(RuntimeError, match="go_to is disabled"):
        asyncio.run(fake.go_to(0.0, 0.0, 0.2, 0.0, 1.0))

    assert execute["code"] == "execution_not_implemented"
    assert fake.go_to_calls == []


def test_crazyswarm2_adapter_takeoff_request_uses_group_zero_and_duration_msg() -> None:
    adapter = crazyswarm_adapter.Crazyswarm2Adapter(robot_id="cf231", hardware_flight_enabled=True)
    adapter._takeoff_client = FakeServiceClient()
    adapter._takeoff_request_type = FakeTakeoffRequest
    adapter._duration_msg_type = FakeDuration

    asyncio.run(adapter.takeoff(0.284, 2.5))

    request = adapter._takeoff_client.requests[0]
    assert request.group_mask == 0
    assert request.height == pytest.approx(0.284)
    assert request.duration.sec == 2
    assert request.duration.nanosec == 500_000_000


async def _start_and_poll(server: Any, message: dict[str, Any]) -> dict[str, Any]:
    started = await server.handle_message_async(message)
    assert started["type"] == "cf8_acceptance_state"
    for _ in range(50):
        status = await server.handle_message_async({"type": "get_cf8_acceptance", "request_id": "poll"})
        if status["type"] == "cf8_acceptance_result":
            return status
        await asyncio.sleep(0.01)
    raise AssertionError("CF8 result was not produced")


def _fast_policy(**updates: Any) -> Any:
    values = {
        "takeoff_delta_m": 0.25,
        "takeoff_duration_s": 0.2,
        "hover_duration_s": 0.01,
        "land_duration_s": 0.2,
        "takeoff_timeout_s": 0.5,
        "landing_timeout_s": 0.5,
        "service_timeout_s": 0.1,
        "z_acceptance_tolerance_m": 0.03,
        "xy_drift_tolerance_m": 0.05,
        "settle_velocity_mps": 0.08,
        "settle_duration_s": 0.01,
        "max_total_acceptance_time_s": 2.0,
        "poll_interval_s": 0.005,
    }
    values.update(updates)
    return cf8_acceptance.CF8AcceptancePolicy(**values)


def _event_names(result: Any) -> set[str]:
    return {event.name for event in result.events}


@dataclass(slots=True)
class FakeRobotState:
    robot_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float = 0.0
    battery_voltage: float = 3.95
    pose_age_s: float = 0.0


class FakeAcceptanceAdapter:
    def __init__(
        self,
        *,
        z_m: float = 0.02,
        readiness_issues: list[dict[str, Any]] | None = None,
        fail_after_takeoff: str | None = None,
        takeoff_error: Exception | None = None,
    ) -> None:
        self.robot_id = "cf231"
        self.session = "session-1"
        self.x_m = 1.0
        self.y_m = 2.0
        self.z_m = z_m
        self.vz_mps = 0.0
        self.speed_mps = 0.0
        self.battery_critical = False
        self.flying = False
        self.armed = False
        self.readiness_issues = readiness_issues or []
        self.fail_after_takeoff = fail_after_takeoff
        self.takeoff_error = takeoff_error
        self.takeoff_calls: list[tuple[Any, Any]] = []
        self.land_calls: list[tuple[Any, Any]] = []
        self.arm_calls: list[bool] = []
        self.go_to_calls: list[tuple[Any, ...]] = []
        self.telemetry_reads = 0

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    def capabilities(self) -> Any:
        return protocol_server.BridgeRobot(
            robot_id="cf231",
            connected=True,
            positioning_mode="flow",
            xy_positioning_available=True,
            z_positioning_available=True,
            pose_stream_available=True,
            pose_rate_hz=10.0,
            status_rate_hz=1.0,
            battery_voltage=3.95,
            battery_critical=False,
            battery_critical_voltage=3.7,
            status_age_s=0.0,
            pose_age_s=0.0,
            x_m=self.x_m,
            y_m=self.y_m,
            z_m=self.z_m,
        )

    def latest_state(self) -> FakeRobotState:
        return FakeRobotState("cf231", self.x_m, self.y_m, self.z_m)

    @property
    def readiness_payload(self) -> dict[str, Any]:
        return {"passed": not self.readiness_issues, "issues": list(self.readiness_issues)}

    @property
    def session_id(self) -> str | None:
        return self.session

    def telemetry_payload(self, **_: Any) -> dict[str, Any]:
        self.telemetry_reads += 1
        if self.fail_after_takeoff == "battery_critical" and self.takeoff_calls and self.telemetry_reads > 6:
            self.battery_critical = True
        return {
            "type": "telemetry",
            "robot_id": "cf231",
            "connected": True,
            "pose_stream_available": True,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "battery_voltage": 3.95,
            "battery_critical": self.battery_critical,
            "status_age_s": 0.0,
            "pose_age_s": 0.0,
            "velocity": {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": self.vz_mps, "speed_mps": self.speed_mps},
            "range": {"zrange_m": max(0.02, self.z_m)},
            "supervisor_info_raw": 0x000B if self.armed else 0x0009,
            "supervisor": {
                "can_be_armed": True,
                "armed": self.armed,
                "can_fly": True,
                "flying": self.flying,
                "locked": False,
                "tumbled": False,
                "crashed": False,
                "deck_fault": False,
            },
        }

    async def takeoff(self, height_m: float, duration_s: float) -> None:
        if self.takeoff_error is not None:
            raise self.takeoff_error
        self.takeoff_calls.append((height_m, duration_s))
        self.z_m = height_m
        self.flying = True
        self.armed = True

    async def land(self, height_m: float, duration_s: float) -> None:
        self.land_calls.append((height_m, duration_s))
        self.z_m = height_m
        self.flying = False

    async def arm(self, arm: bool) -> None:
        self.arm_calls.append(arm)
        self.armed = arm

    async def go_to(self, *_: Any) -> None:
        raise RuntimeError("hardware go_to is disabled until CF9 waypoint acceptance")

    def emergency(self) -> None:
        raise RuntimeError("disabled")


class FakeFuture:
    def __init__(self) -> None:
        self._done = True

    def done(self) -> bool:
        return self._done

    def result(self) -> None:
        return None


class FakeServiceClient:
    def __init__(self) -> None:
        self.requests: list[Any] = []

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return True

    def call_async(self, request: Any) -> FakeFuture:
        self.requests.append(request)
        return FakeFuture()


class FakeDuration:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class FakeTakeoffRequest:
    def __init__(self) -> None:
        self.group_mask = 99
        self.height = -1.0
        self.duration = None
