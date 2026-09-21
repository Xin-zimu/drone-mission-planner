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
telemetry = importlib.import_module("dmp_crazyflie_bridge.telemetry")


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


GROUND_SYMPTOM_ISSUES: list[dict[str, Any]] = [
    {"code": "pose_stability_failed", "message": "x", "blocking": True, "stage": "airborne"},
    {"code": "velocity_not_settled", "message": "x", "blocking": True, "stage": "airborne"},
    {"code": "range_invalid", "message": "x", "blocking": True, "stage": "airborne"},
    {"code": "range_ground_nearfield_unreliable", "message": "x", "blocking": False, "stage": "ground"},
]


def test_cf8_airborne_issues_at_ground_stabilize_then_hover_and_land_pass(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Case A: the ground gate skips airborne-only evidence, STABILIZING then verifies it.

    Skipping those issues at preflight is not an acceptance bypass any more: the vehicle
    still has to prove itself in the air before the hover phase is allowed to start.
    """

    def scenario() -> None:
        clock = FakeClock()
        adapter = FakeAcceptanceAdapter(z_m=0.034, readiness_issues=list(GROUND_SYMPTOM_ISSUES))
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
            adapter,
            policy=_stabilizing_policy(),
            clock=clock,
        )
        monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

        result = asyncio.run(runner.run())

        assert result.passed is True
        assert result.failure_code is None
        assert result.state is cf8_acceptance.BridgeExecutionState.LANDED
        events = _event_names(result)
        assert {"takeoff_target_reached", "stabilization_started", "stabilization_satisfied"} <= events
        assert {"hover_started", "hover_settled", "landed_confirmed"} <= events
        assert "stabilization_timeout" not in events
        assert "controlled_land_started" not in events

        # The ground preflight did see the airborne issues and deliberately did not block.
        assert result.preflight["passed"] is True
        preflight_codes = {issue["code"] for issue in result.preflight["readiness"]["issues"]}
        assert {"pose_stability_failed", "velocity_not_settled", "range_invalid"} <= preflight_codes

        assert len(adapter.takeoff_calls) == 1
        assert len(adapter.land_calls) == 1
        assert adapter.go_to_calls == []
        assert adapter.emergency_calls == 0

        report = result.metrics.stabilization
        assert report is not None
        assert report["ok"] is True
        assert report["anchor_s"] == pytest.approx(0.2)
        assert report["observed_s"] >= runner.policy.minimum_airborne_observation_s
        assert report["min_zrange_m"] >= 0.10
        assert report["max_zrange_m"] <= 0.50

    scenario()


def test_cf8_persistent_airborne_issue_times_out_into_controlled_land_and_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Case B: the same ground symptom never recovering in the air must fail the run."""

    def scenario() -> None:
        clock = FakeClock()
        adapter = FakeAcceptanceAdapter(z_m=0.034, readiness_issues=list(GROUND_SYMPTOM_ISSUES))
        # Airborne down-range stays inside the untrustworthy near field the whole time.
        adapter.zrange_m = 0.05
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
            adapter,
            policy=_stabilizing_policy(stabilization_timeout_s=1.0),
            clock=clock,
        )
        monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

        result = asyncio.run(runner.run())

        assert result.passed is False
        assert result.failure_code == "stabilization_timeout"
        events = _event_names(result)
        assert "hover_started" not in events
        assert "stabilization_satisfied" not in events
        assert "stabilization_timeout" in events
        controlled_land = [event for event in result.events if event.name == "controlled_land_started"]
        assert len(controlled_land) == 1
        assert controlled_land[0].monotonic_s == pytest.approx(1.2)
        assert len(adapter.takeoff_calls) == 1
        assert len(adapter.land_calls) == 1
        assert adapter.go_to_calls == []
        assert adapter.emergency_calls == 0

        report = result.metrics.stabilization
        assert report is not None
        assert report["ok"] is False
        assert "no_continuous_stabilization" in str(report["reason"])
        assert report["last_sample_reason"] == "range_out_of_band"

    scenario()


def test_cf8_ground_preflight_still_blocks_ground_stage_issues() -> None:
    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(
            readiness_issues=[
                {"code": "battery_critical", "message": "x", "blocking": True, "stage": "both"},
            ],
        )
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "preflight_failed"
        assert adapter.takeoff_calls == []
        assert adapter.land_calls == []

    asyncio.run(scenario())


def test_cf8_ground_preflight_fails_closed_for_unstaged_issues() -> None:
    """A readiness payload without stage must keep blocking: no silent bypass."""

    async def scenario() -> None:
        adapter = FakeAcceptanceAdapter(
            readiness_issues=[{"code": "pose_stability_failed", "message": "x", "blocking": True}],
        )
        runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())

        result = await runner.run()

        assert result.passed is False
        assert result.failure_code == "preflight_failed"
        assert adapter.takeoff_calls == []

    asyncio.run(scenario())


@pytest.mark.parametrize(
    ("stable_windows", "hover_duration_s", "expected_elapsed_s"),
    [
        ([(3.0, 10.0)], 3.0, 3.5),  # A: one settled sample at minimum is insufficient.
        ([(0.0, 10.0)], 3.0, 3.0),  # B: early settle cannot shorten minimum hover.
        ([(0.0, 0.3), (0.5, 10.0)], 0.3, 1.0),  # C: interrupted window restarts.
    ],
)
def test_cf8_hover_requires_minimum_and_continuous_settle(
    monkeypatch: pytest.MonkeyPatch,
    stable_windows: list[tuple[float, float]],
    hover_duration_s: float,
    expected_elapsed_s: float,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_fast_policy(
            hover_duration_s=hover_duration_s, settle_duration_s=0.5,
            poll_interval_s=0.1, max_total_acceptance_time_s=10.0,
        ),
        clock=clock,
    )
    origin = runner._capture_origin(adapter.session_id)
    original_telemetry = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        adapter.speed_mps = 0.0 if any(a <= clock() <= b for a, b in stable_windows) else 0.2
        return original_telemetry(**kwargs)

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)
    asyncio.run(runner._hover(origin=origin, target_z_m=origin.z_m, start_s=clock()))

    assert clock() == pytest.approx(expected_elapsed_s)
    assert adapter.takeoff_calls == adapter.land_calls == adapter.go_to_calls == []


def test_cf8_hover_never_settles_times_out_and_attempts_controlled_land(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_fast_policy(
            hover_duration_s=0.3, settle_duration_s=0.5, poll_interval_s=0.1,
            takeoff_timeout_s=1.0, landing_timeout_s=1.0,
            max_total_acceptance_time_s=1.25,
        ),
        clock=clock,
    )
    original_telemetry = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        adapter.speed_mps = 0.2 if kwargs.get("flight_state") == "hovering" else 0.0
        return original_telemetry(**kwargs)

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)
    result = asyncio.run(runner.run())

    assert result.passed is False
    assert result.failure_code == "hover_settle_timeout"
    assert len(adapter.takeoff_calls) == len(adapter.land_calls) == 1
    assert adapter.go_to_calls == []
    assert "hover_settled" not in _event_names(result)
    assert "landed_confirmed" in _event_names(result)
    controlled_land = next(e for e in result.events if e.name == "controlled_land_started")
    assert controlled_land.monotonic_s == pytest.approx(1.25)


def test_cf8_hover_cannot_pass_at_total_deadline(monkeypatch: pytest.MonkeyPatch) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_fast_policy(
            hover_duration_s=0.5, settle_duration_s=0.5,
            max_total_acceptance_time_s=0.5, poll_interval_s=0.1,
        ),
        clock=clock,
    )
    origin = runner._capture_origin(adapter.session_id)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)
    with pytest.raises(cf8_acceptance.CF8AcceptanceError) as exc:
        asyncio.run(runner._hover(origin=origin, target_z_m=origin.z_m, start_s=0.0))
    assert exc.value.code == "hover_settle_timeout"
    assert clock() == pytest.approx(0.5)


@pytest.mark.parametrize("phase", ["takeoff", "landing"])
@pytest.mark.parametrize("physical_completion", [True, False])
def test_cf8_motion_confirmation_uses_full_window_from_zero(
    monkeypatch: pytest.MonkeyPatch, phase: str, physical_completion: bool,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_fast_policy(settle_duration_s=0.5, landing_timeout_s=0.6, poll_interval_s=0.1),
        clock=clock,
    )
    origin = runner._capture_origin(adapter.session_id)
    original_telemetry = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        payload = original_telemetry(**kwargs)
        if not physical_completion:
            payload["z_m"] = origin.z_m + 0.2
        return payload

    async def scenario() -> None:
        if phase == "landing":
            await runner._land(origin=origin, reason="test")
        else:
            await runner._wait_for_settle(
                phase=phase, origin=origin, target_z_m=origin.z_m, timeout_s=0.6,
            )

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)
    if physical_completion:
        asyncio.run(scenario())
        assert clock() == pytest.approx(0.5)
    else:
        with pytest.raises(cf8_acceptance.CF8AcceptanceError) as exc:
            asyncio.run(scenario())
        assert exc.value.code == f"{phase}_motion_timeout"
        assert clock() == pytest.approx(0.6)
        assert "landed_confirmed" not in _event_names(runner)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    async def sleep(self, delay: float) -> None:
        self.now = round(self.now + delay, 9)


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


@pytest.mark.parametrize(
    "bad_confirm",
    ["true", "yes", "1", 1, 0, -1, False, None, [], {}, 1.0],
    ids=[
        "string-true",
        "string-yes",
        "string-1",
        "int-1",
        "int-0",
        "int-negative",
        "false",
        "none",
        "list",
        "dict",
        "float-1",
    ],
)
def test_cf8_protocol_rejects_non_boolean_confirm_real_flight(
    monkeypatch: Any,
    bad_confirm: Any,
) -> None:
    """Only a strict JSON boolean true may unlock CF8; truthy look-alikes must not."""
    fake = FakeAcceptanceAdapter()
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda *_, **__: fake)
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231")

    response = asyncio.run(
        server.handle_message_async(
            {
                "type": "run_cf8_acceptance",
                "request_id": "cf8-bad-confirm",
                "confirm_real_flight": bad_confirm,
            }
        )
    )

    assert response["type"] == "error"
    assert response["code"] == "operator_confirmation_required"
    assert fake.takeoff_calls == []
    assert fake.land_calls == []
    assert fake.arm_calls == []
    assert fake.go_to_calls == []
    assert server._cf8_runner is None


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


def test_cf8_stabilization_defaults_match_the_audited_cf81_values() -> None:
    policy = cf8_acceptance.CF8AcceptancePolicy()

    assert policy.minimum_airborne_observation_s == pytest.approx(5.0)
    assert policy.stabilization_window_s == pytest.approx(3.0)
    assert policy.stabilization_timeout_s == pytest.approx(10.0)
    assert policy.stabilization_min_zrange_m == pytest.approx(0.10)
    assert policy.stabilization_max_zrange_m == pytest.approx(0.50)
    assert policy.stabilization_max_abs_velocity_mps == pytest.approx(0.05)
    assert policy.stabilization_max_speed_mps == pytest.approx(0.08)
    assert policy.stabilization_max_xy_span_m == pytest.approx(0.03)
    assert policy.stabilization_max_xy_displacement_m == pytest.approx(0.03)
    assert policy.stabilization_max_sample_gap_s == pytest.approx(0.5)
    assert policy.stabilization_max_pose_age_s == pytest.approx(0.75)
    # takeoff + 5 s of airborne observation + hover + land must still fit the total budget.
    assert policy.takeoff_duration_s + 5.0 + policy.hover_duration_s + policy.land_duration_s < (
        policy.max_total_acceptance_time_s
    )


def test_stabilization_readiness_code_set_excludes_ground_contaminated_pose_stability() -> None:
    """D2: the collector's 200-sample pose deque must never gate the CF8 airborne window."""
    assert "pose_stability_observation_missing" not in cf8_acceptance.STABILIZATION_READINESS_CODES
    assert "pose_stability_failed" not in cf8_acceptance.STABILIZATION_READINESS_CODES
    # ...and the codes the runner recomputes itself must not be consumed twice.
    assert "velocity_not_settled" not in cf8_acceptance.STABILIZATION_READINESS_CODES
    assert "range_invalid" not in cf8_acceptance.STABILIZATION_READINESS_CODES
    assert {
        "range_unavailable",
        "range_stale",
        "velocity_unavailable",
        "velocity_stale",
        "estimator_not_converged",
        "supervisor_cannot_fly",
    } == cf8_acceptance.STABILIZATION_READINESS_CODES


# --- AirborneStabilizationWindow ------------------------------------------------------


def _window_policy(**updates: Any) -> Any:
    values = {
        "minimum_airborne_observation_s": 5.0,
        "stabilization_window_s": 3.0,
        "stabilization_timeout_s": 10.0,
    }
    values.update(updates)
    return cf8_acceptance.CF8AcceptancePolicy(**values)


def _ok_sample(t: float, **updates: Any) -> Any:
    values: dict[str, Any] = {
        "monotonic_s": t,
        "ok": True,
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.27,
        "zrange_m": 0.26,
        "vx_mps": 0.0,
        "vy_mps": 0.0,
        "vz_mps": 0.0,
        "speed_mps": 0.0,
    }
    values.update(updates)
    return cf8_acceptance.AirborneStabilizationSample(**values)


def test_stabilization_window_requires_minimum_observation_then_full_window() -> None:
    window = cf8_acceptance.AirborneStabilizationWindow(_window_policy(), anchor_s=0.0)
    verdicts: dict[float, bool] = {}
    for step in range(0, 71):
        stamp = round(step * 0.1, 9)
        verdicts[stamp] = window.observe(_ok_sample(stamp)).ok

    # The 3 s continuous window is satisfied at 3 s, but the 5 s observation floor dominates.
    assert verdicts[2.9] is False
    assert verdicts[4.9] is False
    assert verdicts[5.0] is True
    assert [stamp for stamp, ok in verdicts.items() if ok] == [
        stamp for stamp in verdicts if stamp >= 5.0
    ]


def test_stabilization_velocity_spike_restarts_the_continuous_window() -> None:
    window = cf8_acceptance.AirborneStabilizationWindow(_window_policy(), anchor_s=0.0)
    verdicts: dict[float, bool] = {}
    for step in range(0, 91):
        stamp = round(step * 0.1, 9)
        sample = (
            _ok_sample(stamp, ok=False, reason="velocity_exceeded", vx_mps=0.2, speed_mps=0.2)
            if stamp == 5.5
            else _ok_sample(stamp)
        )
        verdicts[stamp] = window.observe(sample).ok

    assert verdicts[5.0] is True
    assert verdicts[5.5] is False
    assert verdicts[8.4] is False
    assert verdicts[8.7] is True
    assert window.continuous_since_s == pytest.approx(5.6)


def test_stabilization_window_cannot_be_satisfied_by_pre_anchor_samples() -> None:
    window = cf8_acceptance.AirborneStabilizationWindow(_window_policy(), anchor_s=10.0)
    report = None
    for step in range(0, 61):
        report = window.observe(_ok_sample(round(step * 0.1, 9)))

    assert report is not None
    assert report.observed_s == 0.0
    assert report.ok is False
    assert report.reason is not None
    assert "airborne_observation_too_short" in report.reason
    # Samples older than the window plus one sample gap are pruned: no unbounded growth.
    assert 0 < report.sample_count <= 40


@pytest.mark.parametrize(
    ("zrange_m", "expected_ok"),
    [
        (0.05, False),
        (0.0999, False),
        (0.10, True),
        (0.26, True),
        (0.50, True),
        (0.5001, False),
        (0.75, False),
    ],
)
def test_stabilization_window_enforces_the_reliable_range_band(
    zrange_m: float,
    expected_ok: bool,
) -> None:
    """The band is a window invariant, independent of each sample's own ok flag."""
    window = cf8_acceptance.AirborneStabilizationWindow(_window_policy(), anchor_s=0.0)
    report = None
    for step in range(0, 61):
        report = window.observe(_ok_sample(round(step * 0.1, 9), zrange_m=zrange_m))

    assert report is not None
    assert report.ok is expected_ok
    if not expected_ok:
        assert "stabilization_range_" in str(report.reason)


@pytest.mark.parametrize(
    ("vx_mps", "vy_mps", "vz_mps", "speed_mps", "expected_ok"),
    [
        (0.05, 0.0, 0.0, 0.05, True),
        (-0.05, 0.0, 0.0, 0.05, True),
        (0.0500001, 0.0, 0.0, 0.0500001, False),
        (0.0, 0.0, 0.0500001, 0.0500001, False),
        (0.04, 0.04, 0.04, 0.08, True),
        (0.04, 0.04, 0.04, 0.0800001, False),
    ],
)
def test_stabilization_window_uses_strict_max_velocity_semantics(
    vx_mps: float,
    vy_mps: float,
    vz_mps: float,
    speed_mps: float,
    expected_ok: bool,
) -> None:
    """Strict max over the whole window: no percentile and no per-axis slack."""
    window = cf8_acceptance.AirborneStabilizationWindow(_window_policy(), anchor_s=0.0)
    report = None
    for step in range(0, 61):
        report = window.observe(
            _ok_sample(
                round(step * 0.1, 9),
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                vz_mps=vz_mps,
                speed_mps=speed_mps,
            )
        )

    assert report is not None
    assert report.ok is expected_ok
    assert report.max_abs_vx_mps == pytest.approx(abs(vx_mps))
    assert report.max_speed_mps == pytest.approx(speed_mps)
    if not expected_ok:
        assert "exceeded" in str(report.reason)


@pytest.mark.parametrize(
    ("drift_m", "expected_ok"),
    [(0.0, True), (0.02, True), (0.03, True), (0.0301, False), (0.06, False)],
)
def test_stabilization_window_enforces_xy_span_and_displacement(
    drift_m: float,
    expected_ok: bool,
) -> None:
    # Anchor plus a 3 s observation floor keeps the whole ramp inside the retained window.
    policy = _window_policy(minimum_airborne_observation_s=3.0)
    window = cf8_acceptance.AirborneStabilizationWindow(policy, anchor_s=0.0)
    report = None
    for step in range(0, 61):
        stamp = round(step * 0.1, 9)
        offset = max(0.0, stamp - 3.0)
        report = window.observe(_ok_sample(stamp, x_m=drift_m * offset / 3.0))

    assert report is not None
    assert report.xy_span_m == pytest.approx(drift_m, abs=1e-9)
    assert report.xy_displacement_m == pytest.approx(drift_m, abs=1e-9)
    assert report.ok is expected_ok
    if not expected_ok:
        assert "stabilization_xy_" in str(report.reason)


# --- STABILIZING runner behaviour ------------------------------------------------------


def test_cf8_pre_anchor_ground_samples_never_reach_the_stabilization_window(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Ground noise must not be able to poison the airborne window."""
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter(z_m=0.02)
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    original = adapter.telemetry_payload
    ground_reads: list[float] = []

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        payload = original(**kwargs)
        state = kwargs.get("flight_state")
        if state in {"preflight_recheck", "taking_off"}:
            # Every one of these independently fails a stabilization criterion, and none
            # of them is part of the ground launch gate or of _settled/_landed.
            ground_reads.append(clock())
            payload["range"] = {"zrange_m": 0.006}
            payload["pose_age_s"] = 9.0
            payload["estimator_variance"] = None
        if state == "preflight_recheck":
            payload["velocity"] = {
                "vx_mps": 0.12,
                "vy_mps": 0.09,
                "vz_mps": 0.01,
                "speed_mps": 0.15,
            }
        return payload

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert ground_reads, "the polluted pre-anchor telemetry was never served"
    assert result.passed is True
    report = result.metrics.stabilization
    assert report is not None
    assert report["ok"] is True
    assert report["anchor_s"] == pytest.approx(0.2)
    assert report["window_start_s"] is not None
    assert report["window_start_s"] >= report["anchor_s"]
    # Every polluted read happened at or before the anchor, and none of that leaked in.
    assert all(stamp <= report["anchor_s"] for stamp in ground_reads)
    assert report["min_zrange_m"] >= 0.10
    assert report["max_zrange_m"] <= 0.50
    assert report["max_abs_vx_mps"] <= 0.05
    assert report["max_speed_mps"] <= 0.08
    assert adapter.go_to_calls == []
    assert adapter.emergency_calls == 0


def test_cf8_battery_critical_during_stabilizing_lands_immediately(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    original = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        payload = original(**kwargs)
        payload["battery_critical"] = kwargs.get("flight_state") == "stabilizing"
        return payload

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.failure_code == "battery_critical"
    events = _event_names(result)
    assert "stabilization_timeout" not in events
    assert "hover_started" not in events
    controlled_land = [event for event in result.events if event.name == "controlled_land_started"]
    assert len(controlled_land) == 1
    # Immediately at the anchor, not after the 2.0 s stabilization timeout.
    assert controlled_land[0].monotonic_s == pytest.approx(0.2)
    assert controlled_land[0].monotonic_s < 0.2 + runner.policy.stabilization_timeout_s
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0


def test_cf8_operator_abort_during_stabilizing_lands_immediately(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    original = adapter.telemetry_payload
    aborted = False

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        nonlocal aborted
        if kwargs.get("flight_state") == "stabilizing" and not aborted:
            aborted = True
            runner.request_abort("operator pressed the abort control")
        return original(**kwargs)

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.failure_code == "operator_abort"
    events = _event_names(result)
    assert "abort_requested" in events
    assert "hover_started" not in events
    controlled_land = [event for event in result.events if event.name == "controlled_land_started"]
    assert len(controlled_land) == 1
    assert controlled_land[0].monotonic_s == pytest.approx(0.3)
    assert controlled_land[0].monotonic_s < 0.2 + runner.policy.stabilization_timeout_s
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0


def test_cf8_can_fly_revoked_airborne_lands_immediately(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    original = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        if kwargs.get("flight_state") == "stabilizing":
            adapter.can_fly = False
        return original(**kwargs)

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.failure_code == "supervisor_cannot_fly"
    assert "hover_started" not in _event_names(result)
    controlled_land = [event for event in result.events if event.name == "controlled_land_started"]
    assert len(controlled_land) == 1
    assert controlled_land[0].monotonic_s == pytest.approx(0.2)
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0


def test_cf8_controlled_land_is_sent_once_and_never_emergency(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter()
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    original = adapter.telemetry_payload

    def telemetry(**kwargs: Any) -> dict[str, Any]:
        if kwargs.get("flight_state") == "stabilizing":
            adapter.locked = True
        return original(**kwargs)

    monkeypatch.setattr(adapter, "telemetry_payload", telemetry)
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.failure_code == "supervisor_locked"
    assert [event.name for event in result.events].count("controlled_land_started") == 1
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0
    assert adapter.go_to_calls == []
    assert "hover_started" not in _event_names(result)


def test_cf8_airborne_readiness_cannot_fly_breaks_stabilization(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An AIRBORNE `supervisor_cannot_fly` soft-fails stabilization instead of being skipped."""
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter(
        live_readiness_issues=[
            {
                "code": "supervisor_cannot_fly",
                "message": "supervisor revoked the flyable state",
                "blocking": True,
                "stage": "airborne",
            }
        ],
    )
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(stabilization_timeout_s=1.0),
        clock=clock,
    )
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.failure_code == "stabilization_timeout"
    report = result.metrics.stabilization
    assert report is not None
    assert report["last_sample_reason"] == "supervisor_cannot_fly"
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0


def test_cf8_stabilizing_ignores_ground_only_supervisor_issues(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Manual explicit-arm airborne: armed=True must not be killed by ground-only codes."""
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter(
        live_readiness_issues=[
            {"code": "supervisor_not_flyable", "message": "x", "blocking": True, "stage": "ground"},
            {"code": "supervisor_armed", "message": "x", "blocking": True, "stage": "ground"},
            {"code": "supervisor_flying", "message": "x", "blocking": True, "stage": "ground"},
        ],
    )
    adapter.can_be_armed = False
    adapter.auto_arm = False
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.passed is True
    assert "stabilization_satisfied" in _event_names(result)
    final = result.final_telemetry
    assert final is not None
    supervisor = final["supervisor"]
    assert supervisor["armed"] is True
    assert supervisor["auto_arm"] is False
    assert supervisor["can_be_armed"] is False
    assert supervisor["flying"] is False
    assert {issue["code"] for issue in final["readiness"]["issues"]} == {
        "supervisor_not_flyable",
        "supervisor_armed",
        "supervisor_flying",
    }
    assert len(adapter.land_calls) == 1
    assert adapter.emergency_calls == 0


@pytest.mark.parametrize(
    "code",
    sorted(
        telemetry.GROUND_ONLY_SUPERVISOR_ISSUE_CODES | telemetry.BOTH_STAGE_SUPERVISOR_ISSUE_CODES
    ),
)
def test_cf8_ground_preflight_blocks_ground_and_both_stage_supervisor_issues(
    code: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    adapter = FakeAcceptanceAdapter(
        readiness_issues=[
            {
                "code": code,
                "message": "x",
                "blocking": True,
                "stage": telemetry.SUPERVISOR_ISSUE_STAGE[code].value,
            }
        ],
    )
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(adapter, policy=_fast_policy())

    result = asyncio.run(runner.run())

    assert result.passed is False
    assert result.failure_code == "preflight_failed"
    assert adapter.takeoff_calls == []
    assert adapter.land_calls == []
    assert adapter.arm_calls == []
    assert adapter.go_to_calls == []


@pytest.mark.parametrize("code", sorted(telemetry.AIRBORNE_ONLY_SUPERVISOR_ISSUE_CODES))
def test_cf8_ground_preflight_ignores_airborne_only_supervisor_issues(
    code: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The ground gate must not consume `can_fly`/flow evidence that only exists airborne."""
    clock = FakeClock()
    adapter = FakeAcceptanceAdapter(
        preflight_only_issues=[
            {
                "code": code,
                "message": "x",
                "blocking": True,
                "stage": telemetry.SUPERVISOR_ISSUE_STAGE[code].value,
            }
        ],
    )
    runner = cf8_acceptance.HardwareFlightAcceptanceRunner(
        adapter,
        policy=_stabilizing_policy(),
        clock=clock,
    )
    monkeypatch.setattr(cf8_acceptance.asyncio, "sleep", clock.sleep)

    result = asyncio.run(runner.run())

    assert result.preflight["passed"] is True
    assert result.passed is True
    assert {issue["code"] for issue in result.preflight["readiness"]["issues"]} == {code}
    assert len(adapter.takeoff_calls) == 1
    assert adapter.go_to_calls == []
    assert adapter.emergency_calls == 0


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
        # Shrink the airborne stabilization gate so real-clock tests stay quick; the
        # production defaults (5.0 / 3.0 / 10.0) are asserted separately.
        "minimum_airborne_observation_s": 0.05,
        "stabilization_window_s": 0.05,
        "stabilization_timeout_s": 0.5,
    }
    values.update(updates)
    return cf8_acceptance.CF8AcceptancePolicy(**values)


def _stabilizing_policy(**updates: Any) -> Any:
    """Deterministic policy for the STABILIZING phase (FakeClock friendly, 0.1 s polls)."""
    values = {
        "takeoff_delta_m": 0.25,
        "takeoff_duration_s": 0.2,
        "hover_duration_s": 0.2,
        "land_duration_s": 0.2,
        "takeoff_timeout_s": 1.0,
        "landing_timeout_s": 1.0,
        "service_timeout_s": 0.1,
        "z_acceptance_tolerance_m": 0.03,
        "xy_drift_tolerance_m": 0.05,
        "settle_velocity_mps": 0.08,
        "settle_duration_s": 0.2,
        "max_total_acceptance_time_s": 20.0,
        "poll_interval_s": 0.1,
        "minimum_airborne_observation_s": 0.5,
        "stabilization_window_s": 0.5,
        "stabilization_timeout_s": 2.0,
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
        preflight_only_issues: list[dict[str, Any]] | None = None,
        live_readiness_issues: list[dict[str, Any]] | None = None,
        fail_after_takeoff: str | None = None,
        takeoff_error: Exception | None = None,
    ) -> None:
        self.robot_id = "cf231"
        self.session = "session-1"
        self.x_m = 1.0
        self.y_m = 2.0
        self.z_m = z_m
        self.vx_mps = 0.0
        self.vy_mps = 0.0
        self.vz_mps = 0.0
        self.speed_mps = 0.0
        self.pose_age_s = 0.0
        # None means "derive from the height": near field on the ground, real range airborne.
        self.zrange_m: float | None = None
        self.estimator_converged = True
        self.battery_critical = False
        self.flying = False
        self.armed = False
        self.auto_arm = False
        self.can_be_armed = True
        self.can_fly = True
        self.locked = False
        self.tumbled = False
        self.crashed = False
        self.high_level_active = False
        self.deck_fault = False
        # readiness_issues are visible at preflight *and* live (legacy behaviour).
        self.readiness_issues = readiness_issues or []
        # ...preflight_only_issues gate the launch but are gone once airborne.
        self.preflight_only_issues = preflight_only_issues or []
        # ...live_readiness_issues only exist in the airborne telemetry payload.
        self.live_readiness_issues = live_readiness_issues or []
        self.fail_after_takeoff = fail_after_takeoff
        self.takeoff_error = takeoff_error
        self.takeoff_calls: list[tuple[Any, Any]] = []
        self.land_calls: list[tuple[Any, Any]] = []
        self.arm_calls: list[bool] = []
        self.go_to_calls: list[tuple[Any, ...]] = []
        self.emergency_calls = 0
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
        issues = [*self.readiness_issues, *self.preflight_only_issues]
        return {"passed": not issues, "issues": issues}

    @property
    def session_id(self) -> str | None:
        return self.session

    def supervisor_raw(self) -> int:
        raw = 0
        for bit, enabled in (
            (0x0001, self.can_be_armed),
            (0x0002, self.armed),
            (0x0004, self.auto_arm),
            (0x0008, self.can_fly),
            (0x0010, self.flying),
            (0x0020, self.tumbled),
            (0x0040, self.locked),
            (0x0080, self.crashed),
            (0x0100, self.high_level_active),
            (0x0800, self.deck_fault),
        ):
            if enabled:
                raw |= bit
        return raw

    def supervisor_payload(self) -> dict[str, Any]:
        return {
            "can_be_armed": self.can_be_armed,
            "armed": self.armed,
            "auto_arm": self.auto_arm,
            "can_fly": self.can_fly,
            "flying": self.flying,
            "locked": self.locked,
            "tumbled": self.tumbled,
            "crashed": self.crashed,
            "high_level_control_active": self.high_level_active,
            "deck_fault": self.deck_fault,
        }

    def telemetry_payload(self, **_: Any) -> dict[str, Any]:
        self.telemetry_reads += 1
        if self.fail_after_takeoff == "battery_critical" and self.takeoff_calls and self.telemetry_reads > 6:
            self.battery_critical = True
        if self.zrange_m is not None:
            zrange_m = self.zrange_m
        elif self.flying:
            zrange_m = self.z_m
        else:
            # VL53L1x near field while sitting on the ground: advisory only, never blocking.
            zrange_m = 0.008
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
            "pose_age_s": self.pose_age_s,
            "velocity": {
                "vx_mps": self.vx_mps,
                "vy_mps": self.vy_mps,
                "vz_mps": self.vz_mps,
                "speed_mps": self.speed_mps,
            },
            "range": {"zrange_m": zrange_m},
            "estimator_variance": (
                {"var_x": 0.0001, "var_y": 0.0001, "var_z": 0.0001}
                if self.estimator_converged
                else None
            ),
            "supervisor_info_raw": self.supervisor_raw(),
            "supervisor": self.supervisor_payload(),
            "readiness": {
                "passed": not (self.readiness_issues or self.live_readiness_issues),
                "issues": [*self.readiness_issues, *self.live_readiness_issues],
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
        self.emergency_calls += 1
        raise RuntimeError("emergency is disabled: CF8 must use the controlled landing path")


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
