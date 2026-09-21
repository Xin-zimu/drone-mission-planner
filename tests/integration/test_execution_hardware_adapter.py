from __future__ import annotations

import importlib
import json
import sys
import types
import warnings
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
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

telemetry = importlib.import_module("dmp_crazyflie_bridge.telemetry")
protocol_server = importlib.import_module("dmp_crazyflie_bridge.protocol_server")
crazyswarm_adapter = importlib.import_module("dmp_crazyflie_bridge.crazyswarm_adapter")
capability_probe = importlib.import_module("dmp_crazyflie_bridge.capability_probe")

# Down-range reading inside the reliable band (>= DEFAULT_MIN_RELIABLE_ZRANGE_M = 0.10 m).
# Ground-level readings (0-13 mm) sit inside the sensor's untrustworthy near field, so a
# fully healthy readiness snapshot has to use a value like this one.
HEALTHY_ZRANGE_MM = 260.0
HEALTHY_ZRANGE_M = 0.26


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


def test_deck_parameter_reader_recovers_when_service_becomes_available(tmp_path: Path) -> None:
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=tmp_path / "absent.json",
    )

    adapter._refresh_positioning_capabilities()
    assert adapter._collector.snapshot().positioning.mode == "unknown"

    adapter._parameter_client = FakeParameterClient(values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1})
    adapter._refresh_positioning_capabilities()

    snapshot = adapter._collector.snapshot()
    assert snapshot.positioning.mode == "flow"
    assert snapshot.positioning.xy_available is True
    assert snapshot.positioning.z_available is True
    assert "deck.bcFlow2=1" in snapshot.positioning.evidence


def test_deck_parameter_reader_preserves_z_only_and_all_zero(tmp_path: Path) -> None:
    absent = tmp_path / "absent.json"
    z_only = _adapter_with_client(
        FakeParameterClient(values={"deck.bcFlow2": 0, "deck.bcZRanger2": 1}),
        snapshot_path=absent,
    )
    z_only._refresh_positioning_capabilities()
    assert z_only._collector.snapshot().positioning.mode == "z_ranger"

    none = _adapter_with_client(FakeParameterClient(values={}), snapshot_path=absent)
    none._refresh_positioning_capabilities()
    snapshot = none._collector.snapshot()
    assert snapshot.positioning.mode == "none"
    assert snapshot.positioning.xy_available is False
    assert snapshot.positioning.z_available is False


def test_deck_parameter_reader_blocks_partial_unset_and_timeout(tmp_path: Path) -> None:
    absent = tmp_path / "absent.json"
    partial = _adapter_with_client(FakeParameterClient(value_count=3), snapshot_path=absent)
    partial._refresh_positioning_capabilities()
    partial_snapshot = partial._collector.snapshot()
    assert partial_snapshot.positioning.mode == "unknown"
    assert any("expected 5, received 3" in item for item in partial_snapshot.diagnostics)

    unset = _adapter_with_client(
        FakeParameterClient(unset_names={"deck.bcFlow2"}),
        snapshot_path=absent,
    )
    unset._refresh_positioning_capabilities()
    unset_snapshot = unset._collector.snapshot()
    assert unset_snapshot.positioning.mode == "unknown"
    assert any("listed but unset" in item for item in unset_snapshot.diagnostics)

    timeout = _adapter_with_client(FakeParameterClient(get_timeout=True), snapshot_path=absent)
    timeout._refresh_positioning_capabilities()
    timeout_snapshot = timeout._collector.snapshot()
    assert timeout_snapshot.positioning.mode == "unknown"
    assert any("timed out" in item for item in timeout_snapshot.diagnostics)


def test_deck_parameter_reader_uses_cf231_runtime_names_with_namespace(tmp_path: Path) -> None:
    client = FakeParameterClient(
        namespace_prefix="swarm.",
        values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1},
    )
    adapter = _adapter_with_client(client, snapshot_path=tmp_path / "absent.json")

    adapter._refresh_positioning_capabilities()

    assert adapter._collector.snapshot().positioning.mode == "flow"
    assert client.requested_names == (
        "swarm.cf231.params.deck.bcFlow2",
        "swarm.cf231.params.deck.bcZRanger2",
        "swarm.cf231.params.deck.bcLighthouse4",
        "swarm.cf231.params.deck.bcLoco",
        "swarm.cf231.params.deck.bcDWM1000",
    )


def test_capability_source_priority_prefers_live_ros_over_snapshot(tmp_path: Path) -> None:
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(values={}),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )

    robot = adapter.capabilities()

    assert robot.positioning_mode == "none"
    assert robot.capability_source == "ros_runtime_parameter"
    assert robot.xy_positioning_available is False


def test_deck_parameter_reader_uses_fresh_cflib_snapshot_when_ros_unset(tmp_path: Path) -> None:
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(unset_names={"deck.bcFlow2"}),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )

    robot = adapter.capabilities()

    assert robot.positioning_mode == "flow"
    assert robot.xy_positioning_available is True
    assert robot.z_positioning_available is True
    assert robot.capability_source == "cflib_firmware_parameter_probe"
    assert robot.capability_captured_at is not None
    assert "deck.bcFlow2=1" in robot.positioning_evidence


def test_capability_requires_a_session_and_fails_closed_without_one(tmp_path: Path) -> None:
    """A perfectly fresh snapshot is still not proof while no hardware session exists."""
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1}),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
        session=False,
    )

    robot = adapter.capabilities()

    assert adapter.session_id is None
    assert robot.positioning_mode == "unknown"
    assert robot.xy_positioning_available is False
    assert robot.z_positioning_available is False


def test_startup_with_already_stale_snapshot_fails_closed_as_unknown(tmp_path: Path) -> None:
    """Requirement: a snapshot that is stale before the session starts must never grant."""
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")
    adapter = _adapter_with_client(
        FakeParameterClient(unset_names={"deck.bcFlow2"}),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )
    adapter.capability_snapshot_max_age_s = 3600.0

    robot = adapter.capabilities()

    assert adapter.session_id is not None
    assert robot.xy_positioning_available is False
    assert robot.z_positioning_available is False
    assert robot.positioning_mode == "unknown"
    assert any("stale" in item for item in robot.diagnostics)

    codes = {issue.code for issue in adapter._collector.snapshot().flight_readiness.issues}
    assert "xy_positioning_missing" in codes
    assert "z_positioning_missing" in codes

    # It stays rejected on later calls in the same session: snapshot age only grows.
    for _ in range(3):
        assert adapter.capabilities().positioning_mode == "unknown"


def test_session_capability_survives_snapshot_ttl_expiry_mid_session(tmp_path: Path) -> None:
    """Requirement: a wall-clock TTL must never withdraw capability from a live session."""
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )
    adapter.capability_snapshot_max_age_s = 3600.0

    granted = adapter.capabilities()
    session_id = adapter.session_id
    assert granted.positioning_mode == "flow"
    assert granted.capability_source == "cflib_firmware_parameter_probe"
    assert session_id is not None

    # Simulate the wall clock running well past the snapshot TTL, with the live parameter
    # service still unavailable: without session pinning this revoked capability.
    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")

    after = adapter.capabilities()

    assert adapter.session_id == session_id
    assert after.positioning_mode == "flow"
    assert after.xy_positioning_available is True
    assert after.z_positioning_available is True
    assert after.capability_source == "cflib_firmware_parameter_probe"
    assert after.capability_captured_at == granted.capability_captured_at
    assert not any("stale" in item for item in after.diagnostics)


def test_repeated_capabilities_calls_do_not_flip_a_pinned_session(tmp_path: Path) -> None:
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(values={"deck.bcFlow2": 1, "deck.bcZRanger2": 1}),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )
    adapter.capability_snapshot_max_age_s = 1.0

    assert adapter.capabilities().positioning_mode == "flow"
    pin_session = adapter._capability_pin_session_id
    # Now every source would disagree: live says no decks, the snapshot is stale.
    adapter._parameter_client = FakeParameterClient(values={})
    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")

    verdicts = [adapter.capabilities() for _ in range(5)]

    assert {robot.positioning_mode for robot in verdicts} == {"flow"}
    assert {robot.capability_source for robot in verdicts} == {"ros_runtime_parameter"}
    assert adapter._capability_pin_session_id == pin_session
    assert len({adapter.session_id for _ in range(3)}) == 1


def test_new_session_rejects_the_now_stale_snapshot_again(tmp_path: Path) -> None:
    """Requirement: after the session id changes, freshness is judged from scratch."""
    clock = FakeClock()
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
        clock=clock,
    )
    adapter.capability_snapshot_max_age_s = 3600.0

    assert adapter.capabilities().positioning_mode == "flow"
    first_session = adapter.session_id
    assert first_session is not None

    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")
    clock.advance(adapter.status_stale_timeout_s + 1.0)
    assert adapter.capabilities().positioning_mode == "unknown"  # session dropped
    assert adapter.session_id is None

    _publish_fresh_status(adapter)
    second = adapter.capabilities()

    assert adapter.session_id is not None
    assert adapter.session_id != first_session
    assert second.positioning_mode == "unknown"
    assert second.xy_positioning_available is False


def test_capability_pin_never_leaks_across_sessions(tmp_path: Path) -> None:
    """A verdict granted to session A must not be inherited by session B."""
    clock = FakeClock()
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
        clock=clock,
    )
    adapter.capability_snapshot_max_age_s = 3600.0

    assert adapter.capabilities().positioning_mode == "flow"
    first_pin = adapter._capability_pin_session_id

    clock.advance(adapter.status_stale_timeout_s + 1.0)
    assert adapter.capabilities().positioning_mode == "unknown"
    assert adapter._capability_pin_session_id is None

    # Session B sees a different truth: only the Z-ranger is populated.
    adapter._parameter_client = FakeParameterClient(values={"deck.bcFlow2": 0, "deck.bcZRanger2": 1})
    _publish_fresh_status(adapter)
    second = adapter.capabilities()

    assert second.positioning_mode == "z_ranger"
    assert second.xy_positioning_available is False
    assert second.z_positioning_available is True
    assert adapter._capability_pin_session_id == adapter.session_id
    assert adapter._capability_pin_session_id != first_pin


def test_disconnect_and_reconnect_reverify_capability_freshness(tmp_path: Path) -> None:
    """Requirement: disconnect / reconnect must re-run the freshness decision."""
    clock = FakeClock()
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
        clock=clock,
    )
    adapter.capability_snapshot_max_age_s = 3600.0

    assert adapter.capabilities().positioning_mode == "flow"

    adapter.disconnect()

    assert adapter.session_id is None
    assert adapter._capability_pin_session_id is None
    assert adapter._capability_source is None

    # The last status goes stale, so there is no session and no capability at all.
    clock.advance(adapter.status_stale_timeout_s + 1.0)
    assert adapter.capabilities().positioning_mode == "unknown"

    # Reconnect with the snapshot now stale -> re-verified and still fail closed.
    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")
    _publish_fresh_status(adapter)
    reconnected = adapter.capabilities()

    assert adapter.session_id is not None
    assert reconnected.positioning_mode == "unknown"
    assert reconnected.xy_positioning_available is False

    # Reconnect again with a fresh snapshot -> granted again, never inherited.
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)
    clock.advance(adapter.status_stale_timeout_s + 1.0)
    _publish_fresh_status(adapter)

    assert adapter.capabilities().positioning_mode == "flow"


def test_adapter_never_starts_a_probe_or_restarts_crazyswarm2(monkeypatch: Any, tmp_path: Path) -> None:
    """Requirements 6/7: the Bridge never seizes the radio or bounces Crazyswarm2."""
    probe_calls: list[dict[str, Any]] = []

    def _fail_probe(**kwargs: Any) -> Any:
        probe_calls.append(kwargs)
        raise AssertionError("the bridge must never start a firmware capability probe")

    monkeypatch.setattr(capability_probe, "probe_cflib_capabilities", _fail_probe)

    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True, captured_at_utc="2000-01-01T00:00:00Z")
    adapter = _adapter_with_client(
        FakeParameterClient(service_ready=False),
        snapshot_path=snapshot_path,
        robot_uri=CF231_URI,
    )

    for _ in range(3):
        adapter.capabilities()
        adapter._refresh_positioning_capabilities()

    assert probe_calls == []
    assert adapter.capabilities().positioning_mode == "unknown"

    source = Path(crazyswarm_adapter.__file__).read_text(encoding="utf-8")
    for forbidden in (
        "probe_cflib_capabilities",
        "SyncCrazyflie",
        "crtp",
        "subprocess",
        "Popen",
        "os.system",
        "pkill",
        "ros2 launch",
    ):
        assert forbidden not in source, f"adapter must not reference {forbidden!r}"


def test_cflib_snapshot_rejects_robot_uri_stale_and_missing(tmp_path: Path) -> None:
    snapshot_path = tmp_path / "cf231_capabilities.json"
    _write_snapshot(snapshot_path, flow2=True, zranger2=True)

    mismatch_robot = capability_probe.load_capability_snapshot(
        snapshot_path,
        robot_id="cf999",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not mismatch_robot.valid
    assert "robot mismatch" in mismatch_robot.diagnostic

    mismatch_uri = capability_probe.load_capability_snapshot(
        snapshot_path,
        robot_id="cf231",
        uri="radio://0/1/2M/E7E7E7E701",
        max_age_s=3600.0,
    )
    assert not mismatch_uri.valid
    assert "URI mismatch" in mismatch_uri.diagnostic

    stale_path = tmp_path / "stale.json"
    _write_snapshot(stale_path, captured_at_utc="2000-01-01T00:00:00Z")
    stale = capability_probe.load_capability_snapshot(
        stale_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=1.0,
    )
    assert not stale.valid
    assert "stale" in stale.diagnostic

    missing = capability_probe.load_capability_snapshot(
        tmp_path / "missing.json",
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not missing.valid
    assert "not found" in missing.diagnostic


def test_default_snapshot_path_is_package_runtime() -> None:
    path = capability_probe.default_snapshot_path("cf231")

    assert path.name == "cf231_capabilities.json"
    assert path.parent.name == "runtime"
    assert path.parent.parent.name == "dmp_crazyflie_bridge"
    assert path.parent.parent.parent.name == "src"


def test_cflib_snapshot_rejects_missing_deck_key_invalid_source_future_and_max_age(tmp_path: Path) -> None:
    snapshot_path = tmp_path / "invalid.json"
    _write_snapshot(snapshot_path)
    payload = json.loads(snapshot_path.read_text(encoding="utf-8"))
    del payload["deck"]["bcDWM1000"]
    snapshot_path.write_text(json.dumps(payload), encoding="utf-8")

    missing_deck = capability_probe.load_capability_snapshot(
        snapshot_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not missing_deck.valid
    assert "missing deck keys" in missing_deck.diagnostic

    malformed_path = tmp_path / "malformed.json"
    malformed_payload = dict(payload)
    malformed_payload["deck"] = []
    malformed_path.write_text(json.dumps(malformed_payload), encoding="utf-8")
    malformed = capability_probe.load_capability_snapshot(
        malformed_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not malformed.valid
    assert "missing deck values" in malformed.diagnostic

    invalid_source_path = tmp_path / "source.json"
    _write_snapshot(invalid_source_path)
    invalid_source_payload = json.loads(invalid_source_path.read_text(encoding="utf-8"))
    invalid_source_payload["source"] = "guessed"
    invalid_source_path.write_text(json.dumps(invalid_source_payload), encoding="utf-8")
    invalid_source = capability_probe.load_capability_snapshot(
        invalid_source_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not invalid_source.valid
    assert "unsupported" in invalid_source.diagnostic

    future_path = tmp_path / "future.json"
    future = (datetime.now(UTC) + timedelta(seconds=60)).isoformat(timespec="seconds").replace("+00:00", "Z")
    _write_snapshot(future_path, captured_at_utc=future)
    future_result = capability_probe.load_capability_snapshot(
        future_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert not future_result.valid
    assert "future" in future_result.diagnostic

    skew_path = tmp_path / "skew.json"
    slight_skew = (datetime.now(UTC) + timedelta(seconds=2)).isoformat(timespec="seconds").replace("+00:00", "Z")
    _write_snapshot(skew_path, captured_at_utc=slight_skew)
    skew = capability_probe.load_capability_snapshot(
        skew_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=3600.0,
    )
    assert skew.valid

    invalid_max_age = capability_probe.load_capability_snapshot(
        skew_path,
        robot_id="cf231",
        uri=CF231_URI,
        max_age_s=0.0,
    )
    assert not invalid_max_age.valid
    assert "max age" in invalid_max_age.diagnostic


def test_config_path_is_relative_to_config_file_and_missing_config_fails(tmp_path: Path) -> None:
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    config_path = config_dir / "bridge.yaml"
    config_path.write_text("capability_snapshot_path: ../runtime/cf231.json\n", encoding="utf-8")

    config = protocol_server._load_config(str(config_path))

    assert protocol_server._config_path(config, "capability_snapshot_path") == (tmp_path / "runtime" / "cf231.json").resolve()
    try:
        protocol_server._load_config(str(tmp_path / "missing.yaml"))
    except FileNotFoundError as exc:
        assert "configuration file not found" in str(exc)
    else:
        raise AssertionError("missing explicit config must fail closed")


def test_cflib_snapshot_covers_z_only_and_all_zero(tmp_path: Path) -> None:
    z_only_path = tmp_path / "z_only.json"
    _write_snapshot(z_only_path, flow2=False, zranger2=True)
    z_only = _adapter_with_client(
        FakeParameterClient(value_count=0),
        snapshot_path=z_only_path,
        robot_uri=CF231_URI,
    )
    z_robot = z_only.capabilities()
    assert z_robot.positioning_mode == "z_ranger"
    assert z_robot.xy_positioning_available is False
    assert z_robot.z_positioning_available is True

    none_path = tmp_path / "none.json"
    _write_snapshot(none_path, flow2=False, zranger2=False)
    none = _adapter_with_client(
        FakeParameterClient(value_count=0),
        snapshot_path=none_path,
        robot_uri=CF231_URI,
    )
    none_robot = none.capabilities()
    assert none_robot.positioning_mode == "none"
    assert none_robot.xy_positioning_available is False
    assert none_robot.z_positioning_available is False


def test_probe_failure_and_busy_error_are_reported(monkeypatch: Any) -> None:
    def fail_import(*_: Any, **__: Any) -> None:
        raise ImportError("busy radio")

    with monkeypatch.context() as scoped:
        scoped.setattr("builtins.__import__", fail_import)
        try:
            capability_probe.probe_cflib_capabilities(robot_id="cf231", uri=CF231_URI, timeout_s=0.01)
        except capability_probe.CapabilityProbeError as exc:
            assert "cflib unavailable" in str(exc)
        else:
            raise AssertionError("probe failure should raise CapabilityProbeError")

    cflib = types.ModuleType("cflib")
    crtp = types.ModuleType("cflib.crtp")
    crazyflie_pkg = types.ModuleType("cflib.crazyflie")
    sync_module = types.ModuleType("cflib.crazyflie.syncCrazyflie")

    def init_drivers(*_: Any, **__: Any) -> None:
        return None

    class BusySyncCrazyflie:
        def __init__(self, uri: str) -> None:
            self.uri = uri

        def __enter__(self) -> BusySyncCrazyflie:
            raise OSError("Crazyradio busy")

        def __exit__(self, *_: Any) -> None:
            return None

    crtp.init_drivers = init_drivers  # type: ignore[attr-defined]
    sync_module.SyncCrazyflie = BusySyncCrazyflie  # type: ignore[attr-defined]
    cflib.crtp = crtp  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "cflib", cflib)
    monkeypatch.setitem(sys.modules, "cflib.crtp", crtp)
    monkeypatch.setitem(sys.modules, "cflib.crazyflie", crazyflie_pkg)
    monkeypatch.setitem(sys.modules, "cflib.crazyflie.syncCrazyflie", sync_module)

    try:
        capability_probe.probe_cflib_capabilities(robot_id="cf231", uri=CF231_URI, timeout_s=0.01)
    except capability_probe.CapabilityProbeError as exc:
        assert "Crazyradio busy" in str(exc)
    else:
        raise AssertionError("busy radio should raise CapabilityProbeError")


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


def test_pose_stability_uses_full_window_not_first_last() -> None:
    clock = FakeClock()
    collector = telemetry.TelemetryCollector("cf231", clock=clock)
    for x in [0.0, 0.10, 0.08, 0.01] * 15:
        collector.record_pose(FakePose(x, 0.0, 0.01))
        clock.advance(0.1)

    stability = collector.snapshot().pose_stability

    assert stability.observed is True
    assert stability.stable is False
    assert stability.xy_displacement_m <= 0.03
    assert stability.xy_span_m >= 0.10


def test_pose_stability_blocks_large_gap_short_duration_and_low_sample_count() -> None:
    clock = FakeClock()
    low_samples = telemetry.TelemetryCollector("cf231", clock=clock)
    for _ in range(5):
        low_samples.record_pose(FakePose(0.0, 0.0, 0.0))
        clock.advance(0.1)
    assert low_samples.snapshot().pose_stability.reason == "not_enough_pose_samples"

    short = telemetry.TelemetryCollector("cf231", clock=FakeClock())
    short_clock = short.clock
    assert isinstance(short_clock, FakeClock)
    for _ in range(20):
        short.record_pose(FakePose(0.0, 0.0, 0.0))
        short_clock.advance(0.1)
    assert short.snapshot().pose_stability.reason == "observation_too_short"

    gap_clock = FakeClock()
    gap = telemetry.TelemetryCollector("cf231", clock=gap_clock)
    for index in range(60):
        gap.record_pose(FakePose(0.0, 0.0, 0.0))
        gap_clock.advance(1.0 if index == 30 else 0.1)
    gap_stability = gap.snapshot().pose_stability
    assert gap_stability.observed is True
    assert gap_stability.stable is False
    assert gap_stability.max_sample_gap_s > 0.5


def test_readiness_blocks_z_velocity_range_estimator_and_supervisor() -> None:
    clock = FakeClock()
    collector = telemetry.TelemetryCollector("cf231", clock=clock)
    collector.set_positioning(
        telemetry.positioning_from_deck_params(
            {
                "deck.bcFlow2": 1,
                "deck.bcZRanger2": 0,
                "deck.bcLighthouse4": 0,
                "deck.bcLoco": 0,
                "deck.bcDWM1000": 0,
            }
        )
    )
    collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0))
    for _ in range(60):
        collector.record_pose(FakePose(0.0, 0.0, 0.03))
        clock.advance(0.1)

    codes = {issue.code for issue in collector.snapshot().flight_readiness.issues}

    assert "z_positioning_missing" in codes
    assert "supervisor_not_flyable" in codes
    assert "velocity_unavailable" in codes
    assert "range_unavailable" in codes
    assert "estimator_not_converged" in codes


def test_readiness_passes_static_policy_when_live_signals_are_healthy() -> None:
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
    collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
    for _ in range(60):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
        collector.record_pose(FakePose(0.0, 0.0, 0.03))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values(FakeLogDataGeneric((HEALTHY_ZRANGE_MM, 0.0001, 0.0001, 0.0001)).values)
        clock.advance(0.1)

    snapshot = collector.snapshot()
    readiness = snapshot.flight_readiness

    assert readiness.passed is True
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True
    assert readiness.issues == ()
    assert readiness.range is not None
    assert readiness.range.zrange_m == HEALTHY_ZRANGE_M
    assert snapshot.to_telemetry_payload()["telemetry_sources"]["readiness_log"] == "/cf231/dmp_readiness"


AUTO_ARM_STANDBY_RAW = 526  # 0x20E = armed | auto_arm | can_fly | high_level_trajectory_finished


def test_decode_supervisor_info_auto_arm_standby_raw_526() -> None:
    """Real cf231 auto-arm firmware parks reporting 526 while waiting for takeoff."""
    supervisor = telemetry.decode_supervisor_info(AUTO_ARM_STANDBY_RAW)

    assert supervisor.raw == AUTO_ARM_STANDBY_RAW
    assert supervisor.armed is True
    assert supervisor.auto_arm is True
    assert supervisor.can_be_armed is False
    assert supervisor.can_fly is True
    assert supervisor.flying is False
    assert supervisor.locked is False
    assert supervisor.tumbled is False
    assert supervisor.crashed is False
    assert supervisor.deck_fault is False

    assert supervisor.arming_ready is True
    assert supervisor.unexpectedly_armed is False
    assert supervisor.safe_for_preflight is True


def test_auto_arm_standby_raises_no_supervisor_issues() -> None:
    supervisor = telemetry.decode_supervisor_info(AUTO_ARM_STANDBY_RAW)

    assert telemetry._supervisor_issues(supervisor) == []


def test_auto_arm_standby_readiness_has_no_supervisor_false_positives() -> None:
    """Regression: raw 526 must not produce supervisor_not_flyable / supervisor_armed."""
    clock = FakeClock()
    collector = _ready_collector(clock)
    for _ in range(60):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=AUTO_ARM_STANDBY_RAW))
        collector.record_pose(FakePose(0.0, 0.0, 0.03))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values((HEALTHY_ZRANGE_MM, 0.0001, 0.0001, 0.0001))
        clock.advance(0.1)

    snapshot = collector.snapshot()
    readiness = snapshot.flight_readiness
    codes = {issue.code for issue in readiness.issues}

    assert "supervisor_not_flyable" not in codes
    assert "supervisor_armed" not in codes
    assert readiness.passed is True
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True
    assert readiness.issues == ()

    payload = snapshot.to_telemetry_payload()
    assert payload["supervisor_info_raw"] == AUTO_ARM_STANDBY_RAW
    assert payload["supervisor"]["safe_for_preflight"] is True
    assert payload["readiness"]["supervisor"]["safe_for_preflight"] is True


def test_explicit_arm_mode_keeps_strict_supervisor_semantics() -> None:
    """Non-auto-arm builds: can_be_armed + armed + can_fly stays an anomaly (0x000B)."""
    supervisor = telemetry.decode_supervisor_info(0x000B)

    assert supervisor.can_be_armed is True
    assert supervisor.can_fly is True
    assert supervisor.auto_arm is False
    assert supervisor.armed is True

    assert supervisor.arming_ready is True
    assert supervisor.unexpectedly_armed is True
    assert supervisor.safe_for_preflight is False

    codes = {issue.code for issue in telemetry._supervisor_issues(supervisor)}
    assert "supervisor_armed" in codes
    assert "supervisor_not_flyable" not in codes


def test_auto_arm_armed_but_not_flyable_still_blocks() -> None:
    """0x0006 = armed | auto_arm without can_fly: arming state is fine, flight is not."""
    supervisor = telemetry.decode_supervisor_info(0x0006)

    assert supervisor.auto_arm is True
    assert supervisor.armed is True
    assert supervisor.can_fly is False

    assert supervisor.arming_ready is True
    assert supervisor.safe_for_preflight is False
    assert "supervisor_not_flyable" in {issue.code for issue in telemetry._supervisor_issues(supervisor)}


def test_auto_arm_without_arm_bit_is_not_a_ready_standby() -> None:
    """0x000C = auto_arm | can_fly but not yet armed: still conservative."""
    supervisor = telemetry.decode_supervisor_info(0x000C)

    assert supervisor.auto_arm is True
    assert supervisor.armed is False
    assert supervisor.can_fly is True

    assert supervisor.arming_ready is False
    assert supervisor.safe_for_preflight is False
    assert "supervisor_not_flyable" in {issue.code for issue in telemetry._supervisor_issues(supervisor)}


@pytest.mark.parametrize(
    ("bit", "expected_code"),
    (
        (0x0010, "supervisor_flying"),
        (0x0020, "supervisor_tumbled"),
        (0x0040, "supervisor_locked"),
        (0x0080, "supervisor_crashed"),
        (0x0100, "supervisor_high_level_active"),
        (0x0800, "deck_fault"),
    ),
)
def test_auto_arm_standby_does_not_mask_hard_supervisor_faults(bit: int, expected_code: str) -> None:
    supervisor = telemetry.decode_supervisor_info(AUTO_ARM_STANDBY_RAW | bit)

    assert supervisor.safe_for_preflight is False
    assert expected_code in {issue.code for issue in telemetry._supervisor_issues(supervisor)}


# --- Ground / airborne readiness split ------------------------------------------------
#
# Ground launch safety may only rely on evidence that is independent of altitude and
# optical-flow quality. pose_stability, velocity settle and range plausibility come from
# the PMW3901 (specified for 80 mm to infinity) and the VL53L1x (unreliable below ~4 cm),
# so they are airborne-only and must not gate a ground start.


def _readiness_with_issues(*issues: Any) -> Any:
    return telemetry.FlightReadiness(
        passed=not any(issue.blocking for issue in issues),
        issues=tuple(issues),
        supervisor=None,
        velocity=None,
        range=None,
        estimator_variance=None,
    )


def _collector_readiness(
    *,
    zrange_mm: float | None = HEALTHY_ZRANGE_MM,
    pose_x: float = 0.0,
    vx: float = 0.0,
    supervisor_info: int = 0x0009,
    var: float = 0.0001,
    samples: int = 60,
    non_finite_range: bool = False,
) -> Any:
    clock = FakeClock()
    collector = _ready_collector(clock)
    for index in range(samples):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=supervisor_info))
        collector.record_pose(FakePose(pose_x * index, 0.0, 0.03))
        collector.record_odom(FakeOdom(vx, 0.0, 0.0))
        if non_finite_range:
            collector.record_range_m(float("nan"))
        elif zrange_mm is not None:
            collector.record_readiness_log_values((zrange_mm, var, var, var))
        clock.advance(0.1)
    return collector.snapshot().flight_readiness


def test_ground_gate_ignores_flow_and_nearfield_dependent_issues() -> None:
    """The exact cf231 ground symptom: drift + velocity noise + a near-field range read."""
    readiness = _collector_readiness(zrange_mm=8.0, pose_x=0.002, vx=0.09)

    codes = {issue.code for issue in readiness.issues}
    airborne_codes = {"pose_stability_failed", "velocity_not_settled", "range_invalid"}
    assert airborne_codes <= codes

    # They stay blocking issues in their own right...
    assert all(issue.blocking for issue in readiness.issues if issue.code in airborne_codes)

    # ...but they must not gate a ground start.
    assert {issue.code for issue in readiness.ground_issues if issue.blocking} == set()
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is False
    assert airborne_codes <= {issue.code for issue in readiness.airborne_issues if issue.blocking}


def test_nearfield_ground_range_is_a_non_blocking_ground_advisory() -> None:
    readiness = _collector_readiness(zrange_mm=8.0)

    advisories = [issue for issue in readiness.issues if issue.code == "range_ground_nearfield_unreliable"]

    assert len(advisories) == 1
    assert advisories[0].blocking is False
    assert advisories[0].stage is telemetry.ReadinessStage.GROUND
    assert readiness.ground_passed is True


def test_range_above_reliable_band_blocks_airborne_without_ground_advisory() -> None:
    readiness = _collector_readiness(zrange_mm=600.0)

    range_codes = {issue.code for issue in readiness.issues if issue.code.startswith("range")}

    assert range_codes == {"range_invalid"}
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is False


def test_non_finite_range_fails_closed_on_both_gates() -> None:
    readiness = _collector_readiness(non_finite_range=True)

    range_unavailable = [issue for issue in readiness.issues if issue.code == "range_unavailable"]

    assert len(range_unavailable) == 1
    assert range_unavailable[0].blocking is True
    assert range_unavailable[0].stage is telemetry.ReadinessStage.BOTH
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False


def test_range_inside_reliable_band_raises_no_range_issue() -> None:
    readiness = _collector_readiness(zrange_mm=HEALTHY_ZRANGE_MM)

    assert [issue.code for issue in readiness.issues if issue.code.startswith("range")] == []
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True


@pytest.mark.parametrize(
    "code",
    [
        "xy_positioning_missing",
        "z_positioning_missing",
        "status_missing",
        "status_stale",
        "pose_missing",
        "pose_stale",
        "pose_rate_too_low",
        "battery_critical",
        "supervisor_missing",
        "supervisor_not_flyable",
        "supervisor_armed",
        "supervisor_flying",
        "supervisor_locked",
        "supervisor_tumbled",
        "supervisor_crashed",
        "supervisor_high_level_active",
        "deck_fault",
        "velocity_unavailable",
        "velocity_stale",
        "range_unavailable",
        "range_stale",
        "estimator_not_converged",
    ],
)
def test_ground_gate_still_blocks_on_altitude_independent_issues(code: str) -> None:
    readiness = _readiness_with_issues(telemetry.ReadinessIssue(code, "synthetic"))

    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False


@pytest.mark.parametrize(
    "code",
    [
        "pose_stability_observation_missing",
        "pose_stability_failed",
        "velocity_not_settled",
        "range_invalid",
    ],
)
def test_airborne_only_issues_never_block_the_ground_gate(code: str) -> None:
    issue = telemetry.ReadinessIssue(code, "synthetic", stage=telemetry.ReadinessStage.AIRBORNE)
    readiness = _readiness_with_issues(issue)

    assert readiness.ground_passed is True
    assert readiness.airborne_passed is False


# --- Step 2.5 supervisor stage audit ---------------------------------------------------
#
# `supervisor_not_flyable`, `supervisor_armed`, `supervisor_flying` and
# `supervisor_high_level_active` describe the supervisor's *handover* state, i.e. whether a
# supervised flight may start from here. A vehicle that is legitimately under way reports
# armed=True / flying=True / can_be_armed=False (manual explicit-arm builds) and often
# active high-level control, so re-using those checks airborne would fault every flight.
# `can_fly` is the handover-independent "the supervisor refuses to fly" bit and therefore
# gets its own AIRBORNE code, `supervisor_cannot_fly`.


def test_declared_supervisor_stage_maps_are_disjoint_and_complete() -> None:
    parts = (
        telemetry.GROUND_ONLY_SUPERVISOR_ISSUE_CODES,
        telemetry.AIRBORNE_ONLY_SUPERVISOR_ISSUE_CODES,
        telemetry.BOTH_STAGE_SUPERVISOR_ISSUE_CODES,
    )
    union: set[str] = set().union(*parts)

    assert sum(len(part) for part in parts) == len(union)
    assert set(telemetry.SUPERVISOR_ISSUE_STAGE) == union
    assert set(telemetry.SUPERVISOR_ISSUE_STAGE.values()) == {
        telemetry.ReadinessStage.GROUND,
        telemetry.ReadinessStage.AIRBORNE,
        telemetry.ReadinessStage.BOTH,
    }


def test_every_supervisor_bit_combination_emits_the_declared_stage() -> None:
    """Exhaustive 2**12 sweep: a newly decoded supervisor bit cannot inherit a wrong stage."""
    observed: dict[str, set[Any]] = {}
    for raw in range(1 << 12):
        for issue in telemetry._supervisor_issues(telemetry.decode_supervisor_info(raw)):
            observed.setdefault(issue.code, set()).add(issue.stage)

    assert observed, "the supervisor sweep produced no issues at all"
    for code, stages in observed.items():
        assert stages == {telemetry.SUPERVISOR_ISSUE_STAGE[code]}, (code, stages)

    emitted = (
        telemetry.GROUND_ONLY_SUPERVISOR_ISSUE_CODES
        | telemetry.AIRBORNE_ONLY_SUPERVISOR_ISSUE_CODES
        | (telemetry.BOTH_STAGE_SUPERVISOR_ISSUE_CODES - {"supervisor_missing"})
    )
    # `supervisor_missing` is emitted by _flight_readiness (status is None), not by
    # _supervisor_issues, so it is the one declared code this sweep cannot observe.
    assert set(observed) == emitted


def test_missing_supervisor_telemetry_is_declared_both_stage_and_blocks_both_gates() -> None:
    assert telemetry.SUPERVISOR_ISSUE_STAGE["supervisor_missing"] is telemetry.ReadinessStage.BOTH

    clock = FakeClock()
    collector = _ready_collector(clock)
    for _ in range(60):
        collector.record_pose(FakePose(0.0, 0.0, HEALTHY_ZRANGE_M))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values((HEALTHY_ZRANGE_MM, 0.0001, 0.0001, 0.0001))
        clock.advance(0.1)

    readiness = collector.snapshot().flight_readiness
    missing = [issue for issue in readiness.issues if issue.code == "supervisor_missing"]

    assert len(missing) == 1
    assert missing[0].stage is telemetry.ReadinessStage.BOTH
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False


def test_auto_arm_standby_is_clean_on_both_gates() -> None:
    issues = telemetry._supervisor_issues(telemetry.decode_supervisor_info(AUTO_ARM_STANDBY_RAW))
    readiness = _readiness_with_issues(*issues)

    assert issues == []
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True


@pytest.mark.parametrize(
    ("raw", "expected_codes"),
    [
        # Auto-arm firmware in flight: armed | auto_arm | can_fly | flying.
        (0x001E, {"supervisor_flying"}),
        # Manual explicit-arm in flight: can_be_armed=False, auto_arm=False.
        (
            0x001A,
            {"supervisor_not_flyable", "supervisor_armed", "supervisor_flying"},
        ),
    ],
)
def test_ground_only_supervisor_codes_never_block_the_airborne_gate(
    raw: int,
    expected_codes: set[str],
) -> None:
    """An airborne vehicle must not be faulted for its ground arming/handover state."""
    issues = telemetry._supervisor_issues(telemetry.decode_supervisor_info(raw))

    assert {issue.code for issue in issues} == expected_codes
    assert all(issue.stage is telemetry.ReadinessStage.GROUND for issue in issues)

    readiness = _readiness_with_issues(*issues)
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is True
    assert [issue.code for issue in readiness.airborne_issues if issue.blocking] == []


def test_supervisor_cannot_fly_is_airborne_only_and_never_reuses_not_flyable() -> None:
    """`can_fly=False` needs an airborne code of its own; not_flyable stays ground-only."""
    raw = AUTO_ARM_STANDBY_RAW & ~0x0008  # can_fly cleared, arming standby otherwise intact
    issues = telemetry._supervisor_issues(telemetry.decode_supervisor_info(raw))
    staged = {issue.code: issue.stage for issue in issues}

    assert staged["supervisor_not_flyable"] is telemetry.ReadinessStage.GROUND
    assert staged["supervisor_cannot_fly"] is telemetry.ReadinessStage.AIRBORNE

    readiness = _readiness_with_issues(*issues)
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False
    assert [issue.code for issue in readiness.airborne_issues if issue.blocking] == [
        "supervisor_cannot_fly"
    ]


@pytest.mark.parametrize("bit", [0x0020, 0x0040, 0x0080, 0x0800])
def test_terminal_supervisor_faults_block_both_gates(bit: int) -> None:
    """tumbled / locked / crashed / deck_fault are never survivable in either stage."""
    issues = telemetry._supervisor_issues(
        telemetry.decode_supervisor_info(AUTO_ARM_STANDBY_RAW | bit)
    )
    staged = {issue.code: issue.stage for issue in issues}
    terminal = set(staged) - {"supervisor_flying"}

    assert terminal, "no terminal code was emitted"
    assert all(staged[code] is telemetry.ReadinessStage.BOTH for code in terminal)

    readiness = _readiness_with_issues(*issues)
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False


def test_unclassified_issue_defaults_to_both_stages_fail_closed() -> None:
    issue = telemetry.ReadinessIssue("brand_new_unclassified_code", "synthetic")

    assert issue.stage is telemetry.ReadinessStage.BOTH
    readiness = _readiness_with_issues(issue)
    assert readiness.ground_passed is False
    assert readiness.airborne_passed is False


def test_non_blocking_issue_never_fails_either_gate() -> None:
    issue = telemetry.ReadinessIssue("advisory_only", "synthetic", blocking=False)
    readiness = _readiness_with_issues(issue)

    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True
    assert readiness.passed is True


def test_readiness_payload_exposes_stage_and_partitioned_verdicts() -> None:
    readiness = _collector_readiness(zrange_mm=8.0, pose_x=0.002, vx=0.09)

    payload = readiness.to_payload()

    assert payload["ground_passed"] is True
    assert payload["airborne_passed"] is False
    assert payload["passed"] is False
    by_code = {issue["code"]: issue for issue in payload["issues"]}
    assert by_code["pose_stability_failed"]["stage"] == "airborne"
    assert by_code["range_invalid"]["stage"] == "airborne"
    assert by_code["range_ground_nearfield_unreliable"]["stage"] == "ground"
    assert by_code["range_ground_nearfield_unreliable"]["blocking"] is False


def test_dmp_readiness_log_contract_order_and_unit_conversion() -> None:
    assert telemetry.DMP_READINESS_LOG_TOPIC == "dmp_readiness"
    # Step 3.6: Kalman VELOCITY covariance (cflib wait_for_position_estimator source).
    assert telemetry.DMP_READINESS_LOG_VARIABLES == (
        "range.zrange",
        "kalman.varPX",
        "kalman.varPY",
        "kalman.varPZ",
    )
    assert telemetry.DMP_ESTIMATOR_LOG_VARIABLES == (
        "kalman.varPX",
        "kalman.varPY",
        "kalman.varPZ",
    )
    assert telemetry.DMP_ESTIMATOR_COVARIANCE_KIND == "velocity_covariance"
    assert telemetry.DMP_ESTIMATOR_COVARIANCE_SOURCE == "kalman.varPX/PY/PZ"
    # The block is still 14 bytes: uint16 range + three float32 covariances.
    assert telemetry.DMP_READINESS_LOG_BLOCK_BYTES == 14
    assert telemetry.DMP_READINESS_LOG_BLOCK_BYTES <= telemetry.CRAZYFLIE_LOG_MAX_BYTES
    assert telemetry.DMP_READINESS_RANGE_MM_INDEX == 0
    assert telemetry.DMP_READINESS_VAR_PX_INDEX == 1
    assert telemetry.DMP_READINESS_VAR_PY_INDEX == 2
    assert telemetry.DMP_READINESS_VAR_PZ_INDEX == 3

    sample = telemetry.parse_dmp_readiness_log_values((300.0, 0.001, 0.002, 0.003))

    assert sample.zrange_m == 0.3
    assert sample.var_px == 0.001
    assert sample.var_py == 0.002
    assert sample.var_pz == 0.003


def test_estimator_covariance_payload_is_explicitly_velocity_not_position() -> None:
    """Step 3.6: the wire payload must not reuse the old position-covariance names."""
    variance = telemetry.EstimatorVarianceTelemetry(
        monotonic_s=0.0, var_px=0.0001, var_py=0.0002, var_pz=0.0003
    )

    payload = telemetry.estimator_covariance_payload(variance)

    assert payload["kind"] == "velocity_covariance"
    assert payload["source"] == "kalman.varPX/PY/PZ"
    assert payload["variables"] == ["kalman.varPX", "kalman.varPY", "kalman.varPZ"]
    assert payload["var_px"] == pytest.approx(0.0001)
    assert payload["var_py"] == pytest.approx(0.0002)
    assert payload["var_pz"] == pytest.approx(0.0003)
    # The legacy position-covariance names must be gone: keeping them would silently
    # change their meaning for any consumer.
    assert "var_x" not in payload
    assert "var_y" not in payload
    assert "var_z" not in payload
    assert variance.peak == pytest.approx(0.0003)


def test_telemetry_payload_marks_estimator_covariance_kind_and_source() -> None:
    clock = FakeClock()
    collector = _ready_collector(clock)
    collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
    collector.record_readiness_log_values((HEALTHY_ZRANGE_MM, 0.0001, 0.0002, 0.0003))

    payload = collector.snapshot().to_telemetry_payload()

    assert payload["estimator_variance"]["kind"] == "velocity_covariance"
    assert payload["estimator_variance"]["source"] == "kalman.varPX/PY/PZ"
    assert payload["estimator_variance"]["var_px"] == pytest.approx(0.0001)
    assert "var_x" not in payload["estimator_variance"]


# --- Step 3.6 estimator convergence (kalman.varPX/varPY/varPZ) -------------------------
#
# The convergence check reads the Kalman VELOCITY covariance, i.e. the same source the
# official cflib ``wait_for_position_estimator()`` gates on. On real hardware the position
# covariance (varX/varY) saturates at the firmware ceiling with a zero span, so a
# span-based test on those variables degenerated into "always converged".


def _variance_samples(
    *,
    var_px: float,
    var_py: float | None = None,
    var_pz: float | None = None,
    samples: int = 60,
    step_s: float = 0.1,
) -> tuple[Any, ...]:
    py = var_px if var_py is None else var_py
    pz = var_px if var_pz is None else var_pz
    return tuple(
        telemetry.EstimatorVarianceTelemetry(
            monotonic_s=index * step_s, var_px=var_px, var_py=py, var_pz=pz
        )
        for index in range(samples)
    )


def _estimator_codes(samples: tuple[Any, ...]) -> list[str]:
    return [
        issue.code
        for issue in telemetry._estimator_variance_issues(samples, telemetry.ReadinessPolicy())
    ]


def test_small_stable_velocity_covariance_is_converged() -> None:
    """Healthy cf231 data: varP ~1.2e-4 .. 2.4e-4 with a tiny span -> PASS."""
    samples = tuple(
        telemetry.EstimatorVarianceTelemetry(
            monotonic_s=index * 0.1,
            var_px=0.00012 + index * 0.000002,
            var_py=0.00013 + index * 0.000002,
            var_pz=0.00017 + index * 0.000001,
        )
        for index in range(60)
    )

    assert _estimator_codes(samples) == []


def test_velocity_covariance_span_above_policy_is_not_converged() -> None:
    samples = tuple(
        telemetry.EstimatorVarianceTelemetry(
            monotonic_s=index * 0.1,
            var_px=0.0001 + (0.003 if index % 2 else 0.0),
            var_py=0.0001,
            var_pz=0.0001,
        )
        for index in range(60)
    )

    assert _estimator_codes(samples) == ["estimator_not_converged"]


def test_saturated_velocity_covariance_with_zero_span_fails_closed() -> None:
    """Regression: stable saturation must never read as convergence.

    This is the failure mode that made the old varX/varY check useless: the value sits at
    the firmware ceiling with span exactly 0.
    """
    policy = telemetry.ReadinessPolicy()
    saturated = policy.estimator_max_covariance  # 100.0, the firmware ceiling

    codes = _estimator_codes(_variance_samples(var_px=saturated))
    assert codes == ["estimator_not_converged"]

    # ...including exactly at the saturation guard and just above it.
    guard = saturated * policy.estimator_saturation_fraction
    assert _estimator_codes(_variance_samples(var_px=guard)) == ["estimator_not_converged"]
    assert _estimator_codes(_variance_samples(var_px=guard - 1e-6)) == []

    issues = telemetry._estimator_variance_issues(
        _variance_samples(var_px=saturated), policy
    )
    assert "saturated, not converged" in issues[0].message


def test_stale_nan_and_infinite_velocity_covariance_fail_closed() -> None:
    clock = FakeClock()
    collector = _ready_collector(clock)
    for _ in range(60):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
        collector.record_pose(FakePose(0.0, 0.0, HEALTHY_ZRANGE_M))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values((HEALTHY_ZRANGE_MM, 0.0001, 0.0001, 0.0001))
        clock.advance(0.1)
    assert "estimator_not_converged" not in {
        issue.code for issue in collector.snapshot().flight_readiness.issues
    }

    # Stale: the estimator sample is older than estimator_stale_timeout_s (1.5 s).
    clock.advance(2.0)
    stale_codes = {issue.code for issue in collector.snapshot().flight_readiness.issues}
    assert "estimator_not_converged" in stale_codes

    # NaN / infinite samples are dropped by the parser, so the stream goes stale/absent.
    for bad in (
        (float("nan"), 0.0001, 0.0001, 0.0001),
        (HEALTHY_ZRANGE_MM, float("nan"), 0.0001, 0.0001),
        (HEALTHY_ZRANGE_MM, 0.0001, float("inf"), 0.0001),
        (HEALTHY_ZRANGE_MM, 0.0001, 0.0001, float("-inf")),
    ):
        with pytest.raises(ValueError, match="NaN or infinite"):
            telemetry.parse_dmp_readiness_log_values(bad)

    # A non-finite value that somehow reaches the policy layer is also refused.
    poisoned = (
        telemetry.EstimatorVarianceTelemetry(
            monotonic_s=0.0, var_px=float("nan"), var_py=0.0001, var_pz=0.0001
        ),
        *_variance_samples(var_px=0.0001, samples=59),
    )
    assert _estimator_codes(poisoned) == ["estimator_not_converged"]


def test_healthy_velocity_covariance_lets_the_airborne_estimator_gate_pass() -> None:
    """varP healthy + pose/velocity/range healthy -> no airborne estimator blocker."""
    clock = FakeClock()
    collector = _ready_collector(clock)
    for index in range(60):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
        collector.record_pose(FakePose(0.0, 0.0, HEALTHY_ZRANGE_M))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values(
            (HEALTHY_ZRANGE_MM, 0.00012 + index * 1e-6, 0.00013, 0.00017)
        )
        clock.advance(0.1)

    readiness = collector.snapshot().flight_readiness

    assert {issue.code for issue in readiness.issues} == set()
    assert readiness.ground_passed is True
    assert readiness.airborne_passed is True


def test_dmp_readiness_log_rejects_bad_samples_without_replacing_last_good_value() -> None:
    adapter = _adapter_with_client(FakeParameterClient(values={}))
    adapter._record_readiness_log(FakeLogDataGeneric((80.0, 0.0001, 0.0001, 0.0001)))

    good_snapshot = adapter._collector.snapshot()
    assert good_snapshot.range is not None
    assert good_snapshot.range.zrange_m == 0.08

    for values in (
        (),
        (80.0, 0.0001, 0.0001),
        (80.0, 0.0001, 0.0001, 0.0001, 1.0),
        (float("nan"), 0.0001, 0.0001, 0.0001),
        (80.0, float("inf"), 0.0001, 0.0001),
        (-1.0, 0.0001, 0.0001, 0.0001),
        (80.0, -0.0001, 0.0001, 0.0001),
    ):
        adapter._record_readiness_log(FakeLogDataGeneric(values))

    snapshot = adapter._collector.snapshot()
    assert snapshot.range is not None
    assert snapshot.range.zrange_m == 0.08
    assert any("invalid /cf231/dmp_readiness sample ignored" in item for item in snapshot.diagnostics)


def test_odom_velocity_staleness_blocks_readiness() -> None:
    clock = FakeClock()
    collector = _ready_collector(clock)
    for _ in range(60):
        collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
        collector.record_pose(FakePose(0.0, 0.0, 0.03))
        collector.record_odom(FakeOdom(0.0, 0.0, 0.0))
        collector.record_readiness_log_values((80.0, 0.0001, 0.0001, 0.0001))
        clock.advance(0.1)

    clock.advance(1.0)
    codes = {issue.code for issue in collector.snapshot().flight_readiness.issues}

    assert "velocity_stale" in codes


def test_hardware_session_tracks_status_stale_to_fresh_reconnect() -> None:
    clock = FakeClock()
    adapter = _adapter_with_client(FakeParameterClient(values={}), session=False, clock=clock)

    assert adapter.capabilities().connected is False
    assert adapter.session_id is None

    adapter._collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))
    first = adapter.capabilities()
    first_session = adapter.session_id
    assert first.connected is True
    assert isinstance(first_session, str)

    clock.advance(3.0)
    stale = adapter.capabilities()
    assert stale.connected is False
    assert adapter.session_id is None

    adapter._collector.record_status(FakeStatus(3.96, 0, 37, 6, 10, 10, supervisor_info=0x0009))
    fresh = adapter.capabilities()
    assert fresh.connected is True
    assert isinstance(adapter.session_id, str)
    assert adapter.session_id != first_session


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


def test_frame_origin_rejects_stale_future_malformed_wrong_session_and_optional_binding() -> None:
    now = datetime(2026, 9, 19, 12, 0, tzinfo=UTC)
    policy = protocol_server.FrameOriginPolicy(max_age_s=300.0, allowed_clock_skew_s=5.0)
    base = _hardware_mission(
        {
            "mode": "relative_current_pose",
            "origin_source": "current_pose",
            "origin_captured_at_utc": "2026-09-19T11:59:00Z",
            "origin_session_id": "session-1",
            "origin_mission_id": "mission-1",
            "origin_route_hash": "route-hash",
        }
    )
    issues: list[dict[str, Any]] = []
    protocol_server._append_frame_origin_issues(
        base,
        _flow_robot(),
        issues,
        current_session_id="session-1",
        policy=policy,
        now_utc=now,
    )
    assert issues == []

    cases = {
        "frame_origin_stale": {"origin_captured_at_utc": "2026-09-19T11:00:00Z"},
        "frame_origin_future": {"origin_captured_at_utc": "2026-09-19T12:01:00Z"},
        "frame_origin_invalid": {"origin_captured_at_utc": "not-a-date"},
        "frame_session_mismatch": {"origin_session_id": "old-session"},
        "frame_mission_mismatch": {"origin_mission_id": "mission-2"},
        "frame_route_mismatch": {"origin_route_hash": "other-hash"},
    }
    for expected_code, update in cases.items():
        mission = _hardware_mission({**base["frame"], **update})
        issues = []
        protocol_server._append_frame_origin_issues(
            mission,
            _flow_robot(),
            issues,
            current_session_id="session-1",
            policy=policy,
            now_utc=now,
        )
        assert expected_code in {issue["code"] for issue in issues}


def _capture_hardware_server_kwargs(monkeypatch: Any, config_path: Path) -> dict[str, Any]:
    """Build a hardware-backend server with a stub adapter and return the adapter kwargs."""
    captured: dict[str, Any] = {}

    class _StubAdapter:
        def __init__(self, **kwargs: Any) -> None:
            captured.update(kwargs)

        def connect(self) -> None:
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
            )

    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", _StubAdapter)
    protocol_server.BridgeProtocolServer(
        backend="hardware", robot_id="cf231", config_path=str(config_path)
    )
    return captured


def _capture_readiness_policy(monkeypatch: Any, config_text: str, tmp_path: Path) -> Any:
    """Build a hardware-backend server with a stub adapter and return its readiness policy."""
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text(config_text, encoding="utf-8")
    return _capture_hardware_server_kwargs(monkeypatch, config_path)["readiness_policy"]


def test_deprecated_min_startup_zrange_is_warned_and_ignored(monkeypatch: Any, tmp_path: Path) -> None:
    """D1: the retired local 0.005 must never resurface as the airborne reliable floor."""
    with pytest.warns(DeprecationWarning, match="min_startup_zrange_m is deprecated"):
        policy = _capture_readiness_policy(monkeypatch, "min_startup_zrange_m: 0.005\n", tmp_path)

    assert policy.min_reliable_zrange_m == pytest.approx(0.10)
    assert policy.range_ground_advisory_m == pytest.approx(0.04)
    assert policy.max_reliable_zrange_m == pytest.approx(0.50)


def test_range_band_defaults_when_only_legacy_keys_present(monkeypatch: Any, tmp_path: Path) -> None:
    with pytest.warns(DeprecationWarning):
        policy = _capture_readiness_policy(
            monkeypatch,
            "min_startup_zrange_m: 0.02\nmax_startup_zrange_m: 0.40\n",
            tmp_path,
        )

    assert policy.min_reliable_zrange_m == pytest.approx(0.10)
    assert policy.range_ground_advisory_m == pytest.approx(0.04)
    # The max key was a pure rename, so its legacy value is still honoured.
    assert policy.max_reliable_zrange_m == pytest.approx(0.40)


def test_deprecated_max_startup_zrange_is_warned_but_still_honoured(
    monkeypatch: Any, tmp_path: Path
) -> None:
    with pytest.warns(DeprecationWarning, match="max_startup_zrange_m is deprecated"):
        policy = _capture_readiness_policy(monkeypatch, "max_startup_zrange_m: 0.44\n", tmp_path)

    assert policy.max_reliable_zrange_m == pytest.approx(0.44)


def test_new_max_key_wins_when_both_band_ceiling_keys_are_present(
    monkeypatch: Any, tmp_path: Path
) -> None:
    """Requirement: no ambiguity - the new key wins and the shadowing is announced."""
    config_text = "max_startup_zrange_m: 0.20\nmax_reliable_zrange_m: 0.45\n"
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        policy = _capture_readiness_policy(monkeypatch, config_text, tmp_path)

    deprecations = [str(item.message) for item in caught if issubclass(item.category, DeprecationWarning)]
    assert any("max_startup_zrange_m is deprecated" in message for message in deprecations)
    assert any("max_reliable_zrange_m takes precedence" in message for message in deprecations)
    assert policy.max_reliable_zrange_m == pytest.approx(0.45)


def test_new_range_band_keys_are_read_from_config(monkeypatch: Any, tmp_path: Path) -> None:
    policy = _capture_readiness_policy(
        monkeypatch,
        "range_ground_advisory_m: 0.03\nmin_reliable_zrange_m: 0.12\nmax_reliable_zrange_m: 0.45\n",
        tmp_path,
    )

    assert policy.range_ground_advisory_m == pytest.approx(0.03)
    assert policy.min_reliable_zrange_m == pytest.approx(0.12)
    assert policy.max_reliable_zrange_m == pytest.approx(0.45)


def test_repo_bridge_config_is_migrated_and_keeps_hardware_flight_disabled(
    monkeypatch: Any,
) -> None:
    """The shipped config must not still rely on either retired key."""
    config_path = BRIDGE_SRC / "config" / "bridge.yaml"
    config_text = config_path.read_text(encoding="utf-8")

    assert "min_startup_zrange_m" not in config_text
    assert "max_startup_zrange_m" not in config_text
    assert "max_reliable_zrange_m: 0.50" in config_text
    assert "hardware_flight_enabled: false" in config_text

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        captured = _capture_hardware_server_kwargs(monkeypatch, config_path)

    assert [item for item in caught if issubclass(item.category, DeprecationWarning)] == []
    assert captured["hardware_flight_enabled"] is False
    policy = captured["readiness_policy"]
    assert policy.range_ground_advisory_m == pytest.approx(0.04)
    assert policy.min_reliable_zrange_m == pytest.approx(0.10)
    assert policy.max_reliable_zrange_m == pytest.approx(0.50)


@pytest.mark.parametrize(
    "config_text",
    [
        "range_ground_advisory_m: 0.10\nmin_reliable_zrange_m: 0.10\n",
        "range_ground_advisory_m: 0.20\nmin_reliable_zrange_m: 0.10\n",
        "min_reliable_zrange_m: 0.50\nmax_reliable_zrange_m: 0.50\n",
        "min_reliable_zrange_m: 0.60\nmax_reliable_zrange_m: 0.50\n",
        # Legacy ceiling key must not create an ambiguous, accepted configuration either.
        "min_reliable_zrange_m: 0.60\nmax_startup_zrange_m: 0.50\n",
    ],
)
def test_invalid_range_band_config_fails_closed(config_text: str, tmp_path: Path) -> None:
    config_path = tmp_path / "bad-band.yaml"
    config_path.write_text(config_text, encoding="utf-8")

    with pytest.raises(ValueError, match="must be greater than"):
        protocol_server.BridgeProtocolServer(backend="sim", config_path=str(config_path))


def test_invalid_explicit_safety_config_fails_closed(tmp_path: Path) -> None:
    invalid_values = (
        "minimum_acceptable_pose_rate_hz: not-a-number\n",
        "velocity_stale_timeout_s: -1\n",
        "minimum_estimator_sample_count: 0\n",
        "max_estimator_variance: inf\n",
        "min_startup_zrange_m: 0.50\nmax_startup_zrange_m: 0.02\n",
        "min_reliable_zrange_m: 0.50\nmax_startup_zrange_m: 0.02\n",
        "frame_origin_max_age_s: 0\n",
        "battery_critical_voltage: nan\n",
    )
    for index, config_text in enumerate(invalid_values):
        config_path = tmp_path / f"bad-{index}.yaml"
        config_path.write_text(config_text, encoding="utf-8")
        try:
            protocol_server.BridgeProtocolServer(backend="sim", config_path=str(config_path))
        except ValueError:
            continue
        raise AssertionError(f"invalid config was accepted: {config_text!r}")


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
    supervisor_info: int = 0


class FakePose:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.pose = _Pose(_Point(x, y, z), _Orientation())


class FakeOdom:
    def __init__(self, vx: float, vy: float, vz: float) -> None:
        self.twist = _TwistWithCovariance(_Twist(_Point(vx, vy, vz)))


@dataclass(frozen=True, slots=True)
class FakeLogDataGeneric:
    values: tuple[Any, ...]


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


@dataclass(frozen=True, slots=True)
class _Twist:
    linear: _Point


@dataclass(frozen=True, slots=True)
class _TwistWithCovariance:
    twist: _Twist


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


class FakeStagedReadinessAdapter(FakeHardwareAdapter):
    """Adapter exposing a staged readiness payload (the post-CF8.1 shape)."""

    def __init__(self, robot_id: str, issues: list[dict[str, Any]]) -> None:
        super().__init__(robot_id)
        self._issues = list(issues)

    @property
    def readiness_payload(self) -> dict[str, Any]:
        return {"passed": False, "issues": list(self._issues)}


def _preflight_with_readiness_issues(
    monkeypatch: Any,
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    fake = FakeStagedReadinessAdapter("cf231", issues)
    monkeypatch.setattr(protocol_server, "Crazyswarm2Adapter", lambda robot_id, **_: fake)
    server = protocol_server.BridgeProtocolServer(backend="hardware", robot_id="cf231")
    server.state.selected_robot_id = "cf231"
    return server.handle_message({"type": "run_preflight", "request_id": "pf-staged"})


def test_hardware_preflight_ground_gate_skips_airborne_stage_issues(monkeypatch: Any) -> None:
    """The real bridge gate must not demand flow/near-field evidence from the ground."""
    preflight = _preflight_with_readiness_issues(
        monkeypatch,
        [
            {"code": "pose_stability_failed", "message": "x", "blocking": True, "stage": "airborne"},
            {"code": "velocity_not_settled", "message": "x", "blocking": True, "stage": "airborne"},
            {"code": "range_invalid", "message": "x", "blocking": True, "stage": "airborne"},
            {"code": "range_ground_nearfield_unreliable", "message": "x", "blocking": False, "stage": "ground"},
        ],
    )

    codes = {issue["code"] for issue in preflight["issues"]}
    blocking_codes = {issue["code"] for issue in preflight["issues"] if issue["blocking"]}

    assert codes.isdisjoint({"pose_stability_failed", "velocity_not_settled", "range_invalid"})
    # The only remaining blocker is the missing mission, unrelated to readiness.
    assert blocking_codes <= {"mission_missing"}


def test_hardware_preflight_ground_gate_keeps_ground_issues_and_stage(monkeypatch: Any) -> None:
    preflight = _preflight_with_readiness_issues(
        monkeypatch,
        [
            {"code": "battery_critical", "message": "x", "blocking": True, "stage": "both"},
            {"code": "range_ground_nearfield_unreliable", "message": "x", "blocking": False, "stage": "ground"},
        ],
    )

    by_code = {issue["code"]: issue for issue in preflight["issues"]}

    assert by_code["battery_critical"]["blocking"] is True
    # stage must survive _append_unique_issue, which otherwise rebuilds the dict.
    assert by_code["battery_critical"]["stage"] == "both"
    assert by_code["range_ground_nearfield_unreliable"]["blocking"] is False
    assert by_code["range_ground_nearfield_unreliable"]["stage"] == "ground"
    assert preflight["passed"] is False


def test_hardware_preflight_fails_closed_for_unstaged_readiness_issues(monkeypatch: Any) -> None:
    """A payload without stage must stay blocking: no silent bypass for legacy shapes."""
    preflight = _preflight_with_readiness_issues(
        monkeypatch,
        [{"code": "pose_stability_failed", "message": "x", "blocking": True}],
    )

    by_code = {issue["code"]: issue for issue in preflight["issues"]}

    assert by_code["pose_stability_failed"]["blocking"] is True
    assert "stage" not in by_code["pose_stability_failed"]
    assert preflight["passed"] is False


CF231_URI = "radio://0/80/2M/E7E7E7E705"


def _adapter_with_client(
    client: FakeParameterClient | None,
    *,
    snapshot_path: Path | None = None,
    robot_uri: str | None = None,
    session: bool = True,
    clock: FakeClock | None = None,
) -> Any:
    """Build a hardware adapter wired to a fake parameter client.

    Capability is session-scoped, so by default this also publishes one fresh status to
    create a hardware telemetry session: without a session the adapter reports unknown
    on purpose (fail closed).
    """
    adapter = crazyswarm_adapter.Crazyswarm2Adapter(robot_id="cf231")
    adapter._parameter_client = client
    adapter._capability_read_interval_s = 0.0
    adapter.robot_uri = robot_uri
    adapter.capability_snapshot_path = snapshot_path
    if clock is not None:
        adapter._collector.clock = clock
    if session:
        _publish_fresh_status(adapter)
    return adapter


def _publish_fresh_status(adapter: Any) -> None:
    adapter._collector.record_status(FakeStatus(3.95, 0, 37, 6, 10, 10, supervisor_info=0x0009))


def _ready_collector(clock: FakeClock) -> Any:
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
    return collector


def _flow_robot() -> Any:
    return protocol_server.BridgeRobot(
        robot_id="cf231",
        connected=True,
        positioning_mode="flow",
        xy_positioning_available=True,
        z_positioning_available=True,
        pose_stream_available=True,
        pose_rate_hz=10.0,
    )


def _hardware_mission(frame: dict[str, Any]) -> dict[str, Any]:
    return {
        "mission_id": "mission-1",
        "source_route_hash": "route-hash",
        "frame": frame,
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


def _write_snapshot(
    path: Path,
    *,
    robot_id: str = "cf231",
    uri: str = CF231_URI,
    flow2: bool = True,
    zranger2: bool = True,
    captured_at_utc: str | None = None,
) -> None:
    captured = captured_at_utc
    if captured is None:
        captured = (datetime.now(UTC) - timedelta(seconds=1)).isoformat(timespec="seconds").replace("+00:00", "Z")
    snapshot = capability_probe.HardwareCapabilitySnapshot(
        robot_id=robot_id,
        uri=uri,
        flow2=flow2,
        zranger2=zranger2,
        lighthouse=False,
        loco=False,
        dwm1000=False,
        captured_at_utc=captured,
    )
    capability_probe.save_capability_snapshot(snapshot, path)
