"""Tests for the read-only Flow Deck drift probe.

The probe is a safety-relevant decision aid: its numbers are used to decide whether a
static Flow Deck platform is trustworthy before a first real flight. These tests pin the
probe's pose math to the bridge's own readiness policy so the two can never drift apart.
"""

from __future__ import annotations

import importlib
import importlib.util
import math
import sys
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
BRIDGE_SRC = REPO_ROOT / "integrations" / "crazyflie_ros2" / "src" / "dmp_crazyflie_bridge"
PROBE_PATH = REPO_ROOT / "integrations" / "crazyflie_ros2" / "scripts" / "flow_drift_probe.py"

sys.path.insert(0, str(BRIDGE_SRC))

telemetry = importlib.import_module("dmp_crazyflie_bridge.telemetry")


def _load_probe() -> Any:
    spec = importlib.util.spec_from_file_location("dmp_flow_drift_probe", PROBE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["dmp_flow_drift_probe"] = module
    spec.loader.exec_module(module)
    return module


probe = _load_probe()


def _pose_samples(times: list[float], xs: list[float], ys: list[float], zs: list[float]) -> list[Any]:
    return [
        probe.PoseSample(monotonic_s=t, x_m=x, y_m=y, z_m=z)
        for t, x, y, z in zip(times, xs, ys, zs, strict=True)
    ]


def _bridge_pose_samples(samples: list[Any]) -> tuple[Any, ...]:
    return tuple(
        telemetry.PoseTelemetry(
            monotonic_s=sample.monotonic_s,
            x_m=sample.x_m,
            y_m=sample.y_m,
            z_m=sample.z_m,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
        )
        for sample in samples
    )


def _store_with(poses: list[Any]) -> Any:
    store = probe.ProbeStore()
    store.poses.extend(poses)
    return store


def test_probe_pose_math_matches_bridge_policy_for_drifting_window() -> None:
    """A ~0.149 m span over 20 s window, matching the observed cf231 field symptom."""
    times = [index * 0.1 for index in range(200)]
    xs = [0.0007 * index for index in range(200)]
    ys = [0.0003 * index + 0.01 * math.sin(index / 5.0) for index in range(200)]
    zs = [0.03 + 0.00002 * (index % 7) for index in range(200)]

    samples = _pose_samples(times, xs, ys, zs)
    report = probe._pose_report(_store_with(samples))
    stability = telemetry._pose_stability(_bridge_pose_samples(samples), telemetry.PoseStabilityPolicy())

    assert report["sample_count"] == stability.sample_count
    assert report["duration_s"] == pytest.approx(stability.duration_s)
    assert report["xy_span_m"] == pytest.approx(stability.xy_span_m)
    assert report["xy_displacement_m"] == pytest.approx(stability.xy_displacement_m)
    assert report["xy_std_m"] == pytest.approx(stability.xy_std_m)
    assert report["z_span_m"] == pytest.approx(stability.z_span_m)
    assert report["z_std_m"] == pytest.approx(stability.z_std_m)
    assert report["max_sample_gap_s"] == pytest.approx(stability.max_sample_gap_s)
    assert report["stable"] == stability.stable
    assert report["stable"] is False
    assert report["reason"] == "pose_drift_or_gap_exceeds_limit"
    assert report["violations"]


def test_probe_pose_math_matches_bridge_policy_for_quiet_window() -> None:
    times = [index * 0.1 for index in range(100)]
    xs = [0.001 * math.sin(index / 7.0) for index in range(100)]
    ys = [0.001 * math.cos(index / 9.0) for index in range(100)]
    zs = [0.03 for _ in range(100)]

    samples = _pose_samples(times, xs, ys, zs)
    report = probe._pose_report(_store_with(samples))
    stability = telemetry._pose_stability(_bridge_pose_samples(samples), telemetry.PoseStabilityPolicy())

    assert report["xy_span_m"] == pytest.approx(stability.xy_span_m)
    assert report["z_span_m"] == pytest.approx(stability.z_span_m)
    assert report["stable"] == stability.stable
    assert report["stable"] is True
    assert report["reason"] is None
    assert report["violations"] == []


def test_probe_pose_report_short_window_matches_bridge_reason() -> None:
    times = [index * 0.1 for index in range(30)]
    samples = _pose_samples(times, [0.0] * 30, [0.0] * 30, [0.03] * 30)

    report = probe._pose_report(_store_with(samples))
    stability = telemetry._pose_stability(_bridge_pose_samples(samples), telemetry.PoseStabilityPolicy())

    assert report["stable"] is False
    assert report["reason"] == "observation_too_short"
    assert report["stable"] == stability.stable
    assert stability.reason == "observation_too_short"


def test_probe_pose_report_handles_no_samples() -> None:
    report = probe._pose_report(probe.ProbeStore())

    assert report["sample_count"] == 0
    assert report["reason"] == "no_pose_samples"


def test_probe_readiness_report_converts_millimetres_and_spans() -> None:
    store = probe.ProbeStore()
    for index in range(6):
        store.readiness.append(
            probe.LogSample(
                monotonic_s=index * 0.1, values=(8.0 + index, 1.2e-4, 1.3e-4, 1.7e-4)
            )
        )

    report = probe._readiness_report(store)

    assert report["sample_count"] == 6
    assert report["zrange_min_m"] == pytest.approx(0.008)
    assert report["zrange_max_m"] == pytest.approx(0.013)
    assert report["zrange_last_m"] == pytest.approx(0.013)
    # Step 3.6: velocity covariance (varPX/PY/PZ), explicitly marked as such.
    assert report["variables"] == [
        "range.zrange",
        "kalman.varPX",
        "kalman.varPY",
        "kalman.varPZ",
    ]
    assert report["estimator_covariance_kind"] == "velocity_covariance"
    assert report["estimator_covariance_source"] == "kalman.varPX/PY/PZ"
    assert report["var_px_last"] == pytest.approx(1.2e-4)
    assert report["var_py_last"] == pytest.approx(1.3e-4)
    assert report["var_pz_last"] == pytest.approx(1.7e-4)
    assert report["var_px_span"] == pytest.approx(0.0)
    assert report["var_pz_span"] == pytest.approx(0.0)
    assert "var_x_last" not in report


def test_probe_flow_report_parses_quality_and_motion_flags() -> None:
    store = probe.ProbeStore()
    for index in range(10):
        store.flow.append(
            probe.LogSample(
                monotonic_s=index * 0.1,
                values=(100.0 - index, -50.0 + index, 1.0 if index % 2 == 0 else 0.0, 40.0 + index),
            )
        )

    report = probe._flow_report(store)

    assert report["sample_count"] == 10
    assert report["delta_x_min"] == 91.0
    assert report["delta_x_max"] == 100.0
    assert report["delta_y_min"] == -50.0
    assert report["delta_y_max"] == -41.0
    assert report["motion_flagged_ratio"] == pytest.approx(0.5)
    assert report["squal_min"] == 40.0
    assert report["squal_max"] == 49.0
    assert report["squal_mean"] == pytest.approx(44.5)


def test_probe_flow_report_flags_missing_topic_samples() -> None:
    report = probe._flow_report(probe.ProbeStore())

    assert report["sample_count"] == 0
    assert report["reason"] == "no_flow_samples"


def _healthy_store() -> Any:
    store = probe.ProbeStore()
    for index in range(60):
        store.poses.append(
            probe.PoseSample(
                monotonic_s=index * 0.1,
                x_m=0.0005 * math.sin(index / 5.0),
                y_m=0.0005 * math.cos(index / 6.0),
                z_m=0.03,
            )
        )
        store.odoms.append(
            probe.OdomSample(
                monotonic_s=index * 0.1,
                vx_mps=0.001 * math.sin(index / 4.0),
                vy_mps=0.001 * math.cos(index / 5.0),
                vz_mps=0.0,
            )
        )
        store.readiness.append(
            probe.LogSample(monotonic_s=index * 0.1, values=(80.0, 0.0001, 0.0001, 0.0001))
        )
        store.flow.append(probe.LogSample(monotonic_s=index * 0.1, values=(1.0, 1.0, 0.0, 60.0)))
    return store


def test_probe_static_gate_passes_when_all_live_signals_are_healthy() -> None:
    report = probe._build_report(_healthy_store(), robot_id="cf231", wall_duration_s=6.0)

    assert report["blocking_issues"] == []
    assert report["static_gate_passed"] is True
    assert report["pose"]["stable"] is True
    assert report["odom"]["settled"] is True
    assert report["flow"]["sample_count"] == 60


def test_probe_static_gate_reports_missing_flow_diagnostics() -> None:
    store = _healthy_store()
    store.flow.clear()

    report = probe._build_report(store, robot_id="cf231", wall_duration_s=6.0)

    assert report["static_gate_passed"] is False
    assert any("flow_diagnostics_missing" in issue for issue in report["blocking_issues"])


def test_probe_odom_report_flags_velocity_above_policy() -> None:
    """Mirrors the observed cf231 standby velocity vx=0.052, vy=0.020, speed=0.0557."""
    store = probe.ProbeStore()
    for index in range(10):
        store.odoms.append(
            probe.OdomSample(monotonic_s=index * 0.1, vx_mps=0.052, vy_mps=0.020, vz_mps=0.0)
        )

    report = probe._odom_report(store)

    assert report["settled"] is False
    assert report["vx_abs_max"] == pytest.approx(0.052)
    assert any("|vx|" in violation for violation in report["violations"])
    assert report["speed_max"] == pytest.approx(math.hypot(0.052, 0.020))


def test_probe_odom_report_accepts_velocity_inside_policy() -> None:
    store = probe.ProbeStore()
    for index in range(10):
        store.odoms.append(
            probe.OdomSample(monotonic_s=index * 0.1, vx_mps=0.01, vy_mps=0.01, vz_mps=0.0)
        )

    report = probe._odom_report(store)

    assert report["settled"] is True
    assert report["violations"] == []


def test_probe_static_gate_flags_implausible_range() -> None:
    store = _healthy_store()
    store.readiness.clear()
    for index in range(10):
        store.readiness.append(
            probe.LogSample(monotonic_s=index * 0.1, values=(0.0, 0.0001, 0.0001, 0.0001))
        )

    report = probe._build_report(store, robot_id="cf231", wall_duration_s=6.0)

    assert report["static_gate_passed"] is False
    assert "range_invalid" in report["blocking_issues"]
