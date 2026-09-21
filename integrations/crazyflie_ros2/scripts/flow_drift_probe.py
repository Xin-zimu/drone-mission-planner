#!/usr/bin/env python3
"""Read-only Flow Deck static-drift probe for the CF8.1 hardware bring-up.

Subscribes to exactly the Crazyswarm2 topics the DMP bridge consumes and recomputes
the same pose/velocity/range statistics the bridge readiness policy evaluates, so the
numbers here are directly comparable with ``get_telemetry``.

Safety: this tool only creates subscriptions. It never publishes a command and never
calls a ROS service, so it cannot arm, take off, land or otherwise move a vehicle.

Usage (inside WSL, with the Crazyswarm2 workspace sourced):

    python3 scripts/flow_drift_probe.py --robot-id cf231 --duration 20
    python3 scripts/flow_drift_probe.py --duration 20 --json /tmp/cf231-drift.json
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path
from typing import Any

DMP_READINESS_LOG_TOPIC = "dmp_readiness"
DMP_FLOW_LOG_TOPIC = "dmp_flow"
DMP_READINESS_LOG_VARIABLES = ("range.zrange", "kalman.varX", "kalman.varY", "kalman.varZ")
DMP_FLOW_LOG_VARIABLES = ("motion.deltaX", "motion.deltaY", "motion.motion", "motion.squal")

# Mirrors dmp_crazyflie_bridge.telemetry.PoseStabilityPolicy / ReadinessPolicy defaults.
MINIMUM_SAMPLE_COUNT = 20
MINIMUM_OBSERVATION_DURATION_S = 5.0
MAX_STATIC_XY_DISPLACEMENT_M = 0.03
MAX_STATIC_Z_DISPLACEMENT_M = 0.03
MAX_SAMPLE_GAP_S = 0.5
MAX_ABS_VELOCITY_MPS = 0.05
MAX_SPEED_MPS = 0.08


@dataclass(slots=True)
class PoseSample:
    monotonic_s: float
    x_m: float
    y_m: float
    z_m: float


@dataclass(slots=True)
class OdomSample:
    monotonic_s: float
    vx_mps: float
    vy_mps: float
    vz_mps: float

    @property
    def speed_mps(self) -> float:
        return math.sqrt(self.vx_mps**2 + self.vy_mps**2 + self.vz_mps**2)


@dataclass(slots=True)
class LogSample:
    monotonic_s: float
    values: tuple[float, ...]


@dataclass(slots=True)
class ProbeStore:
    poses: list[PoseSample] = field(default_factory=list)
    odoms: list[OdomSample] = field(default_factory=list)
    readiness: list[LogSample] = field(default_factory=list)
    flow: list[LogSample] = field(default_factory=list)


def _finite(value: float) -> bool:
    return math.isfinite(value)


def _pose_report(store: ProbeStore) -> dict[str, Any]:
    poses = store.poses
    report: dict[str, Any] = {"sample_count": len(poses)}
    if not poses:
        report["reason"] = "no_pose_samples"
        return report

    first, last = poses[0], poses[-1]
    duration = last.monotonic_s - first.monotonic_s
    gaps = [later.monotonic_s - earlier.monotonic_s for earlier, later in pairwise(poses)]
    max_gap = max(gaps) if gaps else None

    xs = [sample.x_m for sample in poses]
    ys = [sample.y_m for sample in poses]
    zs = [sample.z_m for sample in poses]
    center_x = statistics.fmean(xs)
    center_y = statistics.fmean(ys)
    center_z = statistics.fmean(zs)

    xy_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    xy_displacement = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
    z_span = max(zs) - min(zs)
    xy_std = math.sqrt(
        statistics.fmean((sample.x_m - center_x) ** 2 + (sample.y_m - center_y) ** 2 for sample in poses)
    )
    z_std = math.sqrt(statistics.fmean((sample.z_m - center_z) ** 2 for sample in poses))

    report.update(
        {
            "duration_s": duration,
            "pose_rate_hz": (len(poses) - 1) / duration if duration > 0 else None,
            "x_first": first.x_m,
            "y_first": first.y_m,
            "x_last": last.x_m,
            "y_last": last.y_m,
            "x_min": min(xs),
            "x_max": max(xs),
            "y_min": min(ys),
            "y_max": max(ys),
            "z_mean": center_z,
            "xy_span_m": xy_span,
            "xy_displacement_m": xy_displacement,
            "xy_std_m": xy_std,
            "z_span_m": z_span,
            "z_std_m": z_std,
            "max_sample_gap_s": max_gap,
        }
    )

    if len(poses) < MINIMUM_SAMPLE_COUNT:
        report["stable"] = False
        report["reason"] = "not_enough_pose_samples"
    elif duration < MINIMUM_OBSERVATION_DURATION_S:
        report["stable"] = False
        report["reason"] = "observation_too_short"
    else:
        stable = (
            xy_span <= MAX_STATIC_XY_DISPLACEMENT_M
            and z_span <= MAX_STATIC_Z_DISPLACEMENT_M
            and (max_gap is None or max_gap <= MAX_SAMPLE_GAP_S)
        )
        report["stable"] = stable
        report["reason"] = None if stable else "pose_drift_or_gap_exceeds_limit"
        report["violations"] = _pose_violations(xy_span, z_span, max_gap)
    return report


def _pose_violations(xy_span: float, z_span: float, max_gap: float | None) -> list[str]:
    violations: list[str] = []
    if xy_span > MAX_STATIC_XY_DISPLACEMENT_M:
        violations.append(f"xy_span {xy_span:.4f} > {MAX_STATIC_XY_DISPLACEMENT_M}")
    if z_span > MAX_STATIC_Z_DISPLACEMENT_M:
        violations.append(f"z_span {z_span:.4f} > {MAX_STATIC_Z_DISPLACEMENT_M}")
    if max_gap is not None and max_gap > MAX_SAMPLE_GAP_S:
        violations.append(f"max_gap {max_gap:.4f} > {MAX_SAMPLE_GAP_S}")
    return violations


def _odom_report(store: ProbeStore) -> dict[str, Any]:
    odoms = store.odoms
    report: dict[str, Any] = {"sample_count": len(odoms)}
    if not odoms:
        report["reason"] = "no_odom_samples"
        return report

    speeds = [sample.speed_mps for sample in odoms]
    vxs = [sample.vx_mps for sample in odoms]
    vys = [sample.vy_mps for sample in odoms]
    vzs = [sample.vz_mps for sample in odoms]

    report.update(
        {
            "vx_mean": statistics.fmean(vxs),
            "vy_mean": statistics.fmean(vys),
            "vz_mean": statistics.fmean(vzs),
            "speed_mean": statistics.fmean(speeds),
            "vx_abs_max": max(abs(value) for value in vxs),
            "vy_abs_max": max(abs(value) for value in vys),
            "vz_abs_max": max(abs(value) for value in vzs),
            "speed_max": max(speeds),
        }
    )

    violations: list[str] = []
    if report["vx_abs_max"] > MAX_ABS_VELOCITY_MPS:
        violations.append(f"|vx| {report['vx_abs_max']:.4f} > {MAX_ABS_VELOCITY_MPS}")
    if report["vy_abs_max"] > MAX_ABS_VELOCITY_MPS:
        violations.append(f"|vy| {report['vy_abs_max']:.4f} > {MAX_ABS_VELOCITY_MPS}")
    if report["vz_abs_max"] > MAX_ABS_VELOCITY_MPS:
        violations.append(f"|vz| {report['vz_abs_max']:.4f} > {MAX_ABS_VELOCITY_MPS}")
    if report["speed_max"] > MAX_SPEED_MPS:
        violations.append(f"speed {report['speed_max']:.4f} > {MAX_SPEED_MPS}")
    report["violations"] = violations
    report["settled"] = not violations
    return report


def _readiness_report(store: ProbeStore) -> dict[str, Any]:
    samples = store.readiness
    report: dict[str, Any] = {"sample_count": len(samples), "variables": list(DMP_READINESS_LOG_VARIABLES)}
    if not samples:
        report["reason"] = "no_readiness_samples"
        return report

    zranges = [sample.values[0] / 1000.0 for sample in samples if len(sample.values) >= 4]
    var_xs = [sample.values[1] for sample in samples if len(sample.values) >= 4]
    var_ys = [sample.values[2] for sample in samples if len(sample.values) >= 4]
    var_zs = [sample.values[3] for sample in samples if len(sample.values) >= 4]

    if zranges:
        report.update(
            {
                "zrange_min_m": min(zranges),
                "zrange_max_m": max(zranges),
                "zrange_mean_m": statistics.fmean(zranges),
                "zrange_last_m": zranges[-1],
            }
        )
    if var_xs:
        report.update(
            {
                "var_x_last": var_xs[-1],
                "var_y_last": var_ys[-1],
                "var_z_last": var_zs[-1],
                "var_x_span": max(var_xs) - min(var_xs),
                "var_y_span": max(var_ys) - min(var_ys),
                "var_z_span": max(var_zs) - min(var_zs),
            }
        )
    return report


def _flow_report(store: ProbeStore) -> dict[str, Any]:
    samples = store.flow
    report: dict[str, Any] = {"sample_count": len(samples), "variables": list(DMP_FLOW_LOG_VARIABLES)}
    if not samples:
        report["reason"] = "no_flow_samples"
        return report

    usable = [sample.values for sample in samples if len(sample.values) >= 4]
    if not usable:
        report["reason"] = "flow_samples_missing_values"
        return report

    delta_xs = [values[0] for values in usable]
    delta_ys = [values[1] for values in usable]
    motion_flags = [values[2] for values in usable]
    squals = [values[3] for values in usable]

    report.update(
        {
            "delta_x_min": min(delta_xs),
            "delta_x_max": max(delta_xs),
            "delta_y_min": min(delta_ys),
            "delta_y_max": max(delta_ys),
            "motion_flagged_ratio": sum(1 for flag in motion_flags if flag != 0.0) / len(motion_flags),
            "squal_min": min(squals),
            "squal_mean": statistics.fmean(squals),
            "squal_max": max(squals),
        }
    )
    return report


def _build_report(store: ProbeStore, *, robot_id: str, wall_duration_s: float) -> dict[str, Any]:
    pose = _pose_report(store)
    odom = _odom_report(store)
    readiness = _readiness_report(store)
    flow = _flow_report(store)

    blocking: list[str] = []
    if not pose.get("stable", False):
        blocking.append(f"pose_stability_failed ({pose.get('reason') or 'drift'})")
    if not odom.get("settled", False):
        blocking.append("velocity_not_settled")
    if not store.readiness:
        blocking.append("range_unavailable / estimator_not_converged (no dmp_readiness samples)")
    else:
        zrange = readiness.get("zrange_last_m")
        if zrange is None or not _finite(zrange) or zrange <= 0.0:
            blocking.append("range_invalid")
    if not store.flow:
        blocking.append("flow_diagnostics_missing (dmp_flow topic not published)")

    return {
        "tool": "flow_drift_probe",
        "robot_id": robot_id,
        "wall_duration_s": wall_duration_s,
        "policy": {
            "minimum_sample_count": MINIMUM_SAMPLE_COUNT,
            "minimum_observation_duration_s": MINIMUM_OBSERVATION_DURATION_S,
            "max_static_xy_displacement_m": MAX_STATIC_XY_DISPLACEMENT_M,
            "max_static_z_displacement_m": MAX_STATIC_Z_DISPLACEMENT_M,
            "max_sample_gap_s": MAX_SAMPLE_GAP_S,
            "max_abs_velocity_mps": MAX_ABS_VELOCITY_MPS,
            "max_speed_mps": MAX_SPEED_MPS,
        },
        "pose": pose,
        "odom": odom,
        "readiness": readiness,
        "flow": flow,
        "blocking_issues": blocking,
        "static_gate_passed": not blocking,
    }


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="cf231", help="Crazyflie robot id (default: cf231)")
    parser.add_argument("--duration", type=float, default=20.0, help="capture window in seconds (default: 20)")
    parser.add_argument("--json", type=Path, default=None, help="also write the JSON report to this path")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)

    try:
        import rclpy
        from crazyflie_interfaces.msg import LogDataGeneric
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except ImportError as exc:  # pragma: no cover - environment dependent
        print(
            f"error: ROS2 python packages are unavailable ({exc}).\n"
            "Source the environment first:\n"
            "  source ~/crazyswarm-env/bin/activate\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/ros2_ws/install/local_setup.bash",
            file=sys.stderr,
        )
        return 2

    store = ProbeStore()
    robot_id = str(args.robot_id)

    class FlowDriftProbe(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("dmp_flow_drift_probe")
            self.create_subscription(PoseStamped, f"/{robot_id}/pose", self._on_pose, 10)
            self.create_subscription(Odometry, f"/{robot_id}/odom", self._on_odom, 10)
            self.create_subscription(
                LogDataGeneric, f"/{robot_id}/{DMP_READINESS_LOG_TOPIC}", self._on_readiness, 10
            )
            self.create_subscription(LogDataGeneric, f"/{robot_id}/{DMP_FLOW_LOG_TOPIC}", self._on_flow, 10)

        def _on_pose(self, msg: Any) -> None:
            store.poses.append(
                PoseSample(
                    monotonic_s=time.monotonic(),
                    x_m=float(msg.pose.position.x),
                    y_m=float(msg.pose.position.y),
                    z_m=float(msg.pose.position.z),
                )
            )

        def _on_odom(self, msg: Any) -> None:
            twist = msg.twist.twist
            store.odoms.append(
                OdomSample(
                    monotonic_s=time.monotonic(),
                    vx_mps=float(twist.linear.x),
                    vy_mps=float(twist.linear.y),
                    vz_mps=float(twist.linear.z),
                )
            )

        def _on_readiness(self, msg: Any) -> None:
            store.readiness.append(
                LogSample(monotonic_s=time.monotonic(), values=tuple(float(value) for value in msg.values))
            )

        def _on_flow(self, msg: Any) -> None:
            store.flow.append(
                LogSample(monotonic_s=time.monotonic(), values=tuple(float(value) for value in msg.values))
            )

    rclpy.init()
    node = FlowDriftProbe()
    started = time.monotonic()
    try:
        while time.monotonic() - started < float(args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:  # pragma: no cover - interactive
        pass
    finally:
        wall_duration = time.monotonic() - started
        node.destroy_node()
        rclpy.shutdown()

    report = _build_report(store, robot_id=robot_id, wall_duration_s=wall_duration)
    text = json.dumps(report, indent=2, sort_keys=False)
    print(text)
    if args.json is not None:
        args.json.write_text(text + "\n", encoding="utf-8")
        print(f"\nwrote {args.json}", file=sys.stderr)

    return 0 if report["static_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
