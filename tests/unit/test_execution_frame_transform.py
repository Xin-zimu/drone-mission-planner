from __future__ import annotations

import math

import pytest

from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.execution.frame_transform import (
    capture_current_pose_origin,
    transform_planner_waypoint,
    transform_planner_xy,
)
from drone_mission_planner.execution.models import ExecutionFrameCalibration


def test_relative_takeoff_identity_transform() -> None:
    frame = _frame(origin_x=10.0, origin_y=10.0)

    assert transform_planner_xy(11.0, 12.0, frame) == pytest.approx((1.0, 2.0))


def test_relative_takeoff_rotates_by_yaw_offset() -> None:
    frame = _frame(yaw_offset_rad=math.pi / 2.0)

    assert transform_planner_xy(1.0, 0.0, frame) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_relative_takeoff_applies_translated_origin_and_cf_offset() -> None:
    frame = _frame(
        origin_x=10.0,
        origin_y=10.0,
        origin_z=1.0,
        cf_origin_x=0.5,
        cf_origin_y=-0.5,
        cf_origin_z=0.2,
    )

    waypoint = Waypoint(11.0, 12.0, 1.5)

    assert transform_planner_waypoint(waypoint, TerrainModel(), frame) == pytest.approx(
        (1.5, 1.5, 0.7)
    )


def test_relative_current_pose_origin_maps_planner_local_to_estimator_world() -> None:
    frame = capture_current_pose_origin(
        planner_origin_x_m=10.0,
        planner_origin_y_m=20.0,
        planner_origin_z_m=0.0,
        cf_x_m=14.9911,
        cf_y_m=6.6701,
        cf_z_m=0.00965,
        captured_at_utc="2026-09-18T00:00:00Z",
    )

    assert frame.mode == "relative_current_pose"
    assert frame.origin_source == "current_pose"
    assert frame.origin_captured_at_utc == "2026-09-18T00:00:00Z"
    assert transform_planner_xy(11.0, 20.0, frame) == pytest.approx((15.9911, 6.6701))


def _frame(
    *,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_z: float = 0.0,
    cf_origin_x: float = 0.0,
    cf_origin_y: float = 0.0,
    cf_origin_z: float = 0.0,
    yaw_offset_rad: float = 0.0,
) -> ExecutionFrameCalibration:
    return ExecutionFrameCalibration(
        mode="relative_takeoff",
        planner_origin_x_m=origin_x,
        planner_origin_y_m=origin_y,
        planner_origin_z_m=origin_z,
        cf_origin_x_m=cf_origin_x,
        cf_origin_y_m=cf_origin_y,
        cf_origin_z_m=cf_origin_z,
        yaw_offset_rad=yaw_offset_rad,
        validated=True,
    )
