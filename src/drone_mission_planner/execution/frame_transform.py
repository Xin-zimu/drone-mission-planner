from __future__ import annotations

import math

from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import Waypoint, waypoint_msl_altitude

from .models import ExecutionFrameCalibration
from .validation import VALID_FRAME_MODES, ExecutionValidationError


def transform_planner_waypoint(
    waypoint: Waypoint,
    terrain: TerrainModel,
    frame: ExecutionFrameCalibration,
) -> tuple[float, float, float]:
    """Convert a Planner waypoint to the Crazyflie execution frame."""

    _require_valid_transform_frame(frame)
    planner_z_m = waypoint_msl_altitude(waypoint, terrain)
    if not all(math.isfinite(value) for value in (waypoint.x, waypoint.y, planner_z_m)):
        raise ExecutionValidationError("waypoint contains non-finite Planner coordinates")
    cf_x_m, cf_y_m = transform_planner_xy(waypoint.x, waypoint.y, frame)
    cf_z_m = planner_z_m - frame.planner_origin_z_m + frame.cf_origin_z_m
    if not math.isfinite(cf_z_m):
        raise ExecutionValidationError("waypoint transformed z is not finite")
    return cf_x_m, cf_y_m, cf_z_m


def transform_planner_xy(
    x_m: float,
    y_m: float,
    frame: ExecutionFrameCalibration,
) -> tuple[float, float]:
    """Rotate and translate Planner x/east, y/north into Crazyflie x/y metres."""

    _require_valid_transform_frame(frame)
    if not all(math.isfinite(value) for value in (x_m, y_m)):
        raise ExecutionValidationError("Planner coordinates must be finite")
    dx_m = x_m - frame.planner_origin_x_m
    dy_m = y_m - frame.planner_origin_y_m
    cos_yaw = math.cos(frame.yaw_offset_rad)
    sin_yaw = math.sin(frame.yaw_offset_rad)
    return (
        frame.cf_origin_x_m + cos_yaw * dx_m - sin_yaw * dy_m,
        frame.cf_origin_y_m + sin_yaw * dx_m + cos_yaw * dy_m,
    )


def _require_valid_transform_frame(frame: ExecutionFrameCalibration) -> None:
    if frame.mode not in VALID_FRAME_MODES:
        raise ExecutionValidationError(f"execution frame mode {frame.mode} is unsupported")
    values = (
        frame.planner_origin_x_m,
        frame.planner_origin_y_m,
        frame.planner_origin_z_m,
        frame.cf_origin_x_m,
        frame.cf_origin_y_m,
        frame.cf_origin_z_m,
        frame.yaw_offset_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ExecutionValidationError("execution frame calibration contains non-finite values")
