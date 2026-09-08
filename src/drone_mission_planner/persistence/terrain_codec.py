"""Decode a raw `.dmproj` terrain block into a domain ``TerrainModel``.

Shared by the project repository and the format migrations so that a project
loaded from an older schema generates waypoint altitudes with exactly the same
terrain sampling rules as a current project.
"""

from __future__ import annotations

from typing import Any

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.terrain import TerrainModel, TerrainPeak


def terrain_from_data(data: Any, fallback_resolution: float) -> TerrainModel:
    """Build a terrain model from the persisted terrain mapping.

    Unknown or missing fields fall back to the previous repository defaults so
    that older projects keep loading unchanged.
    """

    if not isinstance(data, dict):
        return TerrainModel(resolution=fallback_resolution)
    peaks = [_terrain_peak(item) for item in data.get("peaks", [])]
    grid_altitudes = _terrain_grid_altitudes(data.get("grid_altitudes", []))
    grid_origin_data = data.get("grid_origin")
    grid_origin = _point(grid_origin_data) if isinstance(grid_origin_data, dict) else None
    base_altitude_raw = data.get("base_altitude", data.get("altitude", 0.0))
    base_altitude = float(0.0 if base_altitude_raw is None else base_altitude_raw)
    grid_values = [value for row in grid_altitudes for value in row]
    if grid_values:
        default_min = min(grid_values)
        default_max = max(grid_values)
    else:
        default_min = base_altitude + sum(min(0.0, peak.height) for peak in peaks)
        default_max = base_altitude + sum(max(0.0, peak.height) for peak in peaks)
    return TerrainModel(
        terrain_type=str(data.get("terrain_type", data.get("type", "flat"))),
        resolution=float(data.get("resolution", fallback_resolution)),
        base_altitude=base_altitude,
        min_altitude=float(data.get("min_altitude", default_min)),
        max_altitude=float(data.get("max_altitude", default_max)),
        peaks=peaks,
        grid_origin=grid_origin,
        grid_width=int(data.get("grid_width", len(grid_altitudes[0]) if grid_altitudes else 0)),
        grid_height=int(data.get("grid_height", len(grid_altitudes))),
        grid_altitudes=grid_altitudes,
    )


def _point(data: dict[str, Any]) -> Point:
    return Point(float(data["x"]), float(data["y"]))


def _terrain_peak(data: Any) -> TerrainPeak:
    if isinstance(data, dict):
        if "center" in data:
            center = _point(data["center"])
        else:
            center = Point(float(data["center_x"]), float(data["center_y"]))
        height_raw = data.get("height", data.get("peak_height", 0.0))
        height = float(0.0 if height_raw is None else height_raw)
        return TerrainPeak(center=center, radius=float(data["radius"]), height=height)
    if isinstance(data, (list, tuple)) and len(data) == 4:
        return TerrainPeak(
            center=Point(float(data[0]), float(data[1])),
            radius=float(data[2]),
            height=float(data[3]),
        )
    raise ValueError("terrain peak must be an object or [x, y, radius, height]")


def _terrain_grid_altitudes(data: Any) -> list[list[float]]:
    if data is None:
        return []
    if not isinstance(data, list):
        raise ValueError("terrain grid altitudes must be a list")
    rows: list[list[float]] = []
    for row in data:
        if not isinstance(row, list):
            raise ValueError("terrain grid altitude rows must be lists")
        rows.append([float(value) for value in row])
    return rows
