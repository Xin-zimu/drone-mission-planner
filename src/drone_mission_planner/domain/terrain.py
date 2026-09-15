from __future__ import annotations

import math
from dataclasses import dataclass, field
from math import ceil, floor, isfinite

from .geometry import Point


@dataclass(frozen=True, slots=True)
class TerrainPeak:
    center: Point
    radius: float
    height: float


@dataclass(frozen=True, slots=True)
class TerrainSample:
    """One terrain lookup with validity separate from the numeric elevation."""

    elevation_m: float | None
    valid: bool
    source: str = ""
    reason: str = ""


@dataclass(slots=True)
class TerrainModel:
    terrain_type: str = "flat"
    resolution: float = 25.0
    base_altitude: float = 0.0
    min_altitude: float = 0.0
    max_altitude: float = 0.0
    peaks: list[TerrainPeak] = field(default_factory=list)
    grid_origin: Point | None = None
    grid_width: int = 0
    grid_height: int = 0
    grid_altitudes: list[list[float]] = field(default_factory=list)
    grid_valid_mask: list[list[bool]] = field(default_factory=list)
    source_data_id: str | None = None

    def altitude_at(self, x: float, y: float) -> float:
        if self.terrain_type == "grid" and self.grid_altitudes and self.grid_origin is not None:
            return self._grid_altitude_at(x, y)
        altitude = self.base_altitude
        for peak in self.peaks:
            if peak.radius <= 0:
                continue
            dx = x - peak.center.x
            dy = y - peak.center.y
            altitude += peak.height * math.exp(
                -(dx * dx + dy * dy) / (2.0 * peak.radius * peak.radius)
            )
        return altitude

    def sample_at(self, x: float, y: float) -> TerrainSample:
        """Return terrain elevation plus whether the sample is verifiable."""

        if self.terrain_type == "grid" and self.grid_altitudes and self.grid_origin is not None:
            return self._grid_sample_at(x, y)
        if not isfinite(x) or not isfinite(y):
            return TerrainSample(None, False, self.terrain_type, "coordinates are not finite")
        return TerrainSample(self.altitude_at(x, y), True, self.terrain_type)

    def _grid_altitude_at(self, x: float, y: float) -> float:
        if not isfinite(x) or not isfinite(y) or self.grid_origin is None:
            return self.base_altitude
        x_index = _clamp((x - self.grid_origin.x) / self.resolution, 0.0, self.grid_width - 1.0)
        y_index = _clamp((y - self.grid_origin.y) / self.resolution, 0.0, self.grid_height - 1.0)
        x0 = floor(x_index)
        y0 = floor(y_index)
        x1 = min(self.grid_width - 1, x0 + 1)
        y1 = min(self.grid_height - 1, y0 + 1)
        tx = x_index - x0
        ty = y_index - y0
        top = _lerp(self.grid_altitudes[y0][x0], self.grid_altitudes[y0][x1], tx)
        bottom = _lerp(self.grid_altitudes[y1][x0], self.grid_altitudes[y1][x1], tx)
        return _lerp(top, bottom, ty)

    def _grid_sample_at(self, x: float, y: float) -> TerrainSample:
        source = self.source_data_id or "grid"
        if not isfinite(x) or not isfinite(y) or self.grid_origin is None:
            return TerrainSample(None, False, source, "coordinates are not finite")
        if self.grid_width <= 0 or self.grid_height <= 0:
            return TerrainSample(None, False, source, "terrain grid is empty")
        x_index = (x - self.grid_origin.x) / self.resolution
        y_index = (y - self.grid_origin.y) / self.resolution
        if (
            x_index < -1e-9
            or y_index < -1e-9
            or x_index > self.grid_width - 1 + 1e-9
            or y_index > self.grid_height - 1 + 1e-9
        ):
            return TerrainSample(None, False, source, "point is outside the DEM coverage")
        x_index = _clamp(x_index, 0.0, self.grid_width - 1.0)
        y_index = _clamp(y_index, 0.0, self.grid_height - 1.0)
        x0 = floor(x_index)
        y0 = floor(y_index)
        x1 = min(self.grid_width - 1, x0 + 1)
        y1 = min(self.grid_height - 1, y0 + 1)
        corners = ((x0, y0), (x1, y0), (x0, y1), (x1, y1))
        if self.grid_valid_mask and not all(self.grid_valid_mask[row][column] for column, row in corners):
            return TerrainSample(None, False, source, "DEM sample touches NoData")
        return TerrainSample(self._grid_altitude_at(x, y), True, source)


def flat_terrain(*, altitude: float = 0.0, resolution: float = 25.0) -> TerrainModel:
    return TerrainModel(
        terrain_type="flat",
        resolution=resolution,
        base_altitude=altitude,
        min_altitude=altitude,
        max_altitude=altitude,
    )


def generate_mountain_terrain(
    *,
    width: float,
    height: float,
    resolution: float,
    peaks: list[TerrainPeak],
    base_altitude: float = 0.0,
) -> TerrainModel:
    if width <= 0 or height <= 0:
        raise ValueError("terrain dimensions must be positive")
    if resolution <= 0:
        raise ValueError("terrain resolution must be positive")
    for peak in peaks:
        if peak.radius <= 0:
            raise ValueError("terrain peak radius must be positive")

    model = TerrainModel(
        terrain_type="procedural",
        resolution=resolution,
        base_altitude=base_altitude,
        min_altitude=base_altitude,
        max_altitude=base_altitude,
        peaks=list(peaks),
    )
    samples = _sample_altitudes(model, width=width, height=height, resolution=resolution)
    if samples:
        model.min_altitude = min(samples)
        model.max_altitude = max(samples)
    return model


def grid_terrain(
    *,
    origin: Point,
    resolution: float,
    altitudes: list[list[float]],
    valid_mask: list[list[bool]] | None = None,
    source_data_id: str | None = None,
) -> TerrainModel:
    if not isfinite(origin.x) or not isfinite(origin.y):
        raise ValueError("terrain grid origin must be finite")
    if not isfinite(resolution) or resolution <= 0:
        raise ValueError("terrain resolution must be positive")
    if not altitudes or not altitudes[0]:
        raise ValueError("terrain grid must contain at least one altitude sample")
    width = len(altitudes[0])
    flattened: list[float] = []
    normalized_rows: list[list[float]] = []
    normalized_mask: list[list[bool]] = []
    for row_index, row in enumerate(altitudes):
        if len(row) != width:
            raise ValueError("terrain grid rows must have equal width")
        normalized_row = [float(value) for value in row]
        if not all(isfinite(value) for value in normalized_row):
            raise ValueError("terrain grid altitude values must be finite")
        if valid_mask is None:
            mask_row = [True for _value in normalized_row]
        else:
            if row_index >= len(valid_mask) or len(valid_mask[row_index]) != width:
                raise ValueError("terrain grid valid mask must match altitude dimensions")
            mask_row = [bool(value) for value in valid_mask[row_index]]
        flattened.extend(value for value, valid in zip(normalized_row, mask_row, strict=True) if valid)
        normalized_rows.append(normalized_row)
        normalized_mask.append(mask_row)
    if not flattened:
        raise ValueError("terrain grid must contain at least one valid altitude sample")
    return TerrainModel(
        terrain_type="grid",
        resolution=resolution,
        base_altitude=normalized_rows[0][0],
        min_altitude=min(flattened),
        max_altitude=max(flattened),
        peaks=[],
        grid_origin=origin,
        grid_width=width,
        grid_height=len(normalized_rows),
        grid_altitudes=normalized_rows,
        grid_valid_mask=normalized_mask,
        source_data_id=source_data_id,
    )


def load_from_dem(filepath: str) -> TerrainModel:
    raise NotImplementedError(f"DEM import is not implemented yet: {filepath}")


def _sample_altitudes(
    model: TerrainModel, *, width: float, height: float, resolution: float
) -> list[float]:
    if not all(isfinite(value) for value in (width, height, resolution)):
        return []
    columns = max(1, ceil(width / resolution))
    rows = max(1, ceil(height / resolution))
    samples: list[float] = []
    for row in range(rows + 1):
        y = min(height, row * resolution)
        for column in range(columns + 1):
            x = min(width, column * resolution)
            samples.append(model.altitude_at(x, y))
    return samples


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _lerp(start: float, end: float, ratio: float) -> float:
    return start + (end - start) * ratio
