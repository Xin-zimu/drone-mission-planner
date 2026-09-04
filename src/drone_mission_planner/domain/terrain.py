from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, exp, floor, isfinite

from .geometry import Point


@dataclass(frozen=True, slots=True)
class TerrainPeak:
    center: Point
    radius: float
    height: float


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

    def altitude_at(self, x: float, y: float) -> float:
        if self.terrain_type == "grid" and self.grid_altitudes and self.grid_origin is not None:
            return self._grid_altitude_at(x, y)
        altitude = self.base_altitude
        for peak in self.peaks:
            if peak.radius <= 0:
                continue
            dx = x - peak.center.x
            dy = y - peak.center.y
            altitude += peak.height * exp(-(dx * dx + dy * dy) / (2.0 * peak.radius * peak.radius))
        return altitude

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
    for row in altitudes:
        if len(row) != width:
            raise ValueError("terrain grid rows must have equal width")
        normalized_row = [float(value) for value in row]
        if not all(isfinite(value) for value in normalized_row):
            raise ValueError("terrain grid altitude values must be finite")
        flattened.extend(normalized_row)
        normalized_rows.append(normalized_row)
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
