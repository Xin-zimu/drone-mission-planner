from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, exp, isfinite

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

    def altitude_at(self, x: float, y: float) -> float:
        altitude = self.base_altitude
        for peak in self.peaks:
            if peak.radius <= 0:
                continue
            dx = x - peak.center.x
            dy = y - peak.center.y
            altitude += peak.height * exp(-(dx * dx + dy * dy) / (2.0 * peak.radius * peak.radius))
        return altitude


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
