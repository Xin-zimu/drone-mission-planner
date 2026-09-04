from __future__ import annotations

import csv
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from math import isfinite
from pathlib import Path

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.terrain import TerrainModel, grid_terrain

REQUIRED_COLUMNS = ("x", "y", "elevation")


class TerrainImportError(ValueError):
    """Raised when an elevation import file is malformed."""


@dataclass(frozen=True, slots=True)
class TerrainImportPreview:
    source: str
    sample_count: int
    grid_width: int
    grid_height: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_elevation: float
    max_elevation: float
    resolution: float

    def summary(self) -> str:
        return (
            f"{self.sample_count} samples, {self.grid_width} x {self.grid_height} grid, "
            f"x {self.min_x:.1f}-{self.max_x:.1f} m, "
            f"y {self.min_y:.1f}-{self.max_y:.1f} m, "
            f"elevation {self.min_elevation:.1f}-{self.max_elevation:.1f} m, "
            f"{self.resolution:.1f} m resolution"
        )


@dataclass(frozen=True, slots=True)
class TerrainImportResult:
    preview: TerrainImportPreview
    terrain: TerrainModel


@dataclass(frozen=True, slots=True)
class _TerrainSample:
    x: float
    y: float
    elevation: float


def load_terrain_csv(path: str | Path) -> TerrainImportResult:
    source = Path(path)
    try:
        with source.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            columns = _required_columns(reader.fieldnames)
            samples = _read_samples(reader, columns)
    except OSError as exc:
        raise TerrainImportError(f"Cannot read elevation CSV: {exc}") from exc

    if len(samples) < 2:
        raise TerrainImportError("CSV must contain at least two elevation samples")
    return _build_grid(source, samples)


def _required_columns(fieldnames: Sequence[str] | None) -> dict[str, str]:
    if fieldnames is None:
        raise TerrainImportError("CSV must include a header row with x, y, elevation columns")
    normalized = {name.strip().lower(): name for name in fieldnames if name is not None}
    missing = [column for column in REQUIRED_COLUMNS if column not in normalized]
    if missing:
        raise TerrainImportError(
            "CSV is missing required column(s): " + ", ".join(sorted(missing))
        )
    return {column: normalized[column] for column in REQUIRED_COLUMNS}


def _read_samples(
    reader: csv.DictReader[str], columns: dict[str, str]
) -> list[_TerrainSample]:
    samples: list[_TerrainSample] = []
    seen: set[tuple[float, float]] = set()
    for row in reader:
        if _is_empty_row(row):
            continue
        line_number = reader.line_num
        x = _parse_float(row.get(columns["x"]), "x", line_number)
        y = _parse_float(row.get(columns["y"]), "y", line_number)
        elevation = _parse_float(row.get(columns["elevation"]), "elevation", line_number)
        key = (x, y)
        if key in seen:
            raise TerrainImportError(
                f"CSV line {line_number} duplicates elevation sample at x={x:g}, y={y:g}"
            )
        seen.add(key)
        samples.append(_TerrainSample(x, y, elevation))
    return samples


def _is_empty_row(row: Mapping[str, str | None]) -> bool:
    return all(value is None or value.strip() == "" for value in row.values())


def _parse_float(value: str | None, column: str, line_number: int) -> float:
    if value is None or value.strip() == "":
        raise TerrainImportError(f"CSV line {line_number} column {column} is empty")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise TerrainImportError(
            f"CSV line {line_number} column {column} must be numeric"
        ) from exc
    if not isfinite(parsed):
        raise TerrainImportError(f"CSV line {line_number} column {column} must be finite")
    return parsed


def _build_grid(source: Path, samples: list[_TerrainSample]) -> TerrainImportResult:
    x_values = sorted({sample.x for sample in samples})
    y_values = sorted({sample.y for sample in samples})
    expected_samples = len(x_values) * len(y_values)
    if expected_samples != len(samples):
        raise TerrainImportError(
            "CSV samples must fill a complete regular grid; "
            f"expected {expected_samples} samples from unique x/y values, got {len(samples)}"
        )

    resolution = _grid_resolution(x_values, y_values)
    sample_map = {(sample.x, sample.y): sample.elevation for sample in samples}
    altitudes = [[sample_map[(x, y)] for x in x_values] for y in y_values]
    terrain = grid_terrain(
        origin=Point(x_values[0], y_values[0]),
        resolution=resolution,
        altitudes=altitudes,
    )
    preview = TerrainImportPreview(
        source=str(source),
        sample_count=len(samples),
        grid_width=len(x_values),
        grid_height=len(y_values),
        min_x=x_values[0],
        max_x=x_values[-1],
        min_y=y_values[0],
        max_y=y_values[-1],
        min_elevation=terrain.min_altitude,
        max_elevation=terrain.max_altitude,
        resolution=terrain.resolution,
    )
    return TerrainImportResult(preview=preview, terrain=terrain)


def _grid_resolution(x_values: list[float], y_values: list[float]) -> float:
    x_step = _regular_step(x_values, "x")
    y_step = _regular_step(y_values, "y")
    steps = [step for step in (x_step, y_step) if step is not None]
    if not steps:
        raise TerrainImportError("CSV must contain at least two distinct x or y values")
    resolution = steps[0]
    if any(not _close(step, resolution) for step in steps[1:]):
        raise TerrainImportError(
            "CSV x and y spacing must match because terrain grids use one resolution"
        )
    return resolution


def _regular_step(values: list[float], axis: str) -> float | None:
    if len(values) < 2:
        return None
    deltas = [right - left for left, right in pairwise(values)]
    step = deltas[0]
    if step <= 0 or not isfinite(step):
        raise TerrainImportError(f"CSV {axis} values must be strictly increasing")
    for delta in deltas[1:]:
        if delta <= 0 or not isfinite(delta) or not _close(delta, step):
            raise TerrainImportError(f"CSV {axis} values must form a regular grid")
    return step


def _close(left: float, right: float) -> bool:
    return abs(left - right) <= max(1e-6, abs(right) * 1e-6)
