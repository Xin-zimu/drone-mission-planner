from __future__ import annotations

from pathlib import Path

import pytest

from drone_mission_planner.persistence.terrain_import import (
    TerrainImportError,
    load_terrain_csv,
)


def test_load_terrain_csv_builds_grid_preview_and_model(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "10,20,100\n"
        "20,20,110\n"
        "10,30,120\n"
        "20,30,140\n",
        encoding="utf-8",
    )

    result = load_terrain_csv(path)

    assert result.preview.sample_count == 4
    assert result.preview.grid_width == 2
    assert result.preview.grid_height == 2
    assert result.preview.min_elevation == 100.0
    assert result.preview.max_elevation == 140.0
    assert result.preview.resolution == 10.0
    assert result.terrain.terrain_type == "grid"
    assert result.terrain.altitude_at(15.0, 25.0) == pytest.approx(117.5)
    assert "4 samples" in result.preview.summary()


def test_load_terrain_csv_rejects_missing_columns(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text("x,z\n0,10\n", encoding="utf-8")

    with pytest.raises(TerrainImportError, match="missing required column"):
        load_terrain_csv(path)


def test_load_terrain_csv_reports_non_numeric_line_and_column(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,not-a-number\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="line 3 column elevation"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_duplicate_samples(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "0,0,20\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="duplicates elevation sample"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_incomplete_grid(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,20\n"
        "0,10,30\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="complete regular grid"):
        load_terrain_csv(path)


def test_load_terrain_csv_rejects_irregular_spacing(tmp_path: Path) -> None:
    path = tmp_path / "terrain.csv"
    path.write_text(
        "x,y,elevation\n"
        "0,0,10\n"
        "10,0,20\n"
        "25,0,30\n",
        encoding="utf-8",
    )

    with pytest.raises(TerrainImportError, match="x values must form a regular grid"):
        load_terrain_csv(path)
