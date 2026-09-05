from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, MapModel, MissionTask
from drone_mission_planner.simulation.engine import SimulationEngine
from drone_mission_planner.simulation.replay import ReplayRecorder, export_replay


def _engine() -> SimulationEngine:
    model = MapModel(width=100, height=100, grid_size=5.0)
    model.bases.append(BaseStation("B-01", "Base", Point(10, 10)))
    model.drones.append(
        Drone(
            "D-01",
            "Alpha",
            Point(10, 10),
            "B-01",
            max_speed=10,
            energy_per_meter=0.1,
            assigned_tasks=["T-01"],
            planned_path=[Point(10, 10), Point(30, 10), Point(10, 10)],
        )
    )
    model.tasks.append(MissionTask("T-01", "Task", Point(30, 10)))
    return SimulationEngine(model, fixed_dt=0.05)


def test_recorder_samples_at_fixed_interval() -> None:
    engine = _engine()
    engine.run_until_complete()
    frames = engine.replay.frames

    assert frames
    assert frames[0].time == pytest.approx(0.0)
    times = [frame.time for frame in frames]
    assert all(
        later - earlier == pytest.approx(engine.replay.interval, abs=engine.fixed_dt)
        for earlier, later in zip(times, times[1:], strict=False)
    )


def test_frames_carry_three_dimensional_states() -> None:
    engine = _engine()
    engine.run_until_complete()
    frame = engine.replay.frames[len(engine.replay.frames) // 2]

    state = frame.drones[0]
    assert state.drone_id == "D-01"
    assert state.x >= 10.0
    assert state.status in {"flying", "climbing", "descending", "executing", "completed"}
    assert frame.processed_events >= 0


def test_replay_export_round_trip(tmp_path: Path) -> None:
    engine = _engine()
    engine.run_until_complete()

    saved = export_replay(engine, tmp_path / "replay.json")
    document = json.loads(saved.read_text(encoding="utf-8"))

    assert document["interval"] == engine.replay.interval
    assert len(document["frames"]) == len(engine.replay.frames)
    assert document["frames"][0]["drones"][0]["drone_id"] == "D-01"


def test_frame_index_for_time_clamps() -> None:
    recorder = ReplayRecorder(interval=0.5)
    engine = _engine()
    engine.run_until_complete()
    recorder.record(engine)
    del engine

    assert recorder.frame_index_for_time(0.0) == 0
    last = len(recorder.frames) - 1
    assert recorder.frame_index_for_time(10_000.0) == last


def test_reset_clears_frames() -> None:
    engine = _engine()
    engine.run_until_complete()
    assert engine.replay.frames

    engine.reset()

    assert engine.replay.frames == ()


def test_recording_does_not_change_the_result() -> None:
    plain = _engine()
    plain.run_until_complete()
    plain_snapshot = plain.snapshot()

    recorded = _engine()
    recorded.replay = ReplayRecorder(interval=0.1)
    recorded.run_until_complete()

    assert recorded.snapshot() == plain_snapshot
    assert recorded.replay.frames  # recording happened without changing outcomes
