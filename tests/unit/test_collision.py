from __future__ import annotations

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.planning.collision import ConflictDetector, MotionState


def test_predicts_crossing_conflict_and_yields_lower_priority() -> None:
    states = [
        MotionState(
            "D-01",
            Point(10, 50),
            (Point(10, 50), Point(90, 50)),
            1,
            10,
            6,
            9,
        ),
        MotionState(
            "D-02",
            Point(50, 90),
            (Point(50, 90), Point(50, 10)),
            1,
            10,
            6,
            3,
        ),
    ]

    conflicts = ConflictDetector(horizon=5.0).detect(states)

    assert len(conflicts) == 1
    assert conflicts[0].predicted_distance == 0
    assert conflicts[0].time_to_conflict == 4.0
    assert conflicts[0].yielding_drone_id == "D-02"


def test_equal_priority_uses_stable_id_tie_break() -> None:
    states = [
        MotionState("D-10", Point(0, 0), (Point(0, 0), Point(20, 0)), 1, 10, 5, 1),
        MotionState("D-02", Point(20, 0), (Point(20, 0), Point(0, 0)), 1, 10, 5, 1),
    ]
    conflict = ConflictDetector(horizon=1.0).detect(states)[0]
    assert conflict.yielding_drone_id == "D-10"
