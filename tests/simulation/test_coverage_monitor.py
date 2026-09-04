from __future__ import annotations

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import MapModel, SearchArea
from drone_mission_planner.simulation.coverage_monitor import CoverageMonitor


def test_coverage_and_repeat_coverage_are_distinct() -> None:
    model = MapModel(width=100, height=100, grid_size=10)
    model.search_areas.append(SearchArea("S-01", "Area", Rect(10, 10, 60, 40), scan_spacing=20))
    monitor = CoverageMonitor(model)
    initial = monitor.snapshot()[0]

    monitor.update({"D-01": Point(20, 20)})
    after_first = monitor.snapshot()[0]
    monitor.update({"D-02": Point(20, 20), "D-01": Point(50, 20)})
    after_second = monitor.snapshot()[0]

    assert initial.coverage == 0
    assert after_first.coverage > 0
    assert after_first.repeat_coverage == 0
    assert after_second.coverage > after_first.coverage
    assert after_second.repeat_coverage > 0


def test_reset_clears_visits() -> None:
    model = MapModel(width=100, height=100, grid_size=10)
    model.search_areas.append(SearchArea("S-01", "Area", Rect(0, 0, 80, 80)))
    monitor = CoverageMonitor(model)
    monitor.update({"D-01": Point(30, 30)})
    assert monitor.snapshot()[0].covered_cells > 0
    monitor.reset()
    assert monitor.snapshot()[0].covered_cells == 0
