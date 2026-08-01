from __future__ import annotations

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import BaseStation, Drone, ProjectModel
from drone_mission_planner.domain.validation import ProjectValidationError, validate_project


def test_rejects_invalid_battery_and_missing_home_base() -> None:
    project = ProjectModel()
    project.map.drones.append(
        Drone(
            "D-01",
            "Broken",
            Point(10, 10),
            home_base_id="B-missing",
            battery_capacity=50,
            remaining_battery=75,
        )
    )
    with pytest.raises(ProjectValidationError) as captured:
        validate_project(project)
    assert "missing home base" in str(captured.value)
    assert "remaining battery" in str(captured.value)


def test_property_update_rolls_back_invalid_value() -> None:
    service = ProjectService()
    service.project.map.bases.append(BaseStation("B-01", "Base", Point(0, 0)))
    drone = service.add_drone(Point(10, 10))
    with pytest.raises(ProjectValidationError, match="max speed"):
        service.update_property(drone.id, "max_speed", 0.0)
    assert drone.max_speed == 15.0


def test_accepts_maximum_supported_grid_dimensions() -> None:
    project = ProjectModel()
    project.map.width = 5000
    project.map.height = 5000
    project.map.grid_size = 10
    validate_project(project)

    project.map.width = 5010
    with pytest.raises(ProjectValidationError, match="500 x 500"):
        validate_project(project)
