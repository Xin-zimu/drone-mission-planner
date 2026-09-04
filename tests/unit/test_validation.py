from __future__ import annotations

import pytest

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import (
    BaseStation,
    Drone,
    MissionTask,
    NoFlyZone,
    Obstacle,
    ProjectModel,
)
from drone_mission_planner.domain.terrain import TerrainModel, TerrainPeak
from drone_mission_planner.domain.validation import ProjectValidationError, validate_project
from drone_mission_planner.domain.wind import WindModel


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


def test_rejects_invalid_environment_models() -> None:
    project = ProjectModel()
    project.map.terrain.peaks.append(TerrainPeak(Point(100.0, 100.0), 0.0, 50.0))
    project.map.wind = WindModel(direction_to_deg=45.0, speed=-1.0)

    with pytest.raises(ProjectValidationError) as captured:
        validate_project(project)

    assert "terrain peak 1 radius" in str(captured.value)
    assert "wind speed" in str(captured.value)


def test_rejects_invalid_grid_terrain_model() -> None:
    project = ProjectModel()
    project.map.terrain = TerrainModel(
        terrain_type="grid",
        resolution=10.0,
        min_altitude=0.0,
        max_altitude=10.0,
        grid_origin=Point(0.0, 0.0),
        grid_width=2,
        grid_height=1,
        grid_altitudes=[[5.0]],
    )

    with pytest.raises(ProjectValidationError) as captured:
        validate_project(project)

    assert "terrain grid row 1 width" in str(captured.value)
    assert "terrain grid altitude range" in str(captured.value)


def test_rejects_invalid_drone_flight_parameters() -> None:
    project = ProjectModel()
    project.map.bases.append(BaseStation("B-01", "Base", Point(0, 0)))
    project.map.drones.append(
        Drone("D-01", "Broken", Point(10, 10), "B-01", climb_rate=0.0, hover_power=-1.0)
    )

    with pytest.raises(ProjectValidationError) as captured:
        validate_project(project)

    assert "climb/descent speeds" in str(captured.value)
    assert "flight power settings" in str(captured.value)


def test_rejects_invalid_altitude_fields() -> None:
    project = ProjectModel()
    project.map.tasks.append(MissionTask("T-01", "Task", Point(10, 10), target_altitude=-1.0))
    project.map.obstacles.append(Obstacle("O-01", "Obstacle", height=-1.0))
    project.map.no_fly_zones.append(NoFlyZone("N-01", "Zone", ceiling_altitude=-1.0))

    with pytest.raises(ProjectValidationError) as captured:
        validate_project(project)

    assert "target altitude" in str(captured.value)
    assert "height" in str(captured.value)
    assert "ceiling altitude" in str(captured.value)
