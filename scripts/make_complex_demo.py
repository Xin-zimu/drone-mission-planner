"""Build a high-complexity mixed workflow demo project.

Scenario: an urban logistics + rescue drill on a 1000 x 700 m map with
  - two bases (main base + forward outpost),
  - five differentiated drones (heavy lifter, fast scout, workhorse,
    outpost drone, long-endurance relay),
  - eight point missions with mixed priority / deadline / payload,
  - four obstacle buildings and one permanent no-fly zone,
  - one large polygon search area for cooperative coverage.

The project is intentionally authored through the domain model so every
value passes validation and nothing is a hard-coded planning result.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_mission_planner.domain.enums import TaskType  # noqa: E402
from drone_mission_planner.domain.geometry import Point, Rect  # noqa: E402
from drone_mission_planner.domain.models import (  # noqa: E402
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    NoFlyZone,
    Obstacle,
    ProjectModel,
    SearchArea,
)
from drone_mission_planner.persistence.project_repository import ProjectRepository  # noqa: E402


def build_project() -> ProjectModel:
    model = MapModel(width=1000, height=700, grid_size=20)

    # --- Bases -----------------------------------------------------------------
    model.bases.extend(
        [
            BaseStation("B-01", "Main base", Point(60, 60), communication_range=420.0),
            BaseStation("B-02", "Forward outpost", Point(920, 620), communication_range=360.0),
        ]
    )

    # --- Drones: one per role --------------------------------------------------
    model.drones.extend(
        [
            Drone(
                "D-01",
                "Heavy lifter",
                Point(90, 70),
                home_base_id="B-01",
                max_speed=10.0,
                battery_capacity=200.0,
                remaining_battery=200.0,
                energy_per_meter=0.03,
                payload_capacity=12.0,
                communication_range=300.0,
                safety_radius=6.0,
            ),
            Drone(
                "D-02",
                "Fast scout",
                Point(110, 85),
                home_base_id="B-01",
                max_speed=18.0,
                battery_capacity=100.0,
                remaining_battery=100.0,
                energy_per_meter=0.025,
                payload_capacity=2.0,
                communication_range=260.0,
                safety_radius=5.0,
            ),
            Drone(
                "D-03",
                "Workhorse",
                Point(90, 105),
                home_base_id="B-01",
                max_speed=14.0,
                battery_capacity=140.0,
                remaining_battery=140.0,
                energy_per_meter=0.025,
                payload_capacity=5.0,
                communication_range=300.0,
                safety_radius=6.0,
            ),
            Drone(
                "D-04",
                "Outpost drone",
                Point(890, 590),
                home_base_id="B-02",
                max_speed=13.0,
                battery_capacity=120.0,
                remaining_battery=120.0,
                energy_per_meter=0.028,
                payload_capacity=4.0,
                communication_range=280.0,
                safety_radius=5.0,
            ),
            Drone(
                "D-05",
                "Relay (long endurance)",
                Point(860, 570),
                home_base_id="B-02",
                max_speed=12.0,
                battery_capacity=180.0,
                remaining_battery=180.0,
                energy_per_meter=0.02,
                payload_capacity=3.0,
                communication_range=520.0,
                safety_radius=6.0,
            ),
        ]
    )

    # --- Obstacles: four buildings --------------------------------------------
    model.obstacles.extend(
        [
            Obstacle("O-01", "Office block", bounds=Rect(300, 150, 120, 100)),
            Obstacle("O-02", "Warehouse", bounds=Rect(600, 300, 90, 140)),
            Obstacle("O-03", "Market hall", bounds=Rect(150, 450, 100, 80)),
            Obstacle("O-04", "Depot", bounds=Rect(750, 450, 130, 90)),
        ]
    )

    # --- No-fly zones ----------------------------------------------------------
    model.no_fly_zones.append(
        NoFlyZone("N-01", "Residential no-fly", bounds=Rect(420, 180, 140, 100))
    )

    # --- Point missions: mixed priority / deadline / payload -------------------
    model.tasks.extend(
        [
            MissionTask(
                "T-01",
                "Checkpoint Alpha",
                Point(500, 80),
                task_type=TaskType.INSPECTION,
                priority=10,
                deadline=120.0,
                execution_duration=5.0,
            ),
            MissionTask(
                "T-02",
                "Checkpoint Beta",
                Point(890, 280),
                task_type=TaskType.INSPECTION,
                priority=9,
                deadline=150.0,
                execution_duration=5.0,
            ),
            MissionTask(
                "T-03",
                "Express parcel",
                Point(700, 90),
                task_type=TaskType.DELIVERY,
                priority=7,
                required_payload=2.0,
                execution_duration=3.0,
            ),
            MissionTask(
                "T-04",
                "Heavy freight",
                Point(300, 300),
                task_type=TaskType.DELIVERY,
                priority=7,
                required_payload=6.0,
                execution_duration=6.0,
            ),
            MissionTask(
                "T-05",
                "Urgent medical delivery",
                Point(200, 620),
                task_type=TaskType.DELIVERY,
                priority=8,
                required_payload=3.0,
                deadline=100.0,
                execution_duration=4.0,
            ),
            MissionTask(
                "T-06",
                "Routine inspection",
                Point(550, 550),
                task_type=TaskType.INSPECTION,
                priority=5,
                execution_duration=4.0,
            ),
            MissionTask(
                "T-07",
                "Perimeter check",
                Point(860, 600),
                task_type=TaskType.INSPECTION,
                priority=4,
                execution_duration=4.0,
            ),
            MissionTask(
                "T-08",
                "Low-priority survey",
                Point(120, 300),
                task_type=TaskType.WAYPOINT,
                priority=3,
                execution_duration=2.0,
            ),
        ]
    )

    # --- Search area: polygon with an obstacle inside to demonstrate -----------
    # --- accessible-cell denominators and detour-aware strips ------------------
    model.search_areas.append(
        SearchArea(
            "S-01",
            "Drill search sector",
            bounds=Rect(450, 300, 430, 260),
            points=[
                Point(450, 300),
                Point(880, 300),
                Point(880, 560),
                Point(650, 560),
                Point(450, 440),
            ],
            scan_spacing=50.0,
            boundary_margin=10.0,
            target_coverage=0.95,
        )
    )

    project = ProjectModel(
        name="Complex logistics & rescue drill",
        map=model,
        planning_settings={"mission_mode": "point_tasks"},
        simulation_settings={
            "fixed_dt": 0.05,
            "random_seed": 42,
            "communication_policy": "auto_return",
            "communication_grace": 10.0,
        },
    )
    return project


def main() -> int:
    project = build_project()
    target = ProjectRepository().save(project, ROOT / "examples" / "complex_workflow_demo.dmproj")
    map_model = project.map
    print(f"Saved {target}")
    print(
        f"Map {map_model.width}x{map_model.height} m, grid {map_model.grid_size} m | "
        f"{len(map_model.bases)} bases | {len(map_model.drones)} drones | "
        f"{len(map_model.tasks)} missions | {len(map_model.obstacles)} obstacles | "
        f"{len(map_model.no_fly_zones)} no-fly zones | {len(map_model.search_areas)} search areas"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
