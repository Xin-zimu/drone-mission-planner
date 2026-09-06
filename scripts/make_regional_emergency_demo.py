"""Build the regional emergency joint-operations showcase project.

The scenario deliberately combines most product capabilities in one editable
project: heterogeneous aircraft, relay nodes, payload/deadline constraints,
terrain and wind, mixed obstacle geometry, layered no-fly zones, and two
priority coverage sectors with exclusion holes.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_mission_planner.domain.enums import ObstacleShape, TaskType  # noqa: E402
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
from drone_mission_planner.domain.terrain import (  # noqa: E402
    TerrainPeak,
    generate_mountain_terrain,
)
from drone_mission_planner.domain.validation import validate_project  # noqa: E402
from drone_mission_planner.domain.wind import WindModel  # noqa: E402
from drone_mission_planner.persistence.project_repository import ProjectRepository  # noqa: E402


def _drone(
    drone_id: str,
    name: str,
    position: Point,
    home_base_id: str,
    *,
    max_speed: float,
    battery: float,
    energy_per_meter: float,
    payload: float,
    communication_range: float,
    cruise_altitude: float,
    role: str = "mission",
) -> Drone:
    return Drone(
        drone_id,
        name,
        position,
        home_base_id=home_base_id,
        max_speed=max_speed,
        air_speed=max_speed,
        battery_capacity=battery,
        remaining_battery=battery,
        energy_per_meter=energy_per_meter,
        payload_capacity=payload,
        communication_range=communication_range,
        safety_radius=7.0 if role == "mission" else 9.0,
        role=role,
        cruise_altitude=cruise_altitude,
        min_clearance=40.0,
        climb_rate=4.0,
        descent_rate=3.0,
        hover_power=100.0,
        climb_power=155.0,
        descent_power=45.0,
        horizontal_power=120.0,
    )


def build_project() -> ProjectModel:
    width, height = 1400, 900
    terrain = generate_mountain_terrain(
        width=width,
        height=height,
        resolution=25.0,
        base_altitude=18.0,
        peaks=[
            TerrainPeak(Point(390.0, 330.0), 180.0, 105.0),
            TerrainPeak(Point(1040.0, 310.0), 220.0, 145.0),
            TerrainPeak(Point(760.0, 760.0), 155.0, 85.0),
        ],
    )
    model = MapModel(
        width=width,
        height=height,
        grid_size=20.0,
        terrain=terrain,
        wind=WindModel(direction_to_deg=65.0, speed=8.0, gust_factor=0.3, enabled=True),
    )

    model.bases.extend(
        [
            BaseStation("B-01", "West emergency command", Point(70.0, 90.0), 480.0),
            BaseStation("B-02", "East logistics hub", Point(1320.0, 790.0), 460.0),
            BaseStation("B-03", "North field station", Point(700.0, 60.0), 420.0),
        ]
    )

    model.drones.extend(
        [
            _drone(
                "D-01",
                "Rapid medical scout",
                Point(95.0, 105.0),
                "B-01",
                max_speed=21.0,
                battery=430.0,
                energy_per_meter=0.022,
                payload=3.0,
                communication_range=330.0,
                cruise_altitude=220.0,
            ),
            _drone(
                "D-02",
                "Heavy relief lifter",
                Point(120.0, 145.0),
                "B-01",
                max_speed=13.0,
                battery=760.0,
                energy_per_meter=0.032,
                payload=15.0,
                communication_range=360.0,
                cruise_altitude=235.0,
            ),
            _drone(
                "D-03",
                "Multispectral mapper",
                Point(105.0, 185.0),
                "B-01",
                max_speed=17.0,
                battery=520.0,
                energy_per_meter=0.024,
                payload=5.0,
                communication_range=340.0,
                cruise_altitude=245.0,
            ),
            _drone(
                "D-04",
                "East response quad",
                Point(1290.0, 760.0),
                "B-02",
                max_speed=18.0,
                battery=500.0,
                energy_per_meter=0.024,
                payload=4.0,
                communication_range=330.0,
                cruise_altitude=225.0,
            ),
            _drone(
                "D-05",
                "Long-range inspector",
                Point(675.0, 85.0),
                "B-03",
                max_speed=25.0,
                battery=650.0,
                energy_per_meter=0.019,
                payload=2.0,
                communication_range=440.0,
                cruise_altitude=270.0,
            ),
            _drone(
                "D-06",
                "All-weather workhorse",
                Point(725.0, 90.0),
                "B-03",
                max_speed=16.0,
                battery=610.0,
                energy_per_meter=0.026,
                payload=8.0,
                communication_range=390.0,
                cruise_altitude=240.0,
            ),
            _drone(
                "D-07",
                "West relay",
                Point(500.0, 455.0),
                "B-01",
                max_speed=14.0,
                battery=850.0,
                energy_per_meter=0.018,
                payload=1.0,
                communication_range=560.0,
                cruise_altitude=280.0,
                role="relay",
            ),
            _drone(
                "D-08",
                "East relay",
                Point(955.0, 475.0),
                "B-02",
                max_speed=14.0,
                battery=850.0,
                energy_per_meter=0.018,
                payload=1.0,
                communication_range=560.0,
                cruise_altitude=280.0,
                role="relay",
            ),
        ]
    )

    model.obstacles.extend(
        [
            Obstacle("O-01", "Hospital tower", bounds=Rect(330.0, 210.0, 110.0, 140.0), height=115.0),
            Obstacle("O-02", "Railway yard", bounds=Rect(545.0, 405.0, 190.0, 90.0), height=48.0),
            Obstacle(
                "O-03",
                "Water storage tank",
                shape=ObstacleShape.CIRCLE,
                bounds=Rect(930.0, 215.0, 110.0, 110.0),
                radius=55.0,
                height=72.0,
            ),
            Obstacle(
                "O-04",
                "Collapsed industrial blocks",
                shape=ObstacleShape.POLYGON,
                bounds=Rect(990.0, 555.0, 180.0, 140.0),
                points=[
                    Point(990.0, 590.0),
                    Point(1050.0, 555.0),
                    Point(1165.0, 585.0),
                    Point(1140.0, 690.0),
                    Point(1020.0, 675.0),
                ],
                height=64.0,
            ),
            Obstacle("O-05", "Evacuation stadium", bounds=Rect(220.0, 620.0, 175.0, 120.0), height=38.0),
            Obstacle("O-06", "Power substation", bounds=Rect(755.0, 675.0, 135.0, 105.0), height=58.0),
        ]
    )

    model.no_fly_zones.extend(
        [
            NoFlyZone(
                "N-01",
                "Airport emergency corridor",
                bounds=Rect(585.0, 120.0, 290.0, 115.0),
                ceiling_altitude=320.0,
            ),
            NoFlyZone(
                "N-02",
                "Chemical plume exclusion",
                shape=ObstacleShape.POLYGON,
                bounds=Rect(825.0, 420.0, 190.0, 165.0),
                points=[
                    Point(825.0, 455.0),
                    Point(900.0, 420.0),
                    Point(1005.0, 465.0),
                    Point(980.0, 575.0),
                    Point(860.0, 585.0),
                ],
                ceiling_altitude=400.0,
            ),
            NoFlyZone(
                "N-03",
                "Dam control airspace",
                bounds=Rect(625.0, 535.0, 130.0, 105.0),
                ceiling_altitude=260.0,
            ),
        ]
    )

    model.tasks.extend(
        [
            MissionTask("T-01", "Trauma medicine drop", Point(255.0, 175.0), TaskType.DELIVERY, 10, required_payload=2.0, deadline=220.0, execution_duration=8.0, target_altitude=205.0),
            MissionTask("T-02", "Bridge structural inspection", Point(490.0, 305.0), TaskType.INSPECTION, 9, deadline=300.0, execution_duration=12.0, target_altitude=230.0),
            MissionTask("T-03", "Shelter bulk supplies", Point(515.0, 795.0), TaskType.DELIVERY, 9, required_payload=9.0, deadline=520.0, execution_duration=15.0, target_altitude=220.0),
            MissionTask("T-04", "North ridge thermal sweep", Point(1115.0, 165.0), TaskType.INSPECTION, 8, deadline=420.0, execution_duration=10.0, target_altitude=285.0),
            MissionTask("T-05", "Eastern levee survey", Point(1245.0, 455.0), TaskType.WAYPOINT, 8, execution_duration=8.0, target_altitude=240.0),
            MissionTask("T-06", "Fire-retardant payload", Point(1080.0, 765.0), TaskType.DELIVERY, 8, required_payload=6.0, deadline=500.0, execution_duration=14.0, target_altitude=225.0),
            MissionTask("T-07", "Rail junction assessment", Point(765.0, 345.0), TaskType.INSPECTION, 7, execution_duration=10.0, target_altitude=245.0),
            MissionTask("T-08", "Communications mast photo", Point(900.0, 300.0), TaskType.INSPECTION, 7, execution_duration=18.0, target_altitude=290.0),
            MissionTask("T-09", "West road clearance", Point(165.0, 465.0), TaskType.WAYPOINT, 6, execution_duration=7.0, target_altitude=205.0),
            MissionTask("T-10", "Southern shelter inspection", Point(430.0, 845.0), TaskType.INSPECTION, 6, execution_duration=10.0, target_altitude=215.0),
            MissionTask("T-11", "Dam downstream sample", Point(900.0, 825.0), TaskType.DELIVERY, 6, required_payload=3.5, execution_duration=9.0, target_altitude=225.0),
            MissionTask("T-12", "East access-road mapping", Point(1270.0, 835.0), TaskType.WAYPOINT, 5, execution_duration=6.0, target_altitude=210.0),
            MissionTask("T-13", "Central weather sensor", Point(590.0, 655.0), TaskType.DELIVERY, 5, required_payload=1.5, execution_duration=12.0, target_altitude=235.0),
        ]
    )

    model.search_areas.extend(
        [
            SearchArea(
                "S-01",
                "Priority flood-rescue sector",
                bounds=Rect(145.0, 515.0, 500.0, 340.0),
                points=[
                    Point(145.0, 545.0),
                    Point(595.0, 515.0),
                    Point(645.0, 815.0),
                    Point(365.0, 855.0),
                    Point(170.0, 770.0),
                ],
                holes=[
                    [
                        Point(205.0, 605.0),
                        Point(410.0, 605.0),
                        Point(410.0, 755.0),
                        Point(205.0, 755.0),
                    ]
                ],
                scan_spacing=32.0,
                boundary_margin=12.0,
                target_coverage=0.98,
                priority=10,
                scan_direction="horizontal",
            ),
            SearchArea(
                "S-02",
                "Secondary infrastructure mapping",
                bounds=Rect(915.0, 130.0, 390.0, 330.0),
                points=[
                    Point(925.0, 150.0),
                    Point(1275.0, 130.0),
                    Point(1305.0, 420.0),
                    Point(1120.0, 460.0),
                    Point(930.0, 405.0),
                ],
                holes=[
                    [
                        Point(920.0, 205.0),
                        Point(1050.0, 205.0),
                        Point(1050.0, 335.0),
                        Point(920.0, 335.0),
                    ]
                ],
                scan_spacing=38.0,
                boundary_margin=10.0,
                target_coverage=0.95,
                priority=6,
                scan_direction="vertical",
            ),
        ]
    )

    project = ProjectModel(
        name="Regional disaster-response joint operation",
        map=model,
        planning_settings={
            "mission_mode": "point_tasks",
            "assignment_weight_energy": 20.0,
            "assignment_weight_distance": 0.2,
            "assignment_weight_battery_risk": 110.0,
            "assignment_weight_task_load": 100.0,
            "assignment_weight_deadline": 1.4,
        },
        simulation_settings={
            "fixed_dt": 0.05,
            "random_seed": 20260905,
            "communication_policy": "auto_return",
            "communication_grace": 12.0,
        },
    )
    validate_project(project)
    return project


def main() -> int:
    project = build_project()
    target = ProjectRepository().save(
        project,
        ROOT / "examples" / "regional_emergency_demo.dmproj",
    )
    model = project.map
    print(f"Saved {target}")
    print(
        f"{len(model.bases)} bases | {len(model.drones)} drones "
        f"({sum(drone.role == 'relay' for drone in model.drones)} relays) | "
        f"{len(model.tasks)} missions | {len(model.obstacles)} obstacles | "
        f"{len(model.no_fly_zones)} no-fly zones | {len(model.search_areas)} search areas"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
