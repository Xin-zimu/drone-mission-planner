from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from statistics import median
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drone_mission_planner.domain.geometry import Point, Rect  # noqa: E402
from drone_mission_planner.domain.models import (  # noqa: E402
    BaseStation,
    Drone,
    MapModel,
    MissionTask,
    Obstacle,
)
from drone_mission_planner.persistence.project_repository import ProjectRepository  # noqa: E402
from drone_mission_planner.planning.assignment import GreedyAssignmentPlanner  # noqa: E402
from drone_mission_planner.planning.grid import GridMap  # noqa: E402
from drone_mission_planner.planning.route_planner import RoutePlanner  # noqa: E402
from drone_mission_planner.simulation.engine import SimulationEngine  # noqa: E402


def timed(callable_: object, *, repeats: int = 1) -> float:
    values: list[float] = []
    for _ in range(repeats):
        start = perf_counter()
        callable_()  # type: ignore[operator]
        values.append(perf_counter() - start)
    return median(values)


def run_benchmarks() -> dict[str, float | int]:
    route_map = MapModel(width=1000, height=700, grid_size=10)
    route_map.obstacles.extend(
        [
            Obstacle("O-01", "Block", bounds=Rect(300, 100, 120, 420)),
            Obstacle("O-02", "Block", bounds=Rect(600, 200, 100, 420)),
        ]
    )
    drone = Drone("D-01", "Benchmark", Point(40, 40), safety_radius=5)
    route_seconds = timed(lambda: RoutePlanner().plan(route_map, drone, Point(940, 640)), repeats=5)

    max_grid = MapModel(width=5000, height=5000, grid_size=10)
    max_grid.obstacles.append(Obstacle("O-01", "Large", bounds=Rect(1000, 1000, 500, 2500)))
    grid_seconds = timed(lambda: GridMap.from_map(max_grid, safety_radius=8), repeats=3)

    load_seconds = timed(
        lambda: ProjectRepository().load(ROOT / "examples" / "rescue_demo.dmproj"), repeats=10
    )

    simulation_map = MapModel(width=1000, height=700, grid_size=25)
    simulation_map.bases.append(BaseStation("B-01", "Base", Point(20, 20), 2000))
    for index in range(20):
        y = 40 + index * 28
        simulation_map.drones.append(
            Drone(
                f"D-{index + 1:02d}",
                f"Drone {index + 1}",
                Point(30, y),
                "B-01",
                communication_range=2000,
                safety_radius=3,
                planned_path=[Point(30, y), Point(950, y)],
            )
        )
    engine = SimulationEngine(simulation_map)
    simulation_seconds = timed(lambda: [engine.step_once() for _ in range(100)], repeats=1)

    assignment_map = MapModel(width=1000, height=700, grid_size=50)
    assignment_map.bases.append(BaseStation("B-01", "Base", Point(25, 25), 2000))
    for index in range(20):
        assignment_map.drones.append(
            Drone(
                f"D-{index + 1:02d}",
                f"Drone {index + 1}",
                Point(25 + index * 2, 25),
                "B-01",
                battery_capacity=1_000_000,
                remaining_battery=1_000_000,
                energy_per_meter=0.001,
                communication_range=2000,
            )
        )
    for index in range(200):
        assignment_map.tasks.append(
            MissionTask(
                f"T-{index + 1:03d}",
                f"Task {index + 1}",
                Point(75 + (index % 18) * 50, 75 + (index // 18) * 50),
                priority=index % 10,
            )
        )
    assignment_seconds = timed(lambda: GreedyAssignmentPlanner().assign(assignment_map))
    return {
        "route_planning_median_seconds": route_seconds,
        "grid_500x500_seconds": grid_seconds,
        "rescue_project_load_median_seconds": load_seconds,
        "simulation_20_drones_100_steps_seconds": simulation_seconds,
        "assignment_20_drones_200_tasks_seconds": assignment_seconds,
        "python_version": sys.version.split()[0],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    results = run_benchmarks()
    rendered = json.dumps(results, indent=2)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
