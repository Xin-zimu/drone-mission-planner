# Drone Mission Planner

**多无人机协同任务规划与动态仿真平台** — a fully local PySide6 desktop application for composing, planning, validating, simulating, and reporting 2D multi-UAV missions. Version 1.0 is software-only: it does not connect to real aircraft or cloud services.

![Final mountain search-and-rescue dashboard](reports/screenshots/phase-08-final-rescue.png)

## Demonstration

![Search, fault containment, live replanning, and final statistics](docs/media/rescue-demo.gif)

[Download the MP4 demonstration](docs/media/rescue-demo.mp4) · [Open the generated simulation report](reports/final-simulation-report.html)

## What is included

| Area | Capabilities |
|---|---|
| Mission editor | Zoomable grid map; bases, drones, point missions, obstacles, no-fly zones, polygon/rectangular search areas; inspector and object tree |
| Route planning | Deterministic 8-connected A*, Octile heuristic, safety-radius inflation, corner-cut prevention, smoothing, and final validation |
| Assignment | Priority-first multi-drone allocation with payload, remaining battery, safe return, reserve, deadlines, and per-drone rejection reasons |
| Cooperative search | Vertical strip partitioning, obstacle-safe lawnmower passes, return to base, live coverage heatmap, and repeat-coverage metrics |
| Dynamic simulation | Fixed 0.05 s logic steps, 0.5x–10x playback, state machine, battery/distance/task integration, pause/step/reset |
| Live adaptation | Manual or seeded automatic failures, exact-position stop, unfinished-work redistribution, temporary zones, task insertion/cancellation |
| Safety | Time–space conflict prediction, priority yielding, combined safety radii, direct/multi-hop base connectivity, loss grace and auto-return |
| Reporting | Per-aircraft and system statistics, completion/coverage charts, event history, and HTML/JSON/CSV export |
| Persistence | Human-readable `.dmproj` JSON, schema migration from 1.0 to 1.1, validation, and clear corrupt/incompatible-file errors |

## Windows application

The release package contains a standalone `DroneMissionPlanner.exe`; Python is not required on the target computer. The reproducible Windows build is defined in [the PyInstaller spec](packaging/drone_mission_planner.spec) and [GitHub Actions workflow](.github/workflows/build-windows.yml).

To build locally on Windows with Python 3.12:

```powershell
./packaging/build-windows.ps1
```

The one-file EXE is written to `dist/DroneMissionPlanner.exe`.

## Run from source

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
python -m pip install -e ".[dev]"
python run.py
```

## Five-minute workflow

1. Place a base, one or more drones, and missions; drag rectangles for obstacles/no-fly zones.
2. Use **Planning → Auto assign all missions**, or draw a Search area and choose **Plan area coverage**.
3. Press Play, or use Pause, Step, Reset, and the speed selector.
4. Select a drone and press `Ctrl+Shift+F` to test state-preserving fault recovery.
5. Review Events, Safety & links, Coverage, and Statistics; press `Ctrl+E` to export a report.

Press `F1` inside the application for the quick-start guide.

## Examples

| Project | Demonstrates |
|---|---|
| `examples/inspection_demo.dmproj` | Three-drone priority inspection around buildings and a crane exclusion zone |
| `examples/delivery_demo.dmproj` | Light/cargo/heavy delivery allocation using payload and energy constraints |
| `examples/rescue_demo.dmproj` | Required final mountain search: three drones, two no-fly zones, four peaks, two checkpoints, D-02 failure, two-drone recovery, 95%+ coverage |
| `examples/safety_constraints_demo.dmproj` | Crossing-flight priority hold and a three-relay communication chain |
| `examples/fault_replanning_demo.dmproj` | Point-mission failure and redistribution from live state |

## Controls

| Action | Control |
|---|---|
| Select / inspect | Select tool, then click map or object tree |
| Add point object | Choose Base, Drone, or Mission, then click |
| Draw area object | Choose Obstacle, No-fly, or Search area, then drag |
| Pan / zoom / fit | Middle-drag or Space-drag / wheel / `F` |
| Save | `Ctrl+S` |
| Point mission planning | `Ctrl+Shift+P` |
| Cooperative coverage | `Ctrl+Shift+C` |
| Play / step | `Ctrl+Space` / `.` |
| Inject selected-drone fault | `Ctrl+Shift+F` |
| Export report | `Ctrl+E` |
| In-app guide | `F1` |

## Engineering quality

```bash
ruff check .
mypy src
pytest
python scripts/benchmark.py
```

The final suite covers geometry, rasterization, A*, smoothing, energy, assignment, coverage, event handling, state-preserving fault recovery, collision avoidance, multi-hop communication, reporting, persistence migration, UI smoke paths, all three release examples, and performance limits. See [the final test report](reports/final-test-report.md).

## Documentation

- [User guide](docs/user-guide.md) and printable Word/PDF manual
- [Architecture](docs/architecture.md)
- [Algorithms](docs/algorithms.md)
- [Developer guide](docs/developer-guide.md)
- [Data format](docs/data-format.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Project summary](docs/project-summary.md)

## Architecture

The project enforces `UI → application → domain/planning/simulation/persistence` dependency direction. Planning and simulation never depend on PySide6, all domain objects are dataclasses, deterministic behavior accepts a fixed seed, and each core feature has automated tests.

## Scope and roadmap

Version 1.0 is a local 2D planning and simulation platform. Real flight control, MAVLink/PX4, ROS 2, 3D terrain, wind, and hardware telemetry are intentionally outside this release; the project summary documents extension points.

## License

MIT
