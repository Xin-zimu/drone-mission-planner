# Developer guide

## Environment

Use Python 3.12. Create a virtual environment and install `.[dev]`. The runtime dependencies are pinned by compatible major versions in `pyproject.toml`; `requirements.txt` contains the minimal application set.

## Dependency direction

| Package | May depend on | Must not depend on |
|---|---|---|
| `domain` | Standard library | Qt, persistence, UI |
| `planning` | Domain | Qt, UI |
| `simulation` | Domain, planning | Qt, UI |
| `persistence` | Domain, migrations | Qt, UI |
| `app` | Domain, persistence | Graphics widgets |
| `ui` | All application packages | — |

Algorithms accept domain dataclasses and return typed result objects with explicit success/failure details. `SimulationEngine` owns mutable runtime state; snapshots exposed to the UI are immutable. All random behavior is derived from the project seed.

## Important flows

### Point mission

1. `GreedyAssignmentPlanner` orders unfinished tasks.
2. Each aircraft candidate is checked for status, payload, outbound/return reachability, energy, and reserve.
3. `RoutePlanner` rasterizes with the candidate's safety radius and runs A*.
4. `MainWindow` applies assignment IDs and paths, then constructs the engine.

### Coverage mission

1. `CoveragePlanner` intersects scanlines with the search polygon.
2. Vertical strips are assigned deterministically to operational drones.
3. Pass endpoints and return legs use the same validated A* planner.
4. `CoverageMonitor` tracks accessible cells and distinct visiting drone IDs.

### Fault recovery

1. `EventManager` releases due events inside fixed logical steps.
2. The engine marks the failed runtime, freezes it, and releases unfinished work.
3. The application synchronizes current position/battery/status into the domain model.
4. Point assignment or area coverage runs again over operational drones.
5. `apply_replan` replaces future path state without resetting time/statistics/history.

### Safety

`ConflictDetector` samples future motion and returns yielding IDs. `CommunicationMonitor` builds a bidirectional range graph, calculates base-hop counts, and issues a one-shot auto-return request after the configured grace period.

## Project format changes

Change `CURRENT_VERSION`, add a migration in `persistence/migrations.py`, retain older fixtures, and add round-trip plus migration tests. Never silently reinterpret unknown future versions.

## Tests and benchmarks

```bash
ruff format --check .
ruff check .
mypy src
pytest --junitxml=reports/junit.xml
python scripts/benchmark.py --output reports/performance.json
```

Qt smoke tests use `pytest-qt`; algorithm tests do not import PySide6. Performance tests use generous release limits and the benchmark script records more detailed measurements.

## Windows release

`packaging/drone_mission_planner.spec` creates a one-file, windowed PyInstaller binary and embeds assets, examples, docs, icon, and version metadata. Run `packaging/build-windows.ps1`, or dispatch `.github/workflows/build-windows.yml`. Do not commit `build/`, `dist/`, or local environments.
