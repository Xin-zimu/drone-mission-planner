# Drone Mission Planner

**多无人机协同任务规划与动态仿真平台**

Drone Mission Planner is a fully local desktop application for composing 2D multi-UAV missions, validating constraints, planning routes, and simulating execution. Version 1 does not connect to real aircraft or cloud services.

## Current milestone

Phase 6 — fault events and state-preserving dynamic replanning.

- Polished PySide6 desktop shell and zoomable map editor
- Base, drone, obstacle, no-fly-zone, and mission-point editing
- Object tree and editable property inspector
- Versioned `.dmproj` JSON save/load
- Structured logging and deterministic IDs
- Deterministic 8-connected A* with Octile heuristic
- Per-drone obstacle inflation, line-of-sight smoothing, and final validation
- Distance, time, energy, expanded-node, and clear failure reporting
- Priority-first multi-drone greedy assignment
- Payload and safe-return battery validation for every candidate
- Simultaneous color-coded routes, assignment table, and manual reassignment field
- Deterministic 0.05 s simulation steps independent from UI frame rate
- Takeoff, flight, task execution, return, and completion state transitions
- Play, pause, single-step, reset, and 0.5x–10x wall-clock speed controls
- Live position, battery, task status, distance, and timing statistics
- Rectangular or polygonal search areas with editable scan spacing and boundary margin
- Deterministic vertical-strip partitioning across the available drone fleet
- Obstacle-safe alternating lawnmower passes with automatic return to base
- Live covered-cell heatmap, target coverage, and cross-drone repeat-coverage metrics
- Manual and deterministic scheduled drone-failure events
- Immediate failed-aircraft stop with visible failure reason and event history
- Automatic task or coverage redistribution from live positions
- Replanning that preserves simulation time, battery, distance, and completed work
- Dynamic mission insertion, cancellation, and temporary no-fly-zone replanning

## Quick start

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python run.py
```

On Linux or macOS, activate with `source .venv/bin/activate`.

## Controls

| Action | Control |
|---|---|
| Select an object | Select tool, then left-click |
| Add an object | Choose a placement tool, then click the map |
| Draw an obstacle | Choose Obstacle, then drag a rectangle |
| Draw a search area | Choose Search area, then drag a rectangle |
| Plan cooperative sweep | `Ctrl+Shift+C` or Planning → Plan area coverage |
| Fail a drone | Select it, then `Ctrl+Shift+F` |
| Schedule a fault | Simulation → Schedule automatic failure |
| Insert during simulation | Use Mission or No-fly while a simulation exists |
| Cancel a mission | Select it, then Simulation → Cancel selected mission |
| Pan | Middle mouse drag, or hold Space and drag |
| Zoom | Mouse wheel |
| Delete | Delete tool and click, or select and press Delete |
| Save | `Ctrl+S` |

## Quality checks

```bash
python -m pytest
ruff check .
mypy src
```

Open `examples/coverage_demo.dmproj` for cooperative search, or `examples/fault_replanning_demo.dmproj` for live fault recovery.

## Roadmap

The project follows eight independently archived phases: editor, A* planning, multi-UAV assignment, dynamic simulation, coverage planning, fault replanning, collision/communication constraints, and final optimization/packaging.

## License

MIT
