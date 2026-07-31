# Drone Mission Planner

**多无人机协同任务规划与动态仿真平台**

Drone Mission Planner is a fully local desktop application for composing 2D multi-UAV missions, validating constraints, planning routes, and simulating execution. Version 1 does not connect to real aircraft or cloud services.

## Current milestone

Phase 1 — foundation and map editor.

- Polished PySide6 desktop shell
- Zoomable and pannable grid map
- Base, drone, rectangular obstacle, and mission-point editing
- Object tree and editable property inspector
- Versioned `.dmproj` JSON save/load
- Structured logging and deterministic IDs
- Automated domain, persistence, coordinate, and UI smoke tests

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

## Roadmap

The project follows eight independently archived phases: editor, A* planning, multi-UAV assignment, dynamic simulation, coverage planning, fault replanning, collision/communication constraints, and final optimization/packaging.

## License

MIT

