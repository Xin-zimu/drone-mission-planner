# Execution Progress

Last updated: 2026-09-05

## Current checkpoint

M12 (real route export) is complete in the working tree and is intended to be captured by the next checkpoint commit. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Commit: `45c354c Implement M13C three-dimensional route editing`
- Commit: `efa315a Implement M13D three-dimensional simulation with waypoint actions`
- Branch: `main`

## Completed in this pass

- M13D (previous pass): 3D simulation state machine with waypoint altitude profiles, climb/descent rate limits, waypoint speed caps, hover/photo/land actions, scan-gated coverage, photo/landing events, and per-drone max altitude / min clearance / photo report fields; the rescue example was regenerated as schema 1.4 with scan waypoints.
- M12 (this pass): `persistence/route_export.py` exports one drone's 3D route as internal JSON, inspection CSV, QGroundControl `.plan` (MAVLink item sequence: takeoff/waypoint/loiter/camera/land/RTL), or ArduPilot WPL text. All four formats share `build_route_payload`, which refuses exports with critical altitude risks, insufficient battery, missing home base, or missing waypoints, attaches warning-level risk summaries, and marks every file `flyable: false` with the local-coordinate not-flyable warning.
- UI: File → Export route… (`Ctrl+Shift+E`) dispatches by file extension and surfaces validation failures with the offending drone and reason.
- Tests: 8 route-export unit tests (format content for all four writers plus every rejection branch) and 1 UI smoke test (export writes a payload, empty project is rejected).
- Documentation updated in `README.md`, `docs/user-guide.md`, `docs/future-roadmap.md`, and this file.

## Validation

- `.venv313\Scripts\python.exe -m pytest -q`: 157 passed.
- `.venv313\Scripts\python.exe -m mypy src tests`: success.
- `.venv313\Scripts\python.exe -m ruff check src tests`: all checks passed.
- `.venv313\Scripts\python.exe -m compileall -q src tests`: success.
- `git diff --check`: success.

## Remaining

1. M10 report/example polish (showcase screenshots and reports need a GUI pass), M11 local map underlay, M9-full imports (GeoJSON/KML), M14 risk scoring.
2. The 3D main line (M13A–M13D plus M12 export) is complete; remaining phases extend import breadth, basemaps, and risk/collaboration features.
