# Execution Progress

Last updated: 2026-09-05

## Current checkpoint

M13C (three-dimensional route editing) is complete in the working tree and is intended to be captured by the next checkpoint commit. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Branch: `main`
- Git worktree: should be clean after the checkpoint commit is created

## Completed in this pass

- New Waypoints tab (`ui/waypoint_panel.py`): a ten-column table over every drone waypoint (index, drone, x, y, altitude, mode, speed, action, hold, task) with combo editors for altitude mode and action, and a delete button.
- Editing is limited to altitude, mode, speed, action, and hold. `ProjectService.update_waypoint` coerces and validates the value, rolls back on `validate_project` rejection, keeps `planned_path` and `waypoints` synchronized, and marks the project dirty. Speed accepts empty/`auto` to fall back to cruise speed.
- `ProjectService.remove_waypoint` guards structural vertices: departure, task-linked, and return-to-launch waypoints cannot be deleted, and coverage scan routes only allow altitude/speed adjustments.
- `domain/waypoint.py` gained `waypoint_msl_altitude` as the single MSL/AGL conversion used by validation, rendering, and the UI.
- The altitude validator now prefers per-waypoint MSL altitudes (linearly interpolated along each leg) when the waypoint count matches the path, so edited altitudes immediately change terrain/obstacle/no-fly risks; the Altitude profile also feeds waypoint altitudes into the segment energy estimates.
- Selecting a waypoint row highlights it with an amber ring in the 2D/2.5D map (`set_waypoint_highlight`) and the 3D view (`set_highlighted_waypoint`); selecting a drone in the map or tree jumps the table to its first waypoint.
- Deferred to M13D (documented in the roadmap): simulation consuming per-waypoint speed and executing waypoint actions in the state machine.
- Tests: 7 service editing tests, 3 validator waypoint-altitude tests, and 2 Waypoints-tab UI smoke tests (render/edit/highlight plus delete guards).
- Documentation updated in `README.md`, `docs/user-guide.md`, `docs/architecture.md`, `docs/future-roadmap.md`, and this file.

## Validation

- `.venv313\Scripts\python.exe -m pytest -q`: 140 passed.
- `.venv313\Scripts\python.exe -m mypy src tests`: success.
- `.venv313\Scripts\python.exe -m ruff check src tests`: all checks passed.
- `.venv313\Scripts\python.exe -m compileall -q src tests`: success.
- Headless acceptance: lowering a mid-route waypoint to 0 m keeps the terrain risk while raising it to 400 m clears all risks; save/reload restores the edited waypoint and the synchronized point path.

## Remaining

1. Start M13D three-dimensional simulation: state machine consumes waypoint altitudes, per-segment speeds, and waypoint actions (hover, photo, scan, land, RTL).
2. Later 3D-track phase: M12 real route export (JSON/CSV/QGroundControl/ArduPilot).
