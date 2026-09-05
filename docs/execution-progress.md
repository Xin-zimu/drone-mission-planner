# Execution Progress

Last updated: 2026-09-05

## Current checkpoint

M13D (three-dimensional simulation with waypoint actions) is complete in the working tree and is intended to be captured by the next checkpoint commit. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Commit: `45c354c Implement M13C three-dimensional route editing`
- Branch: `main`

## Completed in this pass

- `DroneStatus` extended with climbing, descending, hovering, scanning, and landing; `EventType` gained waypoint photo and waypoint landing records.
- `DroneRuntime` holds waypoint profiles aligned with the path (MSL altitudes, per-vertex speed caps, actions, hold seconds). Drones whose waypoints diverge from their path automatically fall back to the legacy 2D motion model.
- The engine's fixed-step motion advances horizontal distance and altitude together: waypoint altitudes form the commanded MSL profile interpolated along each leg, vertical motion is limited by climb/descent rates, and waypoint speeds cap the horizontal ground speed (the reported flight time uses the capped leg time).
- Waypoint actions execute on vertex arrival: hover holds for the configured time with hover energy, take-photo pauses 2 s, increments the photo counter, and records a `WAYPOINT_PHOTO` event, and land descends to terrain altitude at the final vertex and records `WAYPOINT_LANDING`. Return-to-launch legs reuse the returning flow.
- Coverage contributions are gated: only runtimes currently on a leg with a scan endpoint feed the `CoverageMonitor`, satisfying "coverage only updates during scan-capable actions". The rescue example was regenerated as schema 1.4 with scan waypoints so its pre-planned sweeps keep contributing.
- Reports (HTML/CSV/JSON) gained per-drone photos taken, max altitude, and minimum clearance seen.
- Tests: 8 new simulation tests covering altitude profile with rate limits, hover energy/waiting, photo events, speed caps, landing, scan-gated coverage, AGL-to-MSL resolution, and altitude extremes; coverage tests now carry the planner's waypoints like the UI does.
- Documentation updated in `README.md`, `docs/architecture.md`, `docs/future-roadmap.md`, and this file.

## Validation

- `.venv313\Scripts\python.exe -m pytest -q`: 148 passed.
- `.venv313\Scripts\python.exe -m mypy src tests`: success.
- `.venv313\Scripts\python.exe -m ruff check src tests`: all checks passed.
- `.venv313\Scripts\python.exe -m compileall -q src tests`: success.
- `git diff --check`: success.

## Remaining

1. M12 real route export (JSON/CSV/QGroundControl `.plan`/ArduPilot WPL) — next planned phase.
2. M10 report/example polish, M11 local map underlay, M9-full imports, M14+ follow.
