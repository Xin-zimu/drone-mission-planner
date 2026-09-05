# Execution Progress

Last updated: 2026-09-05

## Current checkpoint

M19 (explainable planning) is complete in the working tree and is intended to be captured by the next checkpoint commit. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Commit: `45c354c Implement M13C three-dimensional route editing`
- Commit: `efa315a Implement M13D three-dimensional simulation with waypoint actions`
- Commit: `b6ed844 Implement M12 real route export`
- Commit: `7e4e2a3 Implement M14 route risk assessment`
- Commit: `6ea4008 Implement M9-full mission data import`
- Commit: `1f7a168 Implement M15 simulation replay system`
- Branch: `main`

## Completed in this pass

- M19 (this pass): configurable `AssignmentWeights` (defaults preserve the legacy ranking) accepted by `GreedyAssignmentPlanner`; `explain_assignments` scores every pending task/drone pair with the same feasibility rules and rejection reasons; `build_assignment_suggestions` turns those reasons into actionable tips. The Assignments table shows the top candidates per task in row tooltips; Planning → Assignment weights… edits and persists scalar weights into `planning_settings`; `SimulationReport.assignment_notes` embeds the assignment rationale in exported reports.
- M15 (previous pass): `simulation/replay.py` records fixed-interval 3D snapshots inside the engine step (per-drone x/y/z/status/battery, coverage, processed-event count), exports replay JSON, and supports event-time jumps. The Replay workspace tab drives a timeline slider that shows historical drone positions as amber rings in the 3D view (labels marked "replay"); live position pushes pause while a replay is active, and Exit replay restores the live scene. Double-clicking an Events row jumps to that event's frame; File → Export replay… writes the JSON. Recording is read-only — a dedicated test proves it cannot change the deterministic simulation result.
- M9-full (previous pass): `persistence/mission_import.py` parses GeoJSON (Polygon/Point/LineString), basic KML placemarks, and waypoint CSVs into a preview with counts, out-of-bounds/duplicate/empty-geometry warnings, and line-numbered errors; `apply_import` creates the objects through ProjectService (polygons keep their points, geometry routes get cruise/clearance altitudes, waypoint routes require a target drone). `ProjectService.replace_waypoints` replaces a whole waypoint list with validation and rollback. UI entry: File → Import mission data…
- M14 (previous pass): `planning/risk_assessment.py` scores every route from battery, communication, terrain, airspace, and action factors (warning 8 / critical 25 penalty points, 0-100 score; any critical factor forces the critical level). Terrain/airspace factors reuse the altitude validator with segment/object attribution; battery compares route energy (incl. hover holds) against remaining capacity with a 15% reserve warning; communication compares the farthest route point with the effective radio range (110% exceedance is critical); action factors flag long hovers and routes that never return.
- `build_risk_matrix` produces a per-drone matrix; the simulation report (JSON/CSV/HTML) embeds it; the Altitude profile table gains a Route risk column with factor tooltips.
- M12 export validation now reuses the same assessment: terrain/airspace criticals or a battery critical refuse the export, and payloads carry risk_score / risk_level / risk_factors.
- Tests: 5 explainability unit tests, 6 replay tests, 1 replay UI smoke, 7 mission-import tests, 9 risk-assessment tests.

## Validation

- `.venv313\Scripts\python.exe -m pytest -q`: 184 passed.
- `.venv313\Scripts\python.exe -m mypy src tests`: success.
- `.venv313\Scripts\python.exe -m ruff check src tests`: all checks passed.
- `.venv313\Scripts\python.exe -m compileall -q src tests`: success.
- `git diff --check`: success.

## Previous passes (short)

- M12: real route export (JSON/CSV/QGC plan/WPL) with pre-export validation.
- M13D: 3D simulation state machine (altitude profiles, rate limits, speed caps, hover/photo/land actions, scan-gated coverage).
- M13C: Waypoints tab editing with guards and dual-representation sync.
- M13B: software-rendered 3D mission view with orbit camera, layers, presets, picking.
- M13A: Waypoint domain model, schema 1.4, planning/persistence integration.

## Remaining

1. M9-full imports (GeoJSON/KML/waypoint CSV) — next planned phase.
2. M15 replay, M19 explainability, M10 report/example polish (GUI screenshots pending), M11 basemap, M16+.
