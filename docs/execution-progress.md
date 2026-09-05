# Execution Progress

Last updated: 2026-09-05

## Current checkpoint

M14 (route risk assessment) is complete in the working tree and is intended to be captured by the next checkpoint commit. Earlier checkpoints:

- Commit: `7ca7778 Initial project import`
- Commit: `a7c0e40 Complete M8 altitude and incremental replanning`
- Commit: `ced0ea7 Revise roadmap around 3D-first plan`
- Commit: `2a222e0 Implement M9-lite terrain CSV import`
- Commit: `1a03418 Implement M13A three-dimensional waypoint model`
- Commit: `cdd3a7e Implement M13B true 3D mission view`
- Commit: `45c354c Implement M13C three-dimensional route editing`
- Commit: `efa315a Implement M13D three-dimensional simulation with waypoint actions`
- Commit: `b6ed844 Implement M12 real route export`
- Branch: `main`

## Completed in this pass

- M14 (this pass): `planning/risk_assessment.py` scores every route from battery, communication, terrain, airspace, and action factors (warning 8 / critical 25 penalty points, 0-100 score; any critical factor forces the critical level). Terrain/airspace factors reuse the altitude validator with segment/object attribution; battery compares route energy (incl. hover holds) against remaining capacity with a 15% reserve warning; communication compares the farthest route point with the effective radio range (110% exceedance is critical); action factors flag long hovers and routes that never return.
- `build_risk_matrix` produces a per-drone matrix; the simulation report (JSON/CSV/HTML) embeds it; the Altitude profile table gains a Route risk column with factor tooltips.
- M12 export validation now reuses the same assessment: terrain/airspace criticals or a battery critical refuse the export, and payloads carry risk_score / risk_level / risk_factors.
- Tests: 9 risk-assessment unit tests; export tests updated for the shared model.
- The same model is the designated source for M19 scoring displays.

## Validation

- `.venv313\Scripts\python.exe -m pytest -q`: 165 passed.
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
