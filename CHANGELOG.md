# Changelog

## 1.2.0 - 2026-09-15

- Completed the v1.2 real-mission planning foundation: authoritative 3D waypoints, cumulative scheduling, dependency and energy accounting, global multi-vehicle optimisation, conflict repair, georeferencing, GIS/DEM import, and target-aware route export validation.
- Added project georeferencing with local-only/georeferenced modes, WGS84/ECEF/ENU conversion, explicit height datum tracking, control-point residuals, spatial bounds, geofences, and guarded real-coordinate export.
- Added traceable GIS/DEM data sources, GeoJSON/KML vector import with topology validation and transactional apply, preserved polygon holes and multipart provenance, and GeoTIFF DEM application with NoData-aware terrain samples.
- Added PX4/QGroundControl multirotor export profiles, L0-L3 validation reports, QGC/WPL capability gates, per-waypoint speed command emission, and mission package manifests with file hashes and data-source summaries.
- Added regional and georeferenced example projects plus regression coverage for georeference migration, GIS import topology, DEM validity, route export gates, optimisation constraints, UI smoke paths, and release workflows.
- Release scope note: v1.2.0 provides automated L0-L2 style validation evidence and package smoke checks. Real ground-station import evidence and SITL log evidence remain explicit external validation follow-ups before operational use.

## 1.1.0 - 2026-09-06

- Completed the M20 productization baseline: transactional undo/redo, configurable recovery autosaves, recent projects, local settings, validation center, and atomic project saves.
- Added sparse-ID-safe object creation and referential cleanup when bases, drones, or missions are deleted.
- Fixed multi-area coverage clearing an earlier high-priority route, Assignment weights raising `KeyError` after confirmation, and stale Qt log handlers writing to destroyed windows.
- Added regression coverage for project history, recovery state, atomic writes, validation metadata, deletion cleanup, and the corrected UI workflows.
- Pinned Windows release builds to the validated Python 3.12 environment after Python 3.13 exhibited intermittent native access violations during long mixed Qt/simulation test runs.
- Fixed packaged Windows startup failure (`DLL load failed while importing QtCore`) by isolating PyInstaller from ambient DLL paths and using PySide6's VC++ 14.44 runtime consistently at the application root.
- Added an end-to-end release GUI regression covering project load, all view modes, assignment, simulation, replay, exports, recent files, undo/redo, recovery, and final validation.
- Fixed the simulation tick starving the UI thread on large projects: every 16 ms tick rebuilt the object tree, all workspace tables, the statistics report, and the full 2D/3D scene, piling up timer events at ~100% CPU until the process died with a native access violation. Ticks now step the engine at 16 ms but rebuild the UI at most every 100 ms, and the object tree is only rebuilt when its displayed content actually changes.

## 1.0.0

- Completed all eight planned development stages.
- Added editor, persistence, deterministic A*, assignment, fixed-step simulation, coverage, live fault recovery, collision/communication constraints, statistics/export, examples, docs, demo, and Windows packaging.
- Added `.dmproj` schema 1.1 with automatic migration from 1.0.
- Added the final mountain search-and-rescue acceptance scenario.

## 0.7.0

- Collision prediction, priority yielding, multi-hop communication, loss/return policies.

## 0.6.0

- Dynamic events, manual/automatic failures, state-preserving replanning.

## 0.5.0

- Cooperative area coverage and live coverage monitoring.

Earlier milestone reports remain under `reports/phase-01.md` through `phase-04.md`.
