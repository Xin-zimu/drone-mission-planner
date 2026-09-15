# `.dmproj` data format

The application uses UTF-8 JSON with a required top-level `version` field. The extension is `.dmproj`.

Required top-level members are `version`, `name`, `georeference`, `data_sources`, `map`, `planning_settings`, and `simulation_settings`. Object IDs are stable strings such as `B-01`, `D-01`, `O-01`, and `T-01`.

The loader rejects malformed JSON and unknown versions with a user-facing error. Future schema changes must add a migration before changing `CURRENT_VERSION`.

Version 1.10 adds top-level `georeference`. Legacy projects migrate to `mode: "local_only"` with no origin, so existing local metre `x/y` values are never treated as latitude/longitude. A georeferenced project stores an `origin` (`latitude_deg`, `longitude_deg`, `height_m`), an explicit `horizontal_crs`, a `height_reference` (`unknown`, `ellipsoid`, `orthometric`, `agl`, or `relative_home`), optional `valid_radius_m`, `control_points`, optional `spatial_bounds`, `geofences`, `validation_status` (`unknown`, `validated`, or `stale`), and a `revision` fingerprint. Internal planning remains local ENU metres: `x = east`, `y = north`, `z = up`; Qt screen-space y is inverted only in the UI layer. Real-coordinate export must reject or mark unverified projects whose mode is local-only, whose height datum is unknown, or whose georeference validation status is not `validated`.

Control points store both the expected local ENU coordinate and the observed geodetic coordinate so calibration residuals can be queried without rewriting the mission geometry. `spatial_bounds` and `geofences` are stored in local ENU metres and validated against project objects during normal project validation. When the project origin changes, all local mission coordinates are re-anchored through their real geodetic positions, georeference-scoped local shapes are reprojected, the validation status becomes `stale`, and downstream cached planning state is invalidated.

Version 1.11 adds top-level `data_sources` for traceable GIS/DEM resources. Each entry stores the source path, source format, source CRS, source-coordinate bounds, converted local ENU bounds when verifiable, horizontal/vertical units, optional accuracy and resolution, height reference, object/elevation counts, SHA-256 hash, revision fingerprint, validation status, and preview warnings. Loading 1.10 files adds an empty list only; no external resource is imported during migration.

F5-b keeps the project format at 1.11. The GeoJSON/KML vector importer now uses a preview-only intermediate model that tracks source feature IDs, part indexes, properties, source/local bounds, conversion config and topology issues before apply. Applied polygon holes are stored in the existing `SearchArea.holes` field, and MultiPolygon parts are persisted as separate search areas. Independent line parts are not merged into a single route.

F6 does not change `.dmproj`; it adds export artifacts outside the project file. QGC `.plan` exports include `targetProfile` and `validationReport` metadata, while route JSON includes the same validation report next to the waypoint payload. Mission export packages write one route file per drone plus `manifest.json` with target profile, coordinate reference, data-source summaries, per-file SHA-256 hashes and validation reports. A package manifest is evidence for the generated files only; changing routes, georeference, terrain, data sources or target profile invalidates it.

Search areas are stored in `map.search_areas`. `bounds` supports the rectangular editor; a non-empty `points` array takes precedence for polygonal coverage. Polygon holes are stored in `holes` as one point ring per excluded interior. `scan_spacing`, `boundary_margin`, and `target_coverage` are persisted with the area. Drone routes live only in the `waypoints` array (see version 1.7 below); `planned_path` is a derived read-only view and is never written.

No-fly zones include a `temporary` flag. Runtime event history is intentionally not written into `.dmproj`: the project stores the deterministic `random_seed`, while each simulation run owns its scheduled/processed events and replan counter.

Version 1.2 adds `map.terrain` and `map.wind`. The first terrain source is procedural: it stores `terrain_type`, `resolution`, `base_altitude`, `min_altitude`, `max_altitude`, and Gaussian `peaks` with center, radius, and height. Version 1.3 extends terrain with imported regular grids using `terrain_type: "grid"`, `grid_origin`, `grid_width`, `grid_height`, and `grid_altitudes`. Grid rows are ordered by increasing `y`, columns by increasing `x`, and `resolution` is the sample spacing in metres. The wind model stores `direction_to_deg`, `speed`, `gust_factor`, and `enabled`; direction is the direction the wind blows toward in map coordinates, where 0 degrees points north and 90 degrees points east.

F5-c keeps the project format at 1.11 and extends grid terrain with optional `grid_valid_mask` and `source_data_id`. The mask has the same row/column shape as `grid_altitudes`; `false` cells represent DEM NoData and remain unverifiable for safety checks. `source_data_id` links the applied terrain grid to the matching `data_sources` entry. Older grids without a mask are interpreted as fully valid.

Version 1.4 adds three-dimensional waypoints. Each drone persists a `waypoints` array; before version 1.7 a legacy `planned_path` point list was stored next to it. A waypoint is an object with `x`, `y`, `altitude`, `altitude_mode` (`msl` or `agl`), optional `speed`, `action` (`fly_to`, `hover`, `take_photo`, `scan`, `land`, or `return_to_launch`), `hold_seconds`, and optional `task_id` linking the waypoint to its mission task.

Version 1.7 makes `waypoints` the single route representation (v1.2 F1). The legacy `planned_path` array is no longer written, and `Drone.planned_path` is a read-only projection of the waypoint list. Loading a 1.6 file runs a migration that drops `planned_path`, and:

- when only the 2D path existed, rebuilds waypoints with the pre-F1 altitude rule (cruise altitude, or terrain plus minimum clearance) and records them in the migration report as rebuilt routes that need re-verification;
- when both representations existed and agreed, records a note and keeps the waypoints (including their actions, speeds and holds);
- when both existed and disagreed, keeps the waypoints and records a conflict with the drone ID and point counts instead of silently overwriting either side.

The load path exposes the report through `ProjectRepository.load_with_report` / `ProjectRepository.last_migration_report` and `ProjectService.last_migration_report`. Migration only transforms the in-memory copy: the source file is never modified, and the next explicit save writes version 1.7.

`simulation_settings.communication_policy` accepts `log_only` or `auto_return`; `communication_grace` stores the loss grace period in seconds. Base stations and drones persist their independent `communication_range`, while every drone persists `safety_radius` for collision prediction and obstacle inflation.

Version 1.8 adds the scheduling fields of plan §10.2. Every mission persists `deadline_policy` (`hard`, `soft`, or `legacy_soft`), `predecessor_ids` (stable mission ids it must wait for) and `min_lag_seconds` (extra wait after a predecessor finishes). Loading a 1.7 file marks missions that already carried a `deadline` as `legacy_soft`, because before F2 that value was only an assignment scoring term; each such mission is listed in the migration report and the deadline value itself is never changed. A mission with no deadline keeps the default `hard` policy, which only matters once a deadline is set.

Version 1.9 adds `ground_idle_power` to every drone. Ground waiting is billed with that power instead of the hover power, so a mission that waits on the apron is not accounted as if it were hovering. Loading a 1.8 file defaults the value to 5.0 and reports the change.

Every drone also persists altitude and energy-model parameters: `cruise_altitude`, `min_clearance`, `climb_rate`, `descent_rate`, `hover_power`, `ground_idle_power`, `climb_power`, `descent_power`, `horizontal_power`, and `air_speed`. Mission tasks persist `target_altitude`, obstacles persist visual `height`, and no-fly zones persist `ceiling_altitude` for 2.5D rendering and future airspace constraints. Older 1.0, 1.1, and 1.2 files are migrated in memory with flat terrain, disabled wind, and default flight/height values; saves always write the current schema.
