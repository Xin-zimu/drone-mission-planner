# `.dmproj` data format

The application uses UTF-8 JSON with a required top-level `version` field. The extension is `.dmproj`.

Required top-level members are `version`, `name`, `map`, `planning_settings`, and `simulation_settings`. Object IDs are stable strings such as `B-01`, `D-01`, `O-01`, and `T-01`.

The loader rejects malformed JSON and unknown versions with a user-facing error. Future schema changes must add a migration before changing `CURRENT_VERSION`.

Search areas are stored in `map.search_areas`. `bounds` supports the rectangular editor; a non-empty `points` array takes precedence for polygonal coverage. `scan_spacing`, `boundary_margin`, and `target_coverage` are persisted with the area. Drone routes live only in the `waypoints` array (see version 1.7 below); `planned_path` is a derived read-only view and is never written.

No-fly zones include a `temporary` flag. Runtime event history is intentionally not written into `.dmproj`: the project stores the deterministic `random_seed`, while each simulation run owns its scheduled/processed events and replan counter.

Version 1.2 adds `map.terrain` and `map.wind`. The first terrain source is procedural: it stores `terrain_type`, `resolution`, `base_altitude`, `min_altitude`, `max_altitude`, and Gaussian `peaks` with center, radius, and height. Version 1.3 extends terrain with imported regular grids using `terrain_type: "grid"`, `grid_origin`, `grid_width`, `grid_height`, and `grid_altitudes`. Grid rows are ordered by increasing `y`, columns by increasing `x`, and `resolution` is the sample spacing in metres. The wind model stores `direction_to_deg`, `speed`, `gust_factor`, and `enabled`; direction is the direction the wind blows toward in map coordinates, where 0 degrees points north and 90 degrees points east.

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
