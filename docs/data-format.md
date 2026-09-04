# `.dmproj` data format

The application uses UTF-8 JSON with a required top-level `version` field. The extension is `.dmproj`.

Required top-level members are `version`, `name`, `map`, `planning_settings`, and `simulation_settings`. Object IDs are stable strings such as `B-01`, `D-01`, `O-01`, and `T-01`.

The loader rejects malformed JSON and unknown versions with a user-facing error. Future schema changes must add a migration before changing `CURRENT_VERSION`.

Search areas are stored in `map.search_areas`. `bounds` supports the rectangular editor; a non-empty `points` array takes precedence for polygonal coverage. `scan_spacing`, `boundary_margin`, and `target_coverage` are persisted with the area. Drone `planned_path` arrays may contain point-mission or coverage-sweep routes.

No-fly zones include a `temporary` flag. Runtime event history is intentionally not written into `.dmproj`: the project stores the deterministic `random_seed`, while each simulation run owns its scheduled/processed events and replan counter.

Version 1.2 adds `map.terrain` and `map.wind`. The first terrain source is procedural: it stores `terrain_type`, `resolution`, `base_altitude`, `min_altitude`, `max_altitude`, and Gaussian `peaks` with center, radius, and height. Version 1.3 extends terrain with imported regular grids using `terrain_type: "grid"`, `grid_origin`, `grid_width`, `grid_height`, and `grid_altitudes`. Grid rows are ordered by increasing `y`, columns by increasing `x`, and `resolution` is the sample spacing in metres. The wind model stores `direction_to_deg`, `speed`, `gust_factor`, and `enabled`; direction is the direction the wind blows toward in map coordinates, where 0 degrees points north and 90 degrees points east.

`simulation_settings.communication_policy` accepts `log_only` or `auto_return`; `communication_grace` stores the loss grace period in seconds. Base stations and drones persist their independent `communication_range`, while every drone persists `safety_radius` for collision prediction and obstacle inflation.

Every drone also persists altitude and energy-model parameters: `cruise_altitude`, `min_clearance`, `climb_rate`, `descent_rate`, `hover_power`, `climb_power`, `descent_power`, `horizontal_power`, and `air_speed`. Mission tasks persist `target_altitude`, obstacles persist visual `height`, and no-fly zones persist `ceiling_altitude` for 2.5D rendering and future airspace constraints. Older 1.0, 1.1, and 1.2 files are migrated in memory with flat terrain, disabled wind, and default flight/height values; saves always write the current schema.
