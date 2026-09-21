# Drone Mission Planner Crazyflie ROS 2 Bridge

This package is the v1.3 Crazyflie bridge for the Drone Mission Planner
execution link. The current checked-in implementation supports the TCP + NDJSON
protocol surface, a deterministic SIM backend for software acceptance, and a
default-locked Crazyswarm2 hardware backend:
`hello`, `ping`, `get_capabilities`, `select_robot`, `load_mission`,
`get_telemetry`, `run_preflight`, `execute_mission`, `abort_land`,
`emergency_stop`, `clear_mission`, `run_cf8_acceptance` and
`get_cf8_acceptance`.

Hardware mission execution is intentionally not enabled from this workspace. The
`hardware` backend subscribes to Crazyswarm2 `/cf231/status`, `/cf231/pose`,
`/cf231/odom` and DMP's `/cf231/dmp_readiness` custom firmware log topic,
reports real battery/link/pose/readiness telemetry, and still returns a
structured `execution_not_implemented` error for mission execution. CF8 adds a
separate supervised takeoff-hover-land acceptance runner only; it does not open
XY `go_to`, waypoint mission execution, trajectories, return-to-home or route
execution.

Default bind:

```bash
127.0.0.1:8765
```

Run the bridge locally in SIM mode:

```bash
python -m dmp_crazyflie_bridge.protocol_server
```

Run explicitly without hardware control:

```bash
python -m dmp_crazyflie_bridge.protocol_server --backend sim
```

Run the read-only hardware telemetry backend after Crazyswarm2 is already
running:

```bash
cd integrations/crazyflie_ros2
export PYTHONPATH="$PWD/src/dmp_crazyflie_bridge:${PYTHONPATH:-}"
python -m dmp_crazyflie_bridge.protocol_server \
  --backend hardware \
  --config src/dmp_crazyflie_bridge/config/bridge.yaml
```

The default hardware robot id is configured in
`src/dmp_crazyflie_bridge/config/bridge.yaml`:

```yaml
robot_id: cf231
hardware_flight_enabled: false
```

The hardware readiness safety thresholds in `bridge.yaml` fail closed if an
explicit value is invalid. Leave a key absent to use the default; do not rely on
invalid values falling back silently.

### Down-range band (`range_ground_advisory_m`, `min_reliable_zrange_m`, `max_reliable_zrange_m`)

The VL53L1x down-range sensor is not trustworthy closer than roughly 4 cm, so a
ground reading of 0–13 mm is **not** a height and cannot be used to judge whether
the sensor works. The single old key `min_startup_zrange_m` applied a near-field
plausibility check to a quantity the sensor cannot measure near-field; neither
`0.02` nor a lowered `0.005` was defensible. It is **deprecated**. The band ceiling
was called `max_startup_zrange_m`; that name described a startup plausibility bound,
but the value bounds where the sensor is *reliable*, which is an airborne property,
so it was renamed to `max_reliable_zrange_m`. The number is unchanged (`0.50`).

```yaml
range_ground_advisory_m: 0.04   # below this on the ground -> non-blocking advisory
min_reliable_zrange_m: 0.10     # airborne reliable band floor
max_reliable_zrange_m: 0.50     # airborne reliable band ceiling
```

Legacy key handling is deliberately unambiguous:

| legacy key | behaviour |
|---|---|
| `min_startup_zrange_m` | read but **ignored**, emits a `DeprecationWarning`. Never mapped onto either new key: the retired value was often lowered to a few millimetres to force a ground reading through, and reusing it as the airborne floor would declare a vehicle that never left the ground to be valid. |
| `max_startup_zrange_m` | a pure rename, so the value **is still honoured** when the new key is absent, with a `DeprecationWarning`. If both keys are present the new key wins outright and a second, explicit warning names the shadowed key — a stale local override can never silently win. |

Configuration must satisfy
`range_ground_advisory_m < min_reliable_zrange_m < max_reliable_zrange_m`;
any other ordering fails at startup with a `ValueError`.

### Capability snapshot freshness is session-scoped

`capability_snapshot_max_age_s` (default one hour) is **not** a recurring cache TTL.
It used to be re-evaluated on every telemetry poll, so once the cflib snapshot file
aged past an hour a healthy Flow2 + ZRanger2 vehicle could silently degrade to
`positioning_mode = unknown` in the middle of a live session - and readiness then
blocked on `xy_positioning_missing` / `z_positioning_missing`.

Freshness is now judged **once per hardware telemetry session**, at the moment a
capability verdict is granted:

1. a new session has no capability at all;
2. live `crazyflie_server` deck parameters win when available;
3. otherwise a capability snapshot that is fresh **at that moment** is accepted;
4. otherwise the adapter fails closed (`positioning_mode = unknown`) and keeps
   retrying on later calls (rate limited) until a source grants a verdict;
5. once granted, the verdict (`Flow2`, `ZRanger2`, `positioning_mode`, source and
   captured-at) is **pinned for the rest of the session** and is never re-read, so
   wall-clock TTL expiry cannot withdraw capability from a vehicle that is flying;
6. the session ending (status gap, `disconnect()`, reconnect, changed session id)
   clears the pin, and the next session re-runs the whole decision from scratch.

A snapshot that is already stale **before** a session starts stays rejected: its age
only grows, so it can never be accepted later in that session. Capability never leaks
across sessions. This is also why the manual refresh procedure below stops Crazyswarm2
first: the Bridge itself never starts a firmware parameter probe (it would seize the
Crazyradio from Crazyswarm2) and never stops or restarts Crazyswarm2 to refresh a
snapshot. `tests/integration/test_execution_hardware_adapter.py` asserts both
behaviourally and by scanning the adapter source.

### Ground vs airborne readiness stages

`get_telemetry.readiness` reports `ground_passed` and `airborne_passed` alongside
`passed`, and every issue carries a `stage` of `ground`, `airborne` or `both`.
Ground launch preflight consumes only `ground`/`both` issues. `pose_stability_*`,
`velocity_not_settled` and `range_invalid` are `airborne`: they depend on the
PMW3901 (specified for 80 mm to infinity) and on near-field ranging, so they are
enforced by the post-takeoff stabilization gate rather than by a ground gate.
Issues without a `stage` (legacy payloads) are treated as `both` — fail-closed.
See `../../docs/v1.3-cf81-ground-airborne-readiness-split.md`.

Supervisor codes are staged too. `supervisor_not_flyable`, `supervisor_armed`,
`supervisor_flying` and `supervisor_high_level_active` describe the supervisor's
*handover* state ("may a supervised flight start from here?") and are **`ground`
only**: a vehicle that is legitimately under way reports `armed=True`,
`flying=True`, `can_be_armed=False` (manual explicit-arm builds) and often active
high-level control, so re-using those checks airborne would fault every normal
flight. `can_fly=False` keeps a ground code (`supervisor_not_flyable`) and gains
its own airborne code `supervisor_cannot_fly`; CF8 also treats a revoked
`can_fly` as an immediate hard health fault. `supervisor_missing`,
`supervisor_locked`, `supervisor_tumbled`, `supervisor_crashed` and `deck_fault`
are `both`. The classification lives in
`telemetry.GROUND_ONLY_SUPERVISOR_ISSUE_CODES` /
`AIRBORNE_ONLY_SUPERVISOR_ISSUE_CODES` / `BOTH_STAGE_SUPERVISOR_ISSUE_CODES` and
is asserted against all `2**12` raw supervisor bit patterns by the tests.

### CF8 post-takeoff stabilization gate

`run_cf8_acceptance` no longer jumps from `taking_off` to `hovering`. The state
order is `taking_off -> stabilizing -> hovering -> landing`, where `STABILIZING`
is the only place flow- and near-field-derived quality is enforced:

- anchor: the monotonic timestamp of the `takeoff_target_reached` event;
- `minimum_airborne_observation_s = 5.0` of post-anchor samples, plus a
  continuous `stabilization_window_s = 3.0` window in which **every** fresh
  sample satisfies `0.10 <= range.zrange <= 0.50`, `max(|vx|)`, `max(|vy|)`,
  `max(|vz|) <= 0.05`, `max(speed) <= 0.08` (strict max, **no percentile**),
  `xy_span <= 0.03`, `xy_displacement <= 0.03`, and the estimator stays fresh
  and converged;
- the window is fed only by samples the runner polls after the anchor — never by
  `TelemetryCollector._pose_samples`, whose 200-sample deque (~20 s at 10 Hz)
  still holds pre-takeoff ground samples after liftoff;
- hard health faults (battery critical, lost status/pose, changed hardware
  session, locked/tumbled/crashed/deck fault, revoked `can_fly`, XY drift)
  go straight to the controlled landing path;
- soft convergence shortfalls wait until `stabilization_timeout_s = 10.0` and
  then fail (`stabilization_timeout`) through the same controlled landing.
  The controlled landing is sent exactly once; emergency stop is never used as a
  substitute.

Progress and events are visible in `get_cf8_acceptance`, and
`metrics.stabilization` carries the strict-max window evidence
(`min_zrange_m`, `max_abs_vx_mps`, `max_speed_mps`, `xy_span_m`, ...). A hard
fault additionally stores the telemetry it fired on in `failure_telemetry`; see
"Failure telemetry" below.

`hardware_flight_enabled` is deliberately checked in as `false`. With that
default, even a `run_cf8_acceptance` request cannot create an arm, takeoff or
land service request. A supervised operator must intentionally set it to `true`
in a local config and send `confirm_real_flight: true` on that specific request.

Bridge config paths are deterministic: an explicitly supplied `--config` must
exist, and relative path values inside that config are resolved relative to the
config file directory. The checked-in capability snapshot path is therefore
`src/dmp_crazyflie_bridge/runtime/cf231_capabilities.json` when launched with the
command above.

## Crazyswarm2 Logging

Crazyswarm2's sample `crazyflies.yaml` enables `pose` and `status` by default.
DMP readiness also requires explicit `odom` velocity logging and a custom
`LogDataGeneric` block:

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10
      status:
        frequency: 1
      odom:
        frequency: 10
    custom_topics:
      # kalman.varPX/varPY/varPZ are the Kalman VELOCITY covariance — see the estimator
      # convergence note below. Do NOT swap these back to kalman.varX/varY/varZ.
      dmp_readiness:
        frequency: 10
        vars:
          - range.zrange
          - kalman.varPX
          - kalman.varPY
          - kalman.varPZ
      # Optional local diagnostic: raw Flow deck v2 (PMW3901) quality/motion data.
      # Must stay on its OWN topic — see the block-size note below.
      dmp_flow:
        frequency: 10
        vars:
          - motion.deltaX
          - motion.deltaY
          - motion.motion
          - motion.squal
```

A complete DMP-owned example is checked in at
`config/crazyflies.dmp.example.yaml`. The `dmp_readiness` custom log block is 14 bytes
(`range.zrange` as `uint16`, plus three `float32` Kalman covariances), below the
Crazyflie firmware log-block limit of 26 bytes.

> **Do not add variables to `dmp_readiness`.** The bridge parses its `values` list
> **positionally**, so changing the variable set or order silently changes the block
> layout. Keep additional diagnostics (such as the `dmp_flow` quality topic above) on a
> separate custom topic.

Live topic contract for `cf231`:

- `/cf231/status`: `crazyflie_interfaces/msg/Status`
- `/cf231/pose`: `geometry_msgs/msg/PoseStamped`
- `/cf231/odom`: `nav_msgs/msg/Odometry`
- `/cf231/dmp_readiness`: `crazyflie_interfaces/msg/LogDataGeneric`
- `/cf231/dmp_flow`: `crazyflie_interfaces/msg/LogDataGeneric` (optional diagnostic;
  `motion.deltaX`, `motion.deltaY`, `motion.motion`, `motion.squal`)

The `dmp_readiness.values` order is exactly
`range.zrange`, `kalman.varPX`, `kalman.varPY`, `kalman.varPZ`. Firmware
`range.zrange` is millimeters; the bridge converts it to meters before applying
startup down-range policy. Missing, stale, malformed, NaN/Inf or negative samples
are ignored or reported as blocking readiness diagnostics rather than guessed.

### Estimator convergence uses Kalman **velocity** covariance

`estimator_not_converged` is judged from `kalman.varPX/varPY/varPZ`, the Kalman **velocity**
covariance, which is the same source the official cflib `wait_for_position_estimator()`
gates on. It is **not** `kalman.varX/varY/varZ`: those are the **position** covariance, and
with flow-only positioning they grow to the firmware `MAX_COVARIANCE` ceiling and then sit
there with a span of exactly zero. Measured on `cf231` the position covariance read
`varX = varY = 99.9995` with `span = 0` while the velocity covariance read
`varPX/PY ≈ 1.2e-4 .. 2.4e-4` — a span-based test on the former could only ever report
"converged".

Two independent fail-closed conditions are applied, in this order:

1. **Saturation guard** — a value at or above
   `estimator_max_covariance * estimator_saturation_fraction` (defaults `100.0 * 0.5 = 50.0`)
   is refused *however stable it looks*. A clamped value pinned near the ceiling means "no
   aiding is reaching the filter", not "the filter converged". This is checked before the
   span test precisely because a saturated value has span 0. The guard is deliberately
   loose: healthy velocity covariance is 5 orders of magnitude below it.
2. **Span policy** — `max(varPX/varPY/varPZ span) > max_estimator_variance_span`
   (default `0.002`, unchanged) means the covariance is not settling.
3. Optionally, `max_estimator_variance` as an absolute ceiling — unset by default.

`max_estimator_variance_span` was **not** retuned when the variable changed.

#### Wire contract: `estimator_variance` is explicitly typed

Because the underlying variables changed meaning, the telemetry payload no longer emits the
ambiguous `var_x` / `var_y` / `var_z` keys. It now emits:

```json
"estimator_variance": {
  "kind": "velocity_covariance",
  "source": "kalman.varPX/PY/PZ",
  "variables": ["kalman.varPX", "kalman.varPY", "kalman.varPZ"],
  "var_px": 0.000152,
  "var_py": 0.000153,
  "var_pz": 0.000199
}
```

The CF8 stabilization gate requires `kind == "velocity_covariance"` and finite
`var_px`/`var_py`/`var_pz`; anything else fails closed as `estimator_not_converged`.

## Static Drift Diagnostics

`scripts/flow_drift_probe.py` is a **read-only** localisation probe. It subscribes to the
same topics the bridge consumes and recomputes the identical pose/velocity/range policy
statistics, so its numbers are directly comparable with `get_telemetry`. It also reports
raw Flow deck quality from `/cf231/dmp_flow`.

```bash
cd ~/ros2_ws
source ~/crazyswarm-env/bin/activate
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 <repo>/integrations/crazyflie_ros2/scripts/flow_drift_probe.py \
  --robot-id cf231 --duration 30 --json /tmp/cf231-drift.json
```

It creates subscriptions only: it never publishes and never calls a service, so it cannot
arm, take off or land a vehicle. Use it to judge whether a ground surface and mounting
height actually support optical flow before treating a readiness failure as a software
problem. A healthy reading needs `squal` well above the noise floor and small
`motion.deltaX` / `motion.deltaY` while the vehicle is stationary.

## CF8 Acceptance

CF7-LIVE is intentionally folded into CF8: skipping a standalone CF7-LIVE stage
does not skip its checks. Before any CF8 takeoff service can be dispatched, the
bridge immediately re-runs its authoritative live readiness gate: connected
status, fresh pose, pose rate, Flow XY, Z, battery, supervisor, static pose
stability, `/odom` velocity, `/dmp_readiness` range/Kalman convergence, fresh
hardware session and finite current pose.

The CF8 protocol is intentionally separate from mission execution:

```json
{"type":"run_cf8_acceptance","request_id":"cf8-1","confirm_real_flight":true}
{"type":"get_cf8_acceptance","request_id":"cf8-poll"}
```

`run_cf8_acceptance` starts the runner and returns a
`cf8_acceptance_state`; clients poll `get_cf8_acceptance` for the eventual
`cf8_acceptance_result`. The runner records an event timeline, origin pose,
target Z, landing target, service-dispatch events, drift/error/speed/battery
metrics and final telemetry.

## Failure telemetry (`failure_telemetry`)

When the in-flight health check detects a hard fault, the result also carries
`failure_telemetry`: a compact snapshot of the telemetry at the instant the fault
fired, so a superseded hardware attempt does not have to be repeated merely to
learn what tripped it.

```json
"failure_telemetry": {
  "captured_at_unix_s": 1790001038.35,
  "battery_voltage": 3.69,
  "battery_critical": true,
  "battery_critical_voltage": 3.7,
  "pm_state": 0,
  "z_m": 0.004, "x_m": 0.59, "y_m": -1.75,
  "velocity": {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "speed_mps": 0.0},
  "supervisor_info_raw": 526,
  "supervisor": {"armed": true, "can_fly": true, "locked": false, "...": "..."},
  "status_age_s": 0.31, "pose_age_s": 0.02,
  "estimator_variance": {"kind": "velocity_covariance", "source": "kalman.varPX/PY/PZ",
                         "var_px": 0.0001, "var_py": 0.0001, "var_pz": 0.0002},
  "range": {"zrange_m": 0.0},
  "positioning_mode": "flow", "connected": true, "pose_stream_available": true
}
```

Contract:

- **Observability only.** The snapshot is taken *after* a fault has already been
  detected, so it cannot influence any verdict. No threshold, the
  `battery_critical` rule, the controlled-landing path, the CF8 state machine and
  the retry semantics are unchanged by it.
- Every key is always present. A value that was unavailable at the triggering
  instant is `null` rather than omitted, so a `status_lost` or `pose_lost` fault
  still carries whatever telemetry was left.
- The **first** fault of a run wins. A later detection does not overwrite it —
  the landing loop deliberately tolerates a battery fault it re-detects while
  descending, and reporting that partly-descended sample would hide the numbers
  the run actually failed on.
- A passing run always reports `failure_telemetry: null`, including the case
  where a battery fault was tolerated during the landing phase.

`metrics.min_battery_voltage` is *not* the same number: the failure returns from
the health check before the metrics update, so it holds the last sample recorded
before the fault, not the faulting one. Read `failure_telemetry.battery_voltage`
for the triggering value.

Takeoff and land heights are absolute Crazyswarm2/Crazyflie high-level
commander heights. CF8 captures an acceptance-only origin at start and computes:

```text
target_z = origin_z + cf8_takeoff_delta_m
landing_target_z = origin_z
```

The service response only means the command was accepted by Crazyswarm2. Takeoff,
hover and landing success are confirmed from telemetry, including Z error, XY
drift, low speed over a settle window, fresh status/pose, unchanged session,
battery and supervisor safety.

Current source review shows the normal Crazyswarm2 examples call takeoff/land
without a preceding arm request, while Crazyswarm2 also exposes an independent
`/<cf>/arm` service. DMP therefore defaults `cf8_explicit_arm_required: false`.
If a local firmware/Crazyswarm2 setup requires explicit arming, the operator can
set that policy true; CF8 will arm before takeoff and only disarm after safe
landing confirmation.

Non-flight hardware capability probe:

```bash
cd integrations/crazyflie_ros2
export PYTHONPATH="$PWD/src/dmp_crazyflie_bridge:${PYTHONPATH:-}"
python -m dmp_crazyflie_bridge.capability_probe \
  --robot-id cf231 \
  --uri radio://0/80/2M/E7E7E7E705
```

The capability probe only connects through cflib, reads selected firmware deck
parameters, writes a runtime JSON snapshot, and disconnects. From DMP it is a
non-flight capability probe: it does not request arm, takeoff, waypoint `go_to`
or land. Run it only while the Crazyflie is landed with motors stopped, while
Crazyswarm2 is stopped, and restart Crazyswarm2 after the probe so cflib and the
C++ backend do not compete for Crazyradio.

Non-flight environment probe:

```bash
python scripts/probe_environment.py
```

The probe only reports local tools, Python modules and visible Crazyflie-related
USB devices. It never enables motor commands.

## Quick Start

1. Start the Planner on Windows.
2. Start the bridge in SIM mode with the command above.
3. In the Planner, compile a Crazyflie execution mission.
4. Select `cf1`, load the mission, run preflight, then execute.
5. Review the Execution Dock live state and execution report summary.

## Hardware Checklist

Do not switch to hardware execution until all of the following are true:

- ROS 2 and Crazyswarm2 are installed and importable in the bridge environment.
- The Crazyflie is connected through Crazyradio and visible to Crazyswarm2.
- If Crazyswarm2 deck ROS parameters are listed but unset, Crazyswarm2 is
  stopped, the read-only cflib capability probe has generated a fresh snapshot,
  and the snapshot robot id and URI match the bridge configuration.
- Battery, deck/status and pose telemetry are streaming at the required rate.
- Flow Deck V2, Lighthouse or equivalent XY positioning capability is verified.
- Z positioning is verified separately from XY positioning. For Flow Deck V2,
  `bcFlow2=1` plus `bcZRanger2=1` is expected because the Flow deck initializes
  its integrated VL53L1 ToF through the `bcZRanger2` driver.
- Live readiness is verified separately from deck capability: supervisor state,
  pose freshness/rate, full-window static pose stability, `/odom` velocity,
  `/dmp_readiness` down-range health, Kalman variance diagnostics and current-session frame
  origin must all pass the Bridge preflight.
- The local execution frame origin and yaw calibration have been reviewed and
  captured in the current hardware session.
- A supervised non-autonomous hardware probe has passed.
- A supervised takeoff/hover/land acceptance run has passed.

## Known Limitations

- SIM execution validates protocol, state machine and report behavior only; it is
  not firmware SIL or real flight evidence.
- The checked-in default config keeps real hardware flight disabled with
  `hardware_flight_enabled: false`.
- CF8 can dispatch only `arm` if explicitly required, `takeoff` and `land`, and
  only through `run_cf8_acceptance` after config unlock, strict per-run operator
  confirmation and fresh live preflight. Hardware `go_to` remains disabled until
  CF9, and `execute_mission` still returns `execution_not_implemented`.
- Protocol `emergency_stop` is not automatically triggered by CF8 faults.
  Ordinary CF8 faults attempt controlled landing instead.
- Deck capability detection first attempts reliable Crazyswarm2/ROS parameter
  values. Some Crazyswarm2 C++ backend runs list `cf231.params.deck.*`
  descriptors but return no parameter values because
  `firmware_params.query_all_values_on_connect` is disabled. In that case the
  bridge may consume a fresh, URI-matched read-only cflib capability snapshot;
  otherwise it reports unknown positioning with diagnostics rather than guessing
  readiness.
- **The cflib capability snapshot is only a session-bootstrap source.**
  `capability_snapshot_max_age_s` defaults to 3600 s, but it is evaluated once per
  hardware telemetry session, not on every poll: a session that already granted
  `positioning_mode=flow` keeps it for the whole session even after the file ages
  past the TTL, so a healthy Flow2 + ZRanger2 vehicle can no longer degrade
  mid-flight. A session whose snapshot was already stale when it started still
  fails closed (`positioning_mode=unknown`). Refreshing a snapshot is always a
  manual operator action and requires **exclusive** radio access — stop
  Crazyswarm2, re-run the capability probe, then restart Crazyswarm2 and the
  bridge — because cflib and the C++ backend cannot share the Crazyradio. The
  Bridge itself never starts a probe and never restarts Crazyswarm2.
- **Ground readiness currently demands signals that are only valid in the air.**
  `pose_stability_*`, `velocity_not_settled` and `range_invalid` all depend on
  optical flow and near-field ranging. The PMW3901 is specified for 80 mm to
  infinity, and the VL53L1x is unreliable below roughly 4 cm, so `range.zrange`
  readings of 0–13 mm while the vehicle rests on the ground are not trustworthy
  heights and neither `min_startup_zrange_m` nor a lowered local value can make
  them so. The ground/airborne split is specified in
  `../../docs/v1.3-cf81-ground-airborne-readiness-split.md`.
- Static Flow deck position estimates drift slowly even on a level, textured
  surface; velocity noise scales with height (`v = flow·h/k`), so a vehicle held
  far above the ground can fail the velocity gate purely because of the mounting
  height. Keep the measured height near the intended hover altitude when judging
  static readiness.
- Readiness is fail-closed. Missing `/odom`, down-range, Kalman variance or
  supervisor telemetry produces specific blocking issue codes such as
  `velocity_unavailable`, `range_unavailable`, `estimator_not_converged` or
  `supervisor_not_flyable`.
- Auto-arm firmware parks reporting `supervisor_info_raw = 526`
  (`armed | auto_arm | can_fly | high_level_trajectory_finished`) with
  `can_be_armed = 0`. That is a healthy standby state: arming readiness is
  `can_be_armed or (auto_arm and armed)`, and `armed` without `auto_arm` is still
  the blocking `supervisor_armed` fault.
- Real waypoint execution remains blocked until the positioning gate is passed.
- Hardware session ids are driven by fresh `/status` telemetry. A stale status
  stream invalidates current-pose origins even if the adapter process did not
  explicitly disconnect.
- Full Windows `pytest tests` can hit a pre-existing native `0xc000001d`
  illegal-instruction crash in `tests/integration/test_examples.py` on this
  host; the Crazyflie-focused and non-example suites pass.

## CF7 Hardware Startup Order

1. Attach Crazyradio to WSL.
2. Ensure Crazyswarm2 is not running.
3. Run the read-only cflib capability probe.
4. Verify the generated snapshot in `runtime/`.
5. Start Crazyswarm2 C++ with `gui:=False mocap:=False teleop:=False` and the
   DMP logging config merged into `crazyflies.yaml`.
6. Start the DMP bridge hardware backend.
7. Verify status, pose, capability source, preflight and frame origin.
8. For CF8 live acceptance only, prepare a local config with
   `hardware_flight_enabled: true`, send `run_cf8_acceptance` with strict
   `confirm_real_flight: true`, and poll `get_cf8_acceptance`.
9. Do not use `execute_mission` for hardware; it remains disabled.

### Repeating steps 2–6

Step 3 needs the Crazyradio to itself, so it can only run while Crazyswarm2 is stopped.
The snapshot it writes is only accepted as a session-bootstrap source while it is younger
than `capability_snapshot_max_age_s` (one hour by default). A session that already granted
capability keeps it until the session ends; only a **new** session re-checks the file. If a
fresh session still reports unknown positioning, repeat steps 2–6 rather than lowering
readiness thresholds. Steps 8–9 additionally require a charged pack; the battery critical
threshold is 3.7 V.
