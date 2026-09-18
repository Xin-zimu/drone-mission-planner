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
      dmp_readiness:
        frequency: 10
        vars:
          - range.zrange
          - kalman.varX
          - kalman.varY
          - kalman.varZ
```

A complete DMP-owned example is checked in at
`config/crazyflies.dmp.example.yaml`. The custom log block is 14 bytes
(`range.zrange` as `uint16`, plus three `float32` Kalman variances), below the
Crazyflie firmware log-block limit of 26 bytes.

Live topic contract for `cf231`:

- `/cf231/status`: `crazyflie_interfaces/msg/Status`
- `/cf231/pose`: `geometry_msgs/msg/PoseStamped`
- `/cf231/odom`: `nav_msgs/msg/Odometry`
- `/cf231/dmp_readiness`: `crazyflie_interfaces/msg/LogDataGeneric`

The `dmp_readiness.values` order is exactly
`range.zrange`, `kalman.varX`, `kalman.varY`, `kalman.varZ`. Firmware
`range.zrange` is millimeters; the bridge converts it to meters before applying
startup down-range policy. Missing, stale, malformed, NaN/Inf or negative samples
are ignored or reported as blocking readiness diagnostics rather than guessed.

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
- Real waypoint execution remains blocked until the positioning gate is passed.
- Readiness is fail-closed. Missing `/odom`, down-range, Kalman variance or
  supervisor telemetry produces specific blocking issue codes such as
  `velocity_unavailable`, `range_unavailable`, `estimator_not_converged` or
  `supervisor_not_flyable`.
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
