# Drone Mission Planner Crazyflie ROS 2 Bridge

This package is the v1.3 Crazyflie bridge for the Drone Mission Planner
execution link. The current checked-in implementation supports the TCP + NDJSON
protocol surface and a deterministic SIM backend for software acceptance:
`hello`, `ping`, `get_capabilities`, `select_robot`, `load_mission`,
`run_preflight`, `execute_mission`, `abort_land`, `emergency_stop` and
`clear_mission`.

Hardware execution is intentionally not enabled from this workspace. The
`hardware` backend returns a structured `execution_not_implemented` error until
ROS 2/Crazyswarm2, telemetry, reliable XY positioning and supervised hardware
acceptance are available.

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
- Battery, deck/status and pose telemetry are streaming at the required rate.
- Flow Deck V2, Lighthouse or equivalent XY positioning is verified stable.
- The local execution frame origin and yaw calibration have been reviewed.
- A supervised non-autonomous hardware probe has passed.
- A supervised takeoff/hover/land acceptance run has passed.

## Known Limitations

- SIM execution validates protocol, state machine and report behavior only; it is
  not firmware SIL or real flight evidence.
- The checked-in hardware adapter remains a placeholder and does not send real
  `takeoff`, `go_to`, `land` or emergency commands.
- Real waypoint execution remains blocked until the positioning gate is passed.
- Full Windows `pytest tests` can hit a pre-existing native `0xc000001d`
  illegal-instruction crash in `tests/integration/test_examples.py` on this
  host; the Crazyflie-focused and non-example suites pass.
