# Drone Mission Planner Crazyflie ROS 2 Bridge

This package is the v1.3 CF3 bridge skeleton for the Drone Mission Planner to
Crazyflie execution link. It implements only the TCP + NDJSON protocol surface:
`hello`, `ping`, `get_capabilities`, `select_robot`, `load_mission`,
`run_preflight`, `abort_land`, `emergency_stop` and `clear_mission`.

CF3 does not control Crazyswarm2 or real motors. Mission execution is rejected
with a structured `error` response until the later adapter and executor phases.

Default bind:

```bash
127.0.0.1:8765
```

Run the skeleton locally:

```bash
python -m dmp_crazyflie_bridge.protocol_server
```

