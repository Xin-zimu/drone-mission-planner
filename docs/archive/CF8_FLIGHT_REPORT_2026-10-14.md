# CF8 Real Flight Attempt — Session Report 2026-10-14

**Status: the aircraft spun up and was commanded to take off, but aborted at 0.667 s on
`battery_critical` before gaining altitude. Two blocking software bugs were found and fixed along
the way. The abort exposed the real blocker: the 3.7 V threshold compares a loaded voltage against
a rested-voltage number, which makes flight impossible with the official pack.**

Run directory: `D:\Deepseek WorkSpace\cf_evidence_runs\20261014-145822-prepare`

---

## 1. Headline result

| item | value |
|---|---|
| aircraft | cf231 via `radio://0/80/2M/E7E7E7E7E7` (fallback address) |
| did it fly? | **NO — aborted takeoff.** Motors spun up, airframe rose ≤2.7 cm (fused) / 5.5 cm (raw ToF), then the battery abort fired at +0.667 s |
| commanded target | 0.25 m (ramp 2.5 s) |
| failure code | `battery_critical` |
| **resting voltage** | **4.048 V** |
| **voltage at fault instant** | **3.552 V** |
| **sag under load** | **0.496 V** |
| product threshold that triggered | `battery_critical_voltage: 3.7 V` |
| outcome | autonomous controlled landing, `landed` confirmed +6.071 s |
| damage | none — `tumbled=false`, `crashed=false`, `deck_fault=false`, final z ≈ 0 |
| `hardware_flight_enabled` | restored to `false` (verified) |

**IMPORTANT — correcting an earlier misreading of this run.** An initial pass read
`recorder.ndjson` in sample-index order and concluded the aircraft "climbed to 0.9 m". Aligning on
**wall clock** instead shows that is wrong, and the recorder's own summary fields are misleading
because they aggregate the whole 140 s recording, not just the flight:

- `max_z = 0.802 m`, `max_speed = 3.07 m/s`, `max_xy_drift = 1.48 m` all occurred at
  **07:01:03**, which is **20 s BEFORE** the acceptance command — i.e. when the operator picked the
  idle aircraft up. That explains the physically impossible 3.07 m/s against a `max_speed_mps = 0.08`
  policy: it was hand motion, not the controller. **There is no control defect.**
- **Inside the real flight window** (acceptance sent 07:01:24 → +8 s, 74 samples): peak fused z
  **0.0265 m**, peak raw ToF `zrange` **0.0550 m**. The aircraft never got meaningfully airborne and
  never approached the commanded 0.25 m.
- The 3.552 V minimum occurred at **+1.53 s** (≈+0.7 s after the captured origin, matching the abort
  timestamp), during motor spin-up — i.e. the sag is real and happened on the **actual** (brief)
  takeoff attempt, not during the operator lift.

So the honest result is **an aborted takeoff**, not a flight. This still exercises the whole
software→hardware chain and the battery-abort path, but it does **not** validate hover, the
STABILIZING gate, or the Flow Deck in sustained flight.

Note the physical read: at 4.05 V the pack sags 0.5 V within 0.7 s of the motors spinning up and
crosses the 3.7 V line before the aircraft can even reach 3 cm. The old pack sagged 0.586 V
(4.043 → 3.457 V); this one sags 0.496 V. **Both official packs are unusable under load** — and
this is the finding that drives the threshold change in §6.

Note the physical read: at 4.05 V the pack delivers enough thrust to lift off, but sags 0.5 V
within two thirds of a second and crosses the 3.7 V critical line. It is a **weak pack**, not a
software fault. The old pack sagged 0.586 V (4.043 → 3.457 V); this one sags 0.496 V. Both packs
are unusable for flight.

---

## 2. The two bugs that were blocking every attempt

### Bug 1 — DDS discovery scope (the "Link FAIL" that was never a link problem)

**Symptom.** `cf_prepare.sh` printed `Battery PASS / Radio PASS / Link FAIL` on every run. The
recorder counted `{"pose":0,"odom":0,"readiness":0,"flow":0,"status":0}` for 121 s, and bring-up
failed with "recorder never reported ready within 120 s".

**Why it looked like a radio fault.** `launch.log` showed heavy packet anomalies
(`Unexpected number of unicast packets. Sent: 881. Received: 46330`) and `High latency`. That was
a red herring — those same warnings appear in the 2026-09-22 *successful* flight log.

**Actual cause.** Crazyswarm2 **was** publishing: `ros2 topic hz /cf231/pose` measured 10.18 Hz and
`ros2 topic info -v` showed `Publisher count: 1 / Subscription count: 1`. The recorder simply never
discovered the publisher. This WSL runs in `mirrored` networking mode with a live but flaky NIC
(`eth4`, 192.168.43.9/24), and `ROS_AUTOMATIC_DISCOVERY_RANGE` defaulted to `SUBNET`, so
participant discovery rode multicast over that interface and intermittently never completed.
25 stale `fastrtps_*` segments were also sitting in `/dev/shm` from killed processes.

**Proof it was discovery, not radio.** Starting *any* extra ROS participant unblocked the recorder
instantly. Direct cflib delivered 100% of samples (`num_rx_unicast == num_tx_unicast`) the whole
time.

**Fix.** `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Every participant in this pipeline
(Crazyswarm2, recorder, DMP bridge, preflight) runs on this one host, so scoping discovery to the
host is both correct and sufficient. The app reaches the bridge over its own TCP port 8765, not via
ROS, so nothing external needs to discover these topics.

Applied in: `cf_ops_lib.sh::cf_ops_source_env`, `cf85_lib.sh`, `cf85_recorder.sh`,
`start_crazyswarm2.sh`, `start_dmp_bridge.sh`.

**Result.** Link went from 0 samples/121 s to **331 samples at 10.27 Hz sustained over 30 s** on
all four 10 Hz streams, 33 samples at 1.03 Hz on status. `health.json` now reports `"passed": true`.

### Bug 2 — `cf85_stop_stack: command not found`

**Symptom.** After the first successful prepare, the next run died with
`State FAIL — leftover processes could not be cleaned`, and `state.log` held only
`cf_prepare.sh: line 65: cf85_stop_stack: command not found`.

**Cause.** `cf_prepare.sh` sources `cf_ops_lib.sh`, but `cf85_stop_stack` was defined only in
`cf85_lib.sh`. Latent since the beginning — it could not surface until prepare *succeeded* and
deliberately left the stack online for the next stage to find.

**Fix.** Added `cf_ops_stop_stack()` to `cf_ops_lib.sh` (SIGTERM to the recorder so it still
flushes its timeline), with `cf85_stop_stack` kept as an alias.

### Bug 3 (cosmetic, would have broken Stage 5) — `grep -c` double-zero

`grep -c` prints `0` on no-match **and** exits 1, so `n="$(grep -ic ... || echo 0)"` captured
`"0\n0"`, which aborted the flight-service-call audit with:

```
./cf_prepare.sh: line 236: 0
0: syntax error in expression (error token is "0")
```

Fixed to `|| true` in all 19 call sites across 11 scripts. Backups: `*.bak-20261014-grepcount`.

---

## 3. The flight command sequence (verified working)

```bash
cd "/mnt/d/Deepseek WorkSpace"
./cf_prepare.sh                 # -> READY_FOR_CF8
./run_cf8_acceptance.sh         # -> the ONE authorised attempt
```

`cf_prepare.sh` internals, now all green: state → URI resolve → battery → runtime YAML →
capability snapshot (Stage 2.75, before ROS takes the radio) → recorder+Crazyswarm2 → 30 s link
health → one preflight → READY token (900 s TTL, single-use).

The 5 s abort window before `run_cf8_acceptance.sh` fires is real: Ctrl-C cancels.

---

## 4. Mission log from the flight

```
+ 0.000s  cf8_preflight_started
+ 0.001s  cf8_preflight_passed
+ 0.001s  cf8_origin_captured        origin z = 0.0057 m
+ 0.001s  takeoff_requested          target_z = 0.2557 m
+ 0.011s  takeoff_service_accepted
+ 0.667s  controlled_land_started    <-- reason=battery_critical
+ 0.677s  landing_service_accepted
+ 6.071s  landed_confirmed
+ 6.072s  acceptance_failed
```

The bridge did exactly what it is designed to do: detected the critical voltage, abandoned the
takeoff, and performed a controlled landing instead of dropping. **This is the fail-closed
behaviour working correctly on real hardware** — worth noting as a genuine validation, not just a
failure.

---

## 5. What is now proven on real hardware

- Full software→hardware path: prepare → token → flag enable → bridge restart → one acceptance.
- **Real motors spun up, real takeoff command accepted, real controlled landing** — aircraft
  recovered intact. The full actuation chain works.
- Battery-critical abort and controlled landing path, on a genuine loaded reading.
- Token single-use semantics and flag restoration.
- Telemetry quality: `max_pose_age` 0.080 s, `max_status_age` 0.950 s, 10.19 Hz pose rate.

**NOT yet proven (this attempt aborted before reaching them):** sustained hover, the STABILIZING
gate, Flow Deck positioning *in flight* over time, and the 0.25 m altitude acceptance. A passing
CF8 still needs a pack that survives motor spin-up.

## 6. Why the 3.7 V threshold must change (the core finding)

**The threshold is not "too low" or "too high" — it compares the wrong quantity.** At
[telemetry.py:1051](D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main\integrations\crazyflie_ros2\src\dmp_crazyflie_bridge\dmp_crazyflie_bridge\telemetry.py#L1051)
and [telemetry.py:466-470](D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main\integrations\crazyflie_ros2\src\dmp_crazyflie_bridge\dmp_crazyflie_bridge\telemetry.py#L466-L470)
the bridge does:

```python
status.battery_voltage <= battery_critical_voltage   # 3.7 V
```

`status.battery_voltage` is a **loaded, instantaneous** measurement. 3.7 V is a LiPo's **nominal
rested** voltage. The official 250 mAh pack sags ~0.5 V the instant the motors draw current
(measured: 4.048 → 3.552 V within 0.7 s of spin-up, both today and 3.457 V on 2026-09-22). So the
comparison is guaranteed to fail for a *healthy* official pack at any realistic resting voltage.
**The project cannot progress with this judgement, and no battery swap can fix it.**

Authoritative reference — the firmware's own protection on this aircraft, read live today:

| source | value | meaning |
|---|---|---|
| `pm.lowVoltage` | **3.2 V** | firmware "battery low" warning |
| `pm.criticalLowVoltage` | **3.0 V** | firmware hard power-off |
| DMP `battery_critical_voltage` | **3.7 V** | currently **0.5 V above the manufacturer's warning line** |

A pack at 3.552 V under load is **0.35 V above** the firmware's own warning threshold — the firmware
considered it fine. The DMP aborted a healthy spin-up. The layering we actually want is: software
detects a low pack *before* the firmware's hard cutoff and lands in a controlled way; 3.7 V inverts
that, firing far too early and making flight impossible.

**Agreed design (user-approved 2026-10-14):**

1. **Ground/rested threshold stays 3.7 V.** This protects "don't launch on an empty pack" and it
   passed today at 4.048 V. It must not move.
2. **Airborne threshold 3.3 V + debounce.** 3.3 V sits 0.1 V above the firmware warning (3.2 V) and
   0.3 V above hard cutoff (3.0 V), leaving margin for a controlled landing. It comfortably clears
   the 3.552 V transient that caused today's false abort.
3. **Debounce** so one sagging sample cannot abort flight — reuse the existing continuous-streak
   idiom (`AirborneStabilizationWindow`).

Implementation notes (from a full read-only code map):

- The in-flight abort does **not** use `ReadinessStage`; it reads the top-level
  `telemetry["battery_critical"]` boolean at
  [cf8_acceptance.py:911](D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main\integrations\crazyflie_ros2\src\dmp_crazyflie_bridge\dmp_crazyflie_bridge\cf8_acceptance.py#L911).
  `ReadinessStage` selects *which gate* consumes an issue but carries **no numeric threshold**, so
  the fix is a **parallel debounced signal**, not a stage change on the existing boolean.
- Do **not** repurpose the existing `battery_critical` boolean: it is pinned by six tests and the
  legacy `protocol_server.py:810` path, and changing it would silently move the **ground** threshold
  — the one that must not move.
- Two end-to-end tests will need rewriting because they set the boolean directly and assert an
  immediate abort: `test_execution_cf8_acceptance.py:768` and `:352`.
- The docs that declare the threshold frozen must be updated **in the same commit**, or the change
  violates stated hard constraints: `cf8-operations.md:83`, `:250-251`, `:259`; `session-log.md:24`;
  `progress.md:93-94`, `:1117`. There is precedent — the range gate already uses two thresholds with
  stages (`min_reliable_zrange_m` vs `range_ground_advisory_m`).

## 7. Remaining blockers

1. **Battery judgement (above)** — the active blocker; no battery swap can work around it.
2. **`FALLBACK_ADDRESS_IN_USE`.** The drone answers at `E7E7E7E7E7` while `cf_identity.yaml`
   prefers `E7E7E7E7E705`; the preferred address gets no radio response. Harmless for flying but the
   identity config should be reconciled.
3. Two non-blocking telemetry notes: `deck parameter values unavailable from /crazyflie_server:
   expected 5, received 0` (the known `query_all_values_on_connect=False` behaviour that the
   capability snapshot compensates for), and `range_invalid` on the ground because `zrange` reads
   0 m inside the ToF near-field.

## 8. Invariants — verified after the flight

```
hardware_flight_enabled: false        <- restored and verified
battery_critical_voltage: 3.7         <- ground threshold, unchanged (airborne split pending)
capability_snapshot_max_age_s: 3600   <- untouched
max_estimator_variance_span: 0.002    <- untouched
```

Repo: branch `codex/v1.3-crazyflie-execution`, working tree clean. All tooling changes live in
`D:\Deepseek WorkSpace\_dsh_scratch\` and the two entry scripts; no app source was modified.

## 8. Environment pitfalls (re-confirmed today)

- Crazyradio busid drifts (`3-1` today, was `1-3`/`1-4`) — always resolve at runtime.
- WSL idle shutdown silently drops USB/IP while `usbipd list` still claims `Attached`; keep a
  keepalive running.
- `usbipd attach` fails with "no WSL 2 distribution running" if the distro has gone idle.
- Clear `/dev/shm/fastrtps_*` when no participant is alive.
- PowerShell mangles inline `$` — keep shell logic in `.sh` files and pass only paths/args.
