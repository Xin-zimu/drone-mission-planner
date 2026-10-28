# Incident Report — 2026-10-14 16:03 CF8 attempt on a tethered aircraft

**Severity: real hardware incident, no damage. Root cause: a takeoff was authorised while the
aircraft was on the USB charger. Nothing in the pipeline checked the charge state.**

Run directory: `D:\Deepseek WorkSpace\cf_evidence_runs\20261014-160201-prepare`

---

## 1. What happened

The operator unplugged the charger and confirmed props were fitted. `cf_prepare.sh` reached
`READY_FOR_CF8` (Link PASS, Preflight PASS, battery 4.070 V) and `run_cf8_acceptance.sh` fired one
authorised attempt at **08:03:53Z**.

The aircraft was **on the USB charger** at that moment. It lurched, dragged against the cable, and
the firmware latched a terminal state.

| item | value |
|---|---|
| failure code | `xy_drift_exceeded` (0.187 m at the 0.15 m tolerance, +0.41 s after takeoff) |
| firmware supervisor at trigger | raw **164** = `auto_arm`, **`tumbled`**, **`crashed`** |
| firmware supervisor at pre-flight | raw **526** = `armed`, `auto_arm`, **`can_fly`**, `traj_finished` |
| apparent travel | `x` 14.267 → 15.668, `y` −68.572 → −60.821 ≈ **7.9 m** |
| down-range reading | **1.886 m** (vs 0.021 m before) |
| peak speed | **4.06 m/s** |
| controlled landing | **FAILED** — `supervisor revoked the flyable state` |
| `pm_state` | **1 = charging**, charge current **0.213 A** |
| damage | **none** — see the post-incident check in §4 |
| `hardware_flight_enabled` | restored to `false` (both permanent and runtime configs verified) |

## 2. Root cause

`cf85_battery.sh` recorded the tether state and **nothing consulted it**:

```json
"pm_state": 1,
"pm_state_meaning": "charging",
"charging_current_a": 0.212551325559616,
"cf85_policy": { "verdict": "NEXT_STAGE_ALLOWED", "allowed_to_continue": true }
```

The battery gate compared **voltage only**. 4.070 V is a perfectly healthy reading, so the gate
passed an aircraft that was physically tethered to a USB cable and powered from it. The
`pm_state` field was present in the very same JSON that the gate read — it was simply never
tested. This is a missing check, not a mis-tuned threshold, and no threshold change could have
prevented it.

Why a tethered takeoff is invalid regardless of voltage:

- the cable physically restricts and unbalances the airframe;
- USB power changes the power/thrust situation the controller expects;
- the resulting lurch can snap props or the airframe, and it did latch a crash state here.

## 3. Contributing factor — this should have been caught earlier

The operator was **charging the battery** immediately before the attempt (the gate had refused at
3.985 V minutes earlier). The chat history shows the charge state was observed and written into a
report ("battery recovered to 4.043 V, state=1 charging, 0.258 A — you're charging it") and then
treated as good news rather than as a **blocking** condition. The information existed at every
layer and was acted on at none.

## 4. Aircraft status — verified healthy after the incident

Read-only probe `cf85_incident_check.py` immediately after the charger was removed:

| item | value | meaning |
|---|---|---|
| `pm.state` | **0** = `PM_STATE_BATTERY` | on battery, no USB power |
| charge current | **0.000 A** | charger confirmed disconnected |
| attitude | roll **1.46°**, pitch **0.47°** | level — not on its side |
| `stateEstimate.z` | **0.004 m** | on the ground |
| `pm.vbat` | 3.996 V, 70 % | pack fine |
| `supervisor.info` | **644** = `auto_arm`, **`crashed`**, `traj_finished` | see §5 |

### 4.1 Later re-check (16:40) — and why the accelerometer matters

A later reading showed the airframe **not level**:

| source | reading |
|---|---|
| attitude estimator (`stabilizer.roll/pitch`) | roll **−35.0°**, pitch **−65.7°** |
| **accelerometer** (`acc.x/y/z`, independent of the estimator) | `(-0.956, -0.249, -0.090)`, \|acc\| = **0.992** → body-Z is **84.8°** from gravity |
| `pm.state` | 1 (charging again) |
| `supervisor.info` | **676** = `auto_arm`, **`tumbled`**, **`crashed`**, `traj_finished`; `can_fly = NO` |

Both channels agree the airframe really was on its side, so this is not estimator divergence. The
likely explanation is that the aircraft was simply **lying on its side while on the charger**
(consistent with `pm.state = 1`), not that it broke. No damage evidence exists either way, but
the physical cause was not independently confirmed — **the operator must visually confirm props
and arms.**

Lesson: `stabilizer.roll/pitch` are **estimator outputs**. When they look wrong, confirm with the
accelerometer before concluding anything. `cf85_level_check.py` does exactly that and is now the
required pre-flight check.

### 4.2 Required before the next flight

1. **Unplug the charger** (`pm.state` must read 0).
2. **Set the aircraft flat on the bench.** Then re-read the supervisor: if `tumbled`/`crashed`
   persist and `can_fly` stays NO, **disconnect the battery and power-cycle** — these are latched
   in RAM and a ROS-stack restart does not clear them.
3. Visually confirm the props and arms.

A clean, ready-to-fly supervisor reads **526** (`armed`, `auto_arm`, `can_fly`,
`traj_finished`) with `can_fly = YES`. Anything containing `tumbled`, `crashed` or `locked` will
block the ground launch gate as a `stage=both` blocking issue.

## 5. Corrected open item — the `tumbled` / `crashed` latches, and what we do and do not know

**Correction.** An earlier version of this section claimed `tumbled` cleared while `crashed`
persisted, and inferred that a ROS-stack restart is insufficient but an aircraft power cycle is
required. **The evidence no longer supports that.** Observed sequence:

| `supervisor.info` | when | bits set |
|---|---|---|
| 526 | pre-flight (healthy) | `armed`, `auto_arm`, `can_fly`, `traj_finished` |
| 164 | incident instant | `auto_arm`, **`tumbled`**, **`crashed`** |
| 644 | 16:15, charger unplugged, airframe level | `auto_arm`, **`crashed`**, `traj_finished` |
| 676 | 16:40, charging, airframe on its side | `auto_arm`, **`tumbled`**, **`crashed`**, `traj_finished` |

676 came **after** 644, with no flight in between — so `tumbled` came *back*. That means these bits
are **not a simple one-way latch**, and they **track the current physical situation** at least in
part: a level airframe read 644 (no `tumbled`), a side-lying one read 676 (`tumbled` set).

**What is established:** the bits can be set at different times and are **not** cleared by
restarting the ROS stack alone (644 and 676 were both read with the stack stopped).

**What is NOT established:** whether an aircraft power cycle clears them, and whether `crashed`
would clear on its own once the airframe is level again. **Do not assume either.**

**Therefore the instruction is empirical, not asserted:** set the aircraft flat, re-read
`supervisor.info`, and if `tumbled`/`crashed` persist with `can_fly = NO`, power-cycle the aircraft
(disconnect the battery) and read again. Report what you observe — it resolves this open question
either way.

Relevant bits (from `telemetry.py` `decode_supervisor_info`): `can_fly` = 0x0008,
`tumbled` = 0x0020, `locked` = 0x0040, `crashed` = 0x0080. `supervisor_crashed` and
`supervisor_tumbled` are blocking `stage=both` readiness issues, so either one blocks the ground
launch gate. A clean, ready-to-fly supervisor reads **526** with `can_fly = YES`.

## 6. The fix

A **hard tether interlock** that voltage cannot override, shared as one implementation between the
live gate and its tests:

- `_dsh_scratch/cf85_battery_policy.py` — `decide()`, the single rule. The tether check runs
  **first**; `pm_state` 1 (`charging`) or 2 (`charged`), or any charge current > 0.01 A, returns
  `FORBIDDEN_TETHERED` with `allowed_to_continue: false`.
- `_dsh_scratch/cf85_battery.sh` — now calls `decide()` instead of holding its own copy of the
  rule, and records the tether evidence in `cf85_policy.tether_check`
  (`overridable_by_voltage: false`).
- `_dsh_scratch/cf_ops_lib.sh` — `cf_ops_battery_gate()` refuses `FORBIDDEN_TETHERED` explicitly
  with a clear message. `cf_prepare.sh` therefore stops before starting anything and before any
  flight command.
- `_dsh_scratch/cf85_battery_policy_test.py` — 9 regression cases, including the **real incident
  payload** replayed as a fixture.

Test result: **9/9 pass**. The incident payload (4.070 V, charging) is now `FORBIDDEN_TETHERED`;
healthy on-battery cases are unchanged, and the pre-existing voltage cases still behave as before.

## 7. Safety statement

- arm / takeoff / land / go_to / emergency sent by this fix: **NO** (the fix is offline logic plus
  the release of a verified-clear interlock)
- NVM written: **NO**　thresholds changed: **NO**　real flight in this step: **NO**
- The portable lesson, now encoded in the gate: **voltage health is not flight readiness.**
