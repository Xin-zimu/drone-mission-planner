# Incident Report — 2026-10-14 16:32 CF8 takeoff: airframe tilted, tumble-and-crash

**Severity: real hardware event, aircraft survived. Root cause: a deformed airframe**
**(operator-confirmed "机翼之前有形变，所以会飞不平") produced a static thrust-axis tilt that**
**drove the aircraft into a −X drift on spin-up, tumbled at +0.2 s, and crashed. This is a**
**mechanical-failure incident, not a software-gate defect.**

Run directory: `D:\Deepseek WorkSpace\cf_evidence_runs\20261014-162838-prepare`

---

## 1. What happened

Pre-flight was fully green: `cf85_level_check.py` reported LEVEL (accelerometer tilt 1.3°),
`pm.state=0`, `pm.vbat=4.107 V`, `supervisor.info=526` (`can_fly=YES`, no tumbled/crashed).
`cf_prepare.sh` reached `READY_FOR_CF8` (Battery / Radio / Link / Preflight all PASS).

`run_cf8_acceptance.sh` fired the one authorised attempt. The aircraft lifted, drifted toward
−X, tumbled at +0.2 s, and crashed. Timeline (monotonic seconds, takeoff command = t0):

| time | event |
|---|---|
| t0+0.000 | preflight passed, origin captured, takeoff requested (target 0.25 m) |
| t0+0.010 | takeoff accepted (motors spin up) |
| **t0+0.212** | **supervisor 526 → 286 — `tumbled` latched** |
| t0+0.767 | `xy_drift_exceeded` triggered (drift 0.0946 m vs 0.15 m policy) |
| t0+0.777 | landing accepted |
| t0+3.145 | **controlled landing FAILED — "supervisor revoked the flyable state"** |
| t0+3.145 | acceptance failed |

Key readings:

| item | value |
|---|---|
| supervisor trajectory | **526 (healthy) → 286 (`tumbled`) → 164 (`tumbled`+`crashed`, `can_fly=NO`)** |
| XY drift at trigger | 0.0946 m (policy `xy_drift_tolerance_m`=0.15 m, but drift velocity exceeded the settle band) |
| trigger velocity | vx=−0.321, vy=+0.209, vz=+0.002 m/s (0.383 m/s) — **−X dominant** |
| min battery under load | 3.568 V (`battery_critical=true`, `battery_critical_airborne=false` — debounce 0 samples) |
| post-crash pose | z=1.49 m, speed 6.30 m/s (vz=4.68) — this is the **tumble**, not the drift |

---

## 2. Root cause — deformed airframe (mechanical, not software)

The operator confirmed **the frame has a pre-existing deformation** ("机翼之前有形变，所以会飞不平").
A deformed frame tilts the thrust axis and/or shifts the centre of mass off the geometric centre,
producing a **constant roll/pitch moment** the moment motors spin up.

The offline quantifier `_dsh_scratch/cf85_deformation_quantify.py` reads every
`flight-result.json` and extracts the takeoff→trigger drift window. Across the **three** real
takeoff attempts, the drift direction is **consistent**:

| run | failure | drift (m) | direction | vx (m/s) | vy (m/s) |
|---|---|---|---|---|---|
| 20261014-145822 | battery_critical | 0.056 | 177° | −0.50 | +0.11 |
| 20261014-160201 | xy_drift_exceeded | 0.187 | 159° | −1.35 | −0.03 |
| 20261014-162838 | xy_drift_exceeded | 0.266 | −176° | −0.32 | +0.21 |

- **circular spread of drift direction = 10.6°** (mean 173.5°)
- **−X velocity component: 3/3 attempts (100%)**

A direction stable to 10.6° across three independent attempts (different failure modes:
battery abort, tethered crash, frame crash) is the signature of a **static thrust-axis tilt**,
not takeoff-transient noise. The tilt pushes the aircraft toward **−X**.

**This is the third distinct root cause in one day, and it is mechanical:**

| incident | root cause | type |
|---|---|---|
| 14:58 aborted takeoff | 3.7 V threshold compared loaded vs rested voltage | software (fixed: ground 3.7 / airborne 3.3) |
| 16:03 tethered crash | takeoff authorised while on USB charger | software (fixed: hard tether interlock) |
| **16:32 tumble-and-crash** | **deformed frame → static thrust-axis tilt** | **mechanical — NOT fixable in software without bounded trim** |

---

## 3. Why the crash was unavoidable on THIS airframe

Even a mild frame deformation breaks the hover precondition: the four thrust vectors must
combine into a single force equal to gravity, vertical, and through the centre of mass. A tilted
arm makes one motor's thrust non-vertical, adding a constant horizontal component the onboard PID
must cancel with a steady motor-speed imbalance (trim). Two regimes:

- **mild deformation** → the required trim is small, well inside motor saturation → the PID can
  hold level. **Compensable.**
- **severe deformation** → the required trim drives one side to saturation → the PID cannot
  cancel the moment → the aircraft tips over the instant it leaves the ground. **Not compensable.**

This aircraft tumbled **+0.2 s after takeoff accepted** — before the position controller could
even observe drift, the attitude diverged. That points to a trim requirement near or beyond what
the PID can deliver, i.e. the deformation is at least borderline. The quantification in §2 proves
the tilt exists and points **−X**; the +0.2 s tumble proves the required compensation is **not**
currently being delivered.

---

## 4. What this means for a "trim" fix — the honest engineering read

The operator asked whether an algorithm could compensate ("换不了，能不能想办法写个算法"). The
answer is **conditional and must be validated offline before any hardware write**:

1. **A static roll/pitch trim is the correct, industry-standard mechanism** — Crazyflie firmware
   exposes trim parameters, and Bitcraze's cfclient has roll/pitch trim sliders exactly for this.
   No custom control loop is needed or appropriate.
2. **The trim direction is known (−X, i.e. a roll trim).** The offline quantification gives this
   cleanly (10.6° spread).
3. **The trim magnitude is NOT yet known, and that is the deciding quantity.** If the required
   trim exceeds the motor's available headroom, no software can fix it — the frame must be
   replaced. The magnitude must be measured with a **ground spin-up calibration**
   (low-throttle, motors spin, aircraft does not leave the ground, zero tumble risk), NOT inferred
   from crash telemetry.

**Hard constraints that remain in force:** no threshold/gate/estimator-parameter changes;
`hardware_flight_enabled` stays false; one authorisation = at most one
`run_cf8_acceptance`; any NVM/parameter write (including writing a trim parameter) requires
explicit per-action user authorisation and is out of scope for this report.

---

## 5. Aircraft status after the incident

- supervisor latched **164** (`tumbled`+`crashed`, `can_fly=NO`) — as with the 16:03 incident,
  this is cleared by a **power cycle** (ROS-stack restart does not clear it; confirmed earlier).
- Operator confirmed the aircraft has been **powered off** (latch will clear on next power-up).
- Operator confirmed **props are intact; only the frame is deformed** ("桨叶完整，只有机翼形变").
- `hardware_flight_enabled` restored to **false** by the script (verified in output).

---

## 6. Safety statement

- arm / takeoff / land / go_to / emergency sent this session: **NO / YES (1 takeoff) / YES (1 land) / NO / NO**
- NVM written: **NO**　thresholds changed: **NO**　real flight: **YES** (the aircraft genuinely lifted and tumbled)
- The portable lesson, joining the day's earlier ones: **software gates certify readiness, not
  airworthiness.** A deformed frame is invisible to every gate we have — only the physical
  pre-flight check ("机翼有形变") can catch it, and that check now needs to gate *whether we fly at
  all*, not just *how*.
