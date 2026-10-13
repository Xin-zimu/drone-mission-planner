# CF8 Crazyflie Handoff — 2026-09-22

> 本文件是 **2026-09-22 夜间会话的完整交接**。目标是：一个全新的 DeepSeek Harness 会话**只读这一份文件**即可恢复上下文，不需要重跑任何历史诊断。
> 阅读顺序建议：§1 现状 → §5 电池结论 → §7 今晚执行记录 → §12 下一会话流程 → §4 安全不变量。

---

## 1. 一句话现状

**CF8 真机验收仍未通过。** 直接阻断原因是 **电池在负载下严重压降**：静置 4.043 V 的电池在电机起转瞬间跌到 **3.457 V**（sag **0.586 V**），0.828 s 后触发 `battery_critical` → 受控降落 → `landed`。Flow Deck 光流问题**已通过重新插接修复**（当前证据显示正常）。机体另有**机械变形疑点**待人工检查。**当前不授权任何真实飞行。**

---

## 2. 环境与连接

| 项目 | 值 |
|---|---|
| 主机 | Windows（D: 盘）+ WSL2 Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 飞控栈 | Crazyswarm2（`~/ros2_ws`），python venv `~/crazyswarm-env` |
| 机体 | `cf231` |
| URI | `radio://0/80/2M/E7E7E7E705` |
| 电台 | Crazyradio 2.0，usbipd BUSID `1-3`，USB ID `1915:7777`（**当前已 attach**） |
| 定位 | Flow Deck v2（PMW3901 光流 + VL53L1x 测距）；**无** Lighthouse / Loco / DWM1000 / 运动捕捉 |
| DMP Bridge | NDJSON over TCP `127.0.0.1:8765`，`protocol_version=dmp-cf/1`，`bridge_version=0.1.0-cf3` |
| Bridge 消息 | `hello`, `ping`, `get_capabilities`, `get_telemetry`, `select_robot`, `load_mission`, `run_preflight`, `run_cf8_acceptance`, `get_cf8_acceptance`, `execute_mission`, `abort_land`, `emergency_stop`, `clear_mission` |
| 话题 | `/cf231/status`（1 Hz）、`/cf231/pose`、`/cf231/odom`、`/cf231/dmp_readiness`、`/cf231/dmp_flow`（后四者 10 Hz） |

**电台互斥铁律**：cflib 直连探针与 Crazyswarm2 **绝不能同时占用 Crazyradio**。任何 cflib 探针必须在 Crazyswarm2 停止时运行，反之亦然。

若 `lsusb` 看不到 `1915:7777`：`usbipd detach --busid 1-3` → 保活 → `usbipd attach --wsl --busid 1-3`。

---

## 3. 仓库与 git

| 项目 | 值 |
|---|---|
| repo | `drone-mission-planner`（`D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`） |
| branch | `codex/v1.3-crazyflie-execution` |
| **HEAD（基准 checkpoint）** | **`21c668c`** |
| 本地领先上游 | `a195c51` 之上共 4 个 commit（`76c0847`, `4e0bdaa`, `5e8bc79`, `21c668c`） |
| push 状态 | **未 push（明确要求保持）** |
| 工作树 | 干净；仅 2 个未跟踪交接文档（`CF8_1_Crazyflie_handoff_2026-09-21.md`、`DeepSeekHarness_CF8_1_prompt_v2.txt`） |
| 最近提交 | `5e8bc79` CF8 failure_telemetry 观测性修复；`21c668c` 首次真机尝试 + 观测性修复的文档检查点 |

`21c668c` 之上**不应再有代码改动**；今晚没有修改任何 product code。

---

## 4. 安全不变量（硬约束，任何会话都不得违反）

1. `hardware_flight_enabled` 必须以 **`false`** 提交与保存；仅在**明确授权的一次 CF8** 期间临时置 `true`，飞行后**立即恢复 `false`**。
2. **不得**为了让测试通过而降低任何阈值，尤其是 **`battery_critical_voltage = 3.7 V`**。
3. **不得**修改 gate、TTL（`capability_snapshot_max_age_s = 3600`）、estimator 策略（`max_estimator_variance_span = 0.002`）。
4. **不得**绕过 / 删除 readiness gate；**不得**用 emergency stop 替代受控降落（CF8 必须走 `_abort_or_fault()` → `_land()`）。
5. **不得**运行 `hello_world`、硬件 `go_to`、硬件 `execute_mission`、CF9 相关路径。
6. **不得**在未获**明确单次授权**时发送 `run_cf8_acceptance`；**不得自动重试**，一次授权 = 最多一次尝试。
7. cflib 探针与 Crazyswarm2 **不得同时占用电台**。
8. 写代码改动前必须 `git status --short` + `git diff`；**禁止** `git reset --hard` / 覆盖式 `checkout`；**保留**所有本地未提交改动。
9. pytest 必须加 `--basetemp=/tmp/dmp-pytest`（仓库内 `.pytest-tmp` 的 ACL 已损坏）。
10. **push / merge 需明确要求**，默认不做。

---

## 5. 电池与供电（当前最大确定问题）

### 5.1 事实

| 时间 / 来源 | 静置电压 |
|---|---|
| 2026-09-21 会话首次真实充电后 | 4.164 V |
| 两次"已充满"但无效期间 | 3.816 → 3.837 V |
| 2026-09-22 起飞前（FAST PRECHECK） | **4.043 V** |
| **起飞瞬间（failure_telemetry 实测）** | **3.457 V** |
| 落地后 | 4.032 V |
| 异常后只读探针 | 4.028 V |

- **sag = 0.586 V**（本次）与 **≥0.111 V**（上次，实际值当时未被记录）。
- `pm.state = 0`（未在充电 / 未接 USB 外部电源），`pm.chargeCurrent = 0`，`pm.extVbat = 0`。
- 固件自身阈值：`pm.lowVoltage = 3.2 V`、`pm.criticalLowVoltage = 3.0 V`；DMP 的 3.7 V 更保守。

### 5.2 结论

**该电池内阻已高到无法提供起飞电流。** 仅凭静置电压判断电池健康是**错误**的——两块读数漂亮的静置电压（4.04–4.10 V）都在负载下崩到 3.457 V。

### 5.3 下一会话优先检查（用户执行）

电池本体 / 电池插头（1S 插接）/ 线材 / 焊点。若继续使用当前电池，**必须先验证负载压降**（例如电机起转瞬间的 `pm.vbat` 最低值），再考虑任何飞行授权。

### 5.4 允许讨论、但不许拿来掩盖 sag 的事

preflight 的"静置电压门槛"可以重新讨论；**绝不可以**用降低 `battery_critical_voltage` 的方式掩盖压降。

---

## 6. 硬件状态

### 6.1 Flow Deck（已修复，但必须保留故障签名认知）

**历史故障签名（2026-09-22 23:56 那次 PRECHECK BLOCKED 时实测）：**

```
dmp_flow 原始块： [0, 0, 58, 0]   ← 完全冻结（deltaX=0, deltaY=0, motion=58, squal=0）
motion.squal   ： 0（996/996 样本全零）
kalman.varPX   ： 饱和到固件上限 100.0（span 99.99998）
kalman.varPY   ： ≈71
ESTKALMAN      ： 58 秒内 16 次 "State out of bounds, resetting"
readiness      ： estimator_not_converged (stage=both, blocking=True) → ground_passed=false
位置估计       ： x=39.3 m / y=93.0 m，静止时 speed=5.14 m/s（发散）
```

**重新插接后的健康签名（实测）：**

```
dmp_flow 原始块： [0,-5,176,45] [-11,0,176,9] [5,-5,176,39] …（动态、变号）
motion.squal   ： 9–47，mean≈31–35，29/29 非零，最长连续 0 = 0
kalman.varP*   ： ~1.0–1.7e-4（远离 50/100）
estimator_not_converged ： 消失
ground_passed  ： true
```

**当前结论：没有证据表明 Flow Deck 仍处于失效状态。**

### 6.2 机体机械疑点（用户肉眼观察，待人工检查）

用户观察：**起飞效果不好、没有明显垂直升高、表现为低空快速水平移动**；怀疑机体受挤压、结构轻微弯曲。

**下一次飞行前必须先检查（用户执行）：**
1. 四个电机 / 机臂是否**共面**；
2. 桨叶是否变形；
3. Flow Deck 与机体是否平整贴合。

**暂时不要**通过调控制参数去补偿机械变形。

### 6.3 机体当前物理状态（2026-09-22 收尾时，只读 cflib 探针）

- `stateEstimate.z ≈ -6e-5 m`、`range.zrange = 0` → **在地面**
- `supervisor.info = 526`：`flying=false`、`hl_traj_finished=true`、`armed=true`、`can_fly=true`；**`crashed` / `tumbled` / `locked` / `deck_fault` 全为 false**
- 电池 4.028 V，仍开机（**建议关闭**）
- `kalman.varPX ≈ 1.05e-4`

---

## 7. 今晚（2026-09-22）执行记录

### 7.1 时间线（本地 CST = UTC+8）

| 本地时间 | 事件 | 结果 |
|---|---|---|
| 23:56–23:58 | 首次 FAST PRECHECK（Flow Deck 故障期） | **PRECHECK BLOCKED**：`(0,0,58,0)`、squal=0、varPX 饱和、16 次 ESTKALMAN、`estimator_not_converged`。**未发送任何飞行命令** |
| 00:03–00:04 | Flow Deck 重新插接后的 3 s 健康检查 | 6 项中 5 项 PASS；ESTKALMAN = 1（**启动瞬态**，发生在 `Requesting parameters` 之前） |
| 00:06–00:08 | 9 项稳态 gate | 报告 FAIL，但原因是**我方工具条件 7 判据错误**（用绝对 \|xy\| 当 runaway 指标），非机体故障 |
| 00:23 | FAST PATH + 单次极短 preflight | **CF8 FAST PRECHECK = PASS（10/10）** |
| 00:24:18 | 发送 `run_cf8_acceptance`（`confirm_real_flight=true`，严格 JSON 布尔） | 一次且仅一次 |
| 00:24:19.577 | **0.828 s 后 `battery_critical`** | 触发瞬间电压 **3.457 V** |
| 00:24:19.577 | `controlled_land_started` → `/cf231/land` | 受控降落路径生效 |
| 00:24:25.660 | `landed_confirmed`（+6.811 s） | 机体当时确在地面（z≈0.0003，zrange 0） |
| 00:24:42 | 恢复 flag=false、停 recorder/Bridge/Crazyswarm2 | 收尾完成 |

### 7.2 第二次 CF8 的权威事实

| 项目 | 值 |
|---|---|
| 结果 | **FAIL** |
| `failure_code` | **`battery_critical`** |
| `state` | `landed` |
| `session_id` | `3bc16610-7757-48d5-bd44-aa6f262198b0` |
| origin | x `1.09485`, y `-0.34300`, z `0.000872` m |
| target_z | `0.250872` m |
| 起飞 | `takeoff(height=0.250872 m, duration=2.500000 s, group_mask=0)` |
| 降落 | `land(height=0.000872 m, duration=2.500000 s, group_mask=0)` |
| **service 调用次数** | `/arm` **0**、`/takeoff` **1**、`/land` **1**、`/emergency` **0**、`/go_to` **0** |
| 故障→降落下发 | **0.828 s** |
| 降落下发→落地确认 | 5.96 s |
| STABILIZING | **未进入**（`metrics.stabilization = null`） |
| HOVERING | **未进入** |
| timeout / operator abort / emergency / session change | 全部 **NO** |
| `failure_telemetry.battery_voltage` | **3.457 V**（`battery_critical_voltage` 3.7） |
| `metrics.min_battery_voltage` | 4.043 V（**故障前**最后一个被记录样本，**不是**触发值） |
| `metrics` 其他 | `max_xy_drift 0.0494`、`max_z_error 0.2508`、`max_speed 0.1012`、`max_status_age 1.060`、`max_pose_age 0.0822` |

`failure_telemetry` 完整字段见 `cf84-flight-result.json`，含：`battery_voltage`/`battery_critical`/`battery_critical_voltage`/`pm_state`/`z_m`/`x_m`/`y_m`/`velocity`/`supervisor_info_raw`/`supervisor`(解码)/`status_age_s`/`pose_age_s`/`estimator_variance`(varPX/PY/PZ)/`range`/`positioning_mode`/`connected`/`pose_stream_available`。

### 7.3 三次真机尝试对照

| | 第 1 次（09-21 22:30） | 第 2 次（09-22 00:24） |
|---|---|---|
| `failure_code` | `battery_critical` | `battery_critical` |
| 触发延时 | +0.207 s | +0.828 s |
| 起飞前静置 | 3.811 V | 4.043 V |
| 触发电压 | **未记录**（当时无 `failure_telemetry`） | **3.457 V** |
| 已知 sag | ≥0.111 V（仅下界） | **0.586 V** |
| STABILIZING | 未进入 | 未进入 |
| 受控降落 | 是 | 是 |

---

## 8. ⚠️ 重要更正（避免下一会话重蹈）

1. **`landed_confirmed` 之后 1.35 m 的"再次升空"不是无人机自主起飞。**
   **用户已明确说明：CF8 结束后立即用手拿起了 Crazyflie。** 因此 `landed_confirmed` 之后的 z / zrange / velocity 数据**受到人工搬动污染**。
   → **不得**将这一段作为"自主再次起飞"的证据；**不需要**基于它开启 landing termination 紧急整改。
   → 后续分析飞行数据时，**必须截断 operator pickup 之后的 recorder 样本**（本次约在 recording 的 t≈+13 s 之后，即 2026-09-21T16:24:31Z 之后需谨慎对待）。
2. 我曾错误声称"Crazyswarm2 的 ROS 日志不含固件 console，`grep ESTKALMAN` 是空检查"。**这是错的**：`crazyflie_server` **确实**转发 console，例如
   `[crazyflie_server-1] [INFO] [1790006611.660378123] [crazyflie_server]: [cf231] ESTKALMAN: State out of bounds, resetting`。
   上次 grep 到 0 是因为**那次确实没有 reset**，检查本身有效。日志是**带缓冲**的，被 kill 的进程可能丢失尾部行 → 计数可能**低估**。
3. `pose_stability.xy_span / xy_displacement` 是 **airborne-only** 判据，**不是** ground blocker；严格的 XY ≤ 0.03 m 由起飞后的 **STABILIZING** 独立窗口执行。
   → 我曾错误地使用"绝对 \|xy\| < 1.0 m"和"`pose_stability` 位移 < 0.5 m"作为地面判据，**两者都是我自己发明的、不是生产 policy**，不得再用。
4. `metrics.min_battery_voltage` **不等于**触发值（故障在 health check 中提前 return，`metrics.update()` 未执行）。**触发值只看 `failure_telemetry.battery_voltage`**。这一点已在第二次飞行中被实证（4.043 vs 3.457）。
5. recorder 的"活性判定"不得比较 progress 日志的首行/末行；正确做法见 §12。

---

## 9. 生产语义备忘（不得随意改动）

- **Readyiness stage**：26 个 issue code = 22 个 ground-safe（含 9 个 supervisor）+ 4 个 airborne-only。
  - **airborne-only**：`pose_stability_observation_missing`、`pose_stability_failed`、`velocity_not_settled`、`range_invalid`
  - ground gate 只消费 `ground` + `both`；`_preflight_recheck()` 与 `protocol_server._preflight()` 跳过 `stage == "airborne"`
- **estimator 收敛**：使用 **`kalman.varPX/varPY/varPZ`（速度协方差）**，不是 `varX/Y/Z`（位置协方差，flow-only 下会饱和到 100 且 span 恒为 0）。饱和保护 `estimator_max_covariance=100.0` × `estimator_saturation_fraction=0.5` → 护栏 50.0。
- **CF8 状态机**：`PREFLIGHT_RECHECK → (ARMING) → TAKING_OFF → _wait_for_settle() → takeoff_target_reached → STABILIZING → HOVERING → LANDING → LANDED`
- **STABILIZING 参数**：`minimum_airborne_observation_s=5.0`、`stabilization_window_s=3.0`、`stabilization_timeout_s=10.0`；严格最大值语义（`max|vx|,max|vy|,max|vz| ≤ 0.05`、`max speed ≤ 0.08`、`xy_span ≤ 0.03`、`xy_displacement ≤ 0.03`、`0.10 ≤ zrange ≤ 0.50`）
- **CF8 policy**：`takeoff_delta_m=0.25`、`takeoff_duration_s=2.5`、`hover_duration_s=3.0`、`land_duration_s=2.5`、`z_acceptance_tolerance_m=0.08`、`xy_drift_tolerance_m=0.15`、`max_total_acceptance_time_s=30.0`、`cf8_explicit_arm_required=false`（auto-arm）
- **术语**：`squal` = PMW3901 光流质量（`motion.squal`）；`varPX/PY/PZ` = Kalman 速度协方差；`ground_passed` = 地面门（跳过 airborne-only）；`failure_telemetry` = 故障触发瞬间快照（仅 PASS 时为 `null`）

---

## 10. 工具与脚本（全部在 `D:\Deepseek WorkSpace\_dsh_scratch\`，仓库外）

| 脚本 | 用途 |
|---|---|
| `session_wrapup_check.sh` | 收尾只读核查 + /tmp 证据归档 |
| `check_idle_state.sh` | 进程 / 8765 / 电台 / flag 只读核查 |
| `run_cf84_recorder.sh` | 独立只读 recorder（本会话用；输出 `cf84-recorder.*`） |
| `cf84_counts_check.py` | recorder 活性判定（ready-file counts vs 当前 counts，等待约 1 s） |
| `cf84_fast_path.sh` | FAST PATH 编排：recorder → Crazyswarm2 → 活性确认 → Bridge(false) → 单次 preflight |
| `cf84_preflight.py` | 唯一一次极短 preflight（含 ~2 s squal 读取 + Bridge 只读裁决 + ESTKALMAN 计数，stage-aware） |
| `cf82_bridge_true.sh` | 置 `hardware_flight_enabled=true` 并**重启** Bridge（该标志仅在构造时读取） |
| `cf8_live_run.py` | 发送**一次** `run_cf8_acceptance`（严格 `confirm_real_flight: true`）并轮询结果 |
| `cf84_postflight.sh` | 落地后：读末态 → flag=false → 停栈 → 优雅停 recorder → 服务计数 |
| `cf84_state_probe.py` | 只读 cflib 状态探针（`stateEstimate.*` / `range.zrange` / `pm.vbat` / `supervisor.info`） |
| `read_battery_probe.py` | 只读 cflib 电池探针（`pm.*` 参数 + `pm` 日志块） |
| `cf8_flight_recorder.py` | 通用只读 recorder（惰性 ROS 导入、离线回放、SIGTERM 优雅落盘） |
| `run_cf8_recorder.sh` / `run_cf8_recorder_replay.sh` | recorder 的 WSL 包装（消除 PowerShell `$HOME` 引号问题） |
| `cf8_recorder_smoke_test.py` | recorder 离线 smoke test（**109 项检查 0 失败**） |
| `cf8_retro_verify.py` | 第一次飞行复盘核对（14/14 PASS） |
| `cf84_series_dump.py` | 从 timeline 提取 z / zrange / speed / battery 时间序列 |
| `cf82_blocked_evidence.sh` / `cf83_health_check.sh` / `cf83b_health_check.sh` | 历史 PRECHECK / 健康检查流程 |
| `start_crazyswarm2.sh` / `start_dmp_bridge.sh` / `stop_stack.sh` | 栈启停 |

**PowerShell 引号铁律**：不要在 PowerShell 内联命令里写 `$`（会被提前展开成 Windows 路径）。所有 shell 逻辑写进 `.sh` 文件，PowerShell 只传路径与参数。

---

## 11. 证据清单（`D:\Deepseek WorkSpace\cf81_dryrun_evidence\`）

### 今晚（第二次飞行，最重要）

| 文件 | 内容 |
|---|---|
| `cf84-flight-result.json` | **CF8 结果 + `failure_telemetry`（触发电压 3.457 V）** |
| `cf84-flight-window.json` | 飞行窗口聚合（max_z / 离地高 / 漂移 / 速度 / 电池 / squal / varP / zrange） |
| `cf84-recorder.ndjson` | **原始 timeline（623 行）**，含 timestamp/x/y/z/vx/vy/vz/speed/zrange/squal/varPX-PZ/battery/supervisor |
| `cf84-recorder.json` / `cf84-recorder-stdout.log` | recorder 全程汇总与 stdout |
| `cf84-preflight.json` | 起飞前 **FAST PRECHECK = PASS（10/10）** |
| `cf84-crazyswarm2-final.log` / `crazyswarm2-cf84-flight.log` | Crazyswarm2 日志（takeoff×1、land×1、链路告警、ESTKALMAN） |
| `cf84-final-state.json` / `cf84-post-incident-state.json` | 落地后与异常后的只读遥测 |
| `crazyswarm2-cf84-interrupted.log` | 被中断那次 FAST PATH 的日志（含 1 次稳态 ESTKALMAN） |
| `cf84-bridge-flight.log` | **0 字节**（Bridge 飞行期间没有 stdout 输出；权威证据是 `cf84-flight-result.json`） |

### Flow Deck 故障与修复期

| 文件 | 内容 |
|---|---|
| `cf82-precheck.json` | **PRECHECK BLOCKED**（15 项逐条） |
| `cf82-ground-probe.json` / `cf82-ground-probe-second.json` | **`(0,0,58,0)` 冻结 + squal 0 的原始证据** |
| `cf82-recorder.ndjson` | 故障期 timeline（997 行：max_xy_drift 110.8 m、max_speed 5.14 m/s） |
| `cf82-crazyswarm2.log` | 含 **16 条 ESTKALMAN reset** |
| `cf83-health-probe.json` / `cf83-readiness.json` / `cf83-crazyswarm2.log` | 重新插接后的健康检查（squal 9–47、varP ~1e-4、1 次启动瞬态 reset） |
| `cf83b-health-probe.json` / `cf83b-readiness.json` / `cf83b-gate.json` | 9 项稳态 gate（含我方条件 7 判据错误的记录） |

### 第一次飞行（09-21）

| 文件 | 内容 |
|---|---|
| `cf8-flight-result.json` | 第一次 CF8 结果（**不含** `failure_telemetry`，由改动前代码产生） |
| `cf8-postflight.json` / `crazyswarm2-flight.log` | 落地后遥测与飞行日志（arm 0 / takeoff 1 / land 1 / emergency 0） |
| `cf8-retro-verify.json` | 14 项复盘核对（含"历史结果不得含 failure_telemetry"的防改写校验） |
| `cf8-failure-retrospective.md` | 第一次失败的完整复盘文档 |

---

## 12. 下一会话 FAST PATH（唯一允许的飞行前流程）

**不再重复**大量 3 s / 30 s / capability / `pose_stability` 检查。

### 阶段 0：只读准备（不接机体）

1. 读本文件。
2. `git status --short` + `git log --oneline -3`，确认 HEAD 仍是 `21c668c`、工作树干净、未 push。
3. **等用户完成**：机体机械检查（电机/机臂共面、桨叶、Flow Deck 贴合）+ 电池/插头/线材/焊点检查（§6.2、§5.3）。**不要**在用户确认前上电或连接。
4. 只读核查无残留：`bash _dsh_scratch/check_idle_state.sh`。
5. **capability snapshot**：若 `runtime/cf231_capabilities.json` 年龄 > 3600 s，按 README 的**只读 cflib 流程**在电台空闲时刷新（**不改 TTL**）；若仍 fresh 则直接使用，**不要重复 probe**。

### 阶段 1：最短 FAST PATH（用户确认"现在上电"后）

```
1. 启动 recorder（run_cf84_recorder.sh，只读）
2. 启动 Crazyswarm2（start_crazyswarm2.sh）
3. 确认 ready-file 生成 且 recorder ACTIVE
   → cf84_counts_check.py --ready <ready文件> --log <recorder日志> --wait-s 1
   （判定 = ready-file counts vs 当前 counts；pose/odom/readiness/flow 均明显增加。
     不要比较 progress 首行/末行；不要因只有一条 progress line 就 FAIL）
4. 启动 Bridge（start_dmp_bridge.sh），hardware_flight_enabled 保持 false
5. 只跑一次 cf84_preflight.py（极短，~2 s squal + Bridge 只读裁决）
```

### 阶段 2：一次快速生产 preflight 的判定项

必须 PASS：
`connected=true`、`positioning_mode=flow`、`ground_passed=true`、无 `estimator_not_converged`、
`supervisor.safe_for_preflight=true`、本次连接后**无持续** ESTKALMAN reset、
`squal` 非零且未恒 0、静止 velocity 无 gross runaway、`battery ≥ 4.00 V`、无 ground/both blocking issue。

**明确不检查**（airborne-only，留给起飞后的 STABILIZING）：`pose_stability_*`、`velocity_not_settled`、`range_invalid`。
**明确不使用**：绝对 XY、`pose_stability.xy_displacement`、任何自造的 0.5 m pose 阈值、`run_preflight`/F3、重复 capability probe、重复 interlock。

**注意**：`battery ≥ 4.00 V` 只反映**静置**电压。经今晚证明，静置达标**不代表**电池能承受负载（4.043 V → 3.457 V）。若电池未更换/未验证负载压降，**即使 preflight PASS 也应先与用户确认是否值得消耗一次授权**。

### 阶段 3：仅在明确授权后

```
6. cf82_bridge_true.sh    # flag=true + 重启 Bridge（标志仅在构造时读取）
7. cf8_live_run.py        # 发送一次且仅一次 run_cf8_acceptance
8. 落地后 cf84_postflight.sh  # flag=false → 停栈 → 优雅停 recorder → 服务计数
```
**任何真实硬故障按现有 fail-closed 逻辑走 controlled land；不得自动重试。**

### 阶段 4：数据分析注意

分析 recorder 数据时**必须截断 operator pickup 之后的样本**（§8.1），否则会出现虚假的"自主升空"。

---

## 13. 待决事项（用户决策）

1. 更换 / 修复电池与插头，或先做负载压降验证。
2. 机体机械检查与（必要时）校正。
3. 是否重新讨论 preflight 静置电压门槛（可以讨论，但**不得**降低 `battery_critical_voltage`）。
4. 是否再次授权 CF8（一次授权 = 一次尝试）。
5. 既有未决技术项：`session_id` 从未在协议消息中暴露，而 `frame.origin_session_id` 需与其匹配 → flow 任务的 `run_preflight` 会因 `frame_origin_stale` 失败；**阻塞 CF9 任务路径，不阻塞 CF8**。已推迟。

---

## 14. 收尾状态（2026-09-22 结束时）

| 项目 | 状态 |
|---|---|
| `hardware_flight_enabled` | **false** |
| Bridge / Crazyswarm2 / recorder | **全部已停止** |
| direct cflib 占用电台 | **无** |
| 8765 | **空闲** |
| 飞行相关残余进程 | **无** |
| 机体 | 在地面（`stateEstimate.z≈0`、`range.zrange=0`），仍开机（建议关闭） |
| Crazyradio | 仍 attach（`1915:7777`） |
| git | HEAD `21c668c`，工作树干净（仅 2 个未跟踪文档），**未 push** |
| 阈值 / gate / TTL / estimator | **全部未修改** |
