# 会话日志（当前状态唯一事实来源）

> 这是**唯一**的当前状态页。任何新会话先读这里。
> 状态有变化就更新本文件；本文件与 git 工作树不符时以本文件为准并立即核查。

- 最近更新时间：2026-10-20（地面 PID 诊断跑通；4.10 V 过期文档修正；会话闸门经授权降至 3.50 V；发现并记录 F2b 估计器过积分）
- 主线目标：**CF8 真机验收通过**（首次真机 hover 起飞→悬停→降落一次且仅一次）
- 下一里程碑：M3（拿回飞机后恢复基础飞行能力）见 `learning-plans/路线联动推进计划.md`

## 0. 2026-10-20 最新核查（优先于下方历史状态）

### 本轮补充（用户“一切准备就绪”之后）

- 通过 run_probe.sh 单次读取当前机体，连接前检查电台无占用，结束已关闭连接。当前 controller=1，529 个日志变量；最新 pm.vbat=4.048680 V、pm.state=0、supervisor.info=526。此次读数替代下面较早的 3.800586 V。
- 依 cf8-operations.md §4，实测 4.048680 V **已达会话闸门 ≥4.00 V**（4.10 V 是文档过期值，已于 2026-10-20 修正）；本轮仍未运行 prepare/preflight、未写参数、未发送电机或飞行指令。
- 新增仓库外工具 `_dsh_scratch/cf85_pid_diagnostics.py`，已登记工具索引。inventory/build 实测成功；record 已于本轮地面验证通过。
- 证据目录 `D:/Deepseek WorkSpace/cf_evidence_runs/20261020-pid-ground/`：live-before.json、offline-validation.json、preset/manifest.json、crazyflies.original.yaml 与 crazyflies.pid.yaml（后两者均在 preset/ 下）。原 WSL crazyflies.yaml SHA256 保持不变。
- 独立配置将 all.firmware_params.stabilizer.controller 从 2 改为 1，追加六组日志：实际/目标姿态、实际/目标角速度、电机输出/请求、控制输出及电压/状态、实际位置速度、目标位置速度。日志块经本机实时 TOC 核对，均不超过 26 字节；PID pitchRate 比较使用 -gyro.y。电机输出不等于实测 RPM/推力。
- 离线 10 项检查通过，覆盖原文件未变、PID 配置、缺失变量/超长数据包/高优先级控制器冲突拒绝、缺失/低速/中断日志拒绝。mypy 通过；Ruff 的 pairwise 建议已修复并复核。

### 本轮完成（2026-10-20 20:2x，地面 PID 诊断已跑通）

- **4.10 V 过期文档已修正为 4.00 V**（`cf8-operations.md` §2/§4）。核实：`cf_ops_lib.sh` 与 `cf84_preflight.py` 早在 2026-10-14 就是 4.00 V，唯一残留 4.10 的是 `*_bak-20261014-threshold` 备份。
- **会话闸门经用户明确授权降到 3.50 V**（`BATTERY_FORBID_V` / `BATTERY_DIAG_ONLY_V`）。理由：实测 3.9167 V 时 4.00 V 连**不起桨**的地面诊断都拦住。**生产阈值一律未动**：`hardware_flight_enabled=false`、`battery_critical_voltage=3.7`、`battery_critical_airborne_voltage=3.3`、`minimum_battery_critical_samples=3`、固件 `pm.lowVoltage=3.2`/`criticalLowVoltage=3.0`、preflight `MIN_BATTERY_V=4.00`。已用 `cf85_gate_check.sh` 逐项核对。
- **踩过的坑**：`cf85_battery.sh` 在子 bash 里 source `cf85_lib.sh`，只改 `cf_ops_lib.sh` 会被子进程的 4.00 覆盖，第一次跑直接 FAIL。两个库现均为 3.50，并加了防漂移检查。
- **`recovery_required` 已按审计方式释放**：先逐项证明（无栈进程 / 8765 空闲 / flag=false / 无启用 flag 的 runtime 配置 / 实时 `supervisor=526` tumbled+crashed 均清除），再把标记**归档**为 `recovery_required.released-20261020-202131`（不删除）。它原本记录的是 10-14 翻倒事故的 `safe_to_stop=false`，而那个栈早已不存在。
- **地面运行成功**：`cf_evidence_runs/20261020-202302-pid-ground/`。五路链路 30 s 全部达标（pose/odom/dmp_flow/dmp_readiness 各 10.4 Hz，status 1.03 Hz）；六组 PID 流全部通过（20 Hz 段实测 20.66 Hz，10 Hz 段 10.33 Hz，最大间隔 ≤0.10 s）。
- **控制数据**：`controller.roll/pitch`（目标）全程恒 0、`actuatorThrust` 恒 0、`motor.m1..m4` 与 `m*req` **全部为 0** —— 地面确实没有起桨。实测姿态 roll 0.28°/pitch −0.06°（均值），电池 3.8231 V。
- **发现新故障签名 F2b（已写入故障库）**：静止时机体把 flow 位移过积分，`stateEstimate` 在 22 s 内累积到 x=32 m / y=97 m 后被固件钳回原点；`launch.log` 出现 2 次 `ESTKALMAN: State out of bounds, resetting`。`z` 全程贴地 0–0.016 m、`bcFlow2=1`、`dmp_flow` 10 Hz 正常 → 不是 F2 冻结，也不是 F3 未枚举。历史对照：翻倒事故那次 3 次、`20261014-143923` 5 次、其余多为 0，属**间歇性**。含义：这份记录的 `stateEstimate.x/y` **绝对值不可用**，只能看变化量。
- **该签名不阻断本轮结论但会拦飞行**：`cf84_preflight.py` 第 6 项本就把本次连接后的 ESTKALMAN 重置当阻断项，所以不会静默放过。

- 下一步：① 若要继续，先给电池充电并确认静置回升（本轮从 4.0487 掉到 3.82 V）；② F2b 需要抓 `dmp_flow` 原始 `[dX,dY,motion,squal]` 块才能定性（本轮未采）；③ 形变问题**未被本轮任何数据触达** —— 地面无推力，不证明补偿能力。

### 追加（2026-10-20 20:5x）：F2b 不能解释翻倒事故

- 提出并**证伪**了一个会改变优先级的重要假设："事故那 3 次 ESTKALMAN 重置发生在起飞瞬间，所以是估计器把飞机掀翻的"。
- 方法：把 `recovery.json` 的 monotonic 事件与 `launch.log` 的 unix 时间戳对齐到同一起点（新工具 `cf85_reset_timeline.py`）。
- 结果：起飞请求 +0.000 s → `xy_drift_exceeded` 判于 **+0.766 s** → **首次重置 +2.544 s** → `acceptance_failed` +3.145 s → 第二次重置 +5.652 s → 第三次 **+86.953 s**（降落后）。
- 结论：**重置比失控判定晚 1.78 s，是翻倒的后果而非起因。** "支架形变 → 推力偏斜"仍是事故主因，未被推翻。
- 意义：少了一条诱人的错误修复路线（调估计器/换 deck 去救翻倒），也确认 F2b 与事故无关、可与形变问题分开处理。

### 追加（2026-10-20 21:0x–21:1x）：充电完成；上电后 Flow Deck 掉线并已恢复

- **充电确认**：实测 **4.1067 V**、`pm_state=0`（线已拔）、`supervisor=526` 干净。这是自 10-14 以来首次同时越过会话闸门（3.50）与 preflight 飞行放行线（4.00）。
- **随即触发 F3**：充电后首次上电（uptime 155 s、4.10 V）`deck.bc*` **十四项全 0**、`stabilizer.estimator` 从 2 掉到 **1**、`stateEstimate.z` 跑到 **−17 m**；实时流测试 `squal=0`/`zrange=0`/delta 全 0。
- **F3 的旧归因被推翻**：故障库原写"低压引导导致"，但这次是 **4.10 V 满电**启动。已改写 F3 条目：低压只是诱因之一，**不是必要条件**。
- **恢复**：用户**拔电重插**后 `bcFlow2=1`/`bcZRanger2=1`、`estimator` 回到 2、`squal` 8–43、`zrange` 8–13 mm、`stateEstimate.z` 回到 0.011 m。第 1 步"充满电再上电"**无效**，第 3 步"拔电重插"**有效**。
- **新环境坑（已写入规范 §6）**：cflib `get_value()` 返回**字符串** `'1'`，`== 1` 静默为 False。本轮因此一度把健康的 deck 误判为缺失。判定方必须归一化，`cf85_pid_diagnostics.py` 只采集不判定是有意设计。
- 新增/修正工具：`cf85_postcharge_check.py`（充电后体检，含数值归一化）、`cf85_params_audit.py`（参数表完整性审计）、`cf85_deck_timeline.py`（跨快照 deck 状态序列）、`cf85_reset_timeline.py`（重置 vs 起飞时间轴）。
- **当前飞机状态**：4.054 V、`pm_state=0`、deck 正常、`estimator=2`、`controller=1`、姿态约 1.6°、`supervisor=526`。**可进入下一步的空中/推力实验前置**，但形变补偿仍未验证。

### 追加（2026-10-20 21:2x–21:4x）：真实推力测试 —— 结论是"电池先崩了"，不是形变

**两次尝试，第一次无效、第二次有效但被电源问题污染。两次都如实记录，都不作为形变结论。**

**尝试 1（无效，已作废）**：手写 `send_position_setpoint` + `send_setpoint`，**未启动 Crazyswarm2**。
飞机当时处于固件默认 `commander.enHighLevel=0`，位置指令根本不会被转成推力，**测试对象错了**。
另外我还犯了两个解读错误：把 `controller.actuatorThrust` 当 0..1 归一化值（实测可达 54231），
把 arm 瞬间暂态当成稳态饱和。产出 `NO_HEADROOM` 判定 —— **已明确作废，未写入故障库**。

**尝试 2（有效，但结论受限）**：经 `cf85_stack_for_takeoff.sh` 启动 Crazyswarm2（PID preset），
确认 `enHighLevel=1` 已写入、`/cf231/takeoff` 服务可用，限位下 **0.10 m / 3 s** 起飞。
降落正常返回。首次拿到"高层位置控制真正接管"的控制数据（300 样本）。

**决定性发现 —— 电源先崩，不是姿态失控：**

| 项目 | 实测 |
|---|---|
| 起桨前静置 | 3.926 V |
| 带载最低 | **2.845 V** |
| **带载 sag** | **1.081 V**（历史同型包为 0.5–0.7 V） |
| 低于固件告警 `pm.lowVoltage`=3.2 V | 30/33 样本 |
| **低于固件硬断电 `pm.criticalLowVoltage`=3.0 V** | **12/33 样本** |
| `controller.cmd_roll` 打满 ±32767 | 27.9% 样本 |
| 电机达满量程 65535 | 26 样本 |

**结论：本轮仍无法回答"飞控能否补偿形变"，但原因换成了更根本的一条 —— 这块电池在起飞电流下达不到
固件自己的保护阈值。** 控制器在电压塌陷时拿不到足够权限，所以无法把"机架太歪"与"电源供不上"分开。

**这改变了三次翻倒事故的解释框架**：10-14 两次 + 今天一次，此前都归因为"纯机械形变"。现在看，
**带载电压塌陷是共同存在的可疑因素**，不能排除它参与了翻倒。原 §5.6 的"单一机械根因"表述应视为待复核。

**安全边界（本轮遵守）**：`hardware_flight_enabled` 全程 false（起飞走的是直接 ROS 服务，非验收流程）；
未写参数、未写 NVM、未改任何阈值；全部产物在 `cf_evidence_runs/20261020-thrust/`。

**飞机状态**：测试后 `supervisor=644`（`crashed` 锁存、`can_fly=NO`，固件对非正常着陆的锁存），
静置电压回到 3.917 V、姿态 1.16°/0.20° 正常、无结构损伤。**需断电重插清除锁存**。

**下一步（按优先级）**：① 换一块已知健康的电池（本包 sag 1.08 V 已不可用于推力实验）；
② 用健康电池重跑同一限位测试，才能把形变问题单独隔离出来；③ 在此之前**不要再提高油门**。

#### 更正（2026-10-20 22:5x）：飞机**确实离地、确实飘、确实翻倒**

**上一条"未离地、未翻倒"的判断是错的，已作废。** 错因：我把飞行窗口定义成 PID 流的
`actuatorThrust > 1000`（只有 2.96 s），把真正的飞行段切在了窗口外。

按 `launch.log` 的 takeoff 时间戳对齐后，完整过程是：

| 相对起飞 | 事件 | 证据 |
|---|---|---|
| +0.00 s | 起飞指令 0.10 m | `takeoff(height=0.100...)` |
| +2.64 s | **链路开始丢包** | `Low unicast receive rate (0.84 < 0.90)` |
| +2.96 s | **离地** | `zrange` 0.057 m（直接测地距，不是估计值） |
| +4.46 s | 升到 0.35 m | `zrange` 峰值 **0.354 m** |
| +4.96 s | **EKF 开始发散** | `stateEstimate.z` 0.99→2.49 m，同期 `zrange` 恒 0.35 |
| +5.20 s | **固件判坠机** | `SUP: Crashed, recovery required` |
| +7.52 s | EKF 越界重置 1 | `ESTKALMAN: State out of bounds` |
| +10.10 s | EKF 越界重置 2 | 同上 |

**三层独立故障（必须分开修，不能混为一谈）**：

1. **电源**：`vbat` 最低 2.845 V < 固件硬断电 3.0 V，12/43 样本在阈值下。
2. **电台链路**：起飞后 2.64 s 丢包（0.84 < 0.90）；起飞前已有 `High latency 11.4 ms`。
3. **位置估计**：起飞后约 5 s 发散，`z` 与 `zrange` 差到 7 倍，随后两次越界重置。

**关键分辨：记录里的 `x span 14.37 m` / `speed 11.29 m/s` 不是机体运动。**
同期 `zrange` 钉在 0.35 m、机体没有相应上升，而 EKF 报 `z`=2.49 m —— **是 EKF 在给假数据**。
被系留的飞机也不可能达到 11 m/s（40 km/h）。因此 **EKF 崩溃之后的位置/速度数字不能用于
判断"飘了多远"**。但"离地、飘了、最后翻倒"这一事实由 `SUP: Crashed`、`zrange` 序列与目视
共同确证，不受影响。

**因此仍然无法回答"飞控能否补偿形变"**，但原因已经换了两轮：不是形变太严重，而是
**链路丢包 + 估计器发散 + 电源塌陷**这三者先发生。**这三者未被排除前，任何形变结论都不成立。**

**飞机当前状态**：`supervisor=526` 无锁存、`pm.state=1` 正在充电、4.175 V、姿态 2.77°/4.26°，物理完好。

### 较早的静态检查（历史读数）

- 用户确认：电机塑料支架固定歪斜、无裂纹；电池据用户检查无异常；支架可以更换，但希望先评估飞控。已明确回复“已上电，静放好了”，仅执行只读检查。
- Crazyradio `1915:7777` 在 `3-1`；WSL 初始未接入电台，现有 `run_probe.sh` 完成 USB/IP 接入。探针检查未发现 Crazyswarm2/bridge/recorder 占用；未启动飞行服务。
- 两次现有只读探针均正常退出并关闭机体连接：`cf85_trim_params_probe.py --timeout-s 20`、`cf85_level_check.py`。均经 `run_probe.sh` 串行执行，外层分别设 55 s / 45 s 超时，未重试。
- 机载当前参数：`stabilizer.controller=1`（PID）、`stabilizer.estimator=2`、`commander.enHighLevel=0`、`motorPowerSet.enable=0`、`powerDist.idleThrust=0`。PID 姿态 roll/pitch 的 Kp=6、Ki=3、Kd=0；角速度 roll/pitch 的 Kp=250、Ki=500、Kd=2.5。本轮没有修改这些值。
- 静止实测：`acc=(+0.016,+0.022,+1.003)`，模长约 1.004 g，机身 Z 轴与重力夹角约 1.6°；估计 roll=1.4337°、pitch=1.0473°。两类读数大体一致，不能据此推断电机轴倾角或带桨动态性能。探针自身“LEVEL”的宽泛判据不作为适航证明。
- 供电实测：`pm.state=0`，`pm.vbat=3.800586 V`。低于项目 4.00 V 的 FAST PATH 门槛；未运行 prepare、preflight 或飞行验收。此读数不证明电池损坏，也不证明带载健康。
- `supervisor.info=526 (0x020e)`：按现有探针解码，armed/auto_arm/can_fly 为真，tumbled/crashed 为假。armed 是固件报告状态，本轮未发 arm；can_fly 位不等于允许对变形机体试飞。
- 已核实历史启动日志 `D:/Deepseek WorkSpace/cf_evidence_runs/20261014-162838-prepare/launch.log` 第 20、23 行：服务将 `stabilizer.controller` 写为 2，固件报告 `Using Mellinger (2) controller`。因此“当前上电 PID”与“事故飞行 Mellinger”是两个不同状态，不能混为一谈。
- 官方历史资料提示 Mellinger 对定位误差更敏感、Flow deck 可优先评估 PID：https://forum.bitcraze.io/viewtopic.php?t=4826 。这是后续对照评估的依据，不是本机事故根因或 PID 可修复变形的证明；具体行为仍需核对本机固件和控制数据。
- 修正旧结论：同向漂移及已知支架歪斜不足以证明事故只有机械单一根因，更不能直接确定 roll/pitch trim 的方向或大小；当前缺少目标姿态、角速度及四电机输出记录。拆桨空转不能标定悬停气动力补偿。`motorPowerSet` 是覆盖电机输出，不是叠加闭环 trim，不应用于本任务的悬停补偿。
- 下一步：准备保持 ROS 高层链路、采用 PID 的可回退对照方案及控制诊断日志，先核对固件可用日志项；目前未修改启动配置、未写参数、未批准或执行试飞。用户可先结束本次上电并充电。
- 安全声明：arm=NO / takeoff=NO / land=NO / go_to=NO / emergency=NO；电机指令=NO；参数写入=NO；NVM=NO；阈值修改=NO；真实飞行=NO。
- 工作树仍含历史未提交改动；另有日志原来未列出的 `NEXT_SESSION_CF8_PROMPT_2026-10-14.txt`。本轮只更新本日志，不提交或覆盖历史改动。

## 1. 仓库状态

| 项 | 值 |
|---|---|
| 分支 | `codex/v1.3-crazyflie-execution` |
| HEAD | `a63fc7f` — docs(workflow): session log HEAD -> 18784fc |
| push 状态 | **ahead 8，未 push**（默认不 push，除非用户明确要求） |
| 工作树 | **有未提交改动（10 改 + 3 新增）**：电池判据分级（4 产品 + 2 测试）、4 个规范文档、3 份报告（`CF8_FLIGHT_REPORT_2026-10-14.md`、`INCIDENT_2026-10-14_tethered_takeoff.md`、`INCIDENT_2026-10-14_tumble_crash.md`）。`_dsh_scratch` 与证据目录在仓库外，不进仓库 |

## 2. 安全不变量（当前值，违反即停）

| 项 | 值 | 位置 |
|---|---|---|
| `hardware_flight_enabled` | **false**（未改） | `.../dmp_crazyflie_bridge/config/bridge.yaml` L7 |
| `battery_critical_voltage` | **3.7 V**（未改，**地面/静置**放行判据） | bridge.yaml |
| `battery_critical_airborne_voltage` | **3.3 V**（**2026-10-14 新增**，空中/带载判据） | bridge.yaml |
| `minimum_battery_critical_samples` | **3**（**2026-10-14 新增**，去抖：连续 N 个低样本才触发） | bridge.yaml |
| `capability_snapshot_max_age_s` | **3600**（未改） | bridge.yaml |
| `max_estimator_variance_span` | **0.002**（未改） | bridge.yaml |
| 电池会话政策 | **挂外电一律禁飞（硬互锁）**；`<3.50 V` 禁飞；`≥3.50 V` 放行地面诊断；"充满"但 `<3.90 V` 判可疑电池。**注意：这是会话闸门，不是飞行放行线** —— 飞行仍由 preflight 的 `MIN_BATTERY_V=4.00 V` 和空中 `battery_critical_airborne_voltage=3.3 V` 把关 | 见 cf8-operations.md §4 |
| 会话闸门 3.50 V 的由来 | **2026-10-20 用户明确授权**（原 4.00 V）。因为实测 3.9167 V 时 4.00 V 连不起桨的地面诊断都拦住。两个 lib（`cf_ops_lib.sh`/`cf85_lib.sh`）必须一致，`cf85_gate_check.sh` 防漂移 | 仓库外 `_dsh_scratch/` |

## 3. 硬件历史状态（cf231，最新读数见 §0）

| 项 | 状态（2026-10-14 16:35 结） |
|---|---|
| 电池 | 官方 250 mAh 1S；体检时静置 **4.107 V / 90%**，带载最低跌到 **3.568 V**（sag ~0.5 V 复现）。带载必 sag ~0.5 V |
| 电台 | Crazyradio 2.0 `1915:7777`，busid **3-1**（会随插拔变化） |
| 固件 | `a54f11eb6d00`（与 09-22 逐字节一致，未变）；`pm.lowVoltage=3.2`、`pm.criticalLowVoltage=3.0` |
| **Flow Deck** | **正常**：`bcFlow2=1`、`bcZRanger2=1`、`TYPE_1W`=1、`squal` 有实时数据 |
| **当前阻断点** | **机架形变（用户确认"机翼之前有形变"）→ 静态推力轴偏斜，起飞即朝 −X 漂移并翻倒。** 飞机已断电（supervisor 锁存 164 会在下次上电清除）。换不了机架 → 正在评估 trim 补偿可行性（见 §5.6） |
| 姿态（双通道） | 体检时 **LEVEL**：加速度计 `acc=(0.000,0.022,1.002)` tilt **1.3°**，估计器 roll 1.16° / pitch −0.11°。起飞后 +0.2 s 翻倒（`tumbled`）→ 姿态崩是**形变推力偏斜**导致，不是上电前姿态问题 |
| 锁存位演化（已闭合） | 526（起飞前健康）→ **286（16:32 起飞 +0.2 s，`tumbled` 先锁）** → **164（翻倒后，`tumbled`+`crashed` 双锁，`can_fly=NO`）**。已证：**断电重启清锁存**（本次断电后下次上电会回 526） |
| **已解问题** | 上一轮"tumbled/crashed 是否单向锁存"已闭合：**不是单向锁存，断电即清**（本会话 676→断电→体检 526 实证）。`tumbled` 是"翻倒"的运行期标志，不是永久损伤 |
| 最近三次真机尝试 | 14:58 空载中止（3.7 V 误触发）；16:03 边充电起飞事故（线缆拖拽）；**16:32 真起飞翻倒事故**（机架形变，`tumbled`+`crashed`，详见 `INCIDENT_2026-10-14_tumble_crash.md`） |
| 下一步 | 机架换不了 → **评估 trim 补偿**：离线已证形变偏斜方向稳定（−X，10.6° 离散度）；待做**地面空转校准** `cf85_trim_calibration.py` 测补偿量是否在电机余量内；**补偿量超限则算法救不了，只能换机架** |

## 4. 环境状态

| 项 | 值 |
|---|---|
| 仓库真路径 | `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`；`D:\dmp` 是 junction |
| WSL | Ubuntu 24.04，ROS 2 Jazzy；cflib **与 ruff/mypy** 都在 venv `~/crazyswarm-env`（系统 python3 没有）；`.bashrc` 里 venv 激活行被注释，入口脚本已自动激活（cf_ops_lib.sh `cf_ops_source_env()`） |
| 电台 busid | 会随插拔变化（历史：1-3 → 1-4 → 3-1）；`cf_ops_radio_busid()` 运行时解析；手动命令先 `usbipd list` 确认 |
| 电台掉线 | WSL 空闲自动关机 → USB/IP 失效；`usbipd list` 仍显示 Attached 是陈旧状态；一律用 `_dsh_scratch/run_probe.sh` |
| pytest | 必须 `--basetemp=<新目录>`（`.pytest-tmp` ACL 坏）；**不要重复 `-q`**（addopts 已含） |
| **ROS 2 发现域** | **必须 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`**（2026-10-14 修复）。本机 WSL 是 mirrored 网络 + 不稳定网卡（eth4），默认 SUBNET 会让 DDS 发现间歇失败 → 服务端在发数据但 recorder 零样本 → 假 `Link FAIL`。已在 `cf_ops_lib.sh`/`cf85_lib.sh`/recorder/Crazyswarm2/bridge 五处 export |
| UI 测试 | 5 个 `test_ui_*.py` 因 venv 缺 PySide6 无法收集，与本线无关；跑测试时 `--ignore` 掉 |

## 5. 2026-10-14 下午会话改动（**未提交**）

### 5.1 两个阻塞性 bug（`_dsh_scratch/` 与两个入口脚本，仓库外）

1. **DDS 发现域**：`cf_prepare.sh` 每次都 `Link FAIL`，recorder 121 s 零样本，但
   `ros2 topic hz /cf231/pose` 明明是 10.18 Hz。根因是 SUBNET 多播发现走不稳的网卡。
   → `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。修复后 **10.27 Hz 稳定 30 s**。
2. **`cf85_stop_stack: command not found`**：`cf_prepare.sh` 只 source `cf_ops_lib.sh`，
   而该函数只在 `cf85_lib.sh` 里。以前 prepare 从不成功所以从未暴露。
   → 在 `cf_ops_lib.sh` 新增 `cf_ops_stop_stack()`，保留 `cf85_stop_stack` 为别名。
3. 附带修 `grep -c ... || echo 0` 双零 bug（`grep -c` 无匹配时既打 0 又退出 1，
   `|| echo 0` 会追加第二个 0 → 算术报错）。改为 `|| true`，共 19 处 11 个文件。

### 5.2 电池判据分级（**产品代码，本轮唯一阈值修改**）

文件：`integrations/crazyflie_ros2/src/dmp_crazyflie_bridge/`

| 文件 | 改动 |
|---|---|
| `dmp_crazyflie_bridge/telemetry.py` | 新增 `DEFAULT_BATTERY_CRITICAL_AIRBORNE_V=3.3`、`DEFAULT_MINIMUM_BATTERY_CRITICAL_SAMPLES=3`；`ReadinessPolicy` 加两个字段；collector 加 `_battery_low_streak` 连续计数；snapshot 加 `battery_critical_airborne(_voltage)` / `battery_critical_consecutive_samples`；**地面 battery issue 的 stage 由 BOTH 改为 GROUND** |
| `dmp_crazyflie_bridge/cf8_acceptance.py` | `_detect_inflight_health_issue` 的空中判据由 `battery_critical` 改为 `battery_critical_airborne`；`failure_telemetry` 增加三个新字段（可审计是哪个阈值、连续几个样本触发） |
| `dmp_crazyflie_bridge/protocol_server.py` | 读入两个新配置键；新增反向校验：空中阈值必须**低于**地面阈值，否则 `ValueError` 拒绝启动 |
| `config/bridge.yaml` | 新增 `battery_critical_airborne_voltage: 3.3`、`minimum_battery_critical_samples: 3`（含完整依据注释） |
| `tests/integration/test_execution_hardware_adapter.py` | 新增 8 个测试：默认值序关系、配置读取、6 种非法配置 fail-closed、sag 不误触发、持续低压才触发、恢复到零、地面 issue 不得伪装成空中 issue |
| `tests/integration/test_execution_cf8_acceptance.py` | 3 个测试改为驱动新信号；fake adapter 增加三个字段 |

**验证**：ruff 全过；mypy 前后均 19 个错误（逐条 diff 完全一致，全是缺 ROS/cflib stub 的既有问题）；
`tests/integration`（排除 5 个缺 PySide6 的 UI 文件）**247 passed**；
两个 CF8/adapter 文件 **209 passed**。

### 5.3 DDS 间歇失败的真正根因（启动顺序）

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` **不是**修复，只是恰好与清 `/dev/shm` 同时发生。
用 `order_test.sh` 各 4 次对照测出真因：

| bring-up 启动顺序 | 结果 |
|---|---|
| recorder 先（旧行为） | **0/4 通过** |
| Crazyswarm2 先 | **4/4 通过** |

订阅者先于发布者创建时，同机 DDS 匹配不上（服务端在发、recorder 零样本、且**永不恢复**）。
→ 改 `cf85_bringup_health.sh`：**先起 Crazyswarm2 并等它连上（`Memories:`），再起 recorder**。
改完真实 bring-up **3/3 通过**。

### 5.4 第二个 bug：ready 文件竞态

recorder 原先只要 `pose` + `status` 到齐就写 ready 文件，但 `cf85_link_health.py`
要求该文件里**五路全非零**。`odom`/`flow`/`readiness` 晚到零点几秒 → 直接被判失败。
→ 改 `cf8_flight_recorder.py`：**五路全到齐才宣布 ready**（消费方本来就假设如此）。

### 5.5 外电（充电）互锁 —— 2026-10-14 16:03 真实事故

**事故**：`run_cf8_acceptance.sh` 在飞机**仍挂在 USB 充电器上**时发起了起飞。飞机猛冲、
被线缆拖拽，固件锁存 `tumbled`+`crashed`（supervisor raw 164），表观位移 7.9 m、
`zrange` 1.886 m、峰值 4.06 m/s、受控降落失败。**飞机经检查无损。**

**根因**：`pm_state=1 (charging)`、`charging_current_a=0.213` **就写在 `battery.json` 里**，
但判据**只看电压**。4.070 V 是完全健康的读数，于是放行了一台物理上被线缆拴住、
且由 USB 供电的飞机。

**修复**（电压不可覆盖）：

| 文件 | 改动 |
|---|---|
| `_dsh_scratch/cf85_battery_policy.py` | **新增**：电池/外电判据的**唯一实现** `decide()`；外电检查**最优先** |
| `_dsh_scratch/cf85_battery.sh` | 改为调用 `decide()`，不再自己复制一份规则；记录 `tether_check` |
| `_dsh_scratch/cf_ops_lib.sh` | `cf_ops_battery_gate()` 显式拒绝 `FORBIDDEN_TETHERED` |
| `_dsh_scratch/cf85_battery_policy_test.py` | **新增** 9 项回归，**含真实事故 payload 回放**（9/9 通过） |

**遗留必须处理**：固件 `crashed` 位（0x0080）**重启 ROS 栈不会清**（`tumbled` 会清、`crashed` 不会），
而 `supervisor_crashed` 是 stage=both 阻断项 → **必须给飞机断电重启**再飞。

### 5.6 真机 tumble-and-crash 事故 + 机架形变量化（本会话，未提交）

> 以下保留 10 月 14 日历史判断；其中“已证明单一机械根因”“方向已知即可确定 trim”“地面空转测悬停补偿量”等推断已由 §0 更正，不可作为下一次飞行的依据。

**事故**：16:32 `run_cf8_acceptance.sh` 真起飞，飞机 +0.2 s 锁 `tumbled`、随后翻倒
`crashed`（supervisor 526→286→164），受控降落失败。**根因是机架形变**（用户确认"机翼之前
有形变"），不是软件 gate 缺陷——起飞前体检全绿（LEVEL、pm.state=0、supervisor=526），
gate 合规放行。详见 `INCIDENT_2026-10-14_tumble_crash.md`。

> **2026-10-20 复核提示**：上述"根因是机架形变"是当时的判断。当天限位起飞实测显示
> **该包带载 sag 达 1.081 V，多次跌破固件 3.0 V 硬断电线**。电压塌陷同样能导致翻倒，
> 且会剥夺控制器权限。因此**不能排除电源参与**了 10-14 的翻倒，这条历史结论应视为
> **"形变是已证实的因素之一"，而非"唯一根因"**。要分离两者，必须先用健康电池重做同一实验。

**形变量化（离线）**：新增工具 `_dsh_scratch/cf85_deformation_quantify.py`（只读扫描
`flight-result.json`，确定性，Ruff clean）。三次真实起飞的"起飞→触发"漂移方向：

| run | failure | drift (m) | 方向 | vx |
|---|---|---|---|---|
| 145822 | battery_critical | 0.056 | 177° | −0.50 |
| 160201 | xy_drift_exceeded | 0.187 | 159° | −1.35 |
| 162838 | xy_drift_exceeded | 0.266 | −176° | −0.32 |

**方向离散度 10.6°，−X 速度 3/3（100%）** → 静态推力轴偏斜真实、方向稳定、可测量。
结论：**有界 trim 补偿是合法候选修复**，但**补偿量未知**，必须地面空转校准测出，
超限则只能换机架（算法救不了严重形变）。

**关键判断**：这是同一天第三个独立根因——① 14:58 阈值用错量（软件，已修）；② 16:03 外电
起飞（软件，已修硬互锁）；③ 16:32 机架形变（**机械，软件无解，只能 trim 或换件**）。
三者共性教训：**软件 gate 认证"可飞性"，认证不了"适航性"；形变这类机械问题任何 gate
都看不见，只能靠起飞前人工检查。**

## 6. 最近会话记录（新→旧）

| 日期 | 会话 | 结论 |
|---|---|---|
| 2026-10-20 | 控制器与静止姿态只读核查 | 当前 PID、静态倾角约 1.6°、3.80 V、无 tumbled/crashed；历史启动日志确认事故时为 Mellinger。补偿可行性未证实；未改参数、未起桨。详见 §0 |
| 2026-10-14 傍晚（后） | **真机 tumble-and-crash + 形变量化** | 真起飞翻倒（机架形变，supervisor 526→286→164，受控降落失败）。体检证明三个物理阻断点已清、tumbled/crashed 断电即清（闭合上轮开放问题）。离线量化：三次起飞漂移方向 −X、离散度 10.6° → 静态推力轴偏斜真实可测。待做地面空转校准测 trim 补偿量，超限则只能换机架 |
| 2026-10-14 傍晚 | **外电起飞事故 + 三个根因修复** | ① DDS 间歇失败真因＝启动顺序（0/4 → 4/4），非 LOCALHOST；② ready 文件竞态（需五路全到齐）；③ **边充电边起飞的硬互锁**（真实事故，飞机无损）。修完 prepare 稳定到 `READY_FOR_CF8` |
| 2026-10-14 下午 | 真机首次尝试 + 电池判据分级 | 真机起桨 0.667 s 后因 3.7 V 判据误触发受控降落（飞机未离地）；判定原判据**用错量**（带载电压比静置阈值）→ 拆为地面 3.7 V / 空中 3.3 V + 去抖 |
| 2026-10-14 凌晨 | CF8 交接文档与 Flow Deck 排障 | ~~判 deck 硬件故障~~ **已被推翻**：重新上电后 deck 完全正常，原因是低压引导 |
| 2026-10-13 | 换新电池上电 | 修复 cf_prepare.sh 两个环境 bug（venv 未激活、busid 写死）；提交 `7c1d904` |
| 2026-09-28 | CF8.5 脚本修复 | 离线验证两个入口 + RUNBOOK |
| 2026-09-22 | CF8 真机联调 | 电池掉压 → 电台链路 → 全断电冷启动恢复；收尾交接 |
| 2026-09-21 | 首次 CF8 | 两次真实尝试均 `battery_critical` 受控降落；修 estimator convergence 用错变量 |

## 7. 待办（跨会话）

- [ ] **评估现有飞控与定位组合**：按 §0 准备 PID 对照方案和目标姿态/角速度/电机输出诊断；不得用拆桨空转推算悬停 trim，不得将 `motorPowerSet` 当闭环补偿。参数写入与真实飞行仍按既有授权边界执行。
- [ ] 电池充满（带载 sag ~0.5 V，静置 ≥4.00 V 才放行；这是会话闸门，非 4.10 V）
- [ ] **提交本轮改动**（用户确认后）：产品代码与规范文档分开提交（3 份报告 + 本会话新增的形变量化工具在仓库外）
- [ ] 形变/trim 问题解决后，用修好的判据 + 补偿重跑真机：`./cf_prepare.sh` → 用户授权 → `./run_cf8_acceptance.sh`
- [ ] 仍待办：`FALLBACK_ADDRESS_IN_USE`（机体答 `E7E7E7E7E7`，`cf_identity.yaml` 偏好 `...E705` 无应答）
- [ ] 软件研发线：M2（默认关闭的真机航点执行路径）见 `learning-plans/路线联动推进计划.md`
