# CF8 真机操作规范（SOP · 故障库 · 工具索引 · 环境坑）

> 本文件是 CF8 真机线的**唯一规范**，替代 `CF85_RUNBOOK.md`（内容已并入并去重）。
> 涉及真机的会话：先读 `session-protocol.md` + `PLAN.md` + `STAGE.md` + `session-log.md`，
> 再读本文件。**阶段判据以 `STAGE.md` 为准，本文件只给操作方法与故障判据。**
> 详细实现背景仍可查 `_dsh_scratch/CF85_RUNBOOK.md` 与 `docs/archive/`，但**以本文件为准**。

## 0. 导航（按"我现在要做什么"找章节）

| 我要… | 去 |
|---|---|
| 知道现在能不能飞、卡在哪 | **不在这里** → `session-log.md` |
| 知道本阶段判据与实测证据 | **不在这里** → `STAGE.md` |
| 跑一次完整准备流程 | §1 状态机、§2 用户物理清单、§3 READY 判定 |
| 确认起飞前的物理条件 | §3.1 物理检查清单 |
| 判断电池电压算不算过关 | §4 电池电压政策、§4.1 生产阈值 |
| 找某个脚本怎么用 | §5 工具索引（入口 / 只读诊断 / 自检） |
| 遇到故障先对照签名 | §6 故障库 F1–F13（**先查表再排查**） |
| 遇到环境/工具链的坑 | §7 环境坑 |
| 看某次运行的产物在哪 | §8 证据布局 |
| 某个报错该看哪个文件 | §9 出问题时看哪里 |
| 确认哪些事绝对不能做 | §10 硬约束 |
| 飞完要报告什么 | §11 飞行后报告清单 |

> **故障库编号已在 2026-10-20 重排**为一编号一主题（旧 F1a/F1b/F1c 等已合并），
> 对照表在 §6 开头。

---

## 1. 一句话状态机

```text
[用户物理准备] → cf_prepare.sh → READY_FOR_CF8 → (用户明确授权) → run_cf8_acceptance.sh
                       │ 任何 FAIL：立即停止、不重试、报告等指示
                       └─ 日常只有这两个命令，没有第三个入口
```

**日常只有两条命令：**

```bash
wsl -d Ubuntu
cd "/mnt/d/Deepseek WorkSpace"
./cf_prepare.sh                # 准备全流程：电池 → capability(deck) → Link → Preflight → READY_FOR_CF8
./run_cf8_acceptance.sh        # 只有用户明确决定真飞时才运行（一次授权 = 最多一次）
```

正常输出四行 + 一个结论：

```
Battery      PASS
Radio        PASS
Link         PASS
Preflight    PASS

READY_FOR_CF8
```

- `cf_prepare.sh --verbose` 显示内部每步；`run_cf8_acceptance.sh --yes` 跳过 5 秒中止窗口。
- **单一入口铁律**：`cf84_preflight.py` 永远不得手工直接调用；preflight 唯一入口是
  `cf85_fast_path.sh`，它只由 `cf_prepare.sh` 调用。
- **READY token**：一次性、默认 900 s 有效、绑定本次 runtime URI（sha256 比对）、
  要求栈仍在运行且 `hardware_flight_enabled` 仍为 false。同一 run dir 不允许第二次
  preflight / 第二次飞行。

## 2. 用户物理操作清单（AI 不能代劳的部分）

按顺序，每步之间等 AI 确认再继续：

1. **充满电**：静置电压 **≥ 4.00 V** 才放行（会话政策，见 §4 与 2026-10-20 修正）。充电电流偏慢（~0.04 A）需留意卡住。
2. **确认机械结构**：机体平整、Flow Deck 压紧、插头线材正常。
3. **上电**：对 AI 说"现在上电"——在此之前 AI 不连 cf231、不跑 probe。
4. **电台插接**：Crazyradio 2.0 插在 PC USB 口（busid 会变，先 `usbipd list` 确认）。

**AI 侧铁律**：在用户说"现在上电"之前，不连接 cf231、不启动 Crazyswarm2、不跑 probe、
不刷新 snapshot。未获用户明确"执行 CF8"前禁止真实 takeoff。

## 3. 起飞前 READY 判定（FAST PRECHECK 只查真正的 ground blocker）

只关注这些（airborne-only 条件留给 STABILIZING）：

| 条件 | 来源 |
|---|---|
| `connected` | runtime URI CRTP 应答 |
| `positioning_mode=flow` | preflight |
| `ground_passed` | 生产 readiness gate |
| `estimator_not_converged` 不存在 | 生产 readiness gate |
| Flow 数据非冻结 | Stage 2.75 capability 刷新 |
| supervisor safe | `supervisor.info` 解码（见故障库 F2） |
| battery / 供电状态 | Stage 1 battery gate（**含 F1 外电互锁**） |
| `pm_state == 0`（未挂外电） | battery gate，`FORBIDDEN_TETHERED` 硬拦 |
| 无持续 ESTKALMAN reset | 健康检查 |

到 READY_FOR_CF8 后停在起飞前，报告当前电池电压和 gate 结果。

### 3.1 物理检查清单（软件看不到，必须人工确认）

2026-10-14 的事故（F1）证明：**软件闸门再严，也拦不住物理前提没满足。**
每次真实起飞前逐项确认，缺一项就不飞：

1. **充电器已拔** —— `pm_state` 必须为 0，charge current 0 A（现在有硬互锁兜底，但别依赖它）
2. **螺旋桨装好且无暗裂** —— 经历过 tumbled/crashed 后尤其要换桨
3. **周围 ≥2 m 空地**
4. **`supervisor` 无 `crashed` / `tumbled` 锁存位** —— 有则**给飞机断电重启**（重启 ROS 栈不够）
5. **全程盯机，随时能拔电**
6. **起栈前跑 `cf85_level_check.py`**：`tumbled` 未锁存、倾斜 < 5°、`|acc|≈1 g`。
   飞机被碰倒/侧躺时起栈，会伪造出“EKF 发散 + 位置漂移”的假故障（见 F15），整轮数据作废

## 4. 电池电压政策（会话级，非生产阈值）

| 静置电压 | 允许动作 | `cf_prepare.sh` 表现 |
|---|---|---|
| **挂外电（`pm_state≠0` 或 charge current >0）** | **禁止一切飞行** | `Battery FAIL`，`FORBIDDEN_TETHERED`（F1 硬互锁） |
| `< 4.00 V` | 禁止进入 FAST PATH（含地面诊断） | `Battery FAIL`，立即停止 |
| `≥ 4.00 V` | 放行（诊断区已合并进放行区） | `Battery PASS` |
| "充满"但 `< 3.90 V` | 判可疑电池，不再用于 CF8 实飞 | `battery.json` 标 `suspect_pack: true` |

> 背景：原电池静置 4.043 V，起跳瞬间崩到 3.457 V（sag 0.586 V）。**静置电压达标 ≠ 电池健康。**

> **2026-10-20 修正（本表此前与本仓库脚本不一致）**：`_dsh_scratch/cf_ops_lib.sh` 第 75–76 行
> 早在 2026-10-14 就把 `BATTERY_FORBID_V` 与 `BATTERY_DIAG_ONLY_V` 一起放宽到 **4.00 V**，
> `cf84_preflight.py` 的 `MIN_BATTERY_V` 也是 4.00 V。旧表里的 `4.00 – 4.10 V 仅诊断` /
> `≥ 4.10 V 放行` 在任何代码里都不存在，只剩 `*_bak-20261014-threshold` 备份文件。
> 该过期表述曾让 2026-10-20 一次 4.049 V 的地面验证被误判为"未达门槛"而停摆，故按
> session-protocol §4.2"规范与现实不符先改规范"修正。
>
> **不可降到 3.5 V 的理由**：本机 `battery_critical_voltage=3.7 V` 是地面生产放行线；实测带载
> sag ~0.5 V（4.048 → 3.552 V），静置 3.5 V 一起桨即逼近固件 `pm.criticalLowVoltage=3.0 V`
> 硬断电。3.5 V 只能用于**不起桨**的地面诊断，不能作为任何飞行放行判据。

### 4.1 生产阈值：地面 / 空中分开（2026-10-14 修改）

**这是 2026-10-14 唯一一次生产阈值修改，用户明确授权。** 原判据是拿**带载瞬时电压**去比
**静置标称电压 3.7 V** —— 官方电池一悬停必掉 ~0.5 V，所以这个比较对健康电池**必然失败**，
项目无法推进。这不是"阈值定高了"，是**判据用错了量**。

| 阈值 | 值 | 应用场景 |
|---|---|---|
| `battery_critical_voltage` | **3.7 V**（未改） | **地面/静置**放行判据：静置都扛不住 3.7 V 的包不许起飞 |
| `battery_critical_airborne_voltage` | **3.3 V**（新增） | **空中/带载**判据，驱动受控降落 |
| `minimum_battery_critical_samples` | **3**（新增） | 去抖：连续 N 个样本都低才判定，单次 sag 不触发 |

锚点依据 —— 本机固件自己的保护（2026-10-14 实测读取）：

| 来源 | 值 |
|---|---|
| `pm.lowVoltage` | 3.2 V（固件告警） |
| `pm.criticalLowVoltage` | 3.0 V（固件硬断电） |
| 实测带载 sag | 4.048 → 3.552 V（0.7 s 内） |

3.3 V 高于固件告警 0.1 V、高于硬断电 0.3 V，保持正确分层：**DMP 先发现电量低并受控降落，
固件硬断电只作最后手段**。原 3.7 V 比厂商自己的告警线还高 0.5 V，会掐断健康飞行
（2026-10-14 实测：飞机还没离地就被判定 battery_critical）。

> `hardware_flight_enabled`(false)、`capability_snapshot_max_age_s`(3600)、
> `max_estimator_variance_span`(0.002)、STABILIZING 全部参数 **仍然不改**。

## 5. 工具索引（全部在 `D:\Deepseek WorkSpace\_dsh_scratch\`，仓库外）

**命名规则**：新脚本 `cf85_` 前缀、用途可猜、写完立即登记到本节；一次性脚本标 `[临时]` 用完删。

> **位置约定**：标 **⭐** 的三个文档自检脚本已**入仓库**（`scripts/docs/`），可用
> `python scripts/docs/<name>.py` 直接跑，路径自适应、可移植。
> 其余（真机探针、入口封装）在仓库外 `D:\Deepseek WorkSpace\_dsh_scratch\`，
> 因为它们依赖 WSL/cflib/ROS，不适合进产品仓库。

### 5.1 入口（只有这两个会发起动作）

| 文件 | 作用 |
|---|---|
| `cf_prepare.sh` | 入口 1：准备全流程（唯一会启动栈的入口） |
| `run_cf8_acceptance.sh` | 入口 2：真实飞行，单独触发，需 READY token |

### 5.2 只读诊断（分组：链路 / 电池 / deck / 系统）

| 文件 | 作用 | 组 |
|---|---|---|
| `run_probe.sh` | 同一 WSL 会话内"确保电台 + 跑探针"（应对电台掉线），`用法: run_probe.sh <busid> <probe.py> [args]` | 链路 |
| `cf85_radio_scan.py` | 电台发现（只听不连） | 链路 |
| `cf85_link_health.py` / `cf85_bringup_health.sh` | 持续型数据链路健康判定（30 s 窗口 + ≥50% 标称速率） | 链路 |
| `cf85_radio_paths_test.sh` | 电台"正常短路/异常重连"两路径自检（零射频） | 链路 |
| `cf85_diagnose_bringup.sh` | 只读 bring-up 诊断（内核 USB/IP 日志等） | 链路 |
| `cf85_battery.sh` | 只读电池读数 + 政策裁决（用 runtime URI） | 电池 |
| `cf85_pid_diagnostics.py` | inventory：独占电台只读 TOC/参数/供电；build：生成独立 PID 配置和原文件备份；**verify**：对活动固件清单核验预设（控制器优先级、未发布变量、单块 ≤26 字节、诊断链覆盖、只允许两处语义差异）；**readback**：把机载回读与预设逐项比对（整数精确、浮点按 float32 容差）；record：ROS 六组控制诊断订阅 + **机载日志时钟**判据。无参数写入或飞行 API。`--self-test` 离线。**2026-10-20 实测 record 30 s 地面通过；verify/readback 于 2026-10-26 加入** | 诊断 |
| `cf85_pid_2b_prepare.py` | **纯离线候选生成**：读取已保存的 PID 预设和 2a TOC；增加按轴配对的双环 P/I/D 三块（各 24 B），计算含默认话题的总日志预算；manifest 覆盖全部自定义块，可交给现有只读 record 做后续地面检查。拒绝覆盖、缺变量、超载块和未审阅的配置覆盖。无射频/ROS/参数写入；不产出 `preset-live*`，不接入起飞流程。pytest：`test_cf85_pid_2b_prepare.py` | 离线 |
| `cf85_pid_ground_run.sh` | **地面 PID 诊断唯一入口**：预设核验闸门 → resolve → 电池闸门 → 回滚副本一致性 → 用独立 PID preset 生成 runtime 配置 → bring-up + 30 s 链路健康 → record → 停栈 → 机载回读**并逐项比对**（回读失败即整轮失败）。自动选最新的 `preset-live*`，可用 `CF_PID_PRESET_DIR` 固定。**不跑 preflight、不开 bridge、不起桨**，因此不产出 READY 令牌 | 诊断 |
| `cf85_pid_analyze.py` | 汇总一次地面记录：目标／实际姿态、角速度、电机输出、推力、电压、位置 | 诊断 |
| `cf85_pid_state_check.py` / `cf85_pid_state_seq.py` | 位置估计体检：识别"平滑斜坡 vs 跳变 vs 冻结"，定位 ESTKALMAN 钳位时刻（见 F6） | 诊断 |
| `cf85_gate_check.sh` | 确认会话闸门与 `cf85_lib.sh` 一致，并核对生产阈值/`MIN_BATTERY_V` 未被连带修改 | 自检 |
| `cf8_token.py` | READY 令牌校验/消费（`inspect`/`claim`）。**只由 `run_cf8_acceptance.sh` 调用**；不连电台、不飞行 | 自检 |
| `cf85_link_diag_harness.sh` | 受控链路实验入口：`usb-open-probe`（测一次 cflib open/close 的内核代价）、`stack-idle`（栈空闲 N 秒 + dmesg 前后对比）。**不发飞行命令** | 链路 |
| `cf85_link_ab.sh` | A/B 链路对照：给定 crazyflies YAML + 保持时长，跑一次栈并输出 `link-ab-summary.json`（丢包次数、最大延迟、EKF 次数、USB reset 次数、各流 Hz）。**不发飞行命令** | 链路 |
| `cf85_link_diag_harness.py` | 上面 harness 的 open/close 探针本体（只读一次 `pm.vbat`） | 链路 |
| `cf85_recovery_assess.sh` / `cf85_recovery_release.sh` | 释放 `recovery_required` 互锁前逐项证明无飞行状态；释放时**归档**标记而非删除 | 自检 |
| `cf85_preset_check.sh` / `cf85_preset_semdiff.sh` | 证明 PID preset 只差 controller 2→1 与新增日志块（语义级 diff，不受 YAML 重排干扰） | 自检 |
| `cf85_pid_preset_check.sh` | **离线**预设闸门：拿永久 `crazyflies.yaml` + 已存的活动清单核验预设，并证明永久配置仍与回滚副本逐字节一致。`--rebuild` 只生成新目录、不安装。零射频、零 ROS | 自检 |
| `cf85_termination_probe.py` | **拆桨远程停机测量探针（唯一接受的终止验证入口）**：**cflib 直连独占电台，ROS 栈必须先停**。流程 = 空闲基线 → arming + 对称油门转机 `--spin-s`（上限 20000 / 10 s）→ 发 CRTP localization emergency-stop（type 3）→ 观察 `--watch-s`。要求 `--confirm-props-removed`，**不接受** `--confirm-restrained`（这里不该有推力）。判据见 `cf85_end_state.termination_verdict`：转机前必须非零、指令后归零并保持到窗末，三条全中才算 `stopped=True`。`finally` 与退出路径都会重发停止并 disarm。拒绝也落盘。`--dry-run` 零发射。**边界**：与遥测共用无线链路，测不出"失链后能否停机"；转机用了 arming，所以测的是"已解锁状态下的停止"，不是未解锁状态。 | 终止 |
| `cf85_end_state.py` | **结束判定的唯一实现**（被探针调用，不再各自维护副本）：`evaluate()` 在**窗口末段**分别测新鲜度/静止/电机归零/机体未侧躺，`termination_verdict()` 测"停机被观测到"。修掉旧实现四处漏洞：不看倾角、不查新鲜度、按整窗取电机峰值（正常降落反被拒）、无法区分"停稳"与"落地"。`range.zrange` 按**毫米**换算。`--self-test` 离线 | 自检 |
| `cf85_onboard_time.py` | **跨块统一时间基**：按块解 24 位回绕 → 解算**单一 epoch**偏移 → 用最小二乘给出 host↔机载斜率与残差（本机计数器快 6–9%，只记录不隐藏）→ 按容差合并各块。用在任何跨块比较之前。`--self-test` 离线 | 自检 |
| `cf85_takeoff_probe.py` | **限位低空探针，仍不能用于 2b 自由飞行**：有 `--confirm-restrained`；调用 takeoff/land 后由 `cf85_end_state.evaluate()` 判定结束（不再自带副本），未确认返回码 3。2026-10-29 修：保存机载时间戳、**早期拒绝也落盘**（此前 refuse 直接 return，不留任何证据）。仍未验证：异常路径下的 land 重试、自由飞行下的适用性。 | 推力（待审） |
| `cf85_end_state_test.py` | **离线反例集**（17 项，零射频）：侧躺/悬停/恒速偏航/缺通道/NaN/变量数不符必须拒绝，通道提前停更不算末期证据，并断言探针**确实委托**共享判定；24 位回绕 `16777166→0→50`、真倒退仍判失败 | 自检 |
| `test_cf85_termination_online.py` | 离线回归 20 项：拒绝路径零发射但**必须落盘并记原因**、空测（全程为零）不算停机、应答≠停机、真停机被接受、10-27 形状不判结束、mm 单位、跨块对齐 | 自检 |
| `test_cf85_probe_evidence.py` | 离线回归：拒绝也写证据、退出码 1/3/0 分义、机载时间戳保留、报告只有一个写入口 | 自检 |
| `cf85_stack_for_takeoff.sh` | 起 Crazyswarm2(PID preset) 并**保持运行**，供起飞探针驱动（`cf85_pid_ground_run.sh` 跑完会停栈） | 推力 |
| `cf85_takeoff_analyse.py` / `cf85_takeoff_raw.py` / `cf85_takeoff_verdict.py` | 起飞数据三级解读：粗汇总 / 原始时间序 / 逐项裁决（电压 vs 姿态 vs 电机） | 推力 |
| `cf85_flight_aligned.py` | **按起飞时刻对齐**看 z(EKF) vs zrange、链路丢包、ESTKALMAN 时刻。判断"是否真离地/真飘"必用它 | 推力 |
| `cf85_drift_real.py` | 用 zrange 交叉验证 EKF：区分"机体在飘"与"估计器在漂" | 推力 |
| `cf85_doc_consistency.py` ⭐ | **收尾必跑（形式）**：悬空编号、日志是否退化成编年史、闸门数值是否漂移 | 自检 |
| `cf85_stage_audit.py` ⭐ | **收尾必跑（事实）**：判据是否带条件+证据路径、FAIL 是否被前置条件闸住、§4 是否有无条件断言 | 自检 |
| `cf85_handoff_sim.py` ⭐ | **收尾必跑（交接）**：模拟冷会话只读三份文档，能否判断不可飞、选对下一步、不重复已排除路线 | 自检 |
| `cf85_handoff_test.py` | 关键词级交接粗筛（`cf85_handoff_sim.py` 更严，优先用它） | 自检 |
| `cf85_supervisor_derive.py` | 从已知真值（526 健康 / 164 翻倒）反推 supervisor 位，而不是凭记忆解码（见 F8） | 自检 |
| `cf85_thrust_headroom.py` | 直连 cflib 的限位姿态测试（含 `--dry-run`）。**注意**：未起 Crazyswarm2 时 `enHighLevel=0`，位置指令不生效，2026-10-20 首测因此作废 | 推力 |
| `cf85_thrust_payload_check.py` | 离线校验日志块 ≤26 字节（实测 `th_attitude` 曾超限 14 字节） | 自检 |
| `cf85_commander_audit.py` / `cf85_enhighlevel_trace.sh` | 确认 `commander.enHighLevel` 由 Crazyswarm2 在启动时写入 1，而非需要手写参数 | 自检 |
| `read_battery_probe.py` / `battery_trend.py` / `read_pm_state.py` | 电池读数/趋势/充电状态 | 电池 |
| `pm_probe.py` | 读固件自身电池阈值（`pm.lowVoltage` / `pm.criticalLowVoltage`）—— 空中判据 3.3 V 的锚点依据 | 电池 |
| `cf85_battery_policy.py` | **电池/外电判据的唯一实现**（含 `FORBIDDEN_TETHERED` 硬互锁）；`cf85_battery.sh` 与测试共用 | 电池 |
| `cf85_battery_policy_test.py` | 上面判据的回归测试 9 项，含**真实事故 payload 回放**；离线可跑 | 电池 |
| `cf85_incident_check.py` | 事故后只读体检：pm.state / 姿态 / z / supervisor（判断是否翻倒、是否还挂着线） | 飞后 |
| `cf85_level_check.py` | **加速度计独立验证姿态**（不信任估计器）+ 供电 + supervisor 锁存位；上电前必跑 | 飞前 |
| `cf85_mechanical_baseline.py` / `cf85_mechanical_run.sh` | 静止机械基线：连续记录 acc/gyro、估计姿态、四电机输出与供电，报告均值/离散度/断档；**并核验机载 `firmware_ms` 时钟**（冻结/停顿/回绕/偏快），不再只看宿主到达率。时间戳按实际 **24 位**（cflib 以 3 字节解包）解绕。不写参数、不启 ROS、不起桨。仅能测 IMU 相对重力，不能识别单个电机轴；需人工确认上电静放。`--self-test` 为离线检查 | 诊断 |
| `flow_deck_live.py --duration 5` | **决定性**：Flow Deck 五路变量（deltaX/deltaY/motion/squal/zrange）是否有真实数据 | deck |
| `flow_raw.py` | 完整故障签名（含 `motion.motion`、`TYPE_1W` 普查）+ 与 09-22 对照 | deck |
| `deck_inspect.py` / `deck_diag.py` / `param_meta.py` / `param_access.py` | deck 参数/1-wire 身份/access 证明（deck.* 全 RO） | deck |
| `force_check.py` | 扫描是否存在可写 deck/force 开关（结论：不存在） | deck |
| `deck_recover.py` | 诊断 + 可选 `stm_power_cycle()` 软件重启（重启需 `--power-cycle --confirm-power-cycle`） | deck |
| `boot_banner.py` | 抓启动横幅（含 deck 发现行）；可 `--power-cycle` | deck |
| `charge_state.py` / `console_and_charge.py` | 充电状态 / 固件 console 转储 | 电池 |
| `cf85_identify.py` / `cf85_toc_enumerate.py` / `log_toc_probe.py` | 机体识别 / TOC 枚举（不猜变量名） | 系统 |
| `cf85_cf2.sh --set-address <addr> --confirm-write` | **唯一**的 CF2 配置块写入口（改地址需断电重启；非日常路径） | 系统 |
| `cf85_smoke_test.sh` | 离线自检 121 项（不碰电台、零飞行命令） | 自检 |
| `cf85_run_offline_tests.sh` | 上面 + pytest 单元测试套件 | 自检 |
| `cf85_verify_battery_change.sh` | 跑 ruff + mypy + execution 测试套件（排除缺 PySide6 的 UI 文件） | 自检 |
| `cf85_mypy_baseline.sh` | mypy 前后对比（对 `git archive HEAD` 跑一遍并 diff，证明没引入新错误） | 自检 |
| `cf85_fix_discovery.sh` | 校验 ROS 发现域已设 LOCALHOST + 清理 `/dev/shm/fastrtps_*` 陈旧段 | 环境 |
| `postflight_analysis.sh` | 飞行后读结果：事件时间线 / 真实触发值 / sag / alt 轨迹 | 飞后 |
| `flight_window_truth.sh` | **按墙钟**切分录制：区分"起飞前被人拿起来"与"真实飞行窗口" | 飞后 |
| `peak_check.sh` | 判定高度峰值是真爬升还是单样本尖峰 | 飞后 |
| `cf85_show_failure.sh` | 单测失败时只打印 traceback（修 PowerShell 吞 `$()` 的问题） | 自检 |
| `cf85_verify_incident_fix.sh` | 事故修复的最终自检：语法 + 门确实用共享判据 + 无重复规则 + 互锁/栈状态 | 自检 |
| `order_test.sh` | **启动顺序 A/B**（recorder 先 vs Crazyswarm2 先）—— 定位 DDS 间歇失败真因 | 链路 |
| `cf85_shm_state.sh` | 查看/清理 `/dev/shm/fastrtps_*`（`--clear`，仅在无存活节点时） | 环境 |
| `dds_matrix.sh` / `dds_matrix_all.sh` / `dds_ab_test.sh` | DDS 发现范围 × 第三方参与者的 2×2 对照实验（证明 LOCALHOST 不是修复） | 链路 |
| `verify_order_fix.sh` | 用真实 bring-up 反复验证顺序修复 | 链路 |

### 5.3 自检（离线，随时可跑）

```bash
bash _dsh_scratch/cf85_smoke_test.sh              # 121 项，零射频
bash _dsh_scratch/cf85_run_offline_tests.sh       # + pytest
bash _dsh_scratch/cf85_radio_paths_test.sh        # 电台路径自检（零射频、不连 cf231）
```

## 5.4 PID 地面诊断的证据要求（2026-10-26 新增）

**这条回答的是"地面诊断要留下什么才算做完了"，不回答"能不能飞"。**
结论仍然是：地面零电机输出无法测出控制余量，也无法证明形变被补偿（见 `STAGE.md` §2.2、§4）。

一次地面 PID 诊断必须同时满足下面四类要求，缺任一项即判整轮无效：

| 类别 | 要求 | 谁检查 |
|---|---|---|
| 可回退 | 永久 `crazyflies.yaml` 与保存的 `crazyflies.original.yaml` **逐字节一致**；预设只改 `all.firmware_params.stabilizer.controller` 与新增日志块 | `cf85_pid_preset_check.sh` |
| 可加载 | 每个日志块 <= **26 字节**（固件上限）；块内变量本机固件确实发布；控制器优先级 `all` -> `robot_types` -> `robots` 逐级解析后**最终值为 1** | 同上 |
| 覆盖完整 | 诊断链的九个问题都能被某个日志块回答，特别是 **`pm.state`**（区分挂外电与自由供电，F1 类） | 同上，`READ_ONLY_EVIDENCE` |
| 机载生效 | 回读不是"看到日志有数就行"：逐项比对预设与机载值；**同时**校验固件日志时钟（`LogDataGeneric.timestamp`）覆盖的机载时间不少于宿主时间的 80% | `cf85_pid_ground_run.sh` 第 7 步 |

**为什么单独需要机载日志时钟判据**：Crazyswarm2 侧订阅速率只证明 ROS 话题在发消息，
不证明固件每次回调都在采样。日志块被固件冻结时，ROS 侧的 Hz 可以完全正常。
判据取 `timestamp` 一阶差分的覆盖比例，并先做 uint32 回绕展开——回绕不是断流。

**已知易错点（都已入代码，不要再靠记忆）**：

1. `cflib` 的参数值是**字符串**（`'1'`）。整数参数按整数精确比较，浮点参数必须给 float32 容差：
   `1e-3` 会读回 `0.0010000000474974513`，用等值或字符串比较会误报不一致。
2. `cf_generate_runtime_config.py` 的 `permanent_*_sha256` 描述的是**传进去的那份 YAML**。
   地面诊断传的是预设，所以那几个字段**不能**当作"永久配置未被改动"的证据；
   回滚一致性必须另外用 `cmp` 对保存的原始副本单独证明。
3. 2026-10-20 的预设**缺 `pm.state`**，且第 7 步回读曾经写成 `|| true`——
   即电台掉了、飞机断电也会得到"成功"的结论。两处均已于 2026-10-26 修正。
4. **机载毫秒计数器偏快（2026-10-26 实测）**：固件 `LogDataGeneric.timestamp` 走完 32.8 s 时，
   宿主 `monotonic` 只过了 30.86 s（两次独立测量分别快 6.3% / 9.2%）。同区间 `/proc/uptime`
   与 `monotonic` 只差 0.5 ms，**宿主侧是正常的**。所以按固件秒算恰好是标称 20.00 Hz，
   按真实秒算是 21.83 Hz。**任何 Hz 与时长都要注明时间基**；切片、对齐、判定间隔一律用固件时间戳，
   不要用宿主 wall 时间戳（本轮 wall 还出现过两次约 2.97 s 跳变，见 §7「WSL 时钟域不一致」）。

**边界**：以上四项全绿只表示"控制器选择与日志确实落到机体上、并且持续出数"。
它不表示 PID 调得好，不表示电源能供起飞电流，也不表示歪支脚造成的推力偏斜被补偿。

## 6. 已知故障库（先对照签名，再动手；禁止跳过对照直接"重新排查"）

> 每条有：签名（怎么认出来）、判据（决定性证据）、修复（已验证的）、曾发生时间。

### 故障编号变更对照（2026-10-20 重编号）

旧编号把**不相关的故障**编成了同一族的后缀（F1a/F1/F1b/F1c 是四件不同的事），
读者会误以为它们是同一故障的子类。现改为按"阻断飞行优先顺序"编号，一编号一主题：

| 旧编号 | 新编号 | 主题 |
|---|---|---|
| F1a | **F1** | 边充电边起飞（外电互锁） |
| F1 | **F2** | 电池电压塌陷（sag） |
| F1b | **F3** | 假 `Link FAIL`（启动顺序） |
| F1c | **F4** | 入口脚本函数缺失 |
| F2 | **F5** | Flow Deck 数据冻结 |
| F2b | **F6** | 位置估计发散（ESTKALMAN） |
| F3 | **F7** | Flow Deck 完全不被检测 |
| F4 | **F8** | 电台链路故障 |
| F5 | **F9** | USB/IP 链路不稳 |
| F6 | **F10** | 环境 bug（脚本层） |
| F6b | **F11** | 判读陷阱：汇总指标的时间窗口 |
| F7 | **F12** | 探针/栈互斥冲突 |
| F8 | **F13** | 位表不可凭记忆 |


### F1 边充电边起飞（USB 线未拔）—— 2026-10-14 真实事故

- **签名**：`battery.json` 里 `pm_state=1`（charging）/ `pm_state_meaning="charging"` /
  `charging_current_a > 0`，而 `cf85_policy.verdict` 仍是 `NEXT_STAGE_ALLOWED`。
  然后 `failure_code` 可能是 `xy_drift_exceeded`，固件 supervisor 出现
  **`tumbled` / `crashed`** 位（事故瞬间 raw 164，健康时是 526）。
- **判据**：`pm_state != 0` 或 `charging_current_a > 0.01` → 飞机挂在外电上。
  `summary_line` 里的 `state=N` 就是它。
- **后果**：线缆拉扯+不平衡、USB 供电改变推力预期 → 猛冲、拖拽、固件锁 crashed。
  2026-10-14 那次表现为：7.9 m 表观位移、`zrange` 1.886 m、峰值 4.06 m/s、受控降落失败。
- **修复**：`cf85_battery_policy.py` 的硬互锁 → `FORBIDDEN_TETHERED`，**电压再高也不能覆盖**。
  `cf_ops_battery_gate` 显式拒绝并提示"UNPLUG THE CHARGER"。回归测试
  `cf85_battery_policy_test.py` 9/9（含真实事故 payload 回放）。
- **发生**：2026-10-14 16:03。
- **教训**：**电压健康 ≠ 适飞。** 判据只看电压，pm_state 就在同一个 JSON 里却没人看。
- **残余**：**`crashed` 位重启 ROS 栈不会清**，必须**给飞机断电重启**，否则
  `supervisor_crashed`（stage=both）会挡住地面闸门。

### F2 电池：电压塌陷（sag）——判据与劣化是两件事

**先分清两个独立问题，过去混为一谈浪费了整整一天：**

| 命题 | 当前结论 | 处理 |
|---|---|---|
| 用**带载**电压去比**静置**阈值 3.7 V | **错**。官方包悬停必 sag ~0.5 V，健康包也必然"失败" | 已拆成地面 3.7 V / 空中 3.3 V + 去抖（§4.1） |
| 这块包**本身**能否供起飞电流 | **不能**。实测 sag 1.081 V | 换包。这不是判据问题 |

- **判据（当前有效）**：起桨前后看 `pm.vbat` 曲线。`static − min(loaded) > 0.7 V` → 包劣化。
  同时看有多少样本低于 `pm.criticalLowVoltage`(3.0 V)。
- **实测（2026-10-20 限位 0.10 m 起飞）**：静置 3.926 V → 带载最低 **2.845 V**，sag **1.081 V**；
  33 个带载样本中 **12 个低于 3.0 V**、**30 个低于 3.2 V**。同型包正常范围 0.5–0.7 V。
- **诊断含义（最重要）**：电压塌陷会**剥夺控制器权限**，使"机架太歪"与"电源供不上"无法区分。
  **换到健康包之前，任何推力/起飞实验都不能用来判断形变补偿能力。**
- **修复**：换已知健康电池，复测 sag < 0.7 V 才算通过。"充满"但静置 < 3.90 V 判可疑包。
- **注意**：静置电压回升**不代表**健康（本包就是从 2.845 V 回升到 4.1 V）；充电电流偏慢（~0.04 A）
  要留意插头接触。
- **历史**：2026-09-21 / 09-22 / 10-14 / 10-20 均发生。

### F3 假 `Link FAIL`（真因是启动顺序，不是链路也不是发现域）

- **签名**：`cf_prepare.sh` 输出 `Battery PASS / Radio PASS / Link FAIL`；recorder 连续
  120 s 全是 `counts={"pose":0,...}`；`health.json` **文件为空或不存在**。
- **判据**：同时看 `launch.log` —— 若里面有 `Logging to /pose at 10 Hz` 且
  `ros2 topic hz /cf231/pose` 有 ~10 Hz，则**服务端在发数据**，问题在启动顺序/发现，不在链路。
  `launch.log` 里的 `Unexpected number of unicast packets` 是**误导**（09-22 成功飞行日志里
  一模一样），不是原因。
- **修复（真因：启动顺序）**：`cf85_bringup_health.sh` 必须**先起 Crazyswarm2 并等 `Memories:`，
  再起 recorder**。用 `order_test.sh` 各 4 次对照：recorder 先 → **0/4**；Crazyswarm2 先 → **4/4**。
  修后真实 bring-up **3/3** 通过。
- **`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 不是修复**：它当时恰好与清 `/dev/shm` 同时发生，
  造成了"修好了"的错觉。仍建议保留该 export（本机 mirrored 网络下更稳），但**别把它当根因**。
- **发生**：2026-10-14（当天连续 3 次复现）。

### F4 入口脚本函数缺失（`cf85_stop_stack: command not found`）

- **签名**：第二次跑 `cf_prepare.sh` 报 `State FAIL`，`state.log` 里是
  `cf_prepare.sh: line N: cf85_stop_stack: command not found`。
- **原因**：`cf_prepare.sh` 只 source `cf_ops_lib.sh`，而该函数原本只在 `cf85_lib.sh` 里。
  **只有 prepare 成功过一次、留下 stack 之后才会暴露**，所以长期潜伏。
- **修复**：`cf_ops_lib.sh` 新增 `cf_ops_stop_stack()`，`cf85_stop_stack` 保留为别名。
- **发生**：2026-10-14（首次 prepare 成功后立刻暴露）。

### F5 Flow Deck 数据冻结（检测到了但数据不走）

- **签名**：`deck.bcFlow2=1`；原始块 `[dX, dY, motion, squal]` 形如 `[0, 0, 58, 0]`
  （`motion.motion` 计数 58 仍在走，但 deltaX/deltaY/squal 恒 0）；`TYPE_1W` 存在。
- **判据**：`flow_raw.py` 的对照表；与 F3 的区分点是 `bcFlow2=1` 且 `motion.motion>0`。
- **修复**：**重新插接 Flow Deck**（2026-09-22 验证有效）。
- **发生**：2026-09-22（冻结期）。
- **注意**：与 F3 是**不同故障**，老经验不通用。

### F6 位置估计发散（`ESTKALMAN: State out of bounds`）

- **签名**（2026-10-20 实测，`cf_evidence_runs/20261020-202302-pid-ground`）：
  `launch.log` 出现 `ESTKALMAN: State out of bounds, resetting`（本次 30 s 内 2 次）；
  同时 `stateEstimate.vx/vy` 在**静止**机体上报到 2.0 / 4.9 m/s，`x/y` 在 22 s 内累积到
  32 m / 97 m 后被固件钳回原点（`(32.05, 97.38) -> (0,0)`）。
  关键鉴别点：**`z` 全程贴地 0–0.016 m**，且 `x/y` 是**平滑斜坡**而非跳变或冻结。
- **判据**：与 F5 区分 —— F5 是 delta 恒 0（deck 冻结），F6 是 delta 有输出但**积分率被放大**。
  与 F3 区分 —— `bcFlow2=1`、`bcZRanger2=1`、`dmp_flow` 10 Hz 正常，deck 在出数。
- **为什么它重要**：这份地面记录里 `dmp_pid_state` 的**绝对坐标不可用**（只有变化量有意义）。
  任何基于 `stateEstimate.x/y` 绝对值的判断都会出错。
- **是否阻断飞行**：本机 `cf84_preflight.py` 第 6 项（`RESET_MARKER = "ESTKALMAN"`，只统计
  本次连接之后的发生次数）本就把这类重置当阻断项，所以它**不会**被静默放过。
  历史对照：`20261014-162838-prepare`（翻倒事故那次）有 **3** 次，`20261014-143923-prepare` 有 5 次，
  其余多次运行为 0 —— 说明它是**间歇性**的，不是恒定状态。
- **已排除的假设（2026-10-20 用时间轴证明）**：曾怀疑"事故那次的重置发生在起飞瞬间，
  所以是估计器把飞机掀翻的"。**不成立**。把 `recovery.json` 的 monotonic 事件与 `launch.log`
  的 unix 时间戳对齐到同一起点（`cf85_reset_timeline.py`）：

  | 相对起飞请求 | 事件 |
  |---|---|
  | +0.000 s | `takeoff_requested` |
  | +0.766 s | `controlled_land_started`（`xy_drift_exceeded` 已判） |
  | +2.544 s | **首次** `ESTKALMAN` 重置 |
  | +3.145 s | `acceptance_failed` |
  | +5.652 s | 第二次重置 |

  首次重置比失控判定晚 **1.78 s**，比翻倒（+0.2 s 级）晚更多 → **重置是翻倒的后果，不是起因。**
  所以"支架形变导致推力偏斜"这条主因判断仍然成立，F6 不能用来解释事故。
  注意：`+86.953 s` 那次发生在降落之后，也属后果。
- **本轮未定性**：没有采到 `dmp_flow` 的原始 `[dX,dY,motion,squal]` 块，无法判定是 squal 偏低、
  地面纹理不足，还是积分率放大。下次复现时应同时抓 `flow_raw.py` 的原始块做对照。
- **发生**：2026-10-20（地面 PID 诊断，未起桨）。

### F7 Flow Deck 完全不被检测（1-wire 枚举为空）

**不要因为这条去换 deck 或送修。** 已两次被误判为硬件故障，两次都是"重新上电就好"。

- **签名**：`deck.bc*` 十四项全 0；实时流 `motion.deltaX/deltaY=0`、`motion.squal=0`、
  `range.zrange=0`；`stabilizer.estimator` 从 2 掉到 **1**（Kalman 退化为互补滤波）；
  `stateEstimate.z` 会跑到 **−17 m** 这类荒谬值（失去测距后的发散）。
  对照：`pm.vbat` / `supervisor` 正常 → 机体本身健康。
- **判据**：**必须用实时流**（`flow_deck_live.py`），不能只看参数表。
  `deck.bc*` 是**启动时**写入的快照，不是实时状态。判据以 `squal`/`zrange` 是否非零为准。
- **诱因（已证两条，都不充分）**：
  - 2026-10-14：静置 ~3.83 V **低压**上电 → 失败；充电后重新上电 → 恢复。
  - 2026-10-20：**4.1067 V 满电**上电 → 仍失败；用户**拔电重插** → 恢复。
  → **低压不是必要条件。** 更可能是**插电瞬间的接触抖动**（1-wire 在抖动中枚举失败）。
  上电时一次插到底、避免弹跳，可能比"充满电"更关键。此推论尚未单独验证。
- **修复顺序（先便宜后贵）**：
  1. **拔电重插**（2026-10-20 证明这一步有效，且比"先充电"更快）；
  2. 整机 + 电台全拔重插冷启动；
  3. `stm_power_cycle()` 软件电源循环；
  4. 物理重插 deck 排线；
  5. 以上都无效才考虑换 deck 对照。
- **已排除**：写参数/写 NVM 不可能恢复（`deck.*` 全 RO、无 force 开关）。
- **黄金对照**：正常启动横幅含 `DECK_DISCOVERY: Backend found: onewire` +
  `PMW: Motion chip id: 0x49:0xB6`。
- **教训**：deck 的 1-wire **只在干净的上电瞬间枚举**。表现为"deck 消失"，极易误判硬件损坏。
  **判硬件故障前必须先排除启动抖动**，而"低压"只是历史诱因之一。
- **历史**：2026-10-13 ~ 10-14；2026-10-20。

### F8 电台链路故障（nRF51 应答、STM32 无应答）

- **签名**：无线 ACK 正常、信道扫描命中，但应用层 CRTP 不应答
  （`RADIO_ACK_ONLY`）；`platform.protocolVersion` 读不到。
- **判据**：**必须应用层 CRTP 真应答才算找到机体**（RUNBOOK §2.2）；
  nRF51 ACK / link quality / 信道扫描命中都不算。
- **修复**：**全断电冷启动（USB + 电池都拔）**——只拔 USB 无效，STM32 应用卡死。
- **发生**：2026-09-22。
- **注意**：现在的 `cf_prepare.sh` 会明确报 `Radio FAIL`，绝不误报成 `Battery FAIL`。

### F9 USB/IP（usbipd/WSL）链路不稳

- **签名**：cflib 探针无限挂死（正常 ~20 s）；CRTP connect 0/5 成功、无 connection_failed
  消息；1M/250K 报 `Too many packets lost`；`dmesg` 大量 `vhci_hcd: vhci_device speed not set`；
  `usbipd list` 显示 Attached 但 WSL 内 `lsusb` 看不到（陈旧状态）。
- **判据**：`lsusb` 在 WSL 内看 + 完整 CRTP connect 双向包。
- **修复**：**物理拔插 Crazyradio**（最可靠）；然后 `usbipd list` 读新 busid →
  `usbipd attach --wsl --busid <NEW>`。WSL 重启后若显示 `already attached` 先 detach 再 attach。
- **发生**：2026-10-14 凌晨（触发源：后台探针占住电台——见通用铁律"探针与栈不同时占电台"）。
- **注意**：WSL 空闲自动关机会使 USB/IP 失效；一律用 `run_probe.sh` 保证同一会话内电台可用。

### F10 环境 bug（脚本层，非飞机问题）

- **签名**：`cf_prepare.sh` 报 `No module named 'cflib'`（被误报成找不到机体）；
  用错 busid 自动修复失败。
- **判据**：入口脚本未激活 venv；busid 写死。
- **修复**：`cf_ops_source_env()` 自动激活 `~/crazyswarm-env`；`cf_ops_radio_busid()` 运行时解析。
- **发生**：2026-10-13（首次真机跑 cf_prepare.sh 暴露；此前只有离线验证）。
- **教训**：**离线验证过的脚本不等于真机跑过**；入口首次上真机前先走一遍只读路径。
- **2026-10-20 同族第二例（已修）**：`cf8_token.py` 把"binding 内容变了（sha/robot）"与
  "binding 完好但地址与 token 授权的不一致"合并成同一句 `runtime binding changed after prepare`。
  后果：`run_cf8_acceptance.sh` 第 67 行的 URI 锁永远不可达，`cf85_smoke_test.sh` 的
  `token authorises` 检查恒 FAIL（121 项里唯一一项）。修复 = 拆成两条独立报错，sha/robot 仍先判。
  **教训**：两个不同故障合并成一句报错，等于让后一个闸门变成死代码；smoke 里那条 FAIL 就是它的探针。

### F11 判读陷阱：汇总指标的时间窗口（把落地后的 EKF 漂移算成飞行漂移）

- **签名**（2026-10-20 实测，`20261020-213220-pid-takeoff/recorder.log`）：
  `cf8_metric_parity` 报 `max_xy_drift_m = 14.58`、`max_speed_mps = 11.29`、`min_battery_voltage = 2.86`。
  一架**系留**的 Crazyflie 不可能达到 11 m/s（40 km/h），也不可能位移 14 m。
- **真因**：这几个指标在整个 102 s 录制窗口上取极值，而**飞行只占其中约 10 s**。
  其中 14.58 m 出现在起飞后 +7.5 s 的 **EKF 发散**阶段（`ESTKALMAN: State out of bounds` 同一时刻），
  此时 `zrange` 一直钉在 0.35 m，机体并没有相应运动。
- **判据**：把 `recorder.ndjson` 按**起飞时刻**切片后重算，而不是信 `recorder.log` 的极值。
  同时用 `zrange`（直接测地距）交叉验证：若 `stateEstimate.z` 与 `zrange` 差一个数量级，
  **EKF 已失效，所有位置/速度数字作废**。
- **修复（本次未改代码，先记录口径）**：解读任何 `recorder.log` 的 `max_*` 指标前，
  必须确认它是在飞行窗口内取值；否则会得出"飞机飞得极快/飘得极远"的错误结论。
- **发生**：2026-10-20。
- **教训**：**汇总指标的时间窗口比数值本身更重要。** 另有一条同源教训：当轮还曾把飞行窗口
  定义成 `actuatorThrust > 1000`（仅 2.96 s），把真正的飞行段切在窗外，得出"没离地"的错误结论。

### F12 探针/栈互斥冲突（自伤类）

- **签名**：某个探针突然挂死；后台 job 占住电台导致第二个探针阻塞。
- **判据**：`ps -eo pid,ppid,etime,stat,cmd` 查残留进程；`sudo fuser -v /dev/bus/usb/...` 查占用。
- **修复**：清理孤儿进程；**cflib 探针与 Crazyswarm2 永不同时占用电台**（硬约束）。
- **发生**：2026-09-22（孤儿进程占 USB）；2026-10-14（后台探针占电台）。

### F13 位表不可凭记忆（`supervisor.info`）—— 2026-10-20 曾因此误判

- **症状**：按"记忆中的位表"解码，`526`（项目里一直当作**健康可飞**的值）会被同时解成
  `isTumbled`，`286` 会被解成 `hasFlipped + isCharging`。据此得出"飞机翻了、还插着充电线"的结论，
  **与实测和目视都不符**（姿态全程 roll ≤13.24°、pitch ≤7.08°，电压在负载下塌陷而非充电态）。
- **本机现状**：**没有固件源码**（`supervisor.h` 不存在），**cflib 也不带解码表**。
  所以任何位定义都是猜测。
- **正确做法**：
  1. **优先用物理量判断**，不要用 supervisor 位：姿态角判断是否翻倒、`zrange` 判断是否离地、
     `squal` 判断光流可信度、`pm.vbat`/`pm.state` 判断供电与充电。
  2. 若必须使用 supervisor 位，先拿**已知真值**反推（本项目已知真值：健康上电=526、
     10-14 真正翻倒事故瞬间=164），并且只把**确实变化过的位**当作信号。
  3. 该轮 `cf85_level_check.py` 自带的宽泛解码（如"can_fly=YES"）同样**不可**作为适航证明。
- **发生**：2026-10-20。教训：**"我以为是这个含义"不等于"固件是这个含义"。**

### F14 `Low unicast receive rate` 与 `Unexpected number of unicast packets`

- **签名**：`launch.log` 里成对出现
  `Unexpected number of unicast packets. Sent: 808. Received: 35912`（Received 是 Sent 的几十倍）
  与 `Low unicast receive rate (0.84 < 0.90). Sent: 179. Received: 149`。
- **判据（必须分开看，二者来源不同）**：
  - **开头那条 “Received 几十倍” 是软件伪影，不是重传。**
    `crazyflie_server.cpp` 第 1085–1086 行 `uint16_t previous_numRxBc/previous_numRxUc;`
    **没有默认初始化**，构造函数初始化列表也未包含它们。首次回调里
    `deltaRxUc = data->numRxUc - <未初始化值>` 下溢，再被 `+= 65535` 的“溢出修正”放大。
    实测比率 82.9× / 44.4× / 47.1× / 48.8× 随机分布，正是未初始化内存的特征。
    **不要用它推断“重传放大”或“机体收到更多包”。**
  - **`Low unicast` 方向可信、幅度不可信。** 它用同一回调的
    `num_tx_unicast`（dongle 发送数）与 `num_rx_unicast`（机体 `radio.numRxUc` 增量），
    分母干净、分子受上面那个偏移污染一次后即自洽，所以“确实丢了一些”成立，
    但 0.84 / 0.87 这类具体值不能当精确测量用。
- **判据（与硬件无关的部分）**：
  - 丢包与**起飞负载无关**（2026-10-21 00:03 地面零飞行命令即复现）；
  - 丢包与 **USB 设备复位无因果**（200 s 窗口 0 reset 仍有 4 次；反向亦成立——
    告警在前，`usb 1-1: reset` 晚 21 s）；
  - `vhci_hcd: urb->status -104` 出现在告警**之后** 47 s，属后果不是原因。
- **已验证的缓解（非修复）**：把日志频率从 10/20 Hz 降到 5/10 Hz 后，935 s **零丢包**；
  但原频率 936 s 也只有 1 次，**样本量不足以判定因果**，只能算弱证据。
- **仍存假设**：主机侧调度/USB 提交延迟、射频 ACK 丢失、dongle 重传策略。
  未排除的物理变量：USB 口（是否经 hub）、USB 线长、信道占用。
- **修复**：**不得**改 `warnings.communication.min_unicast_receive_rate`（默认 0.9）来让告警消失。
  若确需消除伪影，正确做法是给那两个成员加 `{0}` 初始化（上游代码，需单独评估）。
- **发生**：2026-10-20（起飞后 2.64 s）；2026-10-21（地面多次复现）。
- **教训**：**先读源码再解释统计量。** 一个未初始化变量能让整条归因链走错方向。

### F15 机体侧躺/`tumbled` 锁存时起栈，会伪造出“EKF 发散 + 位置漂移”

- **签名**：`launch.log` 里 `ESTKALMAN: State out of bounds, resetting` **高频连续**（本次 156 次 / 8 min）；
  `stateEstimate.y` 平滑单调涨到 10 m 以上；`odom.twist.linear.y` 稳定在 4–9 m/s；
  但 `zrange` 全程 0.015–0.026 m（机体根本没离地）；`squal` 正常 50–90。
- **判据**：**先看 `supervisor.info`**。本次 `548 = 0x0224`（`tumbled` 锁存，`can_fly` 消失）。
  再用**加速度计独立验证**（`cf85_level_check.py`）：本次 `acc=(−0.004, +0.970, −0.279)`、
  机体 Z 轴偏离重力 **74.0°** → **飞机侧躺着**。侧躺时光流芯片看到的是墙面/侧向运动，
  EKF 把它积分成“水平飞出去”，数字看着像漂移，其实是物理姿态。
- **区分**：F6（EKF 真的发散）发生在机体**水平**时；本条发生在机体**倾斜**时。
  两者 `ESTKALMAN` 行一模一样，**只能靠姿态读数区分**。
- **修复**：把机体摆正 → **给飞机断电重启**（`tumbled` 锁存重启 ROS 不清，见 F1 残余）→
  重跑 `cf85_level_check.py` 确认 `tumbled: no` 且倾斜 < 5°。
- **发生**：2026-10-21 14:09（换 USB 口期间飞机被碰倒）。同轮 13:46 那次 `zrange` 冲到
  0.218 m 又落回，是同一件事的前兆，当时没认出来。
- **教训**：**起栈前必须验姿态，不能只看“连上了”。** 一次侧躺能产出 8 分钟看起来
  “很像真故障”的数据，并让整轮实验作废。

### F16 突发型链路劣化（26 秒内连续丢包 + 延迟冲到 923 ms）

- **签名**（2026-10-21 15:03–15:04，`20261021-144819-link-port2c-900`）：
  正常跑了 **900 s**（延迟 ≤16 ms），随后在 **t+904…930 的 26 秒内**连续 4 条
  `Low unicast`（0.66 / 0.30 / 0.68 / 0.84），伴随 `High latency` 574 → 923 → 141 → 435 → 226 ms；
  之后**自行恢复**（t+953 起又回到 10–12 ms）。
- **判据（关键鉴别）**：
  1. 爆发窗口内 `dmesg --time-format=iso` **一条内核 USB 事件都没有**；
  2. `urb->status -104` 在停栈前只有 14:37 那 2 条，其余 37 条**全部在 15:05:05 停栈之后**；
  3. recorder 侧出现 **0.9–1.1 s 的采样断档**（5 次），而 `squal` 全程 13–50 正常、`z` 稳定在 0.015–0.018 m；
  4. `Sent` 数值在爆发期间是 **132–419**（正常约 300–870），说明 dongle 侧发包节奏也被拖慢。
  → **不是 USB 复位、不是 deck、不是姿态**。是**主机或 dongle 侧的瞬时调度/提交停顿**，
  持续约 26 s 后自愈。
- **与 F14 的关系**：F14 说的是“`Received ≫ Sent` 那条是未初始化伪影”；本条讲的是**真实的**
  `Low unicast`，二者的 `Sent/Received` 量级不同（伪影是几十倍，本条是 0.3–0.84）。
- **修复**：**尚无**。可试：换短 USB 线、避免主机进入低功耗、`usbipd` 直通模式。
  **不得**改 `min_unicast_receive_rate`。
- **发生**：2026-10-21 15:03（新 USB 口）；2026-10-20 21:32（起飞后 2.64 s，可能同源）。
- **教训**：**“900 s 跑完没事”不等于稳定**——劣化可以是 26 秒的突发，只看总数会把它平均掉。

## 7. 环境坑（每坑只写一次；新坑先验证再入表）

| 坑 | 现象 | 处理 |
|---|---|---|
| **ROS 2 发现域 = SUBNET** | 服务端在发数据（`topic hz` 正常）但 recorder **零样本**，报假 `Link FAIL`；`health.json` 为空 | **必须 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`**。本机 WSL 是 mirrored 网络 + 不稳网卡（eth4），SUBNET 多播发现会间歇失败。已固化在 5 个脚本里 |
| `/dev/shm` 陈旧 FastDDS 段 | 同上，另加发现不稳 | 无存活节点时 `rm -f /dev/shm/fastrtps_*`（见 `cf85_fix_discovery.sh`） |
| WSL 空闲自动关机 | USB/IP 挂载失效，`usbipd list` 仍显示 Attached（陈旧） | 一律用 `run_probe.sh`（同一 WSL 会话内确保电台再跑探针）；保持 keepalive |
| 电台 busid 会变 | 插拔后 1-3 → 1-4 → 3-1 | `cf_ops_radio_busid()` 运行时解析；手动命令先 `usbipd list` |
| venv 未激活 | 裸 `python3` 无 cflib；**裸 shell 也没有 ruff/mypy** | 入口脚本 `cf_ops_source_env()` 已自动激活 `~/crazyswarm-env`；跑 lint 前也要先 activate |
| `.bashrc` L130 venv 行被注释 | 登录 shell 不自动激活 | 不要依赖自动激活；入口脚本显式激活 |
| `.pytest-tmp` ACL 坏 | `tmp_path` 测试整批 ERROR / WinError 5 | `--basetemp=.pytest-tmp-<新名>`；`.pytest-tmp` 需管理员修，别碰 |
| 重复 `-q` | 看不到 `N passed` 汇总（addopts 已含 `-q`） | 命令行不要再加 `-q` |
| 路径含空格 | DSH 命令静默失败/切分错误 | 用 `D:\dmp` junction（真路径 `D:\Deepseek WorkSpace\...`） |
| PowerShell 内联 `$` | 被提前展开 | shell 逻辑写进 `.sh`；pwsh 内联不写 `$` |
| Windows 改文件 CRLF | `write_text` 产生整文件假 diff | 字节读写 + `replace(b"\r\n", b"\n")` |
| WSL 重启后 attach | 显示 `already attached to a client` | 先显式 `usbipd detach --busid <B>` 再 attach |
| 探针挂死 | 被后台 job 占电台 | 先清进程再跑；绝不并行两个探针（见 F12） |
| **WSL 时钟域不一致** | 用 `boot_epoch + dmesg时间戳` 推算墙钟，会**偏 2 s 以上**，足以把因果判反 | **不要推算**。用 `dmesg --time-format=iso`（本机已支持）直接拿 CST 墙钟。实测：`time.time()` 在 60 s 内跳变 +2.335 s，而 `/proc/uptime` 与 `time.monotonic()` 同步 |
| **WSL 墙钟阶梯跳变** | 2026-10-26 23:05 那轮约 67 s 内，`wall - monotonic` 偏移量先后跳了**约 2.97 s 两次**；不是匀速漂移，采样段内几乎恒定 | 只用 `time.monotonic()` 做间隔与排序，墙钟只用来**标注**时刻。判是否跳变看 `wall - monotonic` 呈阶跃 |
| **机体毫秒计数器偏快** | 不是宿主问题：同区间 `/proc/uptime` 与 `monotonic` 差 0.5 ms，而固件 `timestamp` 快 6.3–9.2% | 日志按固件时间戳排序与切片；报 Hz、时长必须注明时间基（固件秒 vs 真实秒）。判据见 §5.4 第 4 条 |

## 8. 证据布局（每次运行一个目录，永不覆盖历史）

```
D:\Deepseek WorkSpace\cf_evidence_runs\<时间戳>-<阶段>\
    meta.txt / runtime-binding.json / uri-resolution.json / bridge.runtime.yaml /
    crazyflies.runtime.yaml / battery.json / radio.log / recorder.ndjson /
    launch.log / health.json / capability.json / preflight.json /
    prepare.log / ready_for_cf8.json / summary.md
飞行时追加：bridge-flight.log / flight-attempt.log / flight-result.json / final-state.json
```

- 中间诊断必须带时间戳；最终产物用稳定文件名。
- 护栏：脚本拒绝写 `cf81_dryrun_evidence` 或含 `cf84-recorder` 的路径（exit 2）——
  CF84 唯一真实飞行 timeline 不可能被覆盖。
- 分析 recorder 数据必须**截断 operator pickup 之后的样本**（用户结束后会用手拿机体）。

## 9. 出问题时看哪里（对照表）

| 现象 | 先看 | 含义 |
|---|---|---|
| `Radio FAIL` + `NO_VALID_URI` | `uri-resolution.log`、`runtime-binding.json` | 没有地址能 CRTP 应答（没开机 / 只有 nRF51 答 / 地址不对）→ F8 |
| `Radio FAIL` + `AMBIGUOUS` | `runtime-binding.json` 的 valid_candidates | 两个地址都能 CRTP，绝不猜 |
| `Radio FAIL` + `RADIO_ACK_ONLY` | `uri-resolution.log` | 电台层 ACK、应用层无应答 → F8 |
| `Radio FAIL` + `CONFIG_INVALID` | `config/cf_identity.yaml` | 身份配置本身有问题 |
| `Battery FAIL` | `battery.json`、`battery.raw.log` | 电压不达标/读不到 → F2；`FORBIDDEN_TETHERED` → F1 |
| `Link FAIL` | `health.json` | 五路有流没数据 / 速率远低标称 |
| `Preflight FAIL` | `preflight.json` 的 failed_checks | 逐条明细 |
| Stage 2.75 报 "Flow Deck v2 decks are missing" | `flow_deck_live.py`、`flow_raw.py` | deck 仍为 0 → F7（参数为 0）或 F5（有参数无数据） |
| 探针挂死 | 进程列表 + `fuser` | F9（USB/IP）/ F12（探针与栈抢电台） |
| 想看内部步骤 | `prepare.log`、`--verbose` | 全部内部命令与输出 |

`cf_prepare.sh` 任何一步 FAIL 立即停止且不重试；重来就重新跑（自动开新 run dir）。

## 10. 绝对不做的事（硬约束，违反即停）

1. 不改任何阈值/gate/TTL/estimator 策略：`hardware_flight_enabled`(false)、
   `capability_snapshot_max_age_s`(3600)、`max_estimator_variance_span`(0.002)、STABILIZING 全部参数。
   **例外（2026-10-14 用户明确授权、已完成）**：电池判据拆为地面 `battery_critical_voltage`(3.7，未改)
   + 空中 `battery_critical_airborne_voltage`(3.3) + 去抖 `minimum_battery_critical_samples`(3)。
   理由与依据见 §4.1。除此之外不得再改任何阈值。
2. `hardware_flight_enabled` 保持 false，除非用户**明确单次授权** CF8。
3. 不绕过 readiness gate；不用 emergency stop 替代受控降落。
4. 不跑 CF9 / 硬件 `go_to` / `execute_mission` / `hello_world`。
5. 不自动重试 CF8：一次授权 = 最多一次 `run_cf8_acceptance`。
6. 不手工直接调用 `cf84_preflight.py`（唯一入口 `cf_prepare.sh`）。
7. cflib 探针与 Crazyswarm2 不同时占用电台；不并行跑两个探针。
8. 不写机体 NVM、不改 radio address/channel/speed（唯一写入口 `cf85_cf2.sh --confirm-write`）。
9. 不改 `bridge.yaml` / `crazyflies.yaml`；runtime 配置只写 run dir。
   **例外（2026-10-14 已授权、已完成）**：为电池分级判据在 `bridge.yaml` 新增
   `battery_critical_airborne_voltage` / `minimum_battery_critical_samples` 两个键（见 §4.1）。
10. 不把 `cf_identity.yaml` 的 policy 开关改 false 来"修好"失败（会被判 CONFIG_INVALID）。
11. push / merge 需用户明确要求，默认不做。
12. 在用户说"现在上电"前不连 cf231；未授权前绝不真实起飞。

## 11. 飞行后报告清单（必须逐项）

结果 / `failure_code` / 事件时间线 / `failure_telemetry`（真实触发值）/ `max_z` /
实际离地高度 / 最大 XY 漂移 / 最大速度 / 最低电池 / squal / varP / range / supervisor /
是否进入 STABILIZING / service 调用次数 / 是否受控降落 / 最终安全状态。
+ 安全声明逐项：arm / takeoff / land / go_to / emergency = YES/NO；NVM 写入；阈值修改。
