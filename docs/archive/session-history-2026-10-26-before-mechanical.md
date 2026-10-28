# 会话日志（当前状态快照）

> **这是一份快照，不是编年史。** 只写"现在是什么"，不写"以前以为是什么"。
> 历史过程见 `docs/archive/session-history-*.md`；本文件每次会话**重写**，不追加。
> 新会话读完本文件即可开工；发现本文件与现实不符 → 先改本文件，再动手。
>
> **不在这里重复完成判据** —— 它们只在 `STAGE.md` §2，那里直接挂实测证据。
> 本文件只回答：现在能不能飞、飞机是什么状态、环境有什么坑、下一步做什么。

- 最近重写：2026-10-21 15:10（链路诊断轮 B 收尾：换口后两次 900 s 对照，结果相反；定位到“26 秒突发型劣化”，与 USB 事件无时间相关性）
- 当前阶段：**CF8 单机真实 hover**（目标与判据见 `STAGE.md`）
- **一句话状态：不可飞。** 四条判据中三条不成立、一条未测

---

## 1. 为什么现在不能飞

判据原文与证据**只在 `STAGE.md` §2**，此处不复制数值。三条不成立的简述：

| 判据 | 状态 | 一句话 |
|---|---|---|
| 电源 | ❌ | 本包 sag 超标，跌破固件硬断电 |
| 链路 | ❌ | 起飞后约 2.6 s 开始丢包 |
| 定位 | ❌ | 起飞后约 5 s EKF 发散，`z` 与 `zrange` 差一个量级 |

**形变问题未触及**：以上任一条不成立，"飞控能否补偿形变"就无法判断——
不是形变太严重，是别的问题先发生。三次翻倒（10-14 两次 + 10-20 一次）都不能再归因于
"单一机械根因"。

---

## 2. 当前飞机与设备状态（实测）

| 项 | 值 |
|---|---|
| 电池 | 官方 250 mAh 1S。**旧包已劣化**（sag 1.081 V）。**2026-10-21 00:00 已换新包**：静置 4.065 V / 90 % / `pm_state=0` / 充电电流 0 A，地面负载 119 s 最低 4.032 V。**新包尚未起桨，sag 未知**；静置正常不代表能飞 |
| 当前供电状态（2026-10-21 13:19） | **在充电（`pm_state=2` charged，4.154 V，充电电流 0 A）** → `FORBIDDEN_TETHERED`。**起飞前必须拔充电器**，否则电池闸门硬拦 |
| 电台 | Crazyradio 2.0 `1915:7777`。**2026-10-21 14:09 已换口**：busid 3-1 → **1-3**，Windows 侧 `Port_#0003.Hub_#0001`，父设备 `USB\ROOT_HUB30`（**主板根集线器，非外接 hub**）。busid 每次插拔都会变，一律运行时解析 |
| 固件 | `a54f11eb6d00`；`pm.lowVoltage=3.2`、`pm.criticalLowVoltage=3.0` |
| Flow Deck | `bcFlow2=1`、`bcZRanger2=1`，1-wire 读得到 `bcFlow2 A`。**会因上电抖动掉枚举**（F5）：实测 00:03 活（squal 9–52）、00:21 死（全程 0）、13:22 活（63–106）、13:28 活（70–110）、13:46 活（28–155） |
| 机载控制 | `stabilizer.controller=1`(PID)、`estimator=2`(Kalman)、`motorPowerSet.enable=0` |
| `commander.enHighLevel` | **仅由 Crazyswarm2 启动时写为 1**。单独探针读到 0 是固件默认，不是故障 |
| 机架 | 用户确认：电机塑料支脚**固定歪斜、无裂纹**、可更换。倾斜量未测量 |
| 最近姿态（**2026-10-21 14:20 实测，当前状态**） | **侧躺**：加速度计 `(−0.004, +0.970, −0.279)`，机体 Z 轴偏离重力 **74.0°**；`stabilizer.roll 83.7°`；`supervisor 548` → **`tumbled` 已锁存，`can_fly: NO`**。换 USB 口期间被碰倒 |
| 恢复正常姿态所需 | **摆正机体 → 给飞机断电重启**（`tumbled` 锁存重启 ROS 无效，见 F1 残余）→ 跑 `cf85_level_check.py` 确认 `tumbled: no` 且倾斜 < 5° |

---

## 3. 安全不变量（违反即停）

| 项 | 当前值 | 位置 |
|---|---|---|
| `hardware_flight_enabled` | **false** | `bridge.yaml` L7 |
| `battery_critical_voltage` | **3.7 V**（地面/静置放行判据） | bridge.yaml |
| `battery_critical_airborne_voltage` | **3.3 V**（空中/带载判据） | bridge.yaml |
| `minimum_battery_critical_samples` | **3**（去抖） | bridge.yaml |
| preflight `MIN_BATTERY_V` | **4.00 V**（飞行放行线） | `cf84_preflight.py` L41 |
| 会话闸门 `BATTERY_FORBID_V` | **3.50 V**（地面诊断，**非**飞行放行线） | `cf_ops_lib.sh` + `cf85_lib.sh`（两处必须一致） |
| 外电互锁 | `pm_state≠0` 或充电电流 >0.01 A → **一律禁飞**，电压不可覆盖 | `cf85_battery_policy.py` |

**会话闸门 3.50 V 的边界**：它只放行**不起桨**的地面诊断。飞行仍由 preflight 4.00 V 和空中 3.3 V 把关。
两个 lib 曾因只改一个导致子进程静默覆盖——`cf85_gate_check.sh` 用于防漂移。

---

## 4. 仓库状态

| 项 | 值 |
|---|---|
| 分支 | `codex/v1.3-crazyflie-execution` |
| **验证基线** | **`f3189f1`**（文档体系重构完成、三项自检全绿的那次提交）。<br>**这不是"实时 HEAD"**——本字段只记录"最后一次经过验证的代码基线"，允许落后于 HEAD |
| push | ahead 10，未 push（默认不 push，除非用户明确要求） |
| 工作树 | **仍未提交**：2026-10-14 的电池判据分级（5 个产品文件 + 2 个测试）；根目录 3 份 10-14 报告（未跟踪）。`_dsh_scratch/` 与 `cf_evidence_runs/` 在**仓库外** |
| 文档自检 | `python scripts/docs/cf85_{doc_consistency,stage_audit,handoff_sim}.py`（已入仓库，可移植） |
| 证据可获取性 | 证据目录在仓库**外**（`D:\Deepseek WorkSpace\cf_evidence_runs\`）。**全新克隆不含证据**，届时自检会报"证据缺失"，那是正确行为，不是通过 |

---

## 5. 环境事实（每坑只记一次）

| 项 | 值 |
|---|---|
| 仓库真路径 | `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`；`D:\dmp` 是 junction |
| WSL | Ubuntu 24.04 + ROS 2 Jazzy；cflib/ruff/mypy 都在 venv `~/crazyswarm-env`（系统 python3 没有） |
| 电台掉线 | WSL 空闲会关机 → USB/IP 失效；`usbipd list` 显示 Attached 可能是陈旧状态。**一律用 `run_probe.sh`** |
| pytest | 必须 `--basetemp=<新目录>`；5 个 `test_ui_*.py` 因缺 PySide6 需 `--ignore` |
| ROS 发现域 | 必须 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| **DDS 假 Link FAIL 真因** | **启动顺序**，不是发现域：recorder 先起 → 0/4 通过；Crazyswarm2 先起 → 4/4 通过 |
| 探针与栈互斥 | cflib 探针与 Crazyswarm2 **永不同时占电台** |
| cflib `get_value()` | 返回**字符串**（如 `'1'`），`== 1` 恒 False。判定前必须数值归一化 |

---

## 6. 下一步

**推进顺序只在 `STAGE.md` §5**，此处不复制。当前卡在第 1 步：换健康电池、复测 sag。

### 6.1 第 1 步的三个子闸门（2026-10-21 00:05 核对）

| 子闸门 | 状态 | 依据 |
|---|---|---|
| 静态检查 | ✅ 通过 | `cf85_gate_check.sh`（两 lib 闸门同为 3.50 V、生产阈值未动、`MIN_BATTERY_V=4.00`）；`ruff` 全绿；`mypy` 19 错 = HEAD 基线 19 错（未新增）；pytest 247 passed；`cf85_battery_policy_test.py` 9/9；`cf85_smoke_test.sh` **121/121**；三项文档自检全绿 |
| 换健康电池 + 拔充电器 | ✅ **已完成** | 静置 **4.065 V** / 90 % / `pm_state=0` / 充电电流 **0.000 A** / 外电 0 V → `NEXT_STAGE_ALLOWED`。`bcFlow2=1`、`bcZRanger2=1`，1-wire 读到 `bcFlow2 A` |
| 换包后地面体检（未起桨） | ⚠️ **当时通过，但不可外推** | 30 s 窗口五路 10.57 Hz 全绿；119 s 内 0 次 ESTKALMAN；`z/zrange` 中位 1.09；`supervisor_raw` 全程 526；地面负载最低 4.032 V（**这不是 sag**）。但 00:21 同一机体、同一包、未移动时 **squal 全程 0、19 次 ESTKALMAN**——deck 会自己冻结（F5） |

**仍未测的就是 sag 本身**：地面负载不含电机电流。要判判据 1 必须起桨，而起桨需要你给下一次明确授权。

**复测 sag 的前置条件（缺一不可）**：新包静置 ≥4.00 V 且 `pm_state=0` → 限位/系留到位 →
用户明确授权起飞 → 起栈跑 `cf85_stack_for_takeoff.sh` + `cf85_takeoff_probe.py`。

### 6.2 判据 2 链路诊断（2026-10-21 13:22–14:05，非飞行，用户授权 B 线）

| 结论 | 依据 |
|---|---|
| **丢包与起飞负载无关** | 2026-10-21 00:03 地面零飞行命令即复现（服务调用计数全 0） |
| **丢包与 USB 设备复位无因果** | 200 s 窗口 0 次 reset 仍有 4 次丢包；反向亦成立——告警在 00:03:56–58，首次 reset 在 00:04:19（晚 21 s） |
| **“Received 是 Sent 几十倍”是软件伪影** | `crazyflie_server.cpp` 1085–1086 行两个 `uint16_t previous_numRx*` **未初始化**，构造函数未含；首次回调下溢后被 `+= 65535` 放大。四份实测比率 82.9× / 44.4× / 47.1× / 48.8× 随机 |
| **`Low unicast` 方向可信、幅度不可信** | 分母 `num_tx_unicast` 干净，分子受过一次污染 |
| **降频 900 s 零丢包，但证据弱** | 半频 5/10 Hz、935 s → 0 次；原频率 936 s → 仅 1 次。样本量不足以判因果 |
| **`urb->status -104` 是后果** | 13:37:53–55 出现，比 13:37:06 的告警晚 47 s |
| **换口后跑了两次 900 s，结果相反** | 14:30 那次：`Low unicast` **0**、max lat 13 ms。14:48 那次：`Low unicast` **4**（0.66/0.30/0.68/0.84）、max lat **923 ms**。**换口无差异，原“换口改善”说法已撤回** |
| **那 4 次丢包是一次 26 秒的爆发，不是 4 次独立事件** | 全部落在 t+904…930；爆发窗口内**内核 USB 事件 0 条**；`-104` 在停栈前只有 14:37 那 2 条，其余 37 条都在 15:05:05 停栈**之后**。→ 与 USB 复位、`-104` 均无时间相关性（F16） |
| **作废：14:09 那轮不是链路问题** | 开机即 `supervisor=548`（`tumbled`），加速度计 74.0° 倾斜 → 飞机侧躺。`vy` 4–9 m/s、`stateEstimate.y` 涨到 12 m、`ESTKALMAN` 156 次，全是“躺着积分”的结果。**该轮数据作废**（F15） |

**未排除的物理变量（需人工）**：USB 口是否经 hub、USB 线长、信道占用。
**禁止**：改 `min_unicast_receive_rate`(0.9) 让告警消失。

> 上一轮写的“USB/IP 带宽/复位是主因”**已按证据撤回**。完整报告见
> `cf_evidence_runs/20261021-003500-link-diagnosis/LINK_DIAGNOSIS_2026-10-21.md`（含 A/B 追加章节）。

本阶段之外的待办（不阻塞 CF8）：

- [ ] `FALLBACK_ADDRESS_IN_USE`：机体答 `E7E7E7E7E7`，`cf_identity.yaml` 偏好 `...E705` 无应答
- [ ] 提交未提交改动（产品代码与规范文档分开提交）
- [ ] 软件线 M2（真机航点执行，默认关闭）—— 阶段启动时再展开，见 `PLAN.md` §4
