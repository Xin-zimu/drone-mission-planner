# 会话日志（当前状态快照）

> **这是一份快照，不是编年史。** 只写"现在是什么"，不写"以前以为是什么"。
> 历史过程见 `docs/archive/session-history-*.md`；本文件每次会话**重写**，不追加。
> 新会话读完本文件即可开工；发现本文件与现实不符 → 先改本文件，再动手。
>
> **不在这里重复完成判据** —— 它们只在 `STAGE.md` §2，那里直接挂实测证据。
> 本文件只回答：现在能不能飞、飞机是什么状态、环境有什么坑、下一步做什么。

- 最近重写：2026-10-20 23:2x
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
| 电池 | 官方 250 mAh 1S；**本包已劣化**（sag 1.081 V）。静置读数会回升，**静置正常不代表能飞** |
| 电台 | Crazyradio 2.0 `1915:7777`，busid 会变（历史 1-3 → 1-4 → 3-1）；WSL 内需 USB/IP 转接 |
| 固件 | `a54f11eb6d00`；`pm.lowVoltage=3.2`、`pm.criticalLowVoltage=3.0` |
| Flow Deck | 正常（`bcFlow2=1`、`bcZRanger2=1`、`squal` 有数据）。**会因上电抖动掉枚举**，见 §5 F3 |
| 机载控制 | `stabilizer.controller=1`(PID)、`estimator=2`(Kalman)、`motorPowerSet.enable=0` |
| `commander.enHighLevel` | **仅由 Crazyswarm2 启动时写为 1**。单独探针读到 0 是固件默认，不是故障 |
| 机架 | 用户确认：电机塑料支脚**固定歪斜、无裂纹**、可更换。倾斜量未测量 |
| 最近姿态 | 静置 roll 2.77° / pitch 4.26°（加速度计与估计器大体一致） |

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

本阶段之外的待办（不阻塞 CF8）：

- [ ] `FALLBACK_ADDRESS_IN_USE`：机体答 `E7E7E7E7E7`，`cf_identity.yaml` 偏好 `...E705` 无应答
- [ ] 提交未提交改动（产品代码与规范文档分开提交）
- [ ] 软件线 M2（真机航点执行，默认关闭）—— 阶段启动时再展开，见 `PLAN.md` §4
