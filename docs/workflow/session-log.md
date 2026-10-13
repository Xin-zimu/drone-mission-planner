# 会话日志（当前状态唯一事实来源）

> 这是**唯一**的当前状态页。任何新会话先读这里。
> 状态有变化就更新本文件；本文件与 git 工作树不符时以本文件为准并立即核查。

- 最近更新时间：2026-10-14 02:30（由本次"规范体系建立"会话写入）
- 主线目标：**CF8 真机验收通过**（首次真机 hover 起飞→悬停→降落一次且仅一次）
- 下一里程碑：M3（拿回飞机后恢复基础飞行能力）见 `learning-plans/路线联动推进计划.md`

## 1. 仓库状态

| 项 | 值 |
|---|---|
| 分支 | `codex/v1.3-crazyflie-execution` |
| HEAD | `7c1d904` — v1.3 CF8.1: capability snapshot schema 2 — robot identity vs radio address |
| push 状态 | **ahead 5，未 push**（默认不 push，除非用户明确要求） |
| 工作树 | 干净；本次规范体系新增文件待提交（见 §5） |

## 2. 安全不变量（当前值，违反即停）

| 项 | 值 | 位置 |
|---|---|---|
| `hardware_flight_enabled` | **false**（未改） | `integrations/crazyflie_ros2/src/dmp_crazyflie_bridge/dmp_crazyflie_bridge/config/bridge.yaml` L7 |
| `battery_critical_voltage` | **3.7 V**（未改） | bridge.yaml L10 |
| `capability_snapshot_max_age_s` | **3600**（未改） | bridge.yaml L9 |
| `max_estimator_variance_span` | **0.002**（未改） | bridge.yaml L21 |
| 电池会话政策 | `<4.00V` 禁飞；`4.00–4.10V` 仅诊断；`≥4.10V` 放行；"充满"但 `<3.90V` 判可疑电池 | 见 cf8-operations.md §4 |

## 3. 硬件状态（cf231）

| 项 | 状态（2026-10-14 01:50 结） |
|---|---|
| 电池 | **3.964 V，90%，已脱离外电、正在放电**，低于 4.00 V 会话禁飞线 |
| 电台 | Crazyradio 2.0 `1915:7777`，busid **3-1**，拔插后链路已恢复（CRTP connect 2.8s） |
| 固件 | `a54f11eb6d00`（与 09-22 逐字节一致，未变） |
| **当前阻断点** | **Flow Deck 完全不被检测**：`deck.bcFlow2=0`、`bcZRanger2=0`、`TYPE_1W=0`、`TYPE_I2C=1`、三路传感器（deltaX/deltaY/squal/zrange）恒 0；`pm.vbat`/`stateEstimate`/`supervisor` 正常 → 机体健康，单 deck 子系统未起来 |
| 已排除 | 充电（非充电中）、软件电源循环 `stm_power_cycle()`、物理重插 deck、整机+电台全拔重插 → 全部无效 |
| 判定 | **deck 侧或板侧 1-wire 通路硬件故障**（不是固件、不是软件、不是环境） |
| 下一步 | 用户找一块**已知正常的 Flow Deck v2** 换上去测：若 `TYPE_1W` 仍 0 → 板侧 header/pull-up 故障需维修；若恢复 → 原 deck 损坏 |

## 4. 环境状态

| 项 | 值 |
|---|---|
| 仓库真路径 | `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`；`D:\dmp` 是 junction |
| WSL | Ubuntu 24.04，ROS 2 Jazzy；cflib 在 venv `~/crazyswarm-env`（系统 python3 没有）；`.bashrc` 里 venv 激活行被注释，入口脚本已自动激活（cf_ops_lib.sh `cf_ops_source_env()`） |
| 电台 busid | 会随插拔变化（历史：1-3 → 1-4 → 3-1）；`cf_ops_radio_busid()` 运行时解析；手动命令先 `usbipd list` 确认 |
| 电台掉线 | WSL 空闲自动关机 → USB/IP 失效；`usbipd list` 仍显示 Attached 是陈旧状态；一律用 `_dsh_scratch/run_probe.sh` |
| pytest | 必须 `--basetemp=.pytest-tmp-<新名>`（`.pytest-tmp` ACL 坏）；**不要重复 `-q`**（addopts 已含） |

## 5. 本次规范体系改动（待提交）

新增 `docs/workflow/`（session-protocol.md、session-log.md、cf8-operations.md、dev-workflow.md）；
新增 `docs/archive/`（历史交接文档归档）；`AGENTS.md` 补充指向 `docs/workflow/` 的说明。

## 6. 最近会话记录（新→旧）

| 日期 | 会话 | 结论 |
|---|---|---|
| 2026-10-14 凌晨 | CF8 交接文档与 Flow Deck 排障 | 三种补救耗尽：deck 仍不被检测 → 判硬件故障；期间电台链路曾坏（自伤：后台探针占电台），拔插修复；电池掉到 3.964 V |
| 2026-10-13 | 换新电池上电 | 首次新电池上电；修复 cf_prepare.sh 两个环境 bug（venv 未激活、busid 写死）；确认 Flow Deck 完全不被检测（决定性证据）；提交 `7c1d904` |
| 2026-09-28 | CF8.5 脚本修复 | 离线验证两个入口（cf_prepare.sh / run_cf8_acceptance.sh）+ RUNBOOK |
| 2026-09-22 | CF8 真机联调 | 全天排障：电池掉压 → 电台链路（nRF51 应答、STM32 无应答）→ 全断电冷启动恢复；收尾交接 |
| 2026-09-21 | 首次 CF8 | 两次真实尝试均 `battery_critical` 受控降落；修 estimator convergence 用错变量；observability 修复 |

## 7. 待办（跨会话）

- [ ] Flow Deck 硬件确认：换已知正常 deck 对照测试（**用户物理操作**）
- [ ] 电池充满到 ≥4.10 V 并静置复测（充电电流偏慢 ~0.04 A 需留意）
- [ ] deck 恢复后：`./cf_prepare.sh` → 用户授权 → `./run_cf8_acceptance.sh`（一次且仅一次）
- [ ] 本次规范体系提交（见 §5）
- [ ] 软件研发线：M2（默认关闭的真机航点执行路径）见 `learning-plans/路线联动推进计划.md`
