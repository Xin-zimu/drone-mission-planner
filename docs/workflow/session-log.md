# 会话日志（当前状态快照）

> **这是一份快照，不是编年史。** 判据与实测证据只在 `STAGE.md` §2，推进顺序只在 §5。
> 历史过程已归档到 `docs/archive/session-history-2026-10-29-before-pid2b-offline.md`。

- 最近重写：2026-10-29，2b 离线准备第 2 段（结束判定与时间基）。
- 当前阶段：CF8 单机真实 hover。
- **一句话状态：不可飞。** 电源判据通过；链路 ❌、定位 ❌、姿态 ❌ 尚未通过。2a 静态筛查已完成，不再重复。
- 用户目标：用飞控缓解机架歪斜。当前无法判断可补偿程度，尚未决定调增益、积分或混控。

## 1. 当前阻断与下一步

用户已确认场地没有问题，并明确目前只有软件 `land` 可用。2026-10-27 的 `land` 服务应答后飞机仍继续飞，最终撞椅翻倒，不能把同一路 `land` 当作已验证的失控终止手段。

本地 Crazyswarm2 源码存在 `emergency()` → `cf_.emergencyStop()` → CRTP 发送路径。这里只核验了软件入口，未验证当前固件的电机停转效果，也未验证失链行为。它与普通命令、日志共用无线链路，不能当作独立备份。

**下一次实机步骤先拆桨验证终止通路，并检查新增日志负载；不是直接起飞。** 具体顺序见 `STAGE.md` §5。上电、转电机和后续带载分别按当次范围确认；本轮未获得这些授权。

## 2. 已核实的诊断边界

- 2a 已通过静态筛查（证据见 `STAGE.md` §2.4），不能排除起桨后的振动、采样、估计或机械问题。
- 带载日志里最先钳位的是 pitch，之后 yaw、roll；pitch 首次钳位时未见电机限幅，只说明该时刻不同步。
- 对角电机差主要对应偏航，不能作为横滚配平负担的直接量度。控制输出饱和不能证明积分饱和。
- `stabilizer.yaw` 和 `stateEstimate.yaw` 同源；`gyro.z` 是机体系角速度，净积分还会正负抵消。不能据此单独证明估计器漂移。
- `roll_kff/pitch_kff` 不是常值 trim，降低 `thrustBase` 不能直接创造悬停余量；不用 `motorPowerSet` 当闭环 trim。
- 根因仍未分离。下一份带载数据应围绕 pitch 首次钳位对齐目标、实测、双环 P/I/D、电机与机载时间，不预设偏航是起因。

## 3. 当前工具准备状态

| 工具或产物 | 当前状态与限制 |
|---|---|
| `cf85_pid_2b_prepare.py` | 纯离线生成器；根据已保存 TOC，在既有 PID 预设副本上新增按轴配对的双环 P/I/D 三块。无射频、ROS、参数写入或指令 API |
| 候选产物 | `cf_evidence_runs/20261029-pid2b-offline/candidate/`。名称避开 `preset-live*` 自动发现；不是起飞配置，不改变原有起栈脚本 |
| 新 manifest | 覆盖全部 11 个自定义话题；可供现有 `cf85_pid_diagnostics.py record` 做地面日志检查。保存命名字段、宿主到达时间、原始 24 位机载时间戳 |
| `cf85_end_state.py` | **结束判定的唯一实现**。`evaluate()` 在窗口末段分别测新鲜度/静止/电机归零/未侧躺；`termination_verdict()` 测"停机被观测到"。探针调用它，不再各留副本 |
| `cf85_onboard_time.py` | **统一时间基**：解 24 位回绕 → 单一 epoch → host↔机载斜率与残差 → 按容差合并。跨块比较前必过 |
| `cf85_termination_probe.py` | **拆桨停机测量唯一入口**：cflib 直连独占电台（**ROS 栈须先停**）；先 arming+对称油门转机，再发 CRTP emergency-stop，再观察归零并保持。要求 `--confirm-props-removed`，**不接受** `--confirm-restrained`；`--dry-run` 零发射；拒绝也落盘 |
| 旧起飞探针 | `cf85_takeoff_probe.py` 已改为调用共享判定，并补上机载时间戳与**拒绝也落盘**。它仍是有推力的限位探针，不能移除限位开关后直接用于 2b |
| 原 PID 预设 | `cf_evidence_runs/20261026-preset/preset-live/`，曾机载验证。新增三块不继承该次实机通过结论 |
| 验证边界 | 已通过保存 TOC 的变量/字节核验、离线模拟采集、真实历史数据回放；尚未验证当前机载参数、块分配和无线负载。20 Hz 不足以重建每个控制周期 |

本轮定向验证：`cf85_end_state_test` **17 项**、`test_cf85_termination_online` **20 项**、`test_cf85_probe_evidence` **9 项**、既有 `test_cf85_pid_2b_prepare` **13 passed**；Ruff 与 mypy 通过（ROS 依赖模块用 `--ignore-missing-imports`）；三项文档自检 consistent / sound / handoff（证据目录 19/19）。

**停机探针的一个前提性缺陷已修**：初版只发 stop、不转机，拆桨后输出恒为 0，判定必然返回"这次什么都没证明"——即它**永远不可能通过**。已改为 cflib 直连先转机再停止（ROS 栈须先停，原因见 `cf8-operations.md` §5）。

## 4. 硬件与环境事实

| 项 | 最近已知状态（不是本轮实机回读） |
|---|---|
| 固件 | 2a 保存的清单：`a54f11eb6d00`，`revision0=2773422571`、`revision1=27904`、`modified=0` |
| 电池/静态姿态/supervisor | 最近一次观测为 2026-10-28 的 2a，见 `STAGE.md` §2.4。当前是否上电未知，本轮未连接 |
| 机架 | 用户确认塑料支脚固定歪斜、无裂纹、可更换；单个电机轴倾斜尚未量化 |
| 电台 | Crazyradio 2.0 `1915:7777`；最后报告已 detach。busid 随插拔变化，后续须运行时解析，不沿用旧值 |
| 机载时间 | 日志时间戳为 **24 位**；本机计数器实测比宿主快约 **8.2%**（2a 复算 `1.08178`，当轮记录 1.08171）。任何"多少 Hz / 多久"必须说明时间基 |
| Flow Deck | 有掉枚举/冻结历史（F5），必须看实时 flow/range，不能只看 `deck.bc*` 参数 |
| `range.zrange` 单位 | **毫米**（`telemetry.py` 除 1000）。按米处理会把着地机体读成"离地 12 m" |
| 链路历史 | 无电机负载时也复现过丢包；USB 复位与低 unicast 告警没有已证因果。计数器统计伪影及长窗口对照见故障库与 `cf_evidence_runs/20261021-003500-link-diagnosis/LINK_DIAGNOSIS_2026-10-21.md` |

## 5. 安全不变量

| 项 | 当前配置基线 |
|---|---|
| `hardware_flight_enabled` | **false** |
| 地面/空中低压阈值 | `battery_critical_voltage=3.7` / `battery_critical_airborne_voltage=3.3`，连续 3 样本 |
| 飞行 preflight | `MIN_BATTERY_V=4.00` |
| 地面诊断会话闸门 | 两个 lib 的 `BATTERY_FORBID_V=3.50`，仅用于不起桨诊断，不是飞行放行线 |
| 外电互锁 | `pm.state≠0` 或充电电流 >0.01 A 禁飞，电压不能覆盖 |

本轮 arm / takeoff / land / go_to / emergency **均未发送**；未 attach 电台、未启动飞行栈、未写参数或 NVM、未改阈值。后续授权不由离线报告自动获得。

## 6. 仓库与运行注意

- 分支 `codex/v1.3-crazyflie-execution`；最后一次记录的验证基线为 `b154001` 之后的离线改动，不是实时 HEAD。
- **2026-10-29 已清空积压并推送**：此前的产品代码 `3630259`、文档 `f281f59`、归档报告 `9d61814`、文档补充 `b154001` 已在 `origin`。本轮改动**尚未提交**。
- **推送要靠代理**：`~/.gitconfig` 的 `http.proxy` 指 `127.0.0.1:7890`（FlClash），直连 GitHub 时通时断。推送失败先看代理是否在跑，别怀疑凭据。
- 仓库真路径 `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`，`D:\dmp` 为 junction。
- `_dsh_scratch/` 和 `cf_evidence_runs/` 在仓库外，全新克隆不含工具与证据；缺失应如实报错。
- WSL Ubuntu 24.04 / ROS 2 Jazzy；Python、cflib、ruff、mypy 在 `~/crazyswarm-env`。ROS 发现域须为 `LOCALHOST`；Crazyswarm2 先起，再启动 recorder。
- cflib 探针与 Crazyswarm2 不同时占用电台；`get_value()` 返回字符串，比较前归一化。
- `scripts/docs/*.py` 在 Windows GBK 控制台会因 ✅ 等字符报 `UnicodeEncodeError`。这是编码问题不是文档问题，设 `PYTHONUTF8=1` 再跑。
- pytest 使用新的 basetemp；不要触碰历史故障临时目录。
- 本阶段之外的待办：地址策略回退问题、默认关闭的 M2 真机航点路径，均不在本轮扩张。
