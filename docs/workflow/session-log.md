# 会话日志（当前状态快照）

> **这是一份快照，不是编年史。** 判据与实测证据只在 `STAGE.md` §2，推进顺序只在 §5。
> 历史过程已归档到 `docs/archive/session-history-2026-10-29-before-pid2b-offline.md`。

- 最近重写：2026-10-29，2b 离线日志准备。
- 当前阶段：CF8 单机真实 hover。
- **一句话状态：不可飞。** 电源判据通过；链路 ❌、定位 ❌、姿态 ❌ 尚未通过。2a 静态筛查已完成，不再重复。
- 用户目标：用飞控缓解机架歪斜。当前无法判断可补偿程度，尚未决定调增益、积分或混控。

## 1. 当前阻断与下一步

用户已确认场地没有问题，并明确目前只有软件 `land` 可用。2026-10-27 的 `land` 服务应答后飞机仍继续飞，最终撞椅翻倒，不能把同一路 `land` 当作已验证的失控终止手段。

本地 Crazyswarm2 源码存在 `emergency()` → `cf_.emergencyStop()` → CRTP 发送路径。这里只核验了软件入口，未验证当前固件的电机停转效果，也未验证失链行为。它与普通命令、日志共用无线链路，不能当作独立备份。

**下一次实机步骤先拆桨验证终止通路，并检查新增日志负载；不是直接起飞。** 具体顺序见 `STAGE.md` §5。上电、转电机和后续带载分别按当次范围确认；本轮未获得这些授权。系留不是唯一方式，场地也不能替代结束机制。

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
| 验证边界 | 已通过保存 TOC 的变量/字节核验和离线模拟采集；尚未验证当前机载参数、块分配和无线负载。20 Hz 不足以重建每个控制周期 |
| 旧起飞探针 | `cf85_takeoff_probe.py` 只采旧四块且丢弃机载时间戳；结束观察的新鲜度、完整落盘与正常降落窗口仍需修正。不能删除限位开关后直接用于 2b |
| 原 PID 预设 | `cf_evidence_runs/20261026-preset/preset-live/`，曾机载验证。新增三块不继承该次实机通过结论 |

本轮定向验证：`test_cf85_pid_2b_prepare.py` **13 passed**（包括缺变量/超块长/配置覆盖拒绝、文件隔离、完整 manifest、模拟 ROS 采集与 24 位回绕）；Ruff 与 mypy（两个新文件）通过。三项文档自检通过（consistent / sound / handoff，证据目录 19/19）；结果保存在本轮 `validation-summary.json`，不替代物理验证。

## 4. 硬件与环境事实

| 项 | 最近已知状态（不是本轮实机回读） |
|---|---|
| 固件 | 2a 保存的清单：`a54f11eb6d00`，`revision0=2773422571`、`revision1=27904`、`modified=0` |
| 电池/静态姿态/supervisor | 最近一次观测为 2026-10-28 的 2a，见 `STAGE.md` §2.4。当前是否上电未知，本轮未连接 |
| 机架 | 用户确认塑料支脚固定歪斜、无裂纹、可更换；单个电机轴倾斜尚未量化 |
| 电台 | Crazyradio 2.0 `1915:7777`；最后报告已 detach。busid 随插拔变化，后续须运行时解析，不沿用旧值 |
| 机载时间 | 日志时间戳实际为 **24 位**；历史 onboard/host 比值偏离 1，报告速率和时长必须明确时间基 |
| Flow Deck | 有掉枚举/冻结历史（F5），必须看实时 flow/range，不能只看 `deck.bc*` 参数 |
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

- 分支 `codex/v1.3-crazyflie-execution`；最后一次记录的验证基线仍为 `f3189f1`，不是实时 HEAD。
- **2026-10-29 已清空积压并推送**：工作树此前积压的 12 个改动 + 5 个未跟踪文件分成三个提交（产品代码 `3630259`、文档 `f281f59`、归档报告 `9d61814`），已 push 到 `origin`，工作树干净。
- **推送要靠代理**：`~/.gitconfig` 的 `http.proxy` 指 `127.0.0.1:7890`（FlClash），直连 GitHub 时通时断。仓库内已固化 `http.proxy`/`https.proxy` 指向该代理；推送失败先看代理是否在跑，别怀疑凭据（凭据在 Windows 凭据管理器里）。
- 仓库真路径 `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`，`D:\dmp` 为 junction。
- `_dsh_scratch/` 和 `cf_evidence_runs/` 在仓库外，全新克隆不含工具与证据；缺失应如实报错。
- WSL Ubuntu 24.04 / ROS 2 Jazzy；Python、cflib、ruff、mypy 在 `~/crazyswarm-env`。ROS 发现域须为 `LOCALHOST`；Crazyswarm2 先起，再启动 recorder。
- cflib 探针与 Crazyswarm2 不同时占用电台；`get_value()` 返回字符串，比较前归一化。
- pytest 使用新的 basetemp；不要触碰历史故障临时目录。
- 本阶段之外的待办：地址策略回退问题、默认关闭的 M2 真机航点路径，均不在本轮扩张。
- 本轮验证：`ruff` 全绿；`mypy` 3 个错误全部是既有基线（在 `b3a7842` 的干净 worktree 上逐条复现，非本轮引入）；全量 `pytest` **714 passed**；三项文档自检 consistent / sound / handoff 全绿。
