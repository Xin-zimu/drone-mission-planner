# 计划对齐与差距（v1.2）

> 对齐对象：`docs/follow-up-development-plan.md`（§22.1 阶段表）。
> 数据来源：本仓库提交历史与实跑命令输出；**人日均为按计划估算反推，不是实际工时**。
> 生成时间：2026-09-15 · HEAD = v1.2.0 release prep

## 1. 进度总览

| 计划阶段 | 计划人日 | 状态 | 交付/提交 |
|---|---:|---|---|
| F0 基线、单位、版本与夹具 | 4～6 | ✅ 完成 | `9354664` |
| F1 航点统一（单一数据源 + 格式 1.7） | 8～12 | ✅ 完成 | `ea3b03d`、`ce90cea`、`8559ed1`、`b358289` |
| F2 完整调度（时间窗/ETA/依赖/甘特/能源账本） | 12～18 | ✅ 完成 | `c315f5b`（SCH-01～04）、`4a2fc4d`（SCH-05～08） |
| F3 全局多机优化 | 14～22 | ✅ 完成 | `bf1ed44`（OPT-01～03）、F3-b（OPT-04/05）、本轮 F3-c（OPT-06/07 + OPT-08 证据） |
| F4 地理参考（WGS84/ENU/高度基准/围栏） | 10～16 | ✅ 完成 | 本轮 F4（GEO-01～08） |
| F5 GIS/DEM 导入 | 12～20 | ✅ 完成 | F5-a/F5-b/F5-c（data sources、GIS topology、DEM apply） |
| F6 目标导出验证（QGC/PX4，L0～L3） | 12～18 | ✅ 完成（外部证据受限） | F6（profile、validation report、package manifest） |
| F7 集成与发布 | 8～12 | ✅ 完成 | v1.2.0 release metadata、report、Windows packaging gate |
| **v1.2 主线合计** | **80～124** | **100% 主线代码/文档收口** | 外部地面站/SITL 证据仍作为发布范围限制登记 |
| F8～F12（P1/P2 增强） | 73～125 | ⬜ 未开始 | — |

按阶段粗算：8 个主线阶段中 8 个完成。计划人日是原始估算，不按本仓库实际提交节奏折算。

## 2. 已交付能力的可验证证据

| 能力 | 证据（实跑） |
|---|---|
| 可复现绿色基线 | F6/F7 gate 使用 Windows Python 3.12；ruff/mypy 全绿；全量测试清单按隔离策略覆盖 **424 passed** |
| 航点单一来源 | `Drone.planned_path` 只读派生；`src/` 写点 0 处；格式 1.7 迁移带冲突报告 |
| 完整调度 | §10.4 固定 A/B 案例逐项命中（A 30/30/60/100；B 120/150，硬截止失败、软截止迟到 10 s） |
| 依赖与阻塞 | 缺失/自引用/重复/环路四类均检出，环路打印具体路径 |
| 仿真时间线与事件 | 7 类任务事件；事件顺序 arrived ≤ wait_start ≤ wait_end ≤ service_start ≤ service_finish |
| 能源账本 | 8 项分账；地面等待 0.0222 vs 误用悬停功率 0.5000；储备只计一次 |
| 计划/实际偏差 | 计划 5.0 s vs 实际 10.3 s → +5.3 s；无计划时写未知而非 0 |
| 只读甘特 | 计划/实际双泳道；点击块选中任务；无写入口 |
| 全局优化基础 | 5 种显式状态；有向代价（6 m/s 风：18.2 s vs 63.1 s）；小实例与穷举最优一致 |
| 依赖/能源/返航硬约束 | 独立核验器拒绝并给数字：依赖「T-02 starts at 20.0 s before predecessor T-01 finishes at 25.0 s」；能源「consumes 45.0 energy against usable 20.0」；返航腿单独可耗尽储备；求解器答案必经 `verify_outcome` 复算 |
| 覆盖块与跨区串接 | 单机连续服务两个独立区域（`areas_served_by_one_drone 2`，串接 makespan 157.8 s / 能量 137.64）；方向不匹配的入口/出口组合被 ValueError 拒绝；电池耗尽后复用停止 |
| 冲突后闭环复核 | 有界等待修复后重新核验 deadline、依赖、能源/储备和返航；修复预算耗尽或复核失败时拒绝应用 |
| 全局优化 UI 与降级 | Planning → Assignment planning… 可选贪心基线/全局优化、求解预算和修复轮次；自动分配记录 solver 状态与基线 fallback 原因 |
| 地理参考 | 格式 1.10 增加 `local_only`/`georeferenced`、WGS84/ECEF/ENU、显式高度基准、控制点残差、范围/围栏约束、重设原点保持真实位置、真实 WGS84 导出门槛 |
| GIS/DEM 导入 | 格式 1.11 增加 `data_sources`；GeoJSON/KML 中间模型、multipart/holes/provenance、拓扑校验、事务 apply；GeoTIFF DEM apply 带 NoData mask |
| 目标导出验证 | PX4/QGroundControl profile、L0-L3 validation report、QGC/WPL gate、speed command、route package manifest |
| 集成发布 | 产品版本 `1.2.0`、release report、Windows packaging smoke path、tag-ready changelog |

## 3. 未做清单（按计划章节）

### F3 全局优化
- [x] OPT-04 必做/可选任务、依赖、能源与返航硬约束
- [x] OPT-05 覆盖块与跨区域串接（解除「每机只接收一个区域结果」限制）
- [x] OPT-06 冲突后复核与有界修复（总预算 + 最大修复轮次）
- [x] OPT-07 UI（贪心基线/全局优化选项）、取消、降级与解释
- [x] OPT-08 固定贪心劣解、小实例精确核验和性能对比证据（性能预算仍随全量验收更新）

### 明确未接入的部分
- [ ] 甘特拖拽调整排程（§10.8 明确首版只读）
- [ ] 跨机依赖的在线协调与同步起飞（§14.5 要求阻断或明确标注）
- [ ] 迁移报告的 UI 对话框（仓库/服务层已暴露）

### 后续阶段
- [x] F4 地理参考（GEO-01～08）：WGS84/ENU、高度基准、地理围栏
- [x] F5 GIS/DEM 导入（IMP-01～09）：source metadata、vector topology、DEM apply
- [x] F6 目标导出验证（EXP-01～05）：命令白名单、manifest、自动化 L0-L3 validation report
- [x] F7 集成与发布：版本、Windows 包路径、文档与报告
- [ ] F8～F12：更强避碰、三维搜索、仿真真实性、多架次补能、方案比较与 Monte Carlo

## 4. 与计划的差距说明

1. **范围没有缩水，只是分轮**：F2 被拆成 F2-a（调度内核）与 F2-b（时间线/账本/报告/甘特）两轮完成；F3 同理拆成 F3-a/F3-b。
2. **人日不可直接对比**：计划人日包含实现、测试与文档，本会话是「AI 单轮增量」节奏，不能按人日折算实际投入。
3. **依赖治理已提前兑现**：OR-Tools 从「声明未使用」变为实际使用且保持可选导入（计划 §20.6）。
4. **外部证据范围受限**：v1.2.0 不附带真实地面站导入记录或 SITL 日志，因此 release report 明确禁止把自动化导出校验描述成飞行认证。

## 5. 下一步建议（按计划 §22.3 依赖顺序）

1. 打 `v1.2.0` tag 并通过 CI 生成 Windows artifact。
2. 后续补真实地面站导入记录和 SITL 日志，形成外部验证证据包。
3. 按 P1/P2 顺序推进 F8～F12，避免把增强项混入 v1.2.0 发布口径。
