# 已归档的计划文档（2026-09 时期）

> **只查历史，不作为事实基线。** 新会话不要从这里开工。
> 当前计划体系见 `docs/workflow/PLAN.md`（总）、`STAGE.md`（阶段）、`session-log.md`（现况）。

## 为什么被归档

2026-10-20 整理时发现：这六份计划文档**最晚的一份也停在 2026-09-21**，而项目在此之后
又推进了一个月的真实硬件工作。它们不再描述现实，却仍在 `docs/` 根目录下看起来像权威计划。

更具体的两个问题：

1. **没有任何在用的文档引用它们**。`session-protocol.md`、`cf8-operations.md`、
   `session-log.md` 三份活跃文档对它们零引用——说明工作早已脱离它们进行。
2. **`execution-progress.md` 自己变成了编年史**：以 "Last updated: 2026-09-15" 开头，
   然后一条条往下累加 checkpoint。这正是"追加式记录"会退化的证据。

## 有价值的内容去哪了

归档前做过迁移检查（`_dsh_scratch/cf85_migrate_plans.sh`），确认这些资产已在新文档中：

| 内容 | 现位置 |
|---|---|
| v1.3 十三项必做 / 八项明确不做 | `PLAN.md` §2、§3 |
| 不可动摇的架构事实（路线单一来源、不依赖 PySide6、不复用 PX4/QGC、Flow Deck 边界、Z-Ranger 限制） | `PLAN.md` §4 |
| CF8 阶段的完成判据与已排除路线 | `STAGE.md` §2、§4 |
| 真机 SOP / 故障库 / 工具索引 / 环境坑 | `cf8-operations.md` |

## 文件清单

| 文件 | 行数 | 原用途 | 归档原因 |
|---|---|---|---|
| `v1.3-crazyflie-execution-plan.md` | 2510 | v1.3 详细设计（架构、数据模型、协议、坐标系） | 设计已完成并落地到代码；作为未来重构参考保留 |
| `follow-up-development-plan.md` | 1684 | v1.2 之后的开发计划 | 已被 v1.3 计划取代，而 v1.3 又已实施 |
| `v1.3-crazyflie-progress.md` | 1637 | v1.3 进度追踪 | 追加式编年史，与 session-log 职责重复 |
| `future-roadmap.md` | 862 | 长期路线图 | 目标已并入 `PLAN.md` |
| `execution-progress.md` | 556 | 执行进度 checkpoint | 追加式编年史 |
| `plan-alignment.md` | 78 | 计划对齐说明 | 一次性文档，结论已并入 `PLAN.md` |

**其中 `v1.3-crazyflie-execution-plan.md` 仍有参考价值**：它记录了
Execution Domain 数据模型、坐标系设计、Mission Compiler 规则、Planner↔Bridge 协议。
将来若要重构执行层，值得先读它。但它描述的是**设计意图**，代码才是当前事实。