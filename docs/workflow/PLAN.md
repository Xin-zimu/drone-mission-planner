# PLAN —— 项目总计划（北极星）

> **这份文件很少改。** 改了就是方向性变化。
> 它只回答三件事：项目要到哪去、什么算完成、明确不做什么。
> **阶段怎么走**看 `STAGE.md`；**现在卡在哪**看 `session-log.md`。
>
> 历史版本与详细设计见 `docs/archive/`（含原 `v1.3-crazyflie-execution-plan.md`）。

## 1. 项目要到哪去

Drone Mission Planner（DMP）是一个任务规划器。它的终点是：

**让规划出来的航线，经安全校验后由真实无人机执行，并把真实遥测、实际轨迹和执行结果回传到规划器。**

一句话的闭环：

```
任务 → 规划航线 → 执行编译 → 安全预检 → ROS 2/Crazyswarm2 → Crazyradio → 真实 Crazyflie
                                                                              ↓
                                            遥测 / 实际轨迹 / 执行结果 ←────────┘
```

当前硬件平台：Windows PySide6 Planner + Ubuntu/WSL2 ROS 2 Jazzy + Crazyswarm2 +
Crazyradio 2.0 + 单台 Crazyflie 2.1（cf231，Flow Deck V2 定位）。

## 2. 什么叫"做完了"

以下是**产品级**完成标准，不是某个阶段的标准。任何一条不满足，项目就还没到终点。

1. Planner 能与 Execution Bridge 建立连接，并识别执行端能力、目标机器人与定位能力。
2. 能把 `Drone.waypoints` 编译成独立的 Crazyflie Execution Mission（含坐标变换、
   动作映射、速度/时长约束、安全限制）。
3. Crazyswarm2 `backend:=sim` 能完整执行同一份 mission。
4. 真机的连接、参数、遥测、定位能力可被 Bridge 读取。
5. 真实飞行前必须通过 Preflight Gate。
6. 有可靠 XY 定位后，单台机可执行：takeoff → waypoint sequence → hover/hold →
   return/land，以及 operator abort → controlled land。
7. Planner 实时显示：连接状态、定位状态、battery/status、执行状态、当前 waypoint、
   真实 pose、实际轨迹。
8. 执行完成后生成 Plan-vs-Actual 结果。
9. sim 与 hardware 共用同一套上层 mission/状态机语义。
10. 所有 core 功能有 pytest 覆盖；ROS bridge 有独立的单元/集成测试。
11. 每次真实飞行留下可复核的运行日志。

## 3. 明确不做（防止范围蔓延）

- 多 Crazyflie 同时执行 / swarm formation / 多机实时避碰
- 3D 搜索算法重写、动态障碍视觉感知、在线 SLAM
- 自动云端调度、互联网远程无人值守飞行
- 在 v1.3 主线内扩张到 CF9 以上的航点执行能力（需单独评估）

## 4. 不可动摇的架构事实

这些是踩过坑之后确认的边界，不是偏好：

| 事实 | 含义 |
|---|---|
| 路线单一来源 | 现有航点/路线模型不得为了执行层而分叉 |
| `planning`/`simulation` 不依赖 PySide6 | AGENTS.md 铁律；核心可脱离 UI 测试 |
| 不复用 PX4/QGC ExportProfile | 它不是 Crazyflie 控制接口 |
| Flow Deck 是本地相对坐标 | 无 Lighthouse 时不得宣称全局定位 |
| 仅 Z-Ranger 时禁止自动 XYZ 航点 | 只能测高，不能定 XY，不得解锁 |
| 安全阈值受保护 | 改阈值需要用户明确授权，且必须留依据 |

## 5. 文档体系（阅读顺序）

| 顺序 | 文件 | 回答什么 | 更新时机 |
|---|---|---|---|
| 1 | `docs/workflow/session-protocol.md` | 会话怎么开、怎么收、报告格式 | 极少 |
| 2 | **本文件 `PLAN.md`** | 项目到哪去、什么算完成、不做什么 | 方向变化时 |
| 3 | `docs/workflow/STAGE.md` | 本阶段目标、完成判据、前置条件 | 每阶段一次 |
| 4 | `docs/workflow/cf8-operations.md` | 真机 SOP、故障库、工具索引 | 有新故障/新工具时 |
| 5 | `docs/workflow/session-log.md` | **现在**能不能飞、卡在哪、下一步 | **每轮重写** |

**防漂移铁律**：一个事实只有一个权威位置。其他文档需要它时写"见 X"，**不复制数值**。
被推翻的结论**从当前文档删除**并归档，不是加一句"以上作废"。
