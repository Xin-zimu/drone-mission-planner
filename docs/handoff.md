# 新对话交接单（drone-mission-planner / v1.2）

> 用途：新开对话时把这份交给下一个会话，即可直接续做，不必重新侦察。
> 生成时间：2026-09-06 · HEAD `bf1ed44` · 全量 **345 passed**

## 1. 仓库与环境

| 项 | 值 |
|---|---|
| 仓库 | `D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main`（路径含空格） |
| **无空格别名** | **`D:\dmp`**（junction，指向仓库根）——DSH 命令按空格切分，**验证命令一律用 `D:/dmp/...`** |
| Python | `.venv` = Python 3.12.6；依赖 PySide6 6.11.2、pydantic 2.13.5、networkx 3.6.1、ortools 9.15.6755、numpy 2.5.3、pytest 8.4.2、pytest-qt 4.5.0、ruff 0.16.6、mypy 1.20.2 |
| 项目格式版本 | **1.9**（产品版本仍 1.1.0，两者分开） |

### 验证命令（照抄）

```powershell
D:/dmp/.venv/Scripts/python.exe -m pytest D:/dmp/tests --basetemp=D:/dmp/.pytest-tmp-run
D:/dmp/.venv/Scripts/python.exe -m ruff check D:/dmp/src D:/dmp/tests D:/dmp/scripts
D:/dmp/.venv/Scripts/python.exe -m mypy --config-file D:/dmp/pyproject.toml D:/dmp/src D:/dmp/tests
cd D:/dmp; D:/dmp/.venv/Scripts/python.exe scripts/benchmark.py
```

> `--basetemp=.pytest-tmp-run` **必须带**：仓库默认的 `.pytest-tmp` 被高权限进程设置了 ACL，当前用户无法访问。

## 2. 当前状态

- 进度与差距见 **`docs/plan-alignment.md`**；流程优化见 **`docs/workflow-optimization.md`**；逐轮证据见 **`docs/execution-progress.md`**。
- 计划原文：`docs/follow-up-development-plan.md`（§10 调度、§11 全局优化、§22 阶段表）。
- 提交链（新→旧）：`bf1ed44`(F3-a) → `4a2fc4d`(F2-b) → `c315f5b`(F2-a) → `b358289`/`8559ed1`/`ce90cea`/`ea3b03d`(F1) → `9354664`(F0 基线)。
- 测试数演进：234 → 264(F1) → 306(F2-a) → 332(F2-b) → 345(F3-a)。

## 3. 关键架构决策（不要回退）

1. **路线单一来源**：`waypoints` 是唯一权威，`Drone.planned_path` 只读派生、不持久化；写入口只有 `replace_route`/`edit_waypoint`/`remove_waypoint`/`clear_route`。
2. **时间模型唯一**：`planning/scheduling.py` 的 `resolve_start` 是 `start = max(arrival, earliest_start, 前置)` 的唯一实现，评分/仿真/报告都消费它。
3. **硬截止语义**：`deadline_policy` 三态（hard/soft/legacy_soft）；旧项目的 deadline 迁移为 `legacy_soft`，不静默变成硬约束。
4. **能源分项**：`planning/energy_ledger.py` 八项分账；地面等待用 `ground_idle_power`，空中等待/服务用 `hover_power`；储备只计一次。
5. **优化器接口**：`AssignmentProblem`（tick 制）与 `AssignmentSolver` 解耦，结果五态显式；**不支持约束返回 `unsupported_constraint` 而不是忽略**；OR-Tools 可选导入，缺失时基线照常。
6. **甘特只读**：点击只发 `task_selected`，走既有 `select_object`，视图无写入口。

## 4. 未做与下一步

- **F3-b（建议下一轮）**：OPT-04 依赖/能源/返航硬约束 → OPT-05 覆盖块跨区串接 → OPT-06 冲突后复核与有界修复 → OPT-07 UI/取消/解释。
- 之后：F4 地理参考 → F5 GIS/DEM → F6 导出验证（L0～L3）→ F7 发布；F8～F12 为 P1/P2。
- 已知未接入：全局优化**尚未接 UI**；甘特拖拽（首版不做）；跨机依赖在线协调（需阻断或标注）；迁移报告 UI 对话框。

## 5. 已知坑（都踩过）

| 坑 | 现象 | 处理 |
|---|---|---|
| `.pytest-tmp` ACL | `PermissionError: [WinError 5]` | 一律 `--basetemp=.pytest-tmp-run` |
| 路径含空格 | DSH 命令静默失败/切分错误 | 用 `D:/dmp` junction |
| Python 脚本改文件 | `write_text` 在 Windows 把 LF 变 CRLF，产生整文件假 diff（曾虚增 3 倍） | 用字节读写 + `replace(b"\r\n", b"\n")` |
| UI 测试挂死 | MainWindow 在**脏项目**上关闭会弹「是否保存」模态框 | 测试夹具里 `service.dirty = False` |
| 合同 measure 已绿 | 判别力闸拒绝 | 指向增量文件，或在该断言内加 `rationale` 说明是护栏 |
| 组判据指向新文件 | decompose dry-run 记为红 → 组落账被拒 | 组判据只指向已存在的绿文件 |
| `vExpect` 写错 | 新合同档位重置为 `far`，写 `maintain` 非法 | 省略 `vExpect` |

## 6. 记忆节点（engram，可直接 recall）

- `dmp 航点单一来源 F1`、`dmp 调度内核 F2-a`、`dmp F2-b 闭环`
- `dmp 环境与验证命令`、`脚本改写致 CRLF 假 diff`、`DSH 省流流程五条`

## 7. 给下一个会话的建议动作

1. `engram_recall` 查上述节点，再读 `docs/plan-alignment.md` 与 `docs/handoff.md`（本文件）。
2. 直接按 **F3-b** 开单；**合同断言 4～6 条、预测只打系统数字、文档与提交并入同一动作、`vExpect` 省略**。
3. 阶段范围切干净（例如 F3-b 先只做 OPT-04+05），失败回滚成本才可控。
