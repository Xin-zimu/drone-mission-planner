# 软件研发线工作流（扩展 AGENTS.md 的执行细节）

> 写产品代码（`src/`、`integrations/`、`tests/`）前必读。
> 铁律以仓库根 `AGENTS.md` 的 15 条为准，本文件把它们落成可执行步骤并补充踩过的坑。
> 经验来源：`docs/handoff.md`、`docs/workflow-optimization.md`、
> `docs/archive/plans-2026-09/plan-alignment.md`（已归档）。

## 1. 标准改动循环

1. **读**：`README.md` + `AGENTS.md` + 相关模块（Codex 规则 1）；涉及真机行为时
   再读 `docs/workflow/cf8-operations.md`（不变量表）。
2. **对齐目标**：一句话说清改什么、为什么、不动什么，用户确认。
3. **改**：一次只实现一个计划阶段（规则 12）；`planning`/`simulation` 不碰 PySide6
   （规则 3）；规划算法不进 UI 类（规则 2）；领域对象用 dataclass/Pydantic（规则 6）；
   公共函数带类型标注（规则 5）；不引入可变全局状态（规则 7）；异常走类型化错误 + 日志（规则 8）。
4. **测**：核心功能必须配 pytest（规则 4）；随机行为接受确定性种子（规则 15）。
5. **静态检查**：**批量化**——一次 `ruff check --fix` + 一次 `mypy` 收全错误再改，
   不"改一个跑一次"；过程中只跑定向测试（`-k` 子集），全量 pytest 收尾跑一次。
6. **文档**：更新 README 和 docs（规则 10）**并入同一改动**，不另开一单。
7. **提交**：见 §3；然后更新 `docs/workflow/session-log.md`（规则 10 的延伸）。

**禁止**：为演示硬编码规划结果（规则 14）；吞异常（规则 8）；跳过迁移直接改项目格式（规则 9）。

## 2. 验证命令（照抄，不要即兴改）

```powershell
# 全量测试（--basetemp 必须带新名；不要再加 -q，addopts 已含）
D:/dmp/.venv/Scripts/python.exe -m pytest D:/dmp/tests --basetemp=D:/dmp/.pytest-tmp-run1
# 定向测试（开发中）
D:/dmp/.venv/Scripts/python.exe -m pytest D:/dmp/tests/<文件> -k <关键词> --basetemp=D:/dmp/.pytest-tmp-dev1
# 静态检查
D:/dmp/.venv/Scripts/python.exe -m ruff check D:/dmp/src D:/dmp/tests D:/dmp/scripts
D:/dmp/.venv/Scripts/python.exe -m mypy --config-file D:/dmp/pyproject.toml D:/dmp/src D:/dmp/tests
```

- 环境：`.venv` = Python 3.12（`D:/dmp/.venv/Scripts/python.exe`）；3.13 只用于打包兼容检查。
- 全量测试规模（历史演进）：424 passed（v1.2.0）→ CF8 子集 205 passed（HEAD）。
- 已知基线：`mypy src` 有 21 个既有错误（不是新引入的就不动）。
- 判据/探针超时（decompose dry-run 约 5 s，全量 pytest 11~14 s）→ 判据用 `-k` 子集或 `--collect-only`。

## 3. 提交与分支纪律

| 规则 | 内容 |
|---|---|
| 分支 | `codex/v1.3-crazyflie-execution`（当前）；新阶段不另开分支（除非用户要求） |
| 提交粒度 | 一个阶段一个提交；提交信息格式：`v1.3 <阶段>: <动名词短语>`（如 `7c1d904 v1.3 CF8.1: capability snapshot schema 2 — robot identity vs radio address`） |
| 文档/规范 | `docs/workflow/`、handoff 类文档单独提交（`docs: ...` 前缀），与产品代码分开 |
| push | **默认不 push**；用户明确要求才 push |
| 回退 | 不 reset / checkout 覆盖工作树；用新提交修正 |
| 未跟踪文件 | 交接/证据类 md 放 `docs/archive/` 或根目录并显式说明；不把 `_dsh_scratch`、证据目录提交进仓库 |

## 4. 环境坑（都踩过；只写一次）

| 坑 | 现象 | 处理 |
|---|---|---|
| `.pytest-tmp` 真坏 | 连 ACL 都读不了，`tmp_path` 测试整批 ERROR | 只用 `--basetemp=<新名>`；`.pytest-tmp` 需管理员修，别碰 |
| basetemp 偶发 `WinError 5` | 同一目录先前能跑后来报错 | 先查当前执行策略（workspace-write 受限令牌拒写），再换新 basetemp 名 |
| 重复 `-q` | 看不到 `N passed` 汇总（addopts 已含） | 命令行不要再加 `-q` |
| 路径含空格 | DSH 命令静默失败/切分错误 | 用 `D:/dmp` junction |
| Python 脚本改文件 | `write_text` 在 Windows 把 LF 变 CRLF，整文件假 diff | 字节读写 + `replace(b"\r\n", b"\n")` |
| UI 测试挂死 | MainWindow 在脏项目上关闭弹"是否保存"模态框 | 测试夹具 `service.dirty = False` |
| Windows 原生扩展偶发崩溃 | 单进程全量 pytest 在 OR-Tools / coverage simulation 边界 access violation | 按独立 pytest 进程分组跑；单跑触发文件通常通过 |

## 5. 已知基线（不要回退的架构决策）

完整清单见 `docs/handoff.md` §3（13 条）。最关键三条：

1. **路线单一来源**：`waypoints` 是唯一权威；`Drone.planned_path` 只读派生、不持久化；
   写入口只有 `replace_route`/`edit_waypoint`/`remove_waypoint`/`clear_route`。
2. **时间模型唯一**：`planning/scheduling.py` 的 `resolve_start` 是
   `start = max(arrival, earliest_start, 前置)` 的唯一实现。
3. **Qt 坐标轴只在 UI 边界转换**：领域层 `y = north`；不要在规划/持久化/导出层交换 x/y。

## 6. 文档更新规则（单一事实源，强制）

**旧做法（已废止）**：大计划文档做阶段索引 + 单独的 progress 文档追加记录。
结果：五个计划文档一个月未更新，`execution-progress.md` 自己变成了编年史。

**现行三层（见 `session-protocol.md` §0）**：

| 层 | 文件 | 只负责 | 更新触发条件 |
|---|---|---|---|
| 战略 | `docs/workflow/PLAN.md` | 项目总体目标、范围、最终验收标准 | **只有**项目方向或总体验收标准变化时 |
| 验证 | `docs/workflow/STAGE.md` | 阶段判据、实测证据、决策记录 | **两种情况分开**：<br>· 获得有效实测 → 立即更新对应判据行（含条件+证据路径）<br>· 阶段目标/范围变化 → 才改阶段定义 |
| 执行 | `docs/workflow/session-log.md` | 当前状态、阻塞点、下一步操作 | **每轮结束重写** |

**单一事实源原则**：

1. 同一事实**只在一个位置维护**。其他文档可以引用它，但**不复制一套需要独立更新的状态**。
   - 判据状态 → 只在 `STAGE.md`
   - 当前阻断 → 只在 `session-log.md`
   - 项目目标 → 只在 `PLAN.md`
   - 故障判据/操作步骤 → 只在 `cf8-operations.md`
2. 引用时写"见 `STAGE.md` §2"，不写"实测 1.081 V"（那会立刻过期）。
3. **实验历史不靠文档保存**：原始数据在 `_dsh_scratch/` 与
   `D:\Deepseek WorkSpace\cf_evidence_runs\<时间戳>-<阶段>\`，
   文档只留**当前结论 + 测试条件 + 证据路径**三条。
4. **结论必须带条件**：写"在 X 条件下 Y 未达预期"，不写"Y 无效"。条件一旦改变必须重测。
5. **被推翻的结论从当前文档删除**，过程进 `docs/archive/`，不是加一句"以上作废"。

**收尾自检（三跑）**：`cf85_doc_consistency.py`（形式）→ `cf85_stage_audit.py`（事实）
→ `cf85_handoff_sim.py`（交接）。三者全绿仅代表文档自洽可交接，**不代表物理结论成立**。

- 详细设计与历史版本已归档到 `docs/archive/plans-2026-09/`，**只查历史，不作为事实基线**。
- 流程优化结论在 `docs/workflow-optimization.md`；新结论追加，不新开文件。
