# 软件研发线工作流（扩展 AGENTS.md 的执行细节）

> 写产品代码（`src/`、`integrations/`、`tests/`）前必读。
> 铁律以仓库根 `AGENTS.md` 的 15 条为准，本文件把它们落成可执行步骤并补充踩过的坑。
> 经验来源：`docs/handoff.md`、`docs/workflow-optimization.md`、`docs/plan-alignment.md`。

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

## 6. 计划文档与进度文档的定位（防"文档爆炸"）

- 大计划文档（`follow-up-development-plan.md`、`v1.2-real-mission-planning-plan.md`、
  `v1.3-crazyflie-execution-plan.md`）只做**阶段索引**，开工时读对应小节，不全文重读。
- 进度事实以 `docs/v1.3-crazyflie-progress.md`（仓库内）+ `docs/workflow/session-log.md` 为准；
  新阶段完成时在 progress 文档追加一节，**不新开平行文档**。
- 流程优化结论在 `docs/workflow-optimization.md`；新结论追加，不新开文件。
