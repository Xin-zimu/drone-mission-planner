# Drone Mission Planner 后续功能详细计划

Last updated: 2026-09-04

## 1. 当前基线

当前项目已经完成本地桌面端多无人机任务规划与仿真能力，并完成 M6-M8 核心实现。

已具备能力：

- 2D 地图编辑：基地、无人机、任务点、障碍物、禁飞区、搜索区域。
- A* 路径规划：网格化、避障、安全半径膨胀、路径平滑、路径校验。
- 多机任务分配：优先级、载荷、电量、返航、安全余量、地形/风能耗。
- 覆盖规划：多机区域分条、割草式扫描、覆盖率统计、补扫。
- 动态仿真：固定步长、播放/暂停/单步、任务完成、电量消耗。
- 故障恢复：无人机失败后保留现场时间、电量、已完成任务和覆盖历史。
- 通信与冲突：多跳通信状态、通信中断记录、简单冲突让行。
- 报告导出：HTML/JSON/CSV 统计报告，包含环境摘要和高度风险摘要。
- 环境模型：程序化地形、基础海拔、山峰、风向、风速、阵风。
- 2.5D 视图：只读地形视图、地形图例、风场箭头、对象高亮、风险航段。
- M7 高度安全：地形 clearance、障碍物高度、禁飞区高度策略、任务目标高度风险检查。
- M8 增量重规划：故障后只对未完成任务或未覆盖格子重新规划。

重要边界：

- 当前不是完整 3D。已有 2.5D 地形显示和高度风险校验，但路径本体仍是 `Point(x, y)`。
- 当前 A* 仍是 2D 网格搜索，高度是后置校验和能耗参数。
- 当前项目不接真实飞控，不做实时遥测，不做 ROS 2/MAVLink 在线控制。
- 发布方向仍是源码运行、本地可运行包、普通未签名构建；不做签名安装包。

## 2. 原计划

原计划按“真实任务准备能力”推进，顺序是：

```text
M9 真实数据导入
M10 报告/示例/文档打磨
M11 真实地图底图
M12 真实航线导出
M13 三维航点模型
M14 风险评估
M15 回放系统
M16 机型/电池/载荷库
M17 高级覆盖规划
M18 协同避碰与中继优化
M19 可解释规划
M20 产品化基础
```

原计划的优点：

- 工程顺序稳，先解决数据导入、底图、导出这些真实任务准备环节。
- M10 可以较早补齐报告、示例和文档，对展示和交付友好。
- M11 底图校准能让后续导入数据和航线导出更接近真实地图使用场景。

原计划的问题：

- 真 3D 到 M13 才开始，来得太晚。
- M9/M10/M11/M12 如果都建立在 2D `Point` 路径上，后续升级三维航点时会返工。
- QGroundControl/ArduPilot 导出需要高度、速度、动作等字段，放在三维航点之前做不稳。
- 风险评估、回放、高级覆盖等能力都需要更可靠的三维状态，否则只能继续做 2.5D 补丁。

## 3. 新计划

新计划改成“3D 提前成为主线”：

```text
M9-lite 最小高程导入
M13A 三维航点数据模型
M13B 真 3D 视图
M13C 三维航线编辑
M13D 三维仿真与高度动作
M12 真实航线导出
M10 报告/示例/文档打磨
M11 真实地图底图
M9-full 完整真实数据导入
M14 风险评估
M15 回放系统
M16 机型/电池/载荷库
M17 高级覆盖规划
M18 协同避碰与中继优化
M19 可解释规划
M20 产品化基础
```

核心变化：

- M9 不一次做完，先做 `M9-lite`，只解决 3D 地形最需要的高程导入。
- M13 提前，并拆成数据模型、3D 视图、航线编辑、三维仿真四个阶段。
- M12 放到 M13 后面，避免导出格式在三维航点模型稳定前返工。
- M10/M11 后移，但不取消；3D 主线完成后再做示例、截图、底图会更准确。
- M14 之后的风险、回放、覆盖、协同、解释都基于三维航点和三维状态继续扩展。

## 4. 3D 的定义

本项目后续的“真 3D”不是单纯把现有 2D 地图画成立体效果，而是至少包含以下五层。

数据模型 3D：

- 航点有 `x/y/altitude`。
- 高度模式明确区分 MSL 和 AGL。
- 航点可携带速度、动作、悬停时间、任务行为。

规划结果 3D：

- 规划输出不只是一串 `Point`，而是一串 `Waypoint`。
- 点任务、覆盖任务、返航任务都能生成三维航点。
- 旧 2D 路径可自动迁移为默认高度航点。

仿真状态 3D：

- 运行时无人机状态包含真实高度。
- 爬升、下降、悬停、拍照、返航等动作进入状态机。
- 能耗、速度、风险基于三维航段计算。

可视化 3D：

- 3D 地形网格。
- 三维航线折线。
- 障碍物显示成立体体块。
- 禁飞区显示成空域柱体或盒体。
- 风险段在 3D 中用颜色标记。

导出 3D：

- JSON/CSV/QGroundControl/ArduPilot 导出携带高度、速度和动作。
- 导出前验证高度、安全、电量、动作兼容性。

## 5. 阶段总览

| 阶段 | 主题 | 目标 | 状态 |
|---|---|---|---|
| M6 | 2.5D 可用性 | 地形/风视图可辅助判断任务 | 已完成 |
| M7 | 高度安全 | 高度进入安全校验和报告 | 已完成核心实现 |
| M8 | 增量重规划 | 故障后只补剩余任务/未覆盖区域 | 已完成核心实现 |
| M9-lite | 最小高程导入 | 让 3D 地形有外部高程数据来源 | 已完成核心实现 |
| M13A | 三维航点模型 | 建立 `Waypoint` 和兼容迁移 | 已完成核心实现 |
| M13B | 真 3D 视图 | 展示地形、航线、空域和高度 | 已完成核心实现 |
| M13C | 三维航线编辑 | 表格和属性面板编辑高度/速度/动作 | 已完成核心实现 |
| M13D | 三维仿真 | 状态机执行高度变化和航点动作 | 已完成核心实现 |
| M12 | 真实航线导出 | 导出 JSON/CSV/QGC/ArduPilot | 已完成核心实现 |
| M10 | 报告与示例 | 重新生成展示级报告、示例、文档 | 已完成核心实现 |
| M11 | 地图底图 | 本地地图图片、比例尺、坐标校准 | 已完成核心实现 |
| M9-full | 完整真实数据导入 | GeoJSON/KML/航点 CSV 完整导入 | 已完成核心实现 |
| M14 | 风险评估 | 电量/通信/地形/空域风险评分 | 已完成核心实现 |
| M15 | 回放系统 | 时间轴和历史状态复盘 | 已完成核心实现 |
| M16 | 设备库 | 机型、电池、载荷、任务模板 | 已完成核心实现 |
| M17 | 高级覆盖 | 多区域、洞、优先级、方向优化 | 已完成核心实现 |
| M18 | 协同优化 | 时空预约、等待点、通道管制、中继机 | 计划中 |
| M19 | 可解释规划 | 分配解释、失败原因、优化建议 | 已完成核心实现 |
| M20 | 产品化 | 撤销/重做、自动保存、最近项目、设置页 | 计划中 |

## 6. M9-lite：最小高程导入

### 目标

先只做 3D 主线必需的高程数据导入，不完整展开所有 GIS 数据能力。

### 范围

- 支持 CSV 点云式高程：`x,y,elevation`。
- 支持规则网格式高程：宽、高、分辨率、基础原点、二维 elevation 矩阵。
- 坐标仍使用本地米制，不做经纬度投影。
- 导入后生成 `TerrainModel` 或新的 terrain source 结构。
- UI 提供最小入口：选择文件、预览范围、确认导入。

### 不做

- 不做 GeoJSON/KML。
- 不做复杂坐标投影。
- 不做在线地图。
- 不做大规模 DEM 瓦片流式加载。

### 代码任务

1. 新增导入模块 `src/drone_mission_planner/persistence/terrain_import.py`。
2. 解析 CSV 并校验列名、数值、空值、重复点、范围。
3. 输出导入预览对象和 terrain 数据。
4. 保留现有 `flat_terrain`、`generate_mountain_terrain`。
5. 增加从规则网格构造 terrain 的公开函数。
6. 明确插值策略：优先选择简单稳定的双线性。
7. Environment tab 增加导入按钮。
8. 导入前显示行数、边界、海拔范围、分辨率估计。
9. 确认后调用 `ProjectService.update_environment()`。

### 测试

- CSV 正常导入。
- 缺列报错。
- 非数值报错并给出行号。
- 规则网格插值。
- 保存/重新打开后地形一致。

### 验收标准

- 用户能导入一个 `x,y,elevation` CSV 并在 Environment tab 看见高程范围。已完成。
- 2.5D 视图能显示导入后的地形起伏。已完成。
- 路线能耗和高度风险使用导入地形。已完成。
- 错误文件给出明确行号和字段名。已完成。
- `pytest / mypy / ruff` 通过。已完成。

## 7. M13A：三维航点数据模型

### 目标

把内部航线从 `list[Point]` 逐步升级为三维航点序列，为真 3D 视图、三维仿真和真实航线导出打基础。

### 数据设计

新增 `Waypoint`：

```python
@dataclass(slots=True)
class Waypoint:
    x: float
    y: float
    altitude: float
    altitude_mode: AltitudeMode
    speed: float | None = None
    action: WaypointAction = WaypointAction.FLY_TO
    hold_seconds: float = 0.0
    task_id: str | None = None
```

新增枚举：

- `AltitudeMode.MSL`：海拔高度。
- `AltitudeMode.AGL`：相对地形高度。
- `WaypointAction.FLY_TO`
- `WaypointAction.HOVER`
- `WaypointAction.TAKE_PHOTO`
- `WaypointAction.SCAN`
- `WaypointAction.LAND`
- `WaypointAction.RETURN_TO_LAUNCH`

### 兼容策略

- 短期保留 `Drone.planned_path: list[Point]`，新增 `Drone.waypoints: list[Waypoint]`。
- 所有旧项目加载时自动从 `planned_path` 派生默认航点。
- 保存时写入新字段，同时保留旧字段一段时间用于兼容。
- 路径规划短期仍输出 2D path，但 route planner 同时生成 waypoints。

### 代码任务

1. domain：新增 `Waypoint`、`AltitudeMode`、`WaypointAction`。
2. domain：给 `Drone` 增加 `waypoints`。
3. domain：增加 `waypoints_from_path()` 和 `path_from_waypoints()`。
4. persistence：`.dmproj` schema 版本升级。
5. persistence：迁移 v1.2 项目到新版本。
6. planning：`PathResult` 增加 `waypoints`。
7. planning：`RoutePlanner` 基于 terrain/task altitude 生成默认三维航点。
8. planning：`CoveragePlanner` 为覆盖扫描生成 `SCAN` 或 `FLY_TO` 动作。
9. simulation：初步让 engine 读取 waypoints，但可继续用 Point path 推进。
10. UI：Altitude profile 读取 waypoints，属性面板显示航点数量和高度摘要。

### 验收标准

- 旧 `.dmproj` 能打开并迁移出 waypoints。
- 新项目保存后包含三维航点。
- 点任务和覆盖任务规划后都有 waypoints。
- 当前 2D/2.5D 显示不退化。
- 所有现有测试通过，并新增迁移/序列化测试。

## 8. M13B：真 3D 视图

### 目标

新增真正的 3D 任务视图，用于显示地形网格、三维航线、空域体块和无人机高度。

### 技术路线

首选方案：

- 使用 Qt WebEngine 嵌入本地 Three.js 页面。
- Python 侧生成场景 JSON。
- Web 侧负责 3D 渲染、相机控制、拾取和高亮。

备选方案：

- 如果 Qt WebEngine 依赖过重，则先用独立本地 HTML 预览窗口。
- 保留 2D/2.5D PySide6 地图作为主编辑入口。

### 3D 场景对象

- Terrain mesh：来自 TerrainModel 或导入 DEM。
- Route polyline：按 waypoint altitude 绘制三维折线。
- Drone marker：当前 x/y/z 状态。
- Obstacle volume：矩形盒体、圆柱体、简化多边形柱体。
- No-fly volume：禁飞空域盒体或柱体，透明材质。
- Search area：地面多边形或半透明覆盖面。
- Risk segment：红/橙色高亮航段。
- Wind vector：全局风向箭头。

### 交互

- 鼠标旋转、平移、缩放。
- 点击对象同步左侧对象树选中。
- 选中无人机时突出显示其航线。
- 切换显示层：地形、航线、障碍、禁飞、覆盖、风险、标签。
- 视角预设：俯视、斜视、侧视、跟随无人机。

### 代码任务

1. 新增 `src/drone_mission_planner/ui/scene3d_export.py`。
2. 把 MapModel、waypoints、coverage、risks 转为稳定 JSON。
3. 新增 `src/drone_mission_planner/ui/web3d/index.html`。
4. 新增 `src/drone_mission_planner/ui/web3d/scene.js`。
5. 新增 `src/drone_mission_planner/ui/web3d/style.css`。
6. 新增 `ThreeDView` dock 或 central mode。
7. 工具栏增加 `2D / 2.5D / 3D` 视图切换。
8. 3D 下先只读，不允许直接拖对象。

### 视觉质量

- 地形高度颜色与现有 2.5D 保持一致。
- 禁飞区透明但边界清楚。
- 航线、风险段、选中态可一眼区分。
- 标签不遮挡主体。

### 验收标准

- 打开 `mountain_wind_demo.dmproj` 能看到真实 3D 地形和三维航线。
- 障碍物和禁飞区显示成立体体块。
- 高度风险段在 3D 中可见。
- 选中对象能在 2D/2.5D/3D 之间同步。
- 3D 视图加载失败时不影响 2D 主功能。

### 实施记录（M13B，2026-09-05）

- 环境中未安装 QtWebEngine（引入约 200 MB 依赖并影响打包），按备选路线改为零依赖的原生渲染：`ui/scene3d_export.py` 生成确定性场景（纯 Python、无 Qt，可无头测试并导出稳定 JSON），`ui/view3d.py` 用 QPainter + 画家算法软件渲染。
- 已实现：地形网格（与 2.5D 同一高度配色）、三维航线（按航点高度，含风险段红/黄高亮）、障碍/禁飞立体体块、搜索区地面轮廓、覆盖格、无人机标记（随仿真实时位置更新）、风向箭头、轨道相机（旋转/平移/缩放）、七层显示开关、俯视/斜视/侧视/跟随视角、点击拾取与 2D/2.5D/3D 选择同步。
- 3D 为只读；因无外部加载组件，"3D 失败不影响 2D" 由结构本身保证（原生控件，无 WebEngine 加载环节）。
- `mountain_wind_demo.dmproj` 自动分配后，三架无人机均生成 110–250 m MSL 三维航线，低净空航段以黄色风险段显示。

## 9. M13C：三维航线编辑

### 目标

让三维航点不只是内部数据，而能被用户检查和编辑。

### UI 功能

- 新增 Waypoints tab。
- 表格列：index、drone、x、y、altitude、altitude mode、speed、action、hold seconds、task id。
- 选中表格行时地图和 3D 视图高亮对应航点。
- 支持编辑 altitude、altitude mode、speed、action、hold seconds。
- 对自动规划航点和手动编辑航点做 dirty 标记。

### 行为规则

- 修改航点高度后，重新计算能耗和高度风险。
- 修改航点动作后，仿真状态机按新动作执行。
- 删除航点前检查是否破坏起点、任务点、返航点。
- 对覆盖航线，默认不允许随意删除扫描端点，先只允许调整高度/速度。

### 测试

- 表格渲染 waypoints。
- 编辑高度后项目 dirty，Altitude profile 刷新。
- 编辑 speed 后仿真使用该段速度。
- 选择航点同步地图高亮。

### 验收标准

- 用户能在 UI 中查看并修改三维航点。
- 修改结果能保存到 `.dmproj` 并重新打开恢复。
- 修改高度会影响风险和能耗。
- 旧的任务规划、覆盖规划、仿真测试不退化。

### 实施记录（M13C，2026-09-05）

- 新增 `ui/waypoint_panel.py`（Waypoints 标签页）：index/drone/x/y/altitude/mode/speed/action/hold/task 十列；mode、action 用下拉编辑，speed 支持留空回退巡航速度；覆盖扫描航线只允许改高度/速度。
- 编辑经 `ProjectService.update_waypoint`/`remove_waypoint` 落地：类型校验 + `validate_project` 失败回滚 + `planned_path`/`waypoints` 双表示同步 + dirty 标记；删除守卫覆盖起点、任务关联点、返航点和覆盖扫描航线。
- `domain/waypoint.py` 新增 `waypoint_msl_altitude` 作为 MSL/AGL 换算唯一实现；高度校验器在航点数量与路径一致时改用航点 MSL 高度（沿段线性插值），编辑高度即刻改变地形/障碍/禁飞风险；Altitude profile 能耗也改用航点高度作为起止高度。
- 选中表格行时 2D/2.5D 地图与 3D 视图同步显示琥珀色航点高亮环。
- 延后至 M13D：仿真状态机按航点 speed 执行段速度、按 action 执行拍照/悬停等动作（本阶段只完成编辑数据流与展示）。

### 实施记录（M13D，2026-09-05）

- `DroneStatus` 扩展 CLIMBING/DESCENDING/HOVERING/SCANNING/LANDING；`DroneRuntime` 持有与路径对齐的航点剖面（MSL 海拔、速度上限、动作、悬停秒数），路径与航点数量或坐标不一致时自动退回旧 2D 模型。
- 运动模型：水平推进使用航点 speed 作为段速度上限；高度沿航点剖面线性内插，并以 climb_rate/descent_rate 限速逼近；能耗仍由 estimate_segment_energy 计算，3D 航段的飞行时间取实际限速时间。
- 动作执行：HOVER 停留 hold_seconds（计悬停能耗与等待时间）；TAKE_PHOTO 记录 WAYPOINT_PHOTO 事件并短暂停留 2 s；LAND 在终点降落至地形高度并记录 WAYPOINT_LANDING 事件；RETURN_TO_LAUNCH 沿用返航航段。SCAN 开始/结束不单独记事件，以 SCANNING 状态呈现（避免事件表刷屏，偏差已在文档标注）。
- 覆盖门控：只有当前航段两端点之一为 SCAN 的运行时才向 CoverageMonitor 提供位置（验收标准"coverage 只在 SCAN 或可覆盖动作中更新"）；rescue 示例已重新生成为 1.4 schema 并携带 SCAN 航点。
- 报告：每机新增 photos_taken、max_altitude、min_clearance（HTML/CSV/JSON 同步）。

## 10. M13D：三维仿真与高度动作

### 目标

让仿真真正执行三维航点，而不是只在 2D 路径上附带高度估算。

### 状态机扩展

- `CLIMBING`
- `DESCENDING`
- `HOVERING`
- `SCANNING`
- `LANDING`
- `RETURNING`

### 运动模型

- 每个 step 同时推进水平距离和高度。
- 爬升/下降受 `climb_rate`、`descent_rate` 限制。
- waypoint speed 覆盖默认巡航速度。
- MSL：目标高度为绝对海拔。
- AGL：目标高度为 `terrain_altitude + altitude`。

### 动作模型

- `FLY_TO`：普通航段。
- `HOVER`：停留 `hold_seconds`。
- `TAKE_PHOTO`：短暂停留和事件记录。
- `SCAN`：覆盖任务扫描动作，影响 coverage 更新。
- `LAND`：结束或返航降落。
- `RETURN_TO_LAUNCH`：标记返航段。

### 报告和事件

- 报告每架无人机最大高度、最小 clearance、总爬升、总下降。
- 事件表记录拍照、扫描开始/结束、降落。
- 回放系统未来可直接消费三维状态。

### 验收标准

- 仿真中无人机高度随 waypoint 改变。
- AGL 航点在不同地形上能转成正确 MSL 高度。
- 爬升/下降速率限制生效。
- 悬停/拍照动作增加等待时间和能耗。
- coverage 只在 `SCAN` 或可覆盖动作中更新。

## 11. M12：真实航线导出

### 目标

在三维航点模型稳定后，导出真实任务软件可检查的任务文件。

### 导出格式

1. 内部 JSON
   - 完整保留 drone、waypoints、altitude mode、speed、action、risk summary。

2. 通用 CSV
   - 便于 Excel 检查。
   - 一行一个航点或动作。

3. QGroundControl `.plan`
   - 基础 Mission items。
   - 支持起飞、航点、悬停、拍照、返航。
   - 坐标仍是本地任务坐标，若没有真实经纬度校准，需要明确标记不可直接飞行。

4. ArduPilot Mission Planner WPL
   - 输出常见 MAV_CMD。
   - 不支持的动作给出降级或错误。

### 导出前校验

- 高度风险是否存在。
- 电量是否不足。
- 是否缺少 home base。
- 是否缺少真实坐标校准。
- 动作是否被目标格式支持。

### 验收标准

- 一个简单巡检任务能导出 JSON/CSV。
- 有坐标校准后能导出 QGroundControl `.plan`。
- 没有坐标校准时 UI 明确说明“只能导出本地坐标检查文件，不能直接飞”。
- 导出前风险提示可定位到具体 drone/waypoint。

### 实施记录（M12，2026-09-05）

- 新增纯 Python 模块 `persistence/route_export.py`：`build_route_payload` 统一做导出前校验（无航点/无 home base/critical 高度风险/电量不足即抛 `RouteExportError`，警告级风险写入 payload），四种导出共享同一数据源。
- JSON 保留全部航点字段（含 altitude_msl 换算与风险摘要）；CSV 一行一航点；QGC `.plan` 输出 MAVLink 命令序列（22 起飞 / 16 航点 / 19 悬停 / 2000 拍照 / 21 降落 / 20 返航），WPL 输出 `QGC WPL 110` 文本。
- 坐标仍为本地米制：两种文件均写入 `coordinate_reference: local_metric_ungeoreferenced`、`flyable: false` 与"不可直接飞"注释，满足验收标准的明确提示要求。
- UI 入口 File → Export route…（`Ctrl+Shift+E`），按扩展名分发格式，校验失败弹窗定位到具体原因。

## 12. M10：报告、示例和文档打磨

### 目标

把 M6-M8 和 M13/M12 的结果整理成可展示、可复现、可交付的材料。

### 任务

- 重新生成救援示例报告。
- 新增 3D 地形巡检示例。
- 新增高度风险避障示例。
- 新增三维航点编辑示例。
- README 增加 3D 视图依赖说明。
- 更新用户手册 Word/PDF。
- 更新算法文档，解释 2D A*、三维航点和 3D 可视化之间的边界。
- 生成截图或短视频。

### 验收标准

- 新用户按 README 能运行项目并打开 3D 示例。
- 示例项目都能打开、规划、仿真、导出报告。
- 文档明确说明哪些输出可直接飞、哪些只是本地仿真。

### 实施记录（M10，2026-09-05）

- 新增三个示例并纳入打开/仿真集成测试：`3d_inspection_demo.dmproj`（双机山地巡检、目标高度、拍照/悬停动作、三维仿真）、`altitude_risk_demo.dmproj`（低巡航撞高障碍，演示 critical 风险与导出拒绝）、`waypoint_edit_demo.dmproj`（AGL/MSL、逐点速度、拍照/悬停/降落动作展示）。
- `docs/algorithms.md` 顶部补充 2D A* / 三维航点 / 3D 可视化三层的边界说明；README 示例表同步。
- 偏差说明：演示截图、短视频与 Word/PDF 手册重制需要真实 GUI 会话，作为 M10 收尾遗留（其余验收项已完成）。

## 13. M11：真实地图底图

### 目标

把任务规划叠加到用户提供的本地地图图片上，并建立本地坐标到真实地图坐标的基础校准。

### 任务

- 支持 PNG/JPG 底图导入。
- 设置透明度、锁定、隐藏。
- 两点比例尺校准。
- 坐标原点设置。
- x/y 轴方向设置。
- 地图旋转。
- `.dmproj` 保存底图路径、比例、原点、旋转。

### 与 3D 的关系

- 3D 视图可以把底图作为地形纹理。
- QGroundControl/ArduPilot 导出需要地图坐标校准，否则只能导出本地坐标文件。

### 验收标准

- 导入地图图片后，任务对象能对齐显示。
- 两点校准后的距离测量接近用户输入。
- 重新打开项目后底图和校准参数保持一致。

### 实施记录（M11，2026-09-05）

- 新增 `domain/basemap.py`：`BasemapModel`（文件、透明度、可见/锁定、米/像素、原点、y 翻转、旋转）与 `derive_calibration` 两点校准（由两组 world/pixel 对应点反解米每像素、旋转角与原点，支持 y 翻转）；`pixel_to_world`/`world_to_pixel` 互逆。
- `.dmproj` schema 1.5：`map.basemap` 字段 + 1.4→1.5 迁移（老项目 basemap 为 None）；存取完整恢复校准参数（round-trip 测试覆盖）。
- 2D 地图在网格之下渲染底图（透明度、锁定、z 顺序），图像缺失时静默跳过且不影响任务对象。
- UI：Map → Import basemap… 与 Basemap settings…（显示/校准数值 + 两点校准分组）。
- 偏差说明：3D 视图把底图作为地形纹理未实现（记录为后续增强）；校准仍为本地米制坐标，导出保持 not-flyable 标记。

## 14. M9-full：完整真实数据导入

### 目标

在 3D 主线稳定后，补齐外部数据导入能力。

### 任务

- GeoJSON Polygon 导入为搜索区域或禁飞区。
- GeoJSON Point 导入为任务点或基地。
- GeoJSON LineString 导入为参考路线或航点序列。
- KML Point/Polygon 基础导入。
- 航点 CSV 导入为无人机预设 waypoints。
- 导入预览：对象数量、边界、越界、重复 ID、空多边形。
- 导入确认后写入项目并支持撤销。

### 验收标准

- 能导入 GeoJSON 多边形作为搜索区域。
- 能导入 CSV 航点为三维航线。
- 导入错误给出明确行号或对象 ID。
- 保存后 `.dmproj` 能完整恢复导入数据。

### 实施记录（M9-full，2026-09-05）

- 新增纯 Python 模块 `persistence/mission_import.py`：GeoJSON FeatureCollection（Polygon→搜索区/禁飞区、Point→任务/基地、LineString→航点序列）、基础 KML（Point/Polygon Placemark）、航点 CSV（x,y,altitude + 可选 mode/speed/action/hold/task 列，错误带行号）。
- 全部先产出 `MissionImportPreview`（对象计数、越界、重名、空多边形、缺几何警告），用户确认后 `apply_import` 经 ProjectService 写入（多边形回填 points、几何航线按巡航/净空回填高度），复用既有 ID 生成与 dirty 标记。
- `ProjectService.replace_waypoints` 支持整表替换（校验失败回滚），与 M13C 编辑守卫并存。
- UI 入口 File → Import mission data…；坐标仍为本地米制（与 .dmproj 一致，不做投影）。

## 15. M14：风险评估

### 目标

从“能规划”升级为“知道哪里危险、为什么危险”。

### 风险类型

- 电量风险：低电量、返航余量不足、逆风高耗能。
- 通信风险：可能失联航段、中继依赖、auto-return 风险。
- 地形风险：clearance 不足、大爬升、大下降。
- 空域风险：禁飞、障碍物、过低穿越空域。
- 动作风险：悬停过久、任务动作不支持导出。

### 输出

- 每条航线风险分。
- 每个 waypoint/segment 的风险原因。
- 风险矩阵报告。
- UI 风险筛选和定位。

### 验收标准

- UI 能显示每条航线风险分。
- 选中风险段能看到具体原因。
- 报告导出包含风险矩阵。
- 导出前风险校验复用同一套风险模型。

### 实施记录（M14，2026-09-05）

- 新增纯 Python 模块 `planning/risk_assessment.py`：电量/通信/地形/空域/动作五类因素，warning 计 8 分、critical 计 25 分，评分 0-100；任一 critical 因素直接定级 critical。
- 地形/空域因素复用高度校验器结果（含 segment/object 定位）；电量按能耗 vs 剩余电量 + 15% 容量储备阈值；通信按航线最远点与基站/无人机有效电台距离比较（超 110% 记 critical）；动作检查悬停总时长 >60 s 与航线是否以返航/降落收尾。
- `build_risk_matrix` 输出每机矩阵行；仿真报告 JSON/CSV/HTML 附风险矩阵；Altitude profile 表新增 Route risk 列（分值+等级+悬停提示）。
- M12 导出校验改为复用同一评估模型：地形/空域 critical 或电量 critical 时拒绝导出，payload 携带 risk_score/risk_level/risk_factors。

## 16. M15：任务回放系统

### 目标

支持仿真结束后复盘任务过程，而不只是看最终状态。

### 任务

- 每隔固定时间记录三维无人机状态。
- 记录任务状态、覆盖率、事件、通信状态、风险状态。
- 时间轴 UI。
- 支持播放、暂停、拖动、跳到事件。
- 支持导出 replay JSON。
- 报告中嵌入关键帧。

### 验收标准

- 仿真结束后可拖动时间轴查看任意时刻。
- 点击故障事件能跳到故障发生时间。
- 回放不改变真实仿真结果。
- 3D 视图能显示历史无人机位置。

### 实施记录（M15，2026-09-05）

- 新增 `simulation/replay.py`：`ReplayRecorder` 以固定仿真时间间隔（默认 0.5 s）在 `_step` 内捕获三维快照（各机 x/y/z/状态/电量、各区域覆盖率、已处理事件数），`export_replay` 输出 JSON；`frame_index_for_time` 支持事件跳转。记录只读引擎状态，回放不可能改变仿真结果（有专项测试验证）。
- 引擎 `__init__`/`reset` 挂载与清空记录器。
- UI：Workspace 新增 Replay 页（时间轴滑条 + 时间/状态/覆盖率标签 + Exit replay）；拖动时 3D 视图以琥珀色圆环显示历史无人机位置（标签带 replay 标记），回放期间暂停实时位置推送；Events 表双击某行跳到该事件时间的帧。File → Export replay… 导出 JSON。
- 偏差说明：未实现"报告中嵌入关键帧"（截图需真实 GUI），将在 M10 一并处理。

## 17. M16：机型、电池和载荷库

### 目标

减少手工输入，用真实参数模板提升仿真可信度。

### 任务

- 无人机型号库：速度、载荷、通信距离、能耗参数。
- 电池库：容量、电压、可用能量、安全余量、老化系数。
- 载荷库：相机、投放箱、传感器、重量、功耗。
- 任务模板：巡检、搜救、配送、测绘覆盖。
- UI 管理和项目保存。

### 验收标准

- 新建无人机可以从型号库带出默认参数。
- 更换电池型号后能耗余量重新计算。
- 载荷影响任务可行性和能耗。
- 模板数据能随项目或全局配置保存。

### 实施记录（M16，2026-09-05）

- 新增 `domain/equipment.py`：DroneModel（全飞行/能耗参数）、BatteryPack（容量/电压/可用比/安全余量/老化，`effective_capacity()` 折算可用能量）、PayloadType（重量/功耗）、MissionTemplate（预设 planning_settings）；`EquipmentLibrary.default()` 内置 3 机型/3 电池/3 载荷/4 任务模板。
- schema 1.6：`ProjectModel.equipment` 随项目保存，1.5→1.6 迁移；`persistence/equipment_codec.py` 负责编解码，损坏数据回退默认库。
- ProjectService：`create_drone_from_model`（整表带出参数）、`set_drone_battery`（重算可用电量预算）、`attach_payload`（增加当前载荷重量）、`apply_mission_template`。
- 载荷联动：分配与解释的能耗按 `required_payload + current_payload` 计算，载荷可真实打破可行性；电池更换后剩余能量按有效容量重算。
- UI：Planning → Equipment library…（加机型无人机/换电池/挂载荷/套模板）。
- 偏差说明：全局（跨项目）配置存储未做，库随项目保存。

## 18. M17：高级覆盖规划

### 目标

让覆盖规划适应更复杂的真实区域。

### 任务

- 搜索区域支持内部洞。
- 多搜索区域同时规划。
- 每个区域支持目标覆盖率、优先级、扫描方向。
- 高优先级区域优先覆盖。
- 电量不足时保关键区域。
- 补扫路径长度优化。
- 可按风向调整扫描方向。

### 验收标准

- 带洞多边形不会把洞内区域计入目标覆盖。
- 多区域能同时生成覆盖路线。
- 故障后补扫路径明显短于完整重扫。
- 风向调整能减少逆风长航段。

### 实施记录（M17，2026-09-05）

- SearchArea 新增 `holes`（内部洞多边形）、`priority`、`scan_direction`（horizontal/vertical），随 schema 持久化（带默认值，老项目无需迁移步骤）。
- 覆盖目标格与扫描线均扣除洞：`target_cells_for_area` 排除洞内格；`scanline_intervals_with_holes` 在扫描线上做区间减法，条带端点不进入洞。
- `scan_direction="vertical"`：列间距沿 x、扫描沿 y 的转置式 lawnmower（含 `_free_segments_vertical` 采样）。
- `CoveragePlanner.plan_all_areas`：按 (-priority, id) 排序多区域规划，无人机跨区域不复用，电量/可用性不足的区域给出 "No drones available" 失败原因（保关键区域）；UI Planning → Plan all coverage areas。
- 补扫优化：增量扫描行按 scan_spacing 合并（此前按网格分辨率逐行，扫描长度约为所需的 2.5 倍），传感器足迹仍覆盖被合并行。
- 偏差说明：按风向自动选择扫描方向未实现，先提供手动 scan_direction。

## 19. M18：协同避碰与中继优化

### 目标

从运行时简单让行升级为规划阶段协同。

### 任务

- 时空预约表。
- 规划阶段避开已有预约。
- 自动插入等待点。
- 窄通道管制。
- 支持专门通信中继无人机。
- 中继机保持指定位置或跟随队形。
- 冲突解释进入报告。

### 验收标准

- 两条交叉路线在规划阶段错峰。
- 中继机能让远端任务保持多跳通信。
- 报告列出冲突、等待点和让行动作。

## 20. M19：可解释规划

### 目标

让用户知道系统为什么这样规划，而不是只看到结果。

### 任务

- 每个任务显示候选无人机评分。
- 展示距离、能耗、风险、载荷、电量因素。
- 对未选无人机给出拒绝原因。
- 对失败任务生成可执行建议。
- 成本权重 UI。
- 报告导出分配依据。

### 验收标准

- 任务分配表能展开查看候选评分。
- 失败任务建议能对应真实约束。
- 修改权重后方案变化可解释。

### 实施记录（M19，2026-09-05）

- `AssignmentWeights`（energy/distance/battery_risk/task_load/deadline）默认值复现原排序；`GreedyAssignmentPlanner` 接受自定义权重，成本由权重合成。默认行为不变，全部既有测试通过。
- `explain_assignments` 对每个待分配 (task, drone) 对按同一可行性规则评分（不落盘），输出能量/距离/电量风险/负载/截止分量与拒绝原因；`build_assignment_suggestions` 把拒绝原因转成可执行建议（减载/换电/缩短航线/检查禁飞/等待可用）。
- UI：Assignments 表行 tooltip 展示前三候选评分与拒绝原因；Planning → Assignment weights… 对话框把权重写入 planning_settings（标 dirty），重新自动分配即生效。
- 报告：`SimulationReport.assignment_notes` 附分配依据与失败建议，HTML/JSON 导出包含。

## 21. M20：产品化基础

### 目标

提升日常使用体验和稳定性。

### 任务

- 撤销/重做。
- 自动保存。
- 最近项目。
- 设置页。
- 数据校验中心。
- 本地 PyInstaller 构建继续维护。
- 崩溃恢复提示。

### 验收标准

- 常见编辑操作可撤销/重做。
- 异常退出后可恢复草稿。
- 用户能从最近项目快速打开任务。
- 本地构建产物能在同机运行。

## 22. 推荐实施顺序

近期建议：

1. M9-lite：最小高程导入。
2. M13A：三维航点数据模型。
3. M13B：真 3D 视图。
4. M13C：三维航线编辑。
5. M13D：三维仿真与高度动作。
6. M12：真实航线导出。

中期建议：

1. M10：报告、示例、文档打磨。
2. M11：真实地图底图。
3. M9-full：完整真实数据导入。
4. M14：风险评估。
5. M15：回放系统。

后期建议：

1. M16：机型、电池、载荷库。
2. M17：高级覆盖规划。
3. M18：协同避碰与中继优化。
4. M19：可解释规划。
5. M20：产品化基础。

## 23. 下一步可直接开工清单

### Task A：M9-lite CSV 高程导入

- 新增 terrain import parser。已完成。
- 支持 `x,y,elevation`。已完成。
- 生成导入预览。已完成。
- 支持导入后替换 TerrainModel。已完成。
- 单元测试覆盖正常/缺列/非数值/重复点。已完成。

### Task B：Waypoint 数据结构设计

- 新增 `Waypoint`、`AltitudeMode`、`WaypointAction`。
- 新增 path/waypoint 互转函数。
- 给 `PathResult` 和 `Drone` 增加 waypoints。
- 写迁移测试。

### Task C：3D 场景 JSON

- 不急着做 UI。
- 先从 MapModel 导出 terrain mesh、route polyline、object volume。
- 对输出 JSON 做 snapshot-like 单元测试。

### Task D：3D 技术验证

- 验证 Qt WebEngine 是否可用。
- 如果可用，做内嵌 Three.js 原型。
- 如果不可用，做本地 HTML fallback。
- 目标只验证地形网格、航线和相机控制。

## 24. 风险和取舍

### 最大风险

三维航点会触碰 domain、persistence、planning、simulation、UI、reporting，影响面比 M8 更大。必须先做兼容层，不能一次性删除 `planned_path`。

### 控制策略

- 先新增 `waypoints`，保留 `planned_path`。
- 所有旧测试先保持通过。
- 每阶段只迁移一个消费方。
- 数据迁移必须有测试。
- 3D 视图先只读，避免同时做编辑和渲染。
- QGC/ArduPilot 导出必须等三维航点稳定后再做。

### 明确暂不做

- 不做真实飞控连接。
- 不做在线遥测。
- 不做在线地图服务。
- 不做完整 GIS 投影系统。
- 不做完整 3D 路径搜索，除非 M13/M14 后明确需要。
- 不用 3D 取代 2D 主编辑器，短期是并存。

## 25. 完成判断

新路线第一阶段完成的判断不是“3D 看起来炫”，而是：

- 外部高程能导入。
- 航线数据结构真的有三维航点。
- 仿真状态真的有高度动作。
- 3D 视图能从同一份模型渲染。
- 导出能使用同一份三维航点。
- 旧项目和旧测试不被破坏。

达到这些条件后，项目才从 2.5D 演示稳定升级为真 3D 任务准备平台。
