# Drone Mission Planner × Crazyflie 真机联调交接记录

> 日期：2026-09-21  
> 目的：把当前 CF8.1 真机联调状态完整交给 DeepSeek Harness，避免重复排查、误改安全逻辑或误触发真机飞行。

---

## 1. 最终系统架构

用户的目标不是把主软件搬进 WSL，而是：

```text
Windows
┌──────────────────────────────────────┐
│ Drone Mission Planner 主程序 / GUI   │
│ 项目代码位于 D:                      │
│ 通过 TCP 连接 127.0.0.1:8765        │
└──────────────────┬───────────────────┘
                   │ TCP / NDJSON
                   ▼
WSL2 Ubuntu
┌──────────────────────────────────────┐
│ DMP Crazyflie Bridge                 │
│ ROS2 Jazzy                           │
│ Crazyswarm2                          │
│ Crazyradio 2.0                       │
└──────────────────┬───────────────────┘
                   │ radio
                   ▼
              Crazyflie cf231
```

Windows → WSL 的 `127.0.0.1:8765` 已经验证可用。

---

## 2. 项目 / 环境 / 真机信息

### Windows 项目路径

```text
D:\Deepseek WorkSpace\drone-mission-planner-main\drone-mission-planner-main
```

WSL 对应：

```text
/mnt/d/Deepseek WorkSpace/drone-mission-planner-main/drone-mission-planner-main
```

### Git 状态

当前分支：

```text
codex/v1.3-crazyflie-execution
```

基线 HEAD：

```text
a195c51 v1.3 CF8.1: harden hover settle acceptance
```

当前已有本地未提交修改，**不要 reset / checkout 覆盖**。已知至少包括：

1. `integrations/crazyflie_ros2/src/dmp_crazyflie_bridge/config/bridge.yaml`
2. `integrations/crazyflie_ros2/src/dmp_crazyflie_bridge/dmp_crazyflie_bridge/telemetry.py`

另外 capability snapshot 已写入：

```text
integrations/crazyflie_ros2/src/dmp_crazyflie_bridge/runtime/cf231_capabilities.json
```

开始执行前必须先：

```bash
git status --short
git diff
```

确认精确工作区状态，不要假定只有上述文件。

### WSL / ROS 环境

```text
ROS_DISTRO=jazzy
ROS2 workspace: ~/ros2_ws
Python venv: ~/crazyswarm-env
```

`cflib` 在 `~/crazyswarm-env` 中；未激活 venv 时可能 `import cflib` 失败。

### 真机

```text
robot_id: cf231
URI: radio://0/80/2M/E7E7E7E705
Crazyradio: Crazyradio 2.0
本次 Windows usbipd BUSID: 1-3
```

BUSID 可能在重插后变化，因此如需重新 attach，先：

```powershell
usbipd list
```

再：

```powershell
usbipd attach --wsl --busid 1-3
```

WSL 中已验证：

```bash
lsusb
```

能看到 Bitcraze Crazyradio。

---

## 3. Crazyswarm2 当前配置

修改过：

```text
~/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
```

关键 logging 配置现在应为：

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10
      status:
        frequency: 1
      odom:
        frequency: 10
    custom_topics:
      dmp_readiness:
        frequency: 10
        vars:
          - range.zrange
          - kalman.varX
          - kalman.varY
          - kalman.varZ
```

Crazyswarm2 启动时已确认打印：

```text
Logging to dmp_readiness at 10 Hz
Logging to /odom at 10 Hz
Logging to /pose at 10 Hz
Logging to /status at 1 Hz
```

ROS topic 已确认：

```text
/cf231/dmp_readiness
/cf231/odom
/cf231/pose
/cf231/status
```

### Crazyswarm2 启动命令

```bash
cd ~/ros2_ws
source ~/crazyswarm-env/bin/activate
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch crazyflie launch.py gui:=False mocap:=False
```

GUI 不要启用；之前有 NiceGUI 兼容问题。

---

## 4. 已验证的链路

### 4.1 cflib 直接连接

非飞行连接已验证：

```text
CONNECTED OK
```

### 4.2 Crazyswarm2 真机连接

已正常读取参数、Log TOC、pose/status/odom/custom log。

曾有启动瞬间：

```text
High latency
Unexpected number of unicast packets
```

但稳定后实际遥测大致：

```text
latency_unicast: 4~5 ms
rssi: ~37~39
rx/tx 基本匹配
```

### 4.3 Windows → WSL Bridge

WSL 曾确认：

```text
127.0.0.1:8765 LISTEN
python pid=5188
```

PID 只是当时值，继续执行时要重新检查。

Windows：

```powershell
Test-NetConnection 127.0.0.1 -Port 8765
```

结果：

```text
TcpTestSucceeded : True
```

协议 hello：

```json
{"type":"hello","request_id":"win-test-1"}
```

返回：

```json
{"backend":"hardware","bridge_version":"0.1.0-cf3","protocol_version":"dmp-cf/1","request_id":"win-test-1","type":"hello_ack"}
```

因此 Windows 软件 → WSL Bridge 的设计已经实际打通。

---

## 5. Capability Probe 已完成

由于 Crazyswarm2 ROS 参数服务列出了 deck 参数但没有返回值，DMP 使用 cflib capability snapshot fallback。

执行 capability probe 前已停止 Crazyswarm2，避免 cflib 与 C++ backend 抢 Crazyradio。

结果：

```text
Flow2: 1
ZRanger2: 1
Lighthouse4: 0
Loco: 0
DWM1000: 0
```

snapshot：

```text
.../src/dmp_crazyflie_bridge/runtime/cf231_capabilities.json
```

重新启动 Crazyswarm2 后，Windows `get_capabilities` 已确认：

```text
connected: true
positioning_mode: flow
xy_positioning_available: true
z_positioning_available: true
capability_source: cflib_firmware_parameter_probe
pose_rate_hz: ~9.75
```

因此 **Flow Deck V2 + Z-Ranger V2 能力识别链路已经打通**。

---

## 6. 下视测距实测与 bridge.yaml 本地修改

`/cf231/dmp_readiness` 的 values 顺序：

```text
range.zrange
kalman.varX
kalman.varY
kalman.varZ
```

其中 `range.zrange` 原始单位是 mm，Bridge 内部转成 m。

### 实测

无人机平放在有纹理地面、后续稳定采样：

```text
10 mm
8 mm
9 mm
9 mm
8 mm
```

另有一次更早的贴地读数：

```text
4 mm
```

手持到底部无遮挡、离地约 13.7 cm：

```text
137 mm
```

有遮挡/错误支撑情况下曾读到：

```text
0 mm
```

### 当前本地修改

原：

```yaml
min_startup_zrange_m: 0.02
```

当前本地改为：

```yaml
min_startup_zrange_m: 0.005
```

**注意：5 mm 目前只是实机联调暂定值，不是充分验证后的通用默认值。**

因为历史上出现过一次 4 mm 真机贴地读数，DeepSeek Harness 应继续采样并决定：
- 是否作为本地硬件配置；
- 是否调整 validity 判定；
- 是否值得修改仓库默认值。

不要为了“让预检通过”继续盲目降低阈值。

---

## 7. 当前主要真实问题：Flow Deck 静止 XY 漂移

Flow Deck V2 和 Z-Ranger V2 已识别，但当前静止定位仍有明显水平漂移。

曾经一次 Kalman reset 后表现很好：

```text
初始：
x=-0.0127
y=-0.00027

约 16 秒后：
x=-0.0237
y=-0.0020
```

但后续重新启动 / 当前环境下，最新 DMP telemetry：

```text
duration ≈ 20.46 s
sample_count = 200
stable = false
xy_displacement ≈ 0.129 m
xy_span ≈ 0.149 m
xy_std ≈ 0.040 m
z_span ≈ 0.00486 m
```

最新静止 odom：

```text
vx ≈ 0.052 m/s
vy ≈ 0.020 m/s
vz ≈ 0.000 m/s
speed ≈ 0.0557 m/s
```

DMP policy：

```text
max_abs_velocity_mps = 0.05
max_speed_mps = 0.08
```

所以虽然总速度 0.0557 < 0.08，但 `vx=0.052 > 0.05`，仍会触发 `velocity_not_settled`。

Pose stability 默认策略大致为：

```text
minimum_sample_count: 20
minimum_observation_duration_s: 5
max_static_xy_displacement_m: 0.03
max_static_z_displacement_m: 0.03
max_sample_gap_s: 0.5
```

当前 XY span 约 0.149 m，明显过大。

### 当前重点

**不要通过放宽 pose / velocity 阈值来“做绿”。**

应优先排查真实 Flow Deck 条件：

- 地面必须有清晰纹理；
- 避免纯白、玻璃、强反光、过暗、无纹理；
- 光照稳定；
- Flow Deck 底部无遮挡；
- 飞机真正静止；
- 必要时临时增加光流质量 / motion 原始诊断；
- 重置 Kalman 后观察完整稳定窗口；
- 如环境/传感器问题导致持续单向漂移，禁止起飞。

Kalman reset 已验证可用：

```bash
ros2 param set /crazyflie_server cf231.params.kalman.initialX 0.0
ros2 param set /crazyflie_server cf231.params.kalman.initialY 0.0
ros2 param set /crazyflie_server cf231.params.kalman.initialZ 0.0
ros2 param set /crazyflie_server cf231.params.kalman.initialYaw 0.0
ros2 param set /crazyflie_server cf231.params.kalman.resetEstimation 1
sleep 1
ros2 param set /crazyflie_server cf231.params.kalman.resetEstimation 0
```

---

## 8. Kalman variance 100 不是当前直接阻塞点

当前 `/dmp_readiness` 常见：

```text
varX ≈ 100
varY ≈ 100
varZ ≈ 3e-6
```

当前 DMP policy：

```text
max_estimator_variance: None
max_estimator_variance_span: 0.002
minimum_estimator_sample_count: 5
```

当前逻辑主要检查采样窗口中的 variance span 是否稳定，不是默认限制绝对 variance 最大值。

因此不要仅因为 X/Y 是 100 就把它当成当前 blocking issue。

---

## 9. Supervisor auto-arm 兼容问题与本地代码修改

最新真机 telemetry：

```text
supervisor_info_raw = 526
```

DMP 解码后：

```text
armed = true
auto_arm = true
can_be_armed = false
can_fly = true
flying = false
locked = false
crashed = false
tumbled = false
deck_fault = false
```

`526 = 512 + 8 + 4 + 2`。

原 DMP：

```python
if not supervisor.can_be_armed or not supervisor.can_fly:
    issues.append(...)
if supervisor.armed:
    issues.append(...)
```

会把 auto-arm 固件正常待飞状态误判为：

```text
supervisor_not_flyable
supervisor_armed
```

### 当前本地 patch

`telemetry.py` 已改为：

```python
arming_ready = supervisor.can_be_armed or (supervisor.auto_arm and supervisor.armed)
if not arming_ready or not supervisor.can_fly:
    issues.append(ReadinessIssue(
        "supervisor_not_flyable",
        "supervisor does not report a flyable arming state",
    ))

if supervisor.armed and not supervisor.auto_arm:
    issues.append(ReadinessIssue(
        "supervisor_armed",
        "supervisor reports the Crazyflie is already armed",
    ))
```

### 此 patch 尚未完成最终验证

已运行：

```bash
python3 -m pytest tests/integration/test_execution_hardware_adapter.py -q   --basetemp=/tmp/dmp-pytest
```

结果：

```text
27 passed
```

但：
- 尚未确认现有测试专门覆盖 `raw=526 / auto_arm=true / armed=true` 场景；
- 应补 regression test；
- `SupervisorInfo.safe_for_preflight` 属性仍保留旧语义，应全仓搜索实际用途，避免 payload/UI/CF8 语义不一致。

---

## 10. 当前 Bridge 进程状态：必须注意

`telemetry.py` patch 是在 Bridge 已经启动以后修改的。

因此交接时：

**除非用户之后手动重启过，否则当前监听 8765 的 Bridge 仍然加载 patch 前的 Python 代码。**

最后一次已知监听：

```text
127.0.0.1:8765
python pid=5188
```

继续执行时先：

```bash
ss -ltnp | grep 8765
```

然后停止旧 Bridge 并重新启动，以加载新逻辑。

启动命令：

```bash
cd "/mnt/d/Deepseek WorkSpace/drone-mission-planner-main/drone-mission-planner-main/integrations/crazyflie_ros2"

source ~/crazyswarm-env/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash

export PYTHONPATH="$PWD/src/dmp_crazyflie_bridge:${PYTHONPATH:-}"

python -m dmp_crazyflie_bridge.protocol_server   --backend hardware   --config src/dmp_crazyflie_bridge/config/bridge.yaml
```

当前必须继续保持：

```yaml
hardware_flight_enabled: false
```

---

## 11. 当前最新 DMP telemetry（patch 重启前）

```text
connected = true
battery ≈ 3.800 V
battery_critical = false
critical threshold = 3.7 V

positioning.mode = flow
xy_available = true
z_available = true

range = 0.008 m
pose_rate_hz ≈ 9.73

pose_stability.stable = false
xy_span ≈ 0.149 m

velocity:
vx ≈ 0.052
vy ≈ 0.020
vz ≈ 0
speed ≈ 0.0557
```

当时 readiness issues：

```text
supervisor_not_flyable
supervisor_armed
pose_stability_failed
velocity_not_settled
```

其中：
- 前两个预期在正确加载并验证 auto-arm patch 后消失；
- 后两个是真实当前问题，必须继续解决，不能靠降低安全阈值掩盖。

电池只有约 3.8 V，虽然高于 3.7 V critical threshold，但首次 CF8 真飞前建议先充电。

---

## 12. pytest 的 /mnt/d 临时目录问题

项目根目录的：

```text
.pytest-tmp
```

ACL 异常。

WSL：

```text
d--x--x--x?
```

WSL 删除：

```text
Permission denied
```

Windows PowerShell 删除、`icacls` 也返回 Access denied。

目前不用优先修。所有 pytest 统一使用：

```bash
--basetemp=/tmp/dmp-pytest
```

---

## 13. DeepSeek Harness 建议执行顺序

### 阶段 A：非飞行验证

1. `git status --short` + `git diff`，保留现有本地修改。
2. 确认 Crazyswarm2 正在运行。
3. 确认 4 个关键 topics。
4. 重启 DMP Bridge，加载 auto-arm patch。
5. 验证 `hello / get_capabilities / get_telemetry`。
6. 验证 auto-arm 正常状态下不再误报两个 supervisor issue。
7. 全仓搜索 `safe_for_preflight` 与 supervisor 相关逻辑。
8. 增加 auto-arm regression test。
9. 跑 hardware adapter + CF8 软件测试。

### 阶段 B：解决真实 Flow Deck 静止漂移

目标：
- `pose_stability.stable=true`
- XY static span / displacement 落入 policy
- per-axis velocity <= 0.05 m/s
- total speed <= 0.08 m/s
- telemetry fresh
- range valid
- estimator window stable
- 无真实 supervisor 故障

不要先放宽阈值。

### 阶段 C：非飞行安全门

在任何真实电机动作前：

1. 验证 `hardware_flight_enabled=false` 时 `run_cf8_acceptance` 返回 `hardware_flight_disabled`。
2. 验证缺失/错误 `confirm_real_flight` 被拒绝。
3. 检查 CF8 runner 使用实时 readiness。
4. 确认 hardware `go_to` 仍禁用。
5. 确认 hardware `execute_mission` 仍 `execution_not_implemented`。
6. 确认 fault path 仍以 controlled land 为主。

### 阶段 D：首次 CF8 真飞

只有所有非飞行检查通过后，先停止并给用户“首次 CF8 真飞前检查报告”。

没有用户明确确认，不得：
- 把 `hardware_flight_enabled` 改成 true；
- 发送真实 arm/takeoff/land。

确认后 CF8 仅：

```text
takeoff 0.25 m
hover 3 s
controlled land
```

不要进入 CF9。

---

## 14. 绝对不要做

- 不要 `git reset --hard`。
- 不要覆盖当前本地修改。
- 不要自动打开 `hardware_flight_enabled`。
- 不要为了让 readiness 通过随意放宽 drift / velocity / estimator / range 阈值。
- 不要运行 `ros2 run crazyflie_examples hello_world`。
- 不要让 Crazyswarm2 与独立 cflib probe 同时占用 Crazyradio。
- 不要开启 hardware `go_to`。
- 不要进入 CF9。
- 不要解锁 hardware `execute_mission`。
- 不要用 emergency stop 替换 CF8 的 controlled landing fault path。
- 不要在没有用户明确确认时发出真实 arm / takeoff / land。

---

## 15. 当前阶段完成定义

短期目标是让：

```text
Windows DMP
   ↓ TCP
WSL Bridge
   ↓ ROS2
Crazyswarm2
   ↓ Crazyradio
cf231
```

稳定工作，并满足：

```text
Capabilities: PASS
Telemetry: PASS
Flow2/ZRanger2 detection: PASS
DMP readiness topics: PASS
auto-arm semantics: FIXED + TESTED
static pose stability: PASS
static velocity: PASS
range validity: PASS
estimator stability: PASS
hardware flight still locked: PASS
```

达到这里以后，才进入一次人工监督的 CF8 takeoff-hover-land acceptance。

CF8 通过后，才讨论 CF9 waypoint / go_to。
