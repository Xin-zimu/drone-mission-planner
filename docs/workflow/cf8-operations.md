# CF8 真机操作规范（SOP · 故障库 · 工具索引 · 环境坑）

> 本文件是 CF8 真机线的**唯一规范**，替代 `CF85_RUNBOOK.md`（内容已并入并去重）。
> 涉及真机的会话：先读 `session-protocol.md` + `session-log.md`，再读本文件。
> 详细实现背景仍可查 `_dsh_scratch/CF85_RUNBOOK.md` 与 `docs/archive/`，但**以本文件为准**。

---

## 1. 一句话状态机

```text
[用户物理准备] → cf_prepare.sh → READY_FOR_CF8 → (用户明确授权) → run_cf8_acceptance.sh
                       │ 任何 FAIL：立即停止、不重试、报告等指示
                       └─ 日常只有这两个命令，没有第三个入口
```

**日常只有两条命令：**

```bash
wsl -d Ubuntu
cd "/mnt/d/Deepseek WorkSpace"
./cf_prepare.sh                # 准备全流程：电池 → capability(deck) → Link → Preflight → READY_FOR_CF8
./run_cf8_acceptance.sh        # 只有用户明确决定真飞时才运行（一次授权 = 最多一次）
```

正常输出四行 + 一个结论：

```
Battery      PASS
Radio        PASS
Link         PASS
Preflight    PASS

READY_FOR_CF8
```

- `cf_prepare.sh --verbose` 显示内部每步；`run_cf8_acceptance.sh --yes` 跳过 5 秒中止窗口。
- **单一入口铁律**：`cf84_preflight.py` 永远不得手工直接调用；preflight 唯一入口是
  `cf85_fast_path.sh`，它只由 `cf_prepare.sh` 调用。
- **READY token**：一次性、默认 900 s 有效、绑定本次 runtime URI（sha256 比对）、
  要求栈仍在运行且 `hardware_flight_enabled` 仍为 false。同一 run dir 不允许第二次
  preflight / 第二次飞行。

## 2. 用户物理操作清单（AI 不能代劳的部分）

按顺序，每步之间等 AI 确认再继续：

1. **充满电**：静置电压 ≥ 4.10 V 才放行（会话政策）。充电电流偏慢（~0.04 A）需留意卡住。
2. **确认机械结构**：机体平整、Flow Deck 压紧、插头线材正常。
3. **上电**：对 AI 说"现在上电"——在此之前 AI 不连 cf231、不跑 probe。
4. **电台插接**：Crazyradio 2.0 插在 PC USB 口（busid 会变，先 `usbipd list` 确认）。

**AI 侧铁律**：在用户说"现在上电"之前，不连接 cf231、不启动 Crazyswarm2、不跑 probe、
不刷新 snapshot。未获用户明确"执行 CF8"前禁止真实 takeoff。

## 3. 起飞前 READY 判定（FAST PRECHECK 只查真正的 ground blocker）

只关注这些（airborne-only 条件留给 STABILIZING）：

| 条件 | 来源 |
|---|---|
| `connected` | runtime URI CRTP 应答 |
| `positioning_mode=flow` | preflight |
| `ground_passed` | 生产 readiness gate |
| `estimator_not_converged` 不存在 | 生产 readiness gate |
| Flow 数据非冻结 | Stage 2.75 capability 刷新 |
| supervisor safe | `supervisor.info` 解码（见故障库 F2） |
| battery / 供电状态 | Stage 1 battery gate |
| 无持续 ESTKALMAN reset | 健康检查 |

到 READY_FOR_CF8 后停在起飞前，报告当前电池电压和 gate 结果。

## 4. 电池电压政策（会话级，非生产阈值）

| 静置电压 | 允许动作 | `cf_prepare.sh` 表现 |
|---|---|---|
| `< 4.00 V` | 禁止进入 FAST PATH | `Battery FAIL`，立即停止 |
| `4.00 – 4.10 V` | 仅诊断，不允许飞 | `Battery PASS (diagnostics only)`，停止（本轮不飞） |
| `≥ 4.10 V` | 放行 | `Battery PASS` |
| "充满"但 `< 3.90 V` | 判可疑电池，不再用于 CF8 实飞 | `battery.json` 标 `suspect_pack: true` |

> 背景：原电池静置 4.043 V，起跳瞬间崩到 3.457 V（sag 0.586 V）。**静置电压达标 ≠ 电池健康。**
> 生产阈值 `battery_critical_voltage = 3.7 V` 永远不改；会话政策只是更严格。

## 5. 工具索引（全部在 `D:\Deepseek WorkSpace\_dsh_scratch\`，仓库外）

**命名规则**：新脚本 `cf85_` 前缀、用途可猜、写完立即登记到本节；一次性脚本标 `[临时]` 用完删。

### 5.1 入口（只有这两个会发起动作）

| 文件 | 作用 |
|---|---|
| `cf_prepare.sh` | 入口 1：准备全流程（唯一会启动栈的入口） |
| `run_cf8_acceptance.sh` | 入口 2：真实飞行，单独触发，需 READY token |

### 5.2 只读诊断（分组：链路 / 电池 / deck / 系统）

| 文件 | 作用 | 组 |
|---|---|---|
| `run_probe.sh` | 同一 WSL 会话内"确保电台 + 跑探针"（应对电台掉线），`用法: run_probe.sh <busid> <probe.py> [args]` | 链路 |
| `cf85_radio_scan.py` | 电台发现（只听不连） | 链路 |
| `cf85_link_health.py` / `cf85_bringup_health.sh` | 持续型数据链路健康判定（30 s 窗口 + ≥50% 标称速率） | 链路 |
| `cf85_radio_paths_test.sh` | 电台"正常短路/异常重连"两路径自检（零射频） | 链路 |
| `cf85_diagnose_bringup.sh` | 只读 bring-up 诊断（内核 USB/IP 日志等） | 链路 |
| `cf85_battery.sh` | 只读电池读数 + 政策裁决（用 runtime URI） | 电池 |
| `read_battery_probe.py` / `battery_trend.py` / `read_pm_state.py` | 电池读数/趋势/充电状态 | 电池 |
| `flow_deck_live.py --duration 5` | **决定性**：Flow Deck 五路变量（deltaX/deltaY/motion/squal/zrange）是否有真实数据 | deck |
| `flow_raw.py` | 完整故障签名（含 `motion.motion`、`TYPE_1W` 普查）+ 与 09-22 对照 | deck |
| `deck_inspect.py` / `deck_diag.py` / `param_meta.py` / `param_access.py` | deck 参数/1-wire 身份/access 证明（deck.* 全 RO） | deck |
| `force_check.py` | 扫描是否存在可写 deck/force 开关（结论：不存在） | deck |
| `deck_recover.py` | 诊断 + 可选 `stm_power_cycle()` 软件重启（重启需 `--power-cycle --confirm-power-cycle`） | deck |
| `boot_banner.py` | 抓启动横幅（含 deck 发现行）；可 `--power-cycle` | deck |
| `charge_state.py` / `console_and_charge.py` | 充电状态 / 固件 console 转储 | 电池 |
| `cf85_identify.py` / `cf85_toc_enumerate.py` / `log_toc_probe.py` | 机体识别 / TOC 枚举（不猜变量名） | 系统 |
| `cf85_cf2.sh --set-address <addr> --confirm-write` | **唯一**的 CF2 配置块写入口（改地址需断电重启；非日常路径） | 系统 |
| `cf85_smoke_test.sh` | 离线自检 121 项（不碰电台、零飞行命令） | 自检 |
| `cf85_run_offline_tests.sh` | 上面 + pytest 单元测试套件 | 自检 |

### 5.3 自检（离线，随时可跑）

```bash
bash _dsh_scratch/cf85_smoke_test.sh              # 121 项，零射频
bash _dsh_scratch/cf85_run_offline_tests.sh       # + pytest
bash _dsh_scratch/cf85_radio_paths_test.sh        # 电台路径自检（零射频、不连 cf231）
```

## 6. 已知故障库（先对照签名，再动手；禁止跳过对照直接"重新排查"）

> 每条有：签名（怎么认出来）、判据（决定性证据）、修复（已验证的）、曾发生时间。

### F1 电池掉压（sag）

- **签名**：静置 4.0+ V，起跳瞬间 `battery_critical`（原电池崩到 3.457 V，sag 0.586 V）。
- **判据**：`battery_trend.py` 记录起跳前后电压曲线；`failure_telemetry.battery_voltage` 有触发瞬时值。
- **修复**：换已知正常电池（**换电池不能当作无线恢复的证明**）；"充满"后 <3.90 V 判可疑。
- **发生**：2026-09-21 两次 CF8 尝试；2026-09-22。
- **注意**：充电电流偏慢（~0.04 A）要留意插头/电池接触。

### F2 Flow Deck 数据冻结（检测到了但数据不走）

- **签名**：`deck.bcFlow2=1`；原始块 `[dX, dY, motion, squal]` 形如 `[0, 0, 58, 0]`
  （`motion.motion` 计数 58 仍在走，但 deltaX/deltaY/squal 恒 0）；`TYPE_1W` 存在。
- **判据**：`flow_raw.py` 的对照表；与 F3 的区分点是 `bcFlow2=1` 且 `motion.motion>0`。
- **修复**：**重新插接 Flow Deck**（2026-09-22 验证有效）。
- **发生**：2026-09-22（冻结期）。
- **注意**：与 F3 是**不同故障**，老经验不通用。

### F3 Flow Deck 完全不被检测（1-wire 枚举为空）

- **签名**：`deck.bcFlow2=0`、`bcZRanger2=0`（所有 `deck.bc*` 全 0）；`TYPE_1W=0`、
  `TYPE_I2C=1`；原始块 `[0, 0, 0, 0, 0]`（`motion.motion` 恒 0）；`range.zrange` 恒 0。
  对照：`pm.vbat`/`stateEstimate`/`supervisor` 正常 → 机体健康。
- **判据**：`flow_deck_live.py`（五路全 0）+ `flow_raw.py`（TYPE_1W=0）双通道一致；
  黄金对照：09-22 正常启动横幅含 `DECK_DISCOVERY: Backend found: onewire` +
  `PMW: Motion chip id: 0x49:0xB6`。
- **排除路径**（已证明无效）：软件电源循环 `stm_power_cycle()`、物理重插 deck、
  整机+电台全拔重插；写参数/写 NVM 不可能恢复（`force_check.py` 证明 deck.* 全 RO、无 force 开关）。
- **修复**：**换一块已知正常的 Flow Deck v2 对照**——`TYPE_1W` 恢复 → 原 deck 损坏；
  仍为 0 → 板侧 1-wire header/pull-up 故障需维修。
- **发生**：2026-10-13 ~ 10-14（当前阻断点）。

### F4 电台链路故障（nRF51 应答、STM32 无应答）

- **签名**：无线 ACK 正常、信道扫描命中，但应用层 CRTP 不应答
  （`RADIO_ACK_ONLY`）；`platform.protocolVersion` 读不到。
- **判据**：**必须应用层 CRTP 真应答才算找到机体**（RUNBOOK §2.2）；
  nRF51 ACK / link quality / 信道扫描命中都不算。
- **修复**：**全断电冷启动（USB + 电池都拔）**——只拔 USB 无效，STM32 应用卡死。
- **发生**：2026-09-22。
- **注意**：现在的 `cf_prepare.sh` 会明确报 `Radio FAIL`，绝不误报成 `Battery FAIL`。

### F5 USB/IP（usbipd/WSL）链路不稳

- **签名**：cflib 探针无限挂死（正常 ~20 s）；CRTP connect 0/5 成功、无 connection_failed
  消息；1M/250K 报 `Too many packets lost`；`dmesg` 大量 `vhci_hcd: vhci_device speed not set`；
  `usbipd list` 显示 Attached 但 WSL 内 `lsusb` 看不到（陈旧状态）。
- **判据**：`lsusb` 在 WSL 内看 + 完整 CRTP connect 双向包。
- **修复**：**物理拔插 Crazyradio**（最可靠）；然后 `usbipd list` 读新 busid →
  `usbipd attach --wsl --busid <NEW>`。WSL 重启后若显示 `already attached` 先 detach 再 attach。
- **发生**：2026-10-14 凌晨（触发源：后台探针占住电台——见通用铁律"探针与栈不同时占电台"）。
- **注意**：WSL 空闲自动关机会使 USB/IP 失效；一律用 `run_probe.sh` 保证同一会话内电台可用。

### F6 环境 bug（脚本层，非飞机问题）

- **签名**：`cf_prepare.sh` 报 `No module named 'cflib'`（被误报成找不到机体）；
  用错 busid 自动修复失败。
- **判据**：入口脚本未激活 venv；busid 写死。
- **修复**：`cf_ops_source_env()` 自动激活 `~/crazyswarm-env`；`cf_ops_radio_busid()` 运行时解析。
- **发生**：2026-10-13（首次真机跑 cf_prepare.sh 暴露；此前只有离线验证）。
- **教训**：**离线验证过的脚本不等于真机跑过**；入口首次上真机前先走一遍只读路径。

### F7 探针/栈互斥冲突（自伤类）

- **签名**：某个探针突然挂死；后台 job 占住电台导致第二个探针阻塞。
- **判据**：`ps -eo pid,ppid,etime,stat,cmd` 查残留进程；`sudo fuser -v /dev/bus/usb/...` 查占用。
- **修复**：清理孤儿进程；**cflib 探针与 Crazyswarm2 永不同时占用电台**（硬约束）。
- **发生**：2026-09-22（孤儿进程占 USB）；2026-10-14（后台探针占电台）。

## 7. 环境坑（每坑只写一次；新坑先验证再入表）

| 坑 | 现象 | 处理 |
|---|---|---|
| WSL 空闲自动关机 | USB/IP 挂载失效，`usbipd list` 仍显示 Attached（陈旧） | 一律用 `run_probe.sh`（同一 WSL 会话内确保电台再跑探针）；保持 keepalive |
| 电台 busid 会变 | 插拔后 1-3 → 1-4 → 3-1 | `cf_ops_radio_busid()` 运行时解析；手动命令先 `usbipd list` |
| venv 未激活 | 裸 `python3` 无 cflib | 入口脚本 `cf_ops_source_env()` 已自动激活 `~/crazyswarm-env` |
| `.bashrc` L130 venv 行被注释 | 登录 shell 不自动激活 | 不要依赖自动激活；入口脚本显式激活 |
| `.pytest-tmp` ACL 坏 | `tmp_path` 测试整批 ERROR / WinError 5 | `--basetemp=.pytest-tmp-<新名>`；`.pytest-tmp` 需管理员修，别碰 |
| 重复 `-q` | 看不到 `N passed` 汇总（addopts 已含 `-q`） | 命令行不要再加 `-q` |
| 路径含空格 | DSH 命令静默失败/切分错误 | 用 `D:\dmp` junction（真路径 `D:\Deepseek WorkSpace\...`） |
| PowerShell 内联 `$` | 被提前展开 | shell 逻辑写进 `.sh`；pwsh 内联不写 `$` |
| Windows 改文件 CRLF | `write_text` 产生整文件假 diff | 字节读写 + `replace(b"\r\n", b"\n")` |
| WSL 重启后 attach | 显示 `already attached to a client` | 先显式 `usbipd detach --busid <B>` 再 attach |
| 探针挂死 | 被后台 job 占电台 | 先清进程再跑；绝不并行两个探针（见 F7） |

## 8. 证据布局（每次运行一个目录，永不覆盖历史）

```
D:\Deepseek WorkSpace\cf_evidence_runs\<时间戳>-<阶段>\
    meta.txt / runtime-binding.json / uri-resolution.json / bridge.runtime.yaml /
    crazyflies.runtime.yaml / battery.json / radio.log / recorder.ndjson /
    launch.log / health.json / capability.json / preflight.json /
    prepare.log / ready_for_cf8.json / summary.md
飞行时追加：bridge-flight.log / flight-attempt.log / flight-result.json / final-state.json
```

- 中间诊断必须带时间戳；最终产物用稳定文件名。
- 护栏：脚本拒绝写 `cf81_dryrun_evidence` 或含 `cf84-recorder` 的路径（exit 2）——
  CF84 唯一真实飞行 timeline 不可能被覆盖。
- 分析 recorder 数据必须**截断 operator pickup 之后的样本**（用户结束后会用手拿机体）。

## 9. 出问题时看哪里（对照表）

| 现象 | 先看 | 含义 |
|---|---|---|
| `Radio FAIL` + `NO_VALID_URI` | `uri-resolution.log`、`runtime-binding.json` | 没有地址能 CRTP 应答（没开机 / 只有 nRF51 答 / 地址不对）→ F4 |
| `Radio FAIL` + `AMBIGUOUS` | `runtime-binding.json` 的 valid_candidates | 两个地址都能 CRTP，绝不猜 |
| `Radio FAIL` + `RADIO_ACK_ONLY` | `uri-resolution.log` | 电台层 ACK、应用层无应答 → F4 |
| `Radio FAIL` + `CONFIG_INVALID` | `config/cf_identity.yaml` | 身份配置本身有问题 |
| `Battery FAIL` | `battery.json`、`battery.raw.log` | 电压不达标或读不到 → F1 |
| `Link FAIL` | `health.json` | 五路有流没数据 / 速率远低标称 |
| `Preflight FAIL` | `preflight.json` 的 failed_checks | 逐条明细 |
| Stage 2.75 报 "Flow Deck v2 decks are missing" | `flow_deck_live.py`、`flow_raw.py` | deck 仍为 0 → F3（当前阻断点） |
| 探针挂死 | 进程列表 + `fuser` | F5 / F7 |
| 想看内部步骤 | `prepare.log`、`--verbose` | 全部内部命令与输出 |

`cf_prepare.sh` 任何一步 FAIL 立即停止且不重试；重来就重新跑（自动开新 run dir）。

## 10. 绝对不做的事（硬约束，违反即停）

1. 不改任何阈值/gate/TTL/estimator 策略：`battery_critical_voltage`(3.7)、
   `capability_snapshot_max_age_s`(3600)、`max_estimator_variance_span`(0.002)、STABILIZING 全部参数。
2. `hardware_flight_enabled` 保持 false，除非用户**明确单次授权** CF8。
3. 不绕过 readiness gate；不用 emergency stop 替代受控降落。
4. 不跑 CF9 / 硬件 `go_to` / `execute_mission` / `hello_world`。
5. 不自动重试 CF8：一次授权 = 最多一次 `run_cf8_acceptance`。
6. 不手工直接调用 `cf84_preflight.py`（唯一入口 `cf_prepare.sh`）。
7. cflib 探针与 Crazyswarm2 不同时占用电台；不并行跑两个探针。
8. 不写机体 NVM、不改 radio address/channel/speed（唯一写入口 `cf85_cf2.sh --confirm-write`）。
9. 不改 `bridge.yaml` / `crazyflies.yaml`；runtime 配置只写 run dir。
10. 不把 `cf_identity.yaml` 的 policy 开关改 false 来"修好"失败（会被判 CONFIG_INVALID）。
11. push / merge 需用户明确要求，默认不做。
12. 在用户说"现在上电"前不连 cf231；未授权前绝不真实起飞。

## 11. 飞行后报告清单（必须逐项）

结果 / `failure_code` / 事件时间线 / `failure_telemetry`（真实触发值）/ `max_z` /
实际离地高度 / 最大 XY 漂移 / 最大速度 / 最低电池 / squal / varP / range / supervisor /
是否进入 STABILIZING / service 调用次数 / 是否受控降落 / 最终安全状态。
+ 安全声明逐项：arm / takeoff / land / go_to / emergency = YES/NO；NVM 写入；阈值修改。
