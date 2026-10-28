# 历史交接文档归档

> 本目录只存**历史**交接文档，**不作为事实基线**。查历史决策时翻阅。
> 当前事实基线是 `docs/workflow/session-log.md`；当前规范是 `docs/workflow/` 下各文件。
> 新会话请**不要**直接以本目录文件开工。

| 文件 | 时间 | 内容 |
|---|---|---|
| `CF8_1_Crazyflie_handoff_2026-09-21.md` | 2026-09-21 | 首次 CF8 尝试（电池掉压）交接 |
| `CF8_Crazyflie_handoff_2026-09-22.md` | 2026-09-22 | 全天排障（电池/电台/冷启动）交接 |
| `CF8_handoff_2026-10-13.md` | 2026-10-13 | 新电池上电、环境 bug 修复、Flow Deck 不被检测证据 |
| `session-history-2026-10-20.md` | 2026-10-20 | **旧版 session-log 全文**（追加式编年史 317 行）。含当日所有推理过程与已作废结论 |
| `DeepSeekHarness_CF8_1_prompt_v2.txt` | 2026-09-21 | CF8.1 阶段提示词 |
| `NEXT_SESSION_CF8_PROMPT.txt` | 2026-09-22 | 旧版下一会话提示词 |
| `NEXT_SESSION_CF8_PROMPT_2026-10-13.txt` | 2026-10-13 | 旧版下一会话提示词（已被 `docs/workflow/` 体系替代） |
| `NEXT_SESSION_CF8_PROMPT_2026-10-14.txt` | 2026-10-14 | 旧版下一会话提示词（"把这段贴给新会话"，已被三层文档体系替代） |
| `CF8_FLIGHT_REPORT_2026-10-14.md` | 2026-10-14 | 14:58 那次尝试的完整分析（结论是**中止的起飞**，只升到 2.7 cm，不是飞行） |
| `INCIDENT_2026-10-14_tethered_takeoff.md` | 2026-10-14 | 16:03 外电起飞事故：飞机还在 USB 充电器上就放行了起飞 |
| `INCIDENT_2026-10-14_tumble_crash.md` | 2026-10-14 | 16:32 真起飞翻倒事故：机架形变导致推力轴倾斜，`tumbled`+`crashed` |
| `session-history-2026-10-26-before-mechanical.md` | 2026-10-26 | **旧版 session-log 全文**（机械基线轮之前） |
| `session-history-2026-10-29-before-pid2b-offline.md` | 2026-10-29 | **旧版 session-log 全文**（2b 离线准备之前） |

**归档原因**：这些文件互相引用、重复、漂移（例如交接单声称的唯一运行手册与脚本实际行为
不符），已由 2026-10-14 建立的规范体系（`docs/workflow/`）取代。证据链仍在
`D:\Deepseek WorkSpace\_dsh_scratch\CF8_DECK_BLOCKER_*.md` 与 `cf_evidence_runs/`。

## 为什么会有 `session-history-*.md`

2026-10-20 的 reorganize 发现：`session-log.md` 被当成追加式编年史使用，同一页里并存
**互相矛盾的活结论**（例如"已离地"与"未离地"），新会话读到哪条全看运气，直接导致重复犯错。

现在的规则（见 `docs/workflow/session-protocol.md` §0.1）：
**`session-log.md` 是快照，每次收尾重写；过程叙事进本目录。**
被推翻的结论**从当前文档删除**，不是加一句"以上作废"。
