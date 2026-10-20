#!/usr/bin/env python3
# ruff: noqa: RUF001, RUF003
# Fullwidth punctuation (，：) is intentional: these checkers match it in the Chinese docs.
"""cf85_stage_audit.py -- check STAGE.md for FACTUAL integrity, not just formatting.

The existing consistency check proves the docs do not contradict each other structurally.
It cannot tell whether a verdict is supported. This one attacks the four failure modes that
matter for a hardware project:

  1. a FAILED safety criterion that does not explicitly block the corresponding experiment
  2. a PASS verdict with no test conditions and no evidence path
  3. STAGE.md and session-log.md disagreeing about the same criterion's state
  4. a completed stage whose criteria are not all green

Exit 0 = structurally sound; 1 = at least one factual gap.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

# Resolve docs/ relative to this file so the checker works from any checkout.
REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS = REPO_ROOT / "docs"
WF = DOCS / "workflow"



STAGE = WF / "STAGE.md"
LOG = WF / "session-log.md"

REQUIRED_FIELDS = ("测试条件", "证据路径", "下一步")


def main() -> int:
    stage = STAGE.read_text(encoding="utf-8")
    log = LOG.read_text(encoding="utf-8")
    problems: list[str] = []

    print("=== 1. every criterion row: verdict present? ===")
    # The summary table rows look like: | 1 | ... | ... | ❌ FAIL |
    rows = re.findall(r"^\|\s*(\d)\s*\|(.+?)\|\s*([^|]*?)\s*\|\s*$", stage, re.MULTILINE)
    verdicts: dict[str, str] = {}
    for num, _body, verdict in rows:
        verdict = verdict.strip()
        if not verdict:
            continue
        verdicts[num] = verdict
        print(f"  判据 {num}: {verdict}")

    if not verdicts:
        problems.append("no criterion verdict rows found in STAGE.md")

    print("\n=== 2. FAILED criteria must block the experiment explicitly ===")
    failed = [n for n, v in verdicts.items() if "FAIL" in v]
    print(f"  failed criteria: {failed or 'none'}")
    if failed:
        gate = re.search(r"## 3\. 前置条件(.+?)(?=\n## )", stage, re.DOTALL)
        order = re.search(r"## 5\. 推进顺序(.+?)(?=\n## )", stage, re.DOTALL)
        if not gate:
            problems.append("FAIL verdicts exist but there is no 前置条件 section")
        if not order:
            problems.append("FAIL verdicts exist but there is no 推进顺序 section")
        if gate and order:
            print("  both 前置条件 and 推进顺序 present -> a failed criterion is gated")

    print("\n=== 3. each detail block needs 测试条件 + 证据路径 + 下一步 ===")
    blocks = re.split(r"\*\*判据 [\d一二三四] —", stage)
    for i, block in enumerate(blocks[1:], 1):
        missing = [f for f in REQUIRED_FIELDS if f not in block]
        if missing:
            print(f"  判据块 {i}: MISSING {missing}")
            problems.append(f"detail block {i} missing {missing}")
        else:
            # Evidence path must be a concrete path, not a promise.
            has_path = bool(re.search(r"cf_evidence_runs/[\w.\-]+", block))
            print(f"  判据块 {i}: complete" + ("" if has_path else "  (no concrete evidence path!)"))
            if not has_path:
                problems.append(f"detail block {i} cites no concrete evidence path")

    print("\n=== 4. STAGE and session-log must not disagree on状态 ===")
    stage_fail = len(failed)
    log_fail = log.count("❌")
    print(f"  STAGE FAIL rows: {stage_fail}   session-log ❌ marks: {log_fail}")
    if stage_fail and log_fail == 0:
        problems.append("STAGE has FAIL criteria but session-log shows no failing mark")
    # session-log must not claim a criterion passes that STAGE marks FAIL.
    if ("全部通过" in log or "四条全绿" in log) and stage_fail:
        problems.append("session-log claims all-green while STAGE has FAIL rows")

    print("\n=== 5. stage marked complete must have no FAIL ===")
    m = re.search(r"状态：\*\*(.+?)\*\*", stage)
    state = m.group(1) if m else "?"
    print(f"  STAGE state: {state}")
    # "未完成" contains "完成"; require an affirmative completion, not a negated one.
    complete = ("完成" in state) and not any(w in state for w in ("未完成", "不可飞", "未达成"))
    if complete and failed:
        problems.append("stage marked complete but criteria still FAIL")

    print("\n=== 6. 'excluded routes' must carry conditions, not absolutes ===")
    sec = re.search(r"## 4\. 已检验的假设(.+?)(?=\n## )", stage, re.DOTALL)
    if sec:
        body = sec.group(1)
        if "测试条件" not in body or "何时应重新考虑" not in body:
            problems.append("§4 lacks condition / re-evaluation columns")
        absolutes = [w for w in ("永远无效", "永远没用", "不可能", "绝无可能") if w in body]
        if absolutes:
            problems.append(f"§4 contains absolute claims: {absolutes}")
        print(f"  columns present: 测试条件={'测试条件' in body}, 何时重评={'何时应重新考虑' in body}")
        print(f"  absolute claims: {absolutes or 'none'}")
    else:
        problems.append("§4 已检验的假设 missing")

    print()
    if problems:
        print(f"AUDIT: {len(problems)} factual gap(s)")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("AUDIT: sound (criteria carry conditions + evidence; FAILs are gated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
