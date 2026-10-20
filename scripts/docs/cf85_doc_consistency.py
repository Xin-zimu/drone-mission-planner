#!/usr/bin/env python3
# ruff: noqa: RUF001, RUF003
# Fullwidth punctuation (，：) is intentional: these checkers match it in the Chinese docs.
"""Final consistency gate for the reorganised docs.

Fails loudly if the defects that motivated this cleanup are still present:
  1. a dangling reference to a fault id that no longer exists
  2. two contradictory "current" statements about the same fact
  3. an append-style heading in the snapshot log (it must be a snapshot, not a chronicle)
"""
from __future__ import annotations

import re
from pathlib import Path

# Resolve docs/ relative to this file so the checker works from any checkout.
REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS = REPO_ROOT / "docs"
WF = DOCS / "workflow"




OPS = (WF / "cf8-operations.md").read_text(encoding="utf-8")
LOG = (WF / "session-log.md").read_text(encoding="utf-8")
PROTO = (WF / "session-protocol.md").read_text(encoding="utf-8")

problems: list[str] = []


def main() -> int:
    defined = set(re.findall(r"^### (F\d+)\s", OPS, re.MULTILINE))
    print(f"defined fault ids: {sorted(defined, key=lambda s: int(s[1:]))}")

    # 0. the three-layer plan docs must all exist
    for name in ("PLAN.md", "STAGE.md", "session-log.md"):
        if not (WF / name).is_file():
            print(f"\n[!] missing plan doc: {name}")
            problems.append(f"missing {name}")
    print(f"  three-layer docs present: {all((WF / n).is_file() for n in ('PLAN.md', 'STAGE.md', 'session-log.md'))}")

    # 0b. no live doc may point at an archived plan AS A SOURCE.
    # A mention inside a "why it was archived" explanation is fine -- detect that by looking for
    # archival wording near the mention.
    archived_dir = WF.parent / "archive" / "plans-2026-09"
    ARCHIVE_WORDS = ("归档", "已废止", "已移动", "archived", "历史", "旧做法", "不再",
                     "编年史", "未更新", "已被取代")
    # Generic filenames exist in many directories; only judge specific plan filenames.
    GENERIC = {"README.md", "index.md"}
    LIVE_DOCS = {"PLAN.md", "STAGE.md", "session-log.md", "session-protocol.md",
                 "dev-workflow.md", "cf8-operations.md"}
    for doc in WF.glob("*.md"):
        if doc.name not in LIVE_DOCS:
            continue
        text = doc.read_text(encoding="utf-8")
        for target in sorted({m.group(1) for m in re.finditer(r"([\w.-]+\.md)", text)}):
            if target in GENERIC or (WF.parent / target).is_file():
                continue
            if not (archived_dir / target).is_file():
                continue
            for ln_no, line in enumerate(text.splitlines(), 1):
                if target not in line:
                    continue
                if any(w in line for w in ARCHIVE_WORDS):
                    continue
                print(f"\n[!] {doc.name}:{ln_no} points at archived '{target}' as if live")
                problems.append(f"{doc.name} -> archived {target}")

    # 1. dangling references anywhere in docs/workflow
    referenced: dict[str, list[str]] = {}
    for doc in WF.glob("*.md"):
        for m in re.finditer(r"\bF(\d+)\b", doc.read_text(encoding="utf-8")):
            fid = f"F{m.group(1)}"
            if fid in defined:
                continue
            referenced.setdefault(fid, []).append(doc.name)
    if referenced:
        print(f"\n[!] ids referenced but not defined: {sorted(set(referenced))}")
        problems.append("dangling fault references")
    else:
        print("  no dangling fault references")

    # 2. the snapshot log must not contain append-style sections
    bad_heads = [ln for ln in LOG.splitlines()
                 if ln.startswith(("### 追加", "### 本轮补充", "### 本轮完成"))]
    if bad_heads:
        print(f"\n[!] append-style headings in session-log: {bad_heads}")
        problems.append("session-log is a chronicle again")
    else:
        print("  session-log has no append-style headings (it is a snapshot)")

    # 3. the log must not contain superseded verdicts as live prose
    stale = [ln for ln in LOG.splitlines()
             if "已作废" in ln or "上一条" in ln or "已推翻" in ln]
    if stale:
        print("\n[!] superseded verdicts still readable in the snapshot log:")
        for ln in stale:
            print(f"    {ln.strip()[:90]}")
        problems.append("stale verdicts in snapshot log")
    else:
        print("  no superseded verdicts left in the snapshot log")

    # 4. protocol must carry the rewrite rule
    if "快照，不是编年史" in PROTO or "重写" in PROTO:
        print("  protocol carries the rewrite-not-append rule")
    else:
        problems.append("protocol lacks the rewrite rule")

    # 5. every doc that mentions the session gate must agree on the number
    gate_vals = set()
    for doc in DOCS.rglob("*.md"):
        t = doc.read_text(encoding="utf-8", errors="ignore")
        for m in re.finditer(r"BATTERY_FORBID_V[^0-9]{0,20}(\d\.\d+)", t):
            gate_vals.add(m.group(1))
    print(f"  BATTERY_FORBID_V values mentioned in docs: {sorted(gate_vals) or 'none'}")
    if len(gate_vals) > 1:
        problems.append(f"conflicting session-gate values: {sorted(gate_vals)}")

    print()
    if problems:
        print(f"RESULT: {len(problems)} problem(s)")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("RESULT: consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
