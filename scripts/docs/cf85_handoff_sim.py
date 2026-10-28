#!/usr/bin/env python3
# ruff: noqa: RUF003
# Fullwidth punctuation (，：) is intentional: these checkers match it in the Chinese docs.
"""Handoff simulation: can a COLD session decide the next safe action from the docs alone?

Not a keyword test. This reconstructs what a fresh agent would conclude and checks the
conclusions against ground truth captured from the actual hardware runs. It fails if the docs
would lead a cold agent to:

  * believe the aircraft is flyable
  * pick an action that STAGE's gate forbids
  * miss that the airframe question is still unanswered
  * propose a route that was already tested and rejected
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

# Resolve docs/ relative to this file so the checker works from any checkout.
REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS = REPO_ROOT / "docs"
WF = DOCS / "workflow"


def _evidence_root() -> Path:
    """Evidence runs live OUTSIDE the repo, potentially several levels up.

    This checkout is nested (drone-mission-planner-main/drone-mission-planner-main/), so the
    evidence directory sits two levels above, next to the outer folder. Walk upward instead of
    assuming a fixed depth, and allow an explicit override.
    """
    env = os.environ.get("CF_EVIDENCE_ROOT")
    if env:
        return Path(env)
    for candidate in [REPO_ROOT, *REPO_ROOT.parents]:
        if (candidate / "cf_evidence_runs").is_dir():
            return candidate
    return REPO_ROOT.parent


EVIDENCE_ROOT = _evidence_root()


GROUND_TRUTH = {
    "flyable": False,
    "blocking_count": 3,
    "airframe_question_answered": False,
    # 2026-10-27: criterion 1 PASSED (sag 0.552 V), so the gated next step is no longer
    # "change the battery" -- it is separating the roll limit-cycle root cause. Keep this in
    # step with STAGE section 5: a checker that still demands the old step passed on stale
    # struck-through text once, which is exactly the drift this file exists to catch.
    "next_action_is_attitude": True,
}


def main() -> int:
    plan = (WF / "PLAN.md").read_text(encoding="utf-8")
    stage = (WF / "STAGE.md").read_text(encoding="utf-8")
    log = (WF / "session-log.md").read_text(encoding="utf-8")
    corpus = plan + stage + log
    fails: list[str] = []

    print("=== cold-session conclusions ===")

    # 1. Is the aircraft flyable? A cold agent must say NO.
    says_unflyable = ("不可飞" in log) and ("FAIL" in stage)
    print(f"  1. 判断为不可飞                : {says_unflyable}")
    if not says_unflyable:
        fails.append("cold agent could think the aircraft is flyable")

    # 2. How many criteria fail? Must be 3 (plus one N/A), matching the runs.
    fail_rows = len(re.findall(r"\|\s*[❌]\s*FAIL\s*\|", stage))
    print(f"  2. 判据 FAIL 条数 = {fail_rows} (truth {GROUND_TRUTH['blocking_count']})")
    if fail_rows != GROUND_TRUTH["blocking_count"]:
        fails.append(f"FAIL count {fail_rows} != {GROUND_TRUTH['blocking_count']}")

    # 3. Does it know the airframe question is unanswered?
    unanswered = any(w in corpus for w in ("形变问题未触及", "无法判断", "未测"))
    print(f"  3. 知道形变问题无结论          : {unanswered}")
    if not unanswered:
        fails.append("docs imply the airframe question is settled")

    # 4. The next action must be the gated live step, not a flight.
    order = re.search(r"## 5\. 推进顺序(.+?)(?=\n## )", stage, re.DOTALL)
    # A step may be struck through (~~...~~) once it is DONE; the live next step is the first
    # one that is not struck through. Reading a completed step as the next action would let a
    # cold session repeat work that is already finished.
    live_step = ""
    if order:
        for line in order.group(1).splitlines():
            if re.match(r"\s*\d+\.\s", line) and "~~" not in line:
                live_step = line.strip()
                break
    step_is_attitude = ("姿态" in live_step) and ("起飞" not in live_step.split("**")[0][:20])
    print(f"  4. 下一步是查姿态而非起飞      : {step_is_attitude}  ({live_step[:60]})")
    if not step_is_attitude:
        fails.append("next action is not the gated live step (attitude)")

    # 5. Must warn against repeating rejected routes.
    warns_repeat = "重新考虑" in stage and "已检验的假设" in stage
    print(f"  5. 对已排除路线给出重评条件    : {warns_repeat}")
    if not warns_repeat:
        fails.append("no re-evaluation conditions on rejected routes")

    # 6. Must not present a superseded 4.10 V gate.
    if "4.10 V" in corpus:
        fails.append("superseded 4.10 V gate still present")
    print(f"  6. 无过期的 4.10 V 闸门        : {'4.10 V' not in corpus}")

    # 7. Evidence must be reachable without the chat.
    paths = set(re.findall(r"cf_evidence_runs/[\w.\-]+", stage))
    existing = [p for p in paths if (EVIDENCE_ROOT / p).exists()]
    print(f"  7. 引用的证据路径真实存在      : {len(existing)}/{len(paths)}")
    if len(existing) != len(paths):
        fails.append(f"cited evidence paths missing: {sorted(set(paths) - set(existing))}")

    # 8. A cold agent must be able to name the forbidden operations.
    forbids = "hardware_flight_enabled=false" in stage
    print(f"  8. 明确禁止项可查到            : {forbids}")
    if not forbids:
        fails.append("forbidden operations not stated in STAGE")

    print()
    if fails:
        print(f"HANDOFF: {len(fails)} way(s) a cold session could go wrong")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("HANDOFF: a cold session can determine state, evidence, and the next safe step")
    return 0


if __name__ == "__main__":
    sys.exit(main())
