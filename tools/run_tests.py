"""Run every case in tests_out/ and compare with expected.json.

  python tools/make_tests.py          # once: builds the synthetic cases
  python tools/run_tests.py [--render] [--only A_chuck_key]

PASS = every expected (parts, fit, joint) found and nothing extra. Outputs go to out/<case>/.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fitfinder.loader import load_part
from fitfinder.match import run_match
from fitfinder.output import render, summary_lines, write_csv, write_json


def grade(res, exp):
    got = sorted((tuple(sorted(m["parts"])), m["fit"], m["joint"]) for m in res["matches"])
    want = sorted((tuple(sorted(m["parts"])), m["fit"], m["joint"]) for m in exp["matches"])
    problems = []
    for w in want:
        if w in got:
            got.remove(w)
        else:
            problems.append(f"missing {w}")
    for g in got:
        problems.append(f"extra {g}")
    near = {n["part"] for n in res["near_misses"]}
    info = [f"near-miss {n} {'reported' if n in near else 'NOT reported'}" for n in exp.get("near_misses", [])]
    return not problems, problems, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests", default="tests_out")
    ap.add_argument("--out", default="out")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--only")
    a = ap.parse_args()
    cases = sorted(d for d in os.listdir(a.tests) if os.path.isfile(os.path.join(a.tests, d, "selection.json")))
    if a.only:
        cases = [c for c in cases if a.only in c]
    rows = []
    for c in cases:
        d = os.path.join(a.tests, c)
        sel = json.load(open(os.path.join(d, "selection.json")))["faces"]
        exp = json.load(open(os.path.join(d, "expected.json")))
        res = run_match(os.path.join(d, "part.step"), os.path.join(d, "assembly.step"), faces=sel,
                        log=lambda *x: None)
        od = os.path.join(a.out, c)
        os.makedirs(od, exist_ok=True)
        write_json(res, od)
        write_csv(res, od)
        lines = summary_lines(res)
        open(os.path.join(od, "summary.txt"), "w").write("\n".join(lines) + "\n")
        ok, problems, info = grade(res, exp)
        # the green faces stored in part.step must be the same faces as selection.json
        green = sorted(load_part(os.path.join(d, "part.step")).green_ids)
        if green != sorted(sel):
            ok = False
            problems.append(f"green faces {green} != selection {sorted(sel)}")
        if a.render:
            try:
                render(res, od)
            except Exception as e:
                info.append(f"render failed: {type(e).__name__}: {e}")
        rows.append((c, ok, len(res["matches"]), exp["count"], res["seconds"], problems, info, lines))
    print()
    print(f"{'case':<18} {'result':<6} {'found':>5} {'expected':>8} {'secs':>6}")
    for c, ok, n, e, s, problems, info, lines in rows:
        print(f"{c:<18} {'PASS' if ok else 'FAIL':<6} {n:>5} {e:>8} {s:>6}")
        for ln in lines[1:]:
            print("     " + ln)
        for p in problems + info:
            print("     - " + p)
    npass = sum(r[1] for r in rows)
    print(f"\n{npass}/{len(rows)} cases passed")
    return 0 if npass == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
