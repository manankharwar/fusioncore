#!/usr/bin/env python3
"""Generate the robot_localization re-run tables from score_nclt_suite.py output.

Usage: gen_rl_rerun_tables.py ~/nclt/eval12_rl/scores/all.txt
Every number is copied from the score file; nothing is computed except formatting.
"""
import re
import sys

SETS = {"2012-01-08": "held out", "2012-02-04": "dev", "2012-03-31": "held out",
        "2012-05-11": "held out", "2012-06-15": "dev", "2012-08-20": "dev",
        "2012-09-28": "dev", "2012-10-28": "held out", "2012-11-04": "held out",
        "2012-12-01": "held out", "2013-02-23": "held out", "2013-04-05": "dev"}
ROW = re.compile(r"^(\S+) (\S+)\s+(fix_times|full_traj)\s+n\s+(\d+)\s+rmse\s+([\d.]+)\s+median\s+([\d.]+)"
                 r"(?:\s+path\s+([\d.]+)\s+outrate\s+([\d.]+))?\s+\[(\S+) \| (\S+) \| \S+ \| cov ([\d.]+)%\]")

rows, versions = {}, set()
for line in open(sys.argv[1]):
    m = ROW.match(line)
    if m:
        seq, arm, pts, n, rmse, med, path, out, ver, pipe, cov = m.groups()
        rows[(seq, arm, pts)] = dict(n=n, rmse=rmse, med=med, path=path, out=out, pipe=pipe, cov=cov)
        versions.add(ver)
seqs = sorted({k[0] for k in rows})
assert len(versions) == 1, versions
print(f"Scorer `tools/score_nclt_suite.py` {versions.pop()}, {len(seqs)} sequences.\n")

arms = ["raw_gps", "rl_node", "fc_node", "fc_node_repeat", "fc_replay", "fc_reanchor"]
print("| Sequence | Set | Points scored | Raw GPS | robot_localization, node | FusionCore default, node "
      "| FusionCore default, node repeat | FusionCore default, replay | FusionCore re-anchoring, replay |")
print("|---|---|---|---|---|---|---|---|---|")
for s in seqs:
    cells = []
    for a in arms:
        r = rows.get((s, a, "fix_times"))
        cells.append(f"{r['rmse']} / {r['med']}" if r else "n/a")
    n = rows[(s, "raw_gps", "fix_times")]["n"]
    print(f"| {s} | {SETS[s]} | {n} | " + " | ".join(cells) + " |")

print("\n| Sequence | Arm | Pipeline | Poses scored | RMSE | Median | Path | Jitter | Coverage |")
print("|---|---|---|---|---|---|---|---|---|")
for s in seqs:
    for a in ["rl_node", "fc_node_repeat"]:
        r = rows.get((s, a, "full_traj"))
        if r:
            print(f"| {s} | {a} | {r['pipe']} | {r['n']} | {r['rmse']} | {r['med']} | {r['path']} | {r['out']} | {r['cov']}% |")

print("\nCoverage at fix times, rl_node: " + ", ".join(
    f"{s} {rows[(s, 'rl_node', 'fix_times')]['cov']}%" for s in seqs))
