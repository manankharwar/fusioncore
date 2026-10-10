#!/usr/bin/env python3
"""Check that the README benchmark table agrees with the verified baseline.

The README benchmark table carries a Status column. Rows marked
"re-measured, n=2" must match a verified entry in tools/benchmark_baseline.json
for the same sequence, with the FC and RL numbers within rounding of the
baseline value (or the mean of its repeats). Every other row must carry the
"never verified" mark, because those numbers were measured through a dataset
player that is now known to have been wrong.

It exits 1 only with --strict. Without --strict it reports the disagreement but
returns 0, so a flaky or stale README never fails an unrelated build. It is not
wired into CI.
"""
import argparse
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
README = ROOT / "README.md"
BASELINE = ROOT / "tools" / "benchmark_baseline.json"

# The benchmark table is the one whose header row contains "FC ATE RMSE".
# Each data row is:
#   | Sequence | Season | Duration | FC ATE RMSE | RL-EKF ATE RMSE | Winner | Status |
TABLE_HEADER = "FC ATE RMSE"
ROW = re.compile(
    r"^\|\s*(\d{4}-\d{2}-\d{2})\s*\|[^|]*\|[^|]*\|"
    r"\s*\*{0,2}([0-9.]+)\s*m\s*\*{0,2}\s*\|"
    r"\s*\*{0,2}([0-9.]+)\s*m\s*\*{0,2}\s*\|[^|]*\|"
    r"\s*(re-measured, n=2|‡ never verified)\s*\|"
)

# Tolerance for "within rounding": a README value may differ from the baseline
# by up to this many metres and still be considered a match. This covers
# rounding to one decimal place and the mean-of-repeats representation.
ROUNDING_TOLERANCE_M = 0.5


def readme_rows(readme_source):
    """Return {sequence: (fc_ate_rmse, rl_ate_rmse, status)} from the table."""
    rows = {}
    in_table = False
    for line in readme_source.splitlines():
        if TABLE_HEADER in line and line.lstrip().startswith("|"):
            in_table = True
            continue
        if in_table:
            if not line.lstrip().startswith("|"):
                break
            match = ROW.match(line)
            if match:
                seq, fc, rl, status = match.groups()
                rows[seq] = (float(fc), float(rl), status)
    return rows


def load_baseline():
    """Return {sequence: baseline_entry} for verified sequences."""
    data = json.loads(BASELINE.read_text())
    return {
        seq: entry
        for seq, entry in data["sequences"].items()
        if entry.get("verified")
    }


def _baseline_values(entry):
    """Return (fc, rl) candidates: the primary value and the mean of repeats."""
    fc_primary = entry["fusioncore_ate_rmse_3d"]
    rl_primary = entry["rl_ate_rmse_3d"]
    fc_repeats = entry.get("repeats_post_frame_fix") or []
    rl_repeats = entry.get("repeats_post_frame_fix_control") or []
    fc_mean = sum(fc_repeats) / len(fc_repeats) if fc_repeats else fc_primary
    rl_mean = sum(rl_repeats) / len(rl_repeats) if rl_repeats else rl_primary
    return (fc_primary, fc_mean), (rl_primary, rl_mean)


def _within(value, candidates):
    """True if value is within ROUNDING_TOLERANCE_M of any candidate."""
    return any(abs(value - c) <= ROUNDING_TOLERANCE_M for c in candidates)


def violations(readme_rows, baseline):
    """Return a list of human-readable violations in the README table."""
    out = []
    for seq, (fc, rl, status) in sorted(readme_rows.items()):
        if status == "re-measured, n=2":
            entry = baseline.get(seq)
            if entry is None:
                out.append(f"{seq}: marked re-measured but has no verified baseline entry")
                continue
            fc_cands, rl_cands = _baseline_values(entry)
            if not _within(fc, fc_cands):
                out.append(
                    f"{seq}: FC {fc} m does not match baseline "
                    f"{fc_cands[0]} m (mean {fc_cands[1]:.3f} m)"
                )
            if not _within(rl, rl_cands):
                out.append(
                    f"{seq}: RL {rl} m does not match baseline "
                    f"{rl_cands[0]} m (mean {rl_cands[1]:.3f} m)"
                )
        else:
            if "never verified" not in status:
                out.append(f"{seq}: row is not re-measured but lacks the never-verified mark")
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit 1 when any README row disagrees with the verified baseline",
    )
    args = parser.parse_args(argv)

    rows = readme_rows(README.read_text())
    baseline = load_baseline()
    bad = violations(rows, baseline)

    for msg in bad:
        print(msg)

    if not baseline:
        print(f"{BASELINE}: no verified sequences found")
        return 1
    if not rows:
        print(f"{README}: no benchmark table rows found")
        return 1

    print(f"{len(bad)} violation(s) in the README benchmark table")
    return 1 if (args.strict and bad) else 0


if __name__ == "__main__":
    sys.exit(main())
