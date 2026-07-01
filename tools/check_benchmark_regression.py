#!/usr/bin/env python3
"""Catch benchmark regressions before they ship.

The problem this solves: FusionCore has many coupled tuning knobs. A change made
to win one sequence (say a long GPS blackout) can quietly worsen another, and
without a check nobody notices for weeks. This compares fresh run metrics against
a recorded baseline (tools/benchmark_baseline.json) and fails loudly if
FusionCore's error grew by more than the allowed threshold.

Feed it metrics.json files produced by tools/evaluate.py --json:

    python3 tools/check_benchmark_regression.py \\
        benchmarks/nclt/2013-04-05/results_fast/metrics.json

    # or a whole set at once
    python3 tools/check_benchmark_regression.py \\
        --glob 'benchmarks/nclt/*/results_fast/metrics.json'

Exit code is non-zero if any sequence regressed, so it can gate CI or a release.
"""

import argparse
import glob as globmod
import json
import os
import sys

# metric we regression-gate on: XY ATE is what matters for a ground robot
METRIC = 'ate_rmse_xy'
BASELINE_KEY = 'fusioncore_ate_rmse_xy'


def load_metrics(path):
    with open(path) as f:
        data = json.load(f)
    fc = data.get('filters', {}).get('FusionCore')
    if fc is None or METRIC not in fc:
        raise ValueError(f'{path}: no FusionCore/{METRIC} (is this a tools/evaluate.py --json output?)')
    return data['sequence'], fc[METRIC]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('metrics', nargs='*', help='metrics.json file(s) from evaluate.py --json')
    ap.add_argument('--glob', help='glob pattern for metrics.json files')
    ap.add_argument('--baseline', default=os.path.join(os.path.dirname(__file__), 'benchmark_baseline.json'))
    ap.add_argument('--threshold', type=float, default=None,
                    help='allowed %% increase in XY ATE before it is a regression (default: baseline value)')
    args = ap.parse_args()

    with open(args.baseline) as f:
        base = json.load(f)
    seqs = base['sequences']
    threshold = args.threshold if args.threshold is not None else base.get('regression_threshold_pct', 10.0)

    paths = list(args.metrics)
    if args.glob:
        paths += sorted(globmod.glob(args.glob))
    if not paths:
        print('No metrics.json files given. Pass paths or --glob.', file=sys.stderr)
        return 2

    print(f'Regression gate: FusionCore XY ATE, threshold +{threshold:.1f}%  '
          f'(baseline {base.get("baseline_commit","?")}, recorded {base.get("recorded","?")})')
    print(f'{"sequence":<14}{"baseline":>10}{"current":>10}{"change":>10}   status')
    print('-' * 58)

    regressions, unknown = [], []
    for p in paths:
        try:
            seq, cur = load_metrics(p)
        except ValueError as e:
            print(f'  skip: {e}', file=sys.stderr)
            continue
        if seq not in seqs:
            print(f'{seq:<14}{"-":>10}{cur:>10.3f}{"-":>10}   NEW (no baseline)')
            unknown.append(seq)
            continue
        b = seqs[seq][BASELINE_KEY]
        pct = (cur - b) / b * 100.0
        if pct > threshold:
            status = 'REGRESSION'
            regressions.append((seq, b, cur, pct))
        elif pct < -threshold:
            status = 'improved'
        else:
            status = 'ok'
        verified = '' if seqs[seq].get('verified') else '  (baseline provisional)'
        print(f'{seq:<14}{b:>10.3f}{cur:>10.3f}{pct:>+9.1f}%   {status}{verified}')

    print('-' * 58)
    if regressions:
        print(f'FAIL: {len(regressions)} sequence(s) regressed beyond +{threshold:.1f}%:')
        for seq, b, cur, pct in regressions:
            print(f'  {seq}: {b:.3f} m -> {cur:.3f} m ({pct:+.1f}%)')
        return 1
    print('PASS: no FusionCore XY ATE regressions beyond threshold.')
    if unknown:
        print(f'Note: {len(unknown)} sequence(s) had no baseline entry (add them to '
              f'{os.path.basename(args.baseline)} once verified).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
