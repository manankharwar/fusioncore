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

It also refuses to grade a run whose CONTROL moved. robot_localization is played
the same bag in the same launch, so its score is a property of the harness and the
machine, not of any FusionCore change. When it moves, the run measured the machine.

That is not hypothetical. On 2026-10-05 a 2013-04-05 run came back at 51.454 m XY
against a 200.910 m baseline and this gate called it a 74.4% IMPROVEMENT. The run
was worthless: the control had moved -4.4% against an established band under 1%,
after 20 `rl_ekf: Failed to meet update rate` entries with a worst case of 0.266 s
against a 0.05 s budget. The playback took 75 minutes for 4052 s of data. A gate
that hands you a 74% win off a starved machine is worse than no gate, because the
number is the kind you want to believe.
"""

import argparse
import glob as globmod
import json
import os
import sys

# metric we regression-gate on: XY ATE is what matters for a ground robot
METRIC = 'ate_rmse_xy'
BASELINE_KEY = 'fusioncore_ate_rmse_xy'

# The control. robot_localization rides the same playback, so a move in ITS score
# is the harness or the machine talking, never a FusionCore change. 1% is not a
# guess: measured spreads on the control are 0.47% on 2013-04-05, 0.00% on
# 2012-08-20 (identical to two decimals across runs, which is what ruled out the
# input that day) and 1.9% on 2012-06-15. A move past 1% means the run is void.
CONTROL_METRIC = 'ate_rmse_3d'
CONTROL_BASELINE_KEY = 'rl_ate_rmse_3d'
CONTROL_TOLERANCE_PCT = 1.0


def load_metrics(path):
    with open(path) as f:
        data = json.load(f)
    fc = data.get('filters', {}).get('FusionCore')
    if fc is None or METRIC not in fc:
        raise ValueError(f'{path}: no FusionCore/{METRIC} (is this a tools/evaluate.py --json output?)')
    rl = data.get('filters', {}).get('RL-EKF') or {}
    return data['sequence'], fc[METRIC], rl.get(CONTROL_METRIC)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('metrics', nargs='*', help='metrics.json file(s) from evaluate.py --json')
    ap.add_argument('--glob', help='glob pattern for metrics.json files')
    ap.add_argument('--baseline', default=os.path.join(os.path.dirname(__file__), 'benchmark_baseline.json'))
    ap.add_argument('--threshold', type=float, default=None,
                    help='allowed %% increase in XY ATE before it is a regression (default: baseline value)')
    ap.add_argument('--control-tolerance', type=float, default=CONTROL_TOLERANCE_PCT,
                    help=f'%% the RL-EKF control may move before the run is void '
                         f'(default: {CONTROL_TOLERANCE_PCT})')
    args = ap.parse_args()

    with open(args.baseline) as f:
        base = json.load(f)
    seqs = base['sequences']
    threshold = args.threshold if args.threshold is not None else base.get('regression_threshold_pct', 10.0)

    # A baseline entry has to be self-consistent before it can judge anything.
    # ATE RMSE 3D = sqrt(mean(dx^2+dy^2+dz^2)) and XY = sqrt(mean(dx^2+dy^2)), so 3D
    # is >= XY always: adding a non-negative dz^2 cannot shrink it. Two of the three
    # entries in this repo's own baseline violated that (2012-08-20 at 3D 116.444 vs
    # XY 121.669, and 2013-04-05 at 3D 189.700 vs XY 200.910), which means those two
    # numbers came from different runs or were transposed. The gate gates on XY, so a
    # transposed XY silently shifts every regression percentage measured against it.
    bad = []
    for seq, v in seqs.items():
        a, b = v.get('fusioncore_ate_rmse_3d'), v.get(BASELINE_KEY)
        if a is not None and b is not None and a < b:
            bad.append((seq, a, b))
    if bad:
        print(f'FAIL: {len(bad)} baseline entr(ies) are internally impossible, '
              f'3D ATE is below XY ATE:')
        for seq, a, b in bad:
            print(f'  {seq}: 3D {a:.3f} m < XY {b:.3f} m')
        print('  3D cannot be smaller than XY. These came from different runs or were '
              'transposed, so any percentage measured against them is meaningless. '
              'Re-measure before using this baseline to gate anything.')
        return 2

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

    regressions, unknown, voids, unchecked = [], [], [], []
    for p in paths:
        try:
            seq, cur, ctrl = load_metrics(p)
        except ValueError as e:
            print(f'  skip: {e}', file=sys.stderr)
            continue
        if seq not in seqs:
            print(f'{seq:<14}{"-":>10}{cur:>10.3f}{"-":>10}   NEW (no baseline)')
            unknown.append(seq)
            continue

        # The control first. If it moved, nothing else on this row can be read,
        # so do not grade FusionCore at all rather than grading it and hedging.
        ctrl_base = seqs[seq].get(CONTROL_BASELINE_KEY)
        if ctrl is None or ctrl_base in (None, 0):
            unchecked.append(seq)
        else:
            ctrl_pct = (ctrl - ctrl_base) / ctrl_base * 100.0
            if abs(ctrl_pct) > args.control_tolerance:
                voids.append((seq, ctrl_base, ctrl, ctrl_pct))
                print(f'{seq:<14}{"-":>10}{"-":>10}{"-":>10}   VOID '
                      f'(control {ctrl_base:.3f} -> {ctrl:.3f}, {ctrl_pct:+.2f}%)')
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
        if seq in unchecked:
            verified += '  (control UNCHECKED)'
        print(f'{seq:<14}{b:>10.3f}{cur:>10.3f}{pct:>+9.1f}%   {status}{verified}')

    print('-' * 58)
    if voids:
        print(f'FAIL: {len(voids)} sequence(s) VOID, the control moved beyond '
              f'{args.control_tolerance:.1f}% so the run measured the machine:')
        for seq, cb, cc, cpct in voids:
            print(f'  {seq}: control {cb:.3f} m -> {cc:.3f} m ({cpct:+.2f}%)')
        print('  Re-run these on an idle machine. Do not read the FusionCore number.')
    if regressions:
        print(f'FAIL: {len(regressions)} sequence(s) regressed beyond +{threshold:.1f}%:')
        for seq, b, cur, pct in regressions:
            print(f'  {seq}: {b:.3f} m -> {cur:.3f} m ({pct:+.1f}%)')
    if voids or regressions:
        return 1
    print('PASS: no FusionCore XY ATE regressions beyond threshold.')
    if unchecked:
        print(f'Note: {len(unchecked)} sequence(s) had no control to check '
              f'(needs RL-EKF/{CONTROL_METRIC} in metrics.json and '
              f'{CONTROL_BASELINE_KEY} in the baseline). The run was graded anyway, '
              f'so it could be a starved machine and nothing here would say so.')
    if unknown:
        print(f'Note: {len(unknown)} sequence(s) had no baseline entry (add them to '
              f'{os.path.basename(args.baseline)} once verified).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
