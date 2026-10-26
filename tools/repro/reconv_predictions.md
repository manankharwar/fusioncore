# Re-convergence branch predictions, written 2026-10-18 BEFORE running

Three branches for why recovery after a blackout is slow:

  A  returning fixes REJECTED                  -> fix is re-anchoring on consistent fixes
  B  accepted but barely weighted, R/nominal huge -> fix is bounding adaptive R on GNSS
  C  accepted at normal weight, error persists -> heading or velocity left the gap wrong

## 2012-08-20, reconv = 94.0% of squared error: predict BRANCH A

Reasoning from what is already on file rather than from the shape of the number. The
worst excursion on this sequence is +778.75 m in ONE step at t=3972.7, which is 14 s
after a 171 s blackout ends, and two raw fixes 0.07 s apart there differ by 825 m. So a
corrupt fix IS accepted. After it, the filter sits ~778 m from truth, which makes every
subsequent GOOD fix a huge innovation that fails chi2. The record also says
robot_localization takes the same hit and recovers within seconds while FusionCore stays
out for 400 s, which is the signature of the gate locking rather than of slow weighting.
The replay already reports 2121 rejections with CHI2_FAILED=1713 on this sequence.

## 2012-02-04, reconv = 94.5% and 44.5% of the whole run: predict BRANCH B

This sequence has the second worst raw GNSS of the set, 58.672 RMSE and 13.817 median.
Genuinely noisy fixes drive large innovations, adapt_R has no ceiling on the GNSS
channel, so R should inflate for real reasons and then stay inflated. That would weight
every good fix at nearly nothing and explain why it never recovers for nearly half the
run.

## 2012-09-28, reconv = 86.5%: predict BRANCH C

The odd one. Its raw GNSS is GOOD (8.983 RMSE, 6.289 median), its steady-coverage RMSE
is good (9.453), and its gaps are almost nothing (4 samples). Yet 12.3% of the run is
spent not having recovered. Good fixes arriving at normal weight against a filter that
keeps drifting back off is the heading signature: position gets pulled to the fix, then
walks away again because the heading it left the gap with is wrong.

## If these hold

A and B are the same family and both are gate/weighting problems with known fixes. C
would mean re-anchoring must reset HEADING too, from the GNSS track once the robot is
moving straight, not just position.
