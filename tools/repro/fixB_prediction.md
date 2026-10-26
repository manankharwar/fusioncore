# Prediction before testing fix B alone, written 2026-10-18

The gate tests each fix against S = H*P*H' + R. The inflated R is part of what lets the
~19% of fixes through during recovery, because a larger S means a SMALLER Mahalanobis
distance. So shrinking R without also fixing the rejection behaviour should make recovery
WORSE, not better.

  Prediction: with a GNSS R ceiling applied and nothing else changed, the rejection rate
  inside the recovery windows on 2012-08-20 RISES above its current 81.2%, and the
  recovery share of squared error does not improve.

Supporting evidence already on file: a global ceiling of 10x across all sensors sent
2012-08-20 to 16318370.904 m, which is the same mechanism taken to its conclusion.

  Therefore: do NOT ship B without A. B alone tightens the gate on a filter that is
  already rejecting the fixes that would rescue it.
