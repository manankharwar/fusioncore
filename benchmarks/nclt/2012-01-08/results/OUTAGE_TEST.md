# GPS Outage / Dead Reckoning Test

GPS cut from t=120.0s to t=165.0s (45s outage).

## Result: Inconclusive for FC vs RL-EKF

This test cannot cleanly compare FC and RL-EKF dead-reckoning quality with
this experimental setup. Two approaches were tried, both have fatal flaws:

**Raw coordinate comparison**: fails because the GT frame (ENU from RTK origin)
and the filter odom frames differ in both origin and rotation. Numbers are dominated
by frame offset, not actual drift.

**SE(3)-aligned ATE on outage segment**: RL-EKF has a constant 138m global offset
before the outage (its overall ATE). SE(3) alignment on the short 45s segment
perfectly compensates for this constant offset, making RL-EKF appear to have
near-zero dead-reckoning error (0.99m): an artifact, not a real result.

A correct dead-reckoning test requires both filters to start the outage from the
same global error state. That needs a controlled setup (e.g., start the run with
GPS off, let both filters initialize identically, then compare).

## What IS Conclusive

| Finding | Result |
|---------|--------|
| RL-UKF during GPS outage | **Filter diverged at t≈31s: before outage even started** |
| RL-UKF overall | Numerically unstable, completely unusable |
| FC vs RL-EKF outage | Inconclusive with this setup: needs controlled experiment |

## Methodology Note

A fair dead-reckoning benchmark would:
1. Start both filters with GPS disabled from t=0 (no accumulated global error difference)
2. Run 60–120s of pure dead-reckoning
3. Compare ATE at the end using the same initial position

This is left as future work. The main benchmark (ATE over 600s with GPS) and the
spike rejection test are the reliable results for this dataset.
