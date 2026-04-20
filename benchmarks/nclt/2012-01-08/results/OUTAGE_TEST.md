# GPS Outage / Dead Reckoning Test

GPS cut from t=120.0s to t=165.0s (45s outage).

## Results

| Filter | Error at outage start (m) | Error at GPS return (m) | Notes |
|--------|--------------------------|-------------------------|-------|
| FusionCore | 5.51 | 155.06 | +149m over 45s |
| RL-EKF | 137.99 | 229.03 | +91m over 45s |
| RL-UKF | N/A | N/A | Diverged at t≈31s — unusable |

## Interpretation Caveat

The raw position errors above are **not SE(3)-aligned** — RL-EKF starts 138m from
truth while FusionCore starts at 5.5m, so the "net drift" numbers are not a clean
apples-to-apples dead-reckoning comparison. The correct takeaway from this test:

1. **RL-UKF is completely unusable** — it diverges numerically within 31 seconds, before the outage even begins.
2. **Both FC and RL-EKF drift substantially** during 45s of pure dead-reckoning — this is a physics ceiling from MEMS IMU + wheel encoders (see ATE benchmark for context).
3. A fair dead-reckoning comparison requires both filters to start from the same
   error state, which would require a controlled experiment (e.g., align both
   trajectories at outage-start, then measure relative drift). This is left as
   future work.

## Methodology

- Outage duration: 45s
- Both FC and RL-EKF run on IMU + wheel odometry only during outage
- Errors are raw 3D Euclidean distance to RTK GPS ground truth (no SE(3) alignment)
- RL-UKF trajectory excluded — diverged before outage window
