# GPS Spike Rejection Test

A single GPS fix was corrupted to +500m NE (~707m diagonal) at t=120.0s.
Measurement window: 30s before and 30s after the spike.

## Results

| Filter | Baseline error (m) | Max deviation after spike (m) | Position jump from spike (m) | Outcome |
|--------|--------------------|-----------------------------|------------------------------|---------|
| FusionCore | 5.21 | 6.25 | +1.04 | **REJECTED**: Mahalanobis gate blocked spike |
| RL-EKF | 105.48 | 198.97 | +93.49 | **JUMPED**: accepted corrupted fix |
| RL-UKF | N/A | N/A | N/A | **DIVERGED** at t≈31s: filter already invalid |

## Key Finding

FusionCore's position moved only **1m** in response to a **707m** corrupted GPS fix.
The Mahalanobis distance gating (χ² test at 99.9th percentile) detected the spike
as a 3σ+ outlier and rejected it in a single update step.

RL-EKF has no outlier rejection: it accepted the corrupted fix and jumped **93m**,
then slowly recovered as subsequent valid GPS fixes pulled it back.

## Methodology

- One GPS NavSatFix message was corrupted: lat += 500m / lon += 500m (≈707m NE)
- Baseline: mean 3D position error vs RTK GPS ground truth in 30s before spike
- Max deviation: peak 3D error in 30s after spike
- Position jump = max_deviation − baseline
- RL-UKF excluded: diverged numerically before spike window
