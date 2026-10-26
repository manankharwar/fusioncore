# Coast variant: predictions written before running (2026-10-22)

The variant: during a GNSS coast, take speed only from the wheels and heading only
from the gyro. Built as two coast-only R multipliers, default 1.0:

- `gnss_coast_imu_accel_scale` (core field, removed): multiplies R_imu on the three accelerometer rows.
  Large means the accelerometer stops steering AX, so VX follows the encoder.
- `gnss_coast_encoder_wz_scale` (core field): multiplies R_encoder on the WZ row. Large means
  heading rate comes from the gyro. Must be paired with `gnss.coast_imu_wz_scale: 1`,
  because the NCLT config sets 500 there, which sidelines the gyro in coast.

Arms on NCLT, all on the shipped config with re-anchoring off so the coast numbers
are the recorded ones (425.3 m and 153.6 m reproduced from arm_*_baseline.tum):

    base     as shipped: accel in, heading mostly from the encoder (imu_wz_scale 500)
    S        speed from wheels only: accel_scale 1e6
    H        heading from gyro only: encoder_wz_scale 1e6, imu_wz_scale 1
    SH       both, the variant as specified

## What the existing numbers already bound

The like-for-like dead-reckoning row for the shipped coast is `enc @filter yaw`
(encoder speed, encoder heading), because imu_wz_scale 500 gives heading to the
encoder. The coast loses to it by about 10% on both gaps:

    2012-06-15   coast 425.3   enc 385.9   gyro 411.1
    2013-04-05   coast 153.6   enc 140.2   gyro 131.6

The variant as specified corresponds to the `gyro` row.

## Predictions

P1. S (accel out) moves the coast toward the `enc` row on both gaps, because if
    accelerometer integration into VX is what adds drift, removing it should recover
    most of the ~10% gap. If S does not move the coast, accel is NOT the suspect and
    the remaining difference is the sigma-point mean or the inflated process noise.
P2. SH lands near the `gyro` row: better than base on 2013-04-05, WORSE than base on
    2012-06-15. So the variant as specified is not a uniform improvement, and the
    heading source decides more than the speed source.
P3. Nothing here gets below ~89% of distance travelled on either gap. Every method
    starts from the filter's own heading at window entry, and that shared heading
    error dominates. This experiment cannot fix that and should not be read as if it
    could.
P4. Synthetic first: with an accelerometer bias the filter cannot fully separate
    from AX, S beats base on drift with known truth. If S does not beat base even
    there, the accel hypothesis is falsified before touching NCLT.
