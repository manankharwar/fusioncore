# Position-vs-velocity reproduction

**Start with `split.cpp`.** It is the one that isolates the mechanism completely.

Two small programs that reproduce FusionCore's deepest known defect in about 30
seconds, with no ROS, no dataset and no benchmark harness.

```bash
./tools/repro/build.sh
/tmp/dr 60 1
```

## What they show

`dr.cpp` feeds a **perfect** encoder (1.0 m/s, dead straight) and a still IMU
(gravity only, zero rotation). Truth is trivially `x = 1.0 * t`.

```
   t        x      truth      vx      yaw     accX    biasAX
 10.0     2.50     10.00    1.000    -0.00   -2.839  -0.8045
 30.0    -1.47     30.00    1.000    -0.00   -0.580  -1.6755
 60.0  -114.06     60.00    1.000    -0.00
```

Velocity is perfect. Yaw is perfect. **Position travels backwards at ~3 m/s.**

`iso.cpp` starts with the velocity already in the state and lets you turn the
encoder updates off, to separate the predict step from the measurement updates.

## Why it happens

Yaw rate is structurally unobservable: the encoder measures `wz + b_ewz` and the
gyro measures `wz + b_gz`, which is two equations in three unknowns. Process
noise is added **per predict step rather than per second**, so at 100 Hz
`q_angular_vel = 0.1` injects 10 (rad/s)² every second against a gyro accurate to
0.005. Yaw covariance therefore grows without bound, reaching 1291 degrees while
the yaw *estimate* stays correct to 0.01 degrees. Position is predicted as
`R(q)·v·dt` averaged over the sigma points, so once those span multiple full
turns their displacement vectors cancel and position advances at a fraction of
the true speed.

The motion model itself is correct. The loss is in the weighted-mean
reconstruction, and it happens under both motion models.

## Which configurations are affected

`scope.cpp` answers that directly. Truth is 60.00 m after 60 s:

```
configuration                                          x (m)    err (m)   yaw sigma
encoder + 6-axis IMU     (wheels_indoor, f1tenth)    -114.06    174.06     348 deg  BROKEN
+ 2nd velocity source    (icp_indoor / ICP odom)     -144.09    204.09     419 deg  BROKEN
+ VSLAM pose             (vslam_imu)                   60.00      0.00       1 deg  OK
+ 9-axis IMU orientation (has_magnetometer: true)      59.99      0.01       1 deg  OK
```

**The rule: anything that observes YAW protects you; anything that only observes
velocity does not.** VSLAM measures full 6-DOF pose including yaw, and a 9-axis
IMU orientation does the same, so yaw sigma stays at 1 degree and position is
exact. Adding a second velocity source makes it slightly WORSE (204 m vs 174 m),
because it increases confidence in speed while leaving direction just as unknown.

Exposed as shipped: `wheels_indoor.yaml`, `f1tenth_indoor.yaml`, and
`icp_indoor.yaml` in either documented option (ICP is fed as a velocity source
either way). Protected: VSLAM, 9-axis IMU, dual antenna, or continuous GPS.

## Useful arguments

```
/tmp/dr  <seconds> <ground_constraint> [q_accel] [q_accel_bias] [alpha] [q_angular_vel]
/tmp/iso <encoder_updates> [alpha] [motion_model]
```

`alpha` is worth trying: at the shipped 0.1 the accel/accel-bias pair runs away
from zero input (accX -0.58, bias -1.68), and 0.5 fixes that completely (-0.001,
-0.008). That is the `Wm[0] = -99` conditioning hazard documented in `ukf.cpp`.

## Notes for whoever fixes this

It is **not a regression**: byte-identical output on 0.3.6 and on `c8b8b1f` from
19 May. GPS masks it, which is why it only shows up during blackouts.

A covariance cap on the quaternion diagonals was tried and **failed**: scaling a
row and column destroys the cross-covariance a measurement needs to correct yaw,
which broke `MagnetometerTest.UpdateMovesYawTowardMeasurement` and
`ClockSkewTest.SkewedSensorPairDoesNotDiverge`. Bounded covariance may still be
the right idea, but not implemented that way.

## Gauss-Markov bias experiment (2026-08-13)

`gauss_markov_experiment.patch` applies first-order decay to the bias states in
`motion_model.cpp`, so they are bounded processes rather than pure random walks.
It is a patch rather than shipped code because the version tested used
file-static globals to hold the time constants, which is not per-instance and
not thread-safe; FusionCore supports running two instances, so that would be a
defect. Re-apply it with `git apply` to reproduce the numbers below.

Truth after 60 s is x = 60.00, yaw = 0.00, accX = 0, biasAX = 0.

```
tau        x@60    yaw       accX     biasAX
OFF         8.83   143.54    -0.707   -0.0575
300 s       8.14   179.94    -0.009   -0.0553
60 s        5.70   177.38    -0.004    0.0042
10 s       21.20    -0.00     0.000   -0.0000
2 s        42.74    -0.00     0.000    0.0000
```

**What it establishes.** Constraining the bias states fixes the state estimates
completely: at tau = 2 s, yaw, accX and the accel bias are all exactly right, and
position improves 4.8x. So the bias random-walk really is what destroys heading.

**Why it is not the fix.** A physically realistic bias correlation time (300 s,
typical for MEMS) does nothing at all. Only an unphysical 2 s helps, and a 2 s
bias decay is not modelling a bias, it is suppressing the bias state. That points
at a stronger conclusion: FusionCore probably should not estimate these bias
states while they are unobservable, rather than decaying them aggressively.

**And it is not sufficient.** Even at tau = 2 s with yaw estimated perfectly,
position reaches only 42.74 of 60, i.e. 71%. Inverting `exp(-sigma^2/2) = 0.71`
gives about 47 degrees of residual yaw UNCERTAINTY. That last 29% is the UKF
sigma-point averaging itself: the position mean is the average of `R(q_i)*v*dt`
over sigma points that face different directions, so it shrinks even when the
best-estimate heading is exactly right. Fixing that is a filter-design decision
about how the mean is propagated, not a tuning question.


---

# THE MECHANISM, isolated by `split.cpp` (2026-08-13)

GPS for 20 s, then a blackout. Truth is 1.0 m/s dead straight the whole time.
`split.cpp` attributes every metre of position motion to either the predict step
or to a specific measurement update.

```
   t          x      truth   predict_dx   update_dx      vx
 20.0      19.96     20.00       19.78        0.18    0.9997   GPS on
 50.0      11.66     50.00       49.67      -38.00    0.9998   blackout
 80.0    -115.08     80.00       79.64     -194.72    1.0000   blackout

WHICH UPDATE MOVES POSITION (signed X, metres)
  IMU update        :   -199.80     <- the entire error
  encoder update    :      0.79
  ground constraint :     -0.04
  GNSS update       :      4.33
```

**Predict is correct to 0.45%** (79.64 m against a truth of 80.00). The motion
model, quaternion handling and velocity integration are all sound. The IMU
*update* subtracts 199.80 m. Encoder and ground constraint are innocent, and GPS
was pulling forward, which is the only reason this stayed hidden.

## Why an IMU update can move position at all

Position does not appear in `imu_measurement_function` at all. The IMU reaches
position purely through the covariance coupling, which is legitimate Kalman
behaviour when the innovation is honest. It is not honest here:

```
   t      imu_innov      AX      B_AX     P(X,AX)     P(X,VX)
 10.0       -0.1115   -2.714   -0.824      0.0006       0.000
 40.0       -0.9452   -0.228   -1.708      0.3014       0.000
 80.0       -1.7297    0.100   -1.987     -0.0514       0.000
```

1. `AX` and `B_AX` are not separately observable: the accelerometer measures
   their sum plus a gravity term, so nothing distinguishes "accelerating" from
   "biased". The bias wanders to **-1.99 m/s^2** on a perfectly still sensor.
2. Because the internal split is wrong, **the predicted specific force never
   matches the measured one**. The innovation grows to -1.73 m/s^2 against a
   sensor reading exactly gravity. The filter permanently disagrees with a
   perfect measurement.
3. `P(X,AX)` is non-zero, so that standing innovation is applied to position on
   every IMU update, 100 times a second. Ten thousand updates is the -199.80 m.

## The detail that explains the whole confusing signature

**`P(X,VX)` is 0.000 throughout.** Position is coupled to ACCELERATION but not to
VELOCITY. So the update can drag position anywhere while leaving `vx` at a
perfect 1.000. That is precisely the symptom that made this so hard to see from
the field data: velocity right, position wrong, on every run since 2026-08-03.

## What a fix must target

Not the covariance cap, not Q scaling, not the quaternion mean (all tried, all
reverted, see the memory file). The target is **the observability of the
acceleration/bias split**, and the fact that an unobservable split produces a
standing innovation that leaks into position through `P(X,AX)`.

Note this is a SECOND mechanism, distinct from the yaw/heading one measured
earlier the same day. Both are real. This one dominates when heading is fine.

## RESOLVED 2026-08-14: it was the sigma-point weights, and `alpha` is now 1.0

Both mechanisms above were downstream of one thing. At 23 states with `kappa = 0`
and the old `alpha = 0.1`, `lambda = alpha^2*n - n = -22.77`, so the CENTRE sigma
weight is **-99.0** against 46 outer weights of +2.17. They sum to 1, which is
formally correct but only for a tight cluster. Yaw is unobservable, the quaternion
sigma points spread, their forward displacements cancel, and the surviving centre
point (the one still pointing the right way) gets multiplied by -99. Position runs
backwards while velocity and yaw both read perfect.

`blackout.cpp` now sweeps `alpha` rather than `gnss_coast_q_bias_factor`, because
that factor turned out to be a dead knob: identical results at 100 and at 1, under
both the old and the new default.

```
blackout     alpha=0.1     alpha=0.5     alpha=1.0
 30 s          38.34 m       5.02 m        8.02 m
 60 s         195.08 m      16.84 m       20.13 m
120 s         646.34 m      44.68 m       82.60 m
240 s         607.32 m     105.00 m      206.88 m
```

At 120 s the old default put the estimate at x = -411 m against a truth of +120 m.

### Why 1.0 and not 0.5, since 0.5 scores better here

Measured on NCLT 2013-04-05 at 1x against identical ground truth, so this question
is settled with data rather than taste:

```
                        alpha=1.0    alpha=0.5
ATE 3D                    131.85 m     108.07 m     0.5 better by 18%
drift                     24.57 m/km   20.14 m/km   0.5 better by 18%
path length ratio          1.0092       0.9464      1.0 better, and this decides it
RL-EKF control            229.66 m     230.60 m     harness stable to ~1 m
achieved filter rate        88.5 Hz      83.9 Hz     a confound, see below
```

`alpha = 0.5` wins ATE but draws the path **5.4% short**, where 1.0 is within 0.9%
of true length. Short path length is the `exp(-sigma^2/2)` attenuation signature,
and it is expected: `Wm[0]` at 0.5 is still **-3.0**, so some cancellation remains.
Only `alpha = 1.0` gives `lambda = 0` and all 47 weights non-negative, which is the
standard unscaled UKF rather than a tuned point on a U-curve.

Path scale is also the metric that maps onto the field failure this hunt started
from, where the rover drew roughly 1.8x the distance it actually travelled. Trading
that away for ATE on a single sequence is the wrong trade.

Caveat worth keeping: the two runs differed in achieved rate as well as in alpha
(83.9 vs 88.5 Hz), and process noise is injected per predict call rather than per
second, so rate is not neutral. The 18% gap is well outside the ~1 m control noise
and is probably real, but it is not cleanly attributable to alpha alone.

### The remaining problem is heading, not weights

With `alpha = 1.0`, a 120 s blackout still drifts **yaw to -134 degrees from zero
rotation input**, with the encoder reporting `wz = 0` and `b_gz` wandering to
0.0104. Position now tracks velocity correctly, so the path is smooth: it simply
curves away. Both alpha values land near 108-132 m on 2013-04-05 while `c8b8b1f`
(19 May) measures 22.96 m, so a second regression of roughly 5x remains, and this
heading drift is the prime suspect. Note that the frozen `yaw = 0.00` and
`b_gz = 0.00000` seen in earlier runs of this repro were an ARTEFACT of the -99
weighting suppressing those states, not evidence that heading was healthy.
