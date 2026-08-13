# Position-vs-velocity reproduction

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
