# Platform profiles

A profile is a measured answer to one question: **what can this platform actually
observe?** Estimating a state you cannot observe is worse than not estimating it, and
on NCLT that is worth up to 61%.

These are parameter bundles, not code. Copy the block into your own config, or load
the file and override individual keys after it. Nothing here changes a library
default, because the right answer genuinely differs by platform.

## Why this exists

FusionCore carries 23 states. The textbook GNSS/INS filter carries 15. The extra
eight are where three of its open defects live:

| group | states | problem |
|---|---|---|
| angular velocity `WX,WY,WZ` | 3 | gyro measures `WZ + B_GZ`: one equation, two unknowns |
| acceleration `AX,AY,AZ` | 3 | accel measures `AX + B_AX + gravity`: one equation, three unknowns |
| encoder yaw bias `B_EWZ` | 1 | a third unknown on an equation that already had two |

Measured on a 1.8 s in-place spin in `test_observability.cpp`: `r(WZ, B_GZ) = -0.998`. On
NCLT with encoders the same correlation measured -0.63 to -0.04, so real driving locks the
pair far less than that synthetic does. Where it is locked, the pair slides along an unobservable
line at zero measurement cost, and anything integrating `WZ` alone inherits the error.
That is the 81.6% yaw and 80.7% position figures in issue #150.

The literature says the same thing independently. Accelerometer bias is completely
unobservable in straight motion, and for land vehicles the azimuth and its gyro bias
are unobservable in straight motion and under constant acceleration.

## What the profiles are worth

Measured on NCLT at 1x through the ROS node on 2026-10-17, with the dataset player as it
was before the 2026-10-25 tilt fix, so these show the profile's direction rather than
current figures. The control column is robot_localization on the same run: it consumes the
same input and runs none of this code, so a control that holds still is what makes the
FusionCore change attributable. (The win/loss columns that used to sit here compared
FusionCore with robot_localization through that player and are withdrawn; that comparison
is being re-run.)

| sequence | shipped | `ground_robot` | control moved | change |
|---|---|---|---|---|
| 2012-06-15 | 82.011 | **32.149** | -0.05% | **-60.80%** |
| 2013-04-05 | 14.058 | **9.193** | +0.50% | **-34.61%** |
| 2012-08-20 | 155.618 | 157.591 | +0.04% | +1.27% |
| 2012-09-28 | 18.594 | **8.872** | +11.23% | **-52.29%** |

2012-08-20 is about 10x its own 0.13% spread worse, which is small and real; the profile
is not uniformly better and this file is not going to pretend otherwise.

2012-09-28 is the weakest of the four and is listed with its control move rather than
without it. An 11.23% control move would normally void a comparison; this sequence has
a measured control-side spread of 11.05%, so the move is consistent with its own noise
instead of with a changed input. The other three rows need no such argument.

**From 0.5.0 both ground-robot profiles also turn on `gnss.reanchor_enabled`.** None of the
rows above include it. It was measured on its own: lower RMSE on 10 of 12 NCLT sequences
and higher on none, but higher median on 4 and higher output-rate jitter on 3, so it failed
its ship rule on those and is on here as a stated decision. Details in docs/configuration.md
under "Recovering after GPS comes back". Set it false to opt out.

The same bundles are reachable as one parameter, `platform.profile: ground_robot` or
`ground_robot_flat`, documented in `docs/configuration.md`. `tools/test_profiles.py`
pins the node's table to these files so the two forms cannot drift apart and hand two
users two different filters.

## Choosing one

- **`ground_robot.yaml`** needs wheel odometry. It zeroes the process noise and prior of
  the acceleration and bias states, which is only safe because the encoder constrains yaw
  rate directly. **Measured 2026-10-22: "frozen" means slowed, not held.** The profile zeroes the bias process noises and priors, but the UKF adds a 1e-6 identity to P when it builds sigma points, and a zero-variance state triggers its identity-shift repair, so the biases still drift. On NCLT 2013-04-05 the gyro bias reached a median sigma of 0.053 rad/s and roamed up to about 0.18 rad/s. The measured improvements are what the profile does as shipped; the word "freeze" overstates the mechanism. Not yet fixed. Tested
  against a 20 deg/s gyro bias and a 29.7% encoder scale error in simulation and it
  held, but do not use it on a platform without encoders.
- **`ground_robot_flat.yaml`** adds a Z-position constraint for campus, warehouse,
  parking-lot and field terrain. Do not use it where the robot genuinely climbs.
- **No profile** is the right choice for drones, boats, and anything that leaves the
  ground plane. The library defaults stay general on purpose.

## Measured on hardware, 2026-10-17: it does not hurt, and it does not help here

Three rover bags, replayed offline through `fusioncore_core` with `hardware/replay.cpp`
(not the live node, so these closures are not the field numbers). The rover's own
frozen config, with the five keys the profile sets removed so the profile could supply
them. Loop closure in metres, 10 s stillness window:

| bag | driven | `general` | `ground_robot` | `ground_robot_flat` |
|---|---|---|---|---|
| 20260927_1257 | 106.5 m | 1.10 | 1.08 | 1.09 |
| 20260927_1307 | 104.9 m | 0.34 | 0.30 | 0.30 |
| 20260926_1755 | 103.8 m | 0.78 | 0.78 | 0.79 |

Every move is far smaller than the receiver's own 2.6 m sigma, so none of it is a
measurement of accuracy. **What that does establish is that the profile is safe on real
hardware**, which was the open question: freezing seven states did not break anything
on three real bags.

The same three logs with a 120 s GNSS blackout cut into them, every other sensor
untouched, to reach the regime the NCLT gains came from:

| bag | metric | `general` | `ground_robot` | change |
|---|---|---|---|---|
| 20260927_1257 | closure | 1.15 | 1.09 | -5% |
| 20260927_1307 | closure | 0.34 | 0.31 | -9% |
| 20260926_1755 | closure | 0.77 | 0.85 | +10% |
| 20260927_1257 | path length | 198.1 | 189.7 | **-4.3%** |
| 20260927_1307 | path length | 192.2 | 170.9 | **-11.1%** |
| 20260926_1755 | path length | 180.7 | 175.3 | **-3.0%** |

Closure is mixed and inside the noise. Path length falls on 3 of 3, which is the same
direction as the NCLT result (less excess motion), and path length is the metric that
can see a filter wandering and coming back.

**Why the NCLT-scale gain does not appear, stated as a hypothesis and not a result:**
these runs are about 105 m over 350 to 500 s with continuous 1 Hz fixes, so a 120 s
blackout covers roughly 30 m of driving. NCLT 2012-06-15 is 7.5 km with a 461 s
blackout. The unobservable pairs need time and distance with nothing pinning them, and
this course does not provide either. Testing that properly needs a longer course or a
real canopy, not another replay.

## A trap worth knowing before you set this

The rover's own config spells out all four process noises at exactly their library
defaults:

```yaml
    ukf.q_acceleration: 1.0
    ukf.q_gyro_bias: 1.0e-5
    ukf.q_accel_bias: 1.0e-5
    ukf.q_encoder_wz_bias: 1.0e-7
```

A profile only supplies defaults, so on that config it would correctly keep all four
and log `4 kept`, applying almost nothing. Writing a value out, even the value the node
already uses, is indistinguishable from meaning it. **If you adopt a profile, delete
the lines you are handing over to it**, and read the node's log to confirm what it
applied rather than assuming.

## Validated across five sequences in replay, 2026-10-18

The table above is from node runs on three sequences. Since then every sequence with a
COMPLETE exported input bag was run through the deterministic core replay, shipped NCLT
config, only the seven profile keys changed between arms:

Provenance of every number below: **pipeline** replay, **config** shipped NCLT plus the
seven profile keys, **scoring points** full trajectory, **coverage** 99 to 100% of each
sequence.

| sequence | free RMSE | frozen RMSE | free median | frozen median | change in RMSE |
|---|---|---|---|---|---|
| 2012-02-04 | 142.792 | **142.311** | **34.495** | 34.582 | -0.3% |
| 2012-06-15 | 78.139 | **64.295** | 29.978 | **18.164** | **-17.7%** |
| 2012-08-20 | 171.393 | **145.765** | 35.436 | **31.618** | **-15.0%** |
| 2012-09-28 | 23.638 | **22.501** | 7.235 | **7.075** | -4.8% |
| 2013-04-05 | 16.677 | **15.355** | **6.960** | 7.001 | -7.9% |

**FIVE valid bags, not six.** 2012-01-08 was withdrawn on 2026-10-18: its exported input
log covers only 66% of the sequence, 3740 s against 5640 s, because it came from a
benchmark run that was stopped partway through. No complete input bag exists for it, so
it cannot be re-measured without a new node run.

**Better on RMSE on all five.** Better on median on three of five, and the two it loses
are by 0.25% and 0.6%. Path length ratio stays between 0.888 and 1.180 everywhere, so
none of this is a degenerate trajectory flattering the alignment.

n=1 per cell, and 2012-02-04 carries a 14.80% spread of its own, so read the direction
across five sequences rather than any single row.

## Before you trust it on your robot

Run `tools/bag_report.py` on one of your bags first. If it says your IMU and wheels
disagree about turn direction, or that your bag contains no manoeuvre that makes
heading observable, fix that before changing any of this. A profile cannot rescue a
sensor that is wired backwards.
