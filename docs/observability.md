# Observability: drive this, or the filter cannot see it

Some quantities a sensor-fusion filter estimates are only observable if the robot
moves in a particular way. If it never does, the filter is not broken and the
estimate is not trustworthy either, and from the outside those two look identical.

This page says which quantities need which motion, and FusionCore reports it at
runtime so you do not have to guess.

## The report

`FusionCoreStatus::observability` carries a verdict per quantity and the manoeuvre
that would fix the rest. The node logs it on change:

```
Observability: heading UNOBSERVABLE (20.7 deg), yaw gyro bias UNOBSERVABLE
(sigma 0.2003 rad/s, r(WZ,B_GZ) -0.998), GNSS lever arm UNOBSERVABLE.
  TO FIX: drive two figure-eights, each loop 5 m or wider, at 0.5 m/s or more...
```

Four verdicts, and the difference between the middle two is the one that matters:

| | meaning |
|---|---|
| `UNKNOWN` | not enough data yet to judge |
| `UNOBSERVABLE` | **nothing is constraining this. Waiting does not help; driving does** |
| `MARGINAL` | constrained, but not tightly enough to rely on |
| `OBSERVABLE` | fine |

## The manoeuvres

### Stop and wait: a few seconds, wheels still

**This is the only thing that makes the yaw gyro bias observable, and it is the first
thing the filter will ask for.** Two seconds is enough, ten is comfortable.

Measured on a true rate of 0.6 rad/s with a true bias of 0.05 rad/s:

```
  no stop     WZ 0.50  B_GZ 0.15   yaw  84.9% of truth   r(WZ,B_GZ) = -0.998
  10 s stop   WZ 0.60  B_GZ 0.05   yaw 100.1% of truth   r(WZ,B_GZ) = -0.330
```

Why stopping and nothing else: the gyro reports `WZ + B_GZ`, so any
`(WZ + d, B_GZ - d)` fits it identically and the pair is in the null space of the
measurement Jacobian. ZUPT is a **different measurement**, `z = WZ` with no bias
term, so while stationary `WZ` is pinned to zero and the gyro reading becomes a
direct observation of the bias.

### Figure-eight: two loops, each 5 m or wider, at 0.5 m/s or more, stopping at each crossing

The straights give GNSS track heading. **The turns do not separate the yaw rate from
its bias**, and an earlier version of this page said they did. That was wrong and the
correction is worth stating, because the intuition behind it is a common one:

```
  spin one way only           B_GZ error +0.1000   r = -0.9997
  figure-eight, never stops   B_GZ error +0.1000   r = -0.9978
  figure-eight WITH stops     B_GZ error  0.0000   r = -0.1069
```

A figure-eight driven without stopping leaves the bias exactly as wrong as driving in
a circle does. The gyro reads `WZ + B_GZ` at every instant whichever way the robot is
turning, so no path through space adds an independent equation. The **stops** at the
crossings are what do the work.

### Drive straight: 10 m or more, over 0.2 m/s, no turning

GNSS track heading is the bearing between two fixes, so its uncertainty is roughly
`GNSS sigma / distance travelled`. At a 3 m sigma, 5 m of travel is more than a
radian of heading error and 7.5 m is where it first becomes usable. This is why
`heading_validated` flipping at 5 m does not mean heading is known.

### Turn in place: 60 degrees or more, at 1 rad/s or more

Feeds the rotation heading bootstrap, which recovers absolute heading from the arc an
offset GNSS antenna sweeps. Needs a non-zero lever arm and an RTK-grade receiver.

**It must be brisk, and that is measured rather than asserted.** Yaw 1-sigma grows
about 0.2 rad/s while turning with an IMU feeding and about 0.3 rad/s on encoder
alone. Enough rotation and little enough elapsed time pull against each other, so a
slow turn loses track of its own rotation before it has swept enough arc and the
bootstrap correctly declines. A 2.0 rad/s turn bootstraps; the same angle taken at
0.5 rad/s does not.

## What each verdict is computed from

### Heading

From `P` via the quaternion-to-yaw Jacobian, plus whether anything has ever actually
observed it. **A tight prior is not an observation.** At init the heading sigma reads
0.1 degrees and nothing has constrained heading at all, so the report says
`UNOBSERVABLE`. Reporting `OBSERVABLE` off a tight prior is how a user ends up
trusting a heading that never fused.

Thresholds: `OBSERVABLE` under 10 degrees, `MARGINAL` under 30, `UNOBSERVABLE` above.
Measured on the rover, `heading_validated` once went true at 5.04 m carrying 48.6
degrees, and a whole run reported validated at a yaw 1-sigma of 101 degrees.

### Yaw gyro bias

This is issue #150, read straight off `P`. A gyro alone cannot separate a rate from a
bias on that rate: they enter the measurement identically, so the pair is free to
slide along `WZ + B_GZ = const` at no measurement cost, and yaw integrates `WZ` alone
and inherits the error.

The indicator is the correlation `r(WZ, B_GZ)`, because it approaches 1 in magnitude
exactly when the pair is free. Measured on a 1.8 s in-place spin at a true 0.6 rad/s:

```
  WZ = 0.4800   B_GZ = 0.1200   WZ + B_GZ = 0.6000 exactly
  yaw = 0.8808  against a true 1.08  =  81.6%
  r(WZ, B_GZ) = -0.998
```

The sum is right to four decimals while neither half is. The correlation is
**negative** because the pair trades against each other to hold that sum.

Two separate ways for this to read `UNOBSERVABLE`, and they need different responses:

- **high correlation**: the pair is structurally free. Driving straight will not fix
  it, which is why the figure-eight exists.
- **large sigma with low correlation**: nothing has constrained it yet. This is the
  state at init, where `P` is diagonal so the correlation is 0 while the bias sigma is
  0.316 rad/s, which is 18 deg/s. Uncorrelated and unconstrained are different things.

Thresholds: correlation limit 0.95, bias sigma 0.02 rad/s for `OBSERVABLE` and 0.05
for `MARGINAL`.

### GNSS lever arm

Not estimated, configured. What varies is whether the filter is allowed to **use** it,
which is gated on heading uncertainty via `gnss.lever_arm_max_heading_sigma_deg`
(default 20 degrees), because rotating a lever arm by a heading you do not know adds
more position error than it removes. `gnss.apply_lever_arm_pre_heading` overrides the
gate deliberately, and the report respects that rather than contradicting you.

## Why this page exists

Three open issues in this project were observability problems that looked like filter
problems for weeks:

- **#150**: the yaw rate and its bias sliding along a direction nothing measures.
- **Heading never fuses**: 0 bearings in 298 fixes, swept, with no config that fixes it.
- **The lever arm**: configured, resolved from TF, and never once applied on a run,
  because heading validated at 5.3 m with 25.3 degrees against a 20 degree limit.

In each the filter behaved exactly as specified. Nothing told anyone that the quantity
was unconstrained, so the conclusion was that the filter was wrong. The thresholds
above are constants in `fusioncore.hpp` and a test asserts this page's numbers against
them, so the two cannot drift apart.
