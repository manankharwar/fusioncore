# Is your filter's covariance honest?

Most localization debugging asks whether the estimate looks right. This asks a
different question: does the filter's own reported confidence match the errors it
is actually making? A filter can track well and still be lying about its
uncertainty, and when it is, everything downstream that reads the covariance
(outlier gates, Nav2, your own health checks) is working from a wrong number.

You do not need ground truth to check this, which is what makes it practical on a
real robot.

## The measure

FusionCore already records the Normalized Innovation Squared for every GNSS fix
on `/fusion/debug/gnss_status`, as `mahalanobis_sq`:

```
NIS = nu^T S^-1 nu        nu = fix minus prediction
                          S  = H P H^T + R
```

`nu` is how far the fix landed from where the filter expected it. `S` is how far
the filter said it might land. Dividing one by the other gives a number whose
average, for an honest filter, is the dimension of the measurement: **3** for a
GNSS x/y/z position.

| NIS | meaning |
|-----|---------|
| about 3 | the covariance is honest |
| well above 3 | overconfident: `S` too small, gain too low to correct the filter's own error, and the chi2 gate starts rejecting good fixes |
| well below 3 | underconfident: `S` too large, gain too high so the filter chases measurement noise, and the chi2 gate loses its ability to reject anything |

## Running it

Record a run with the debug topic included, then:

```bash
python3 tools/nis_from_bag.py /path/to/your_bag
```

It prints the NIS distribution, the rejection reasons, and the filter's own
position and heading uncertainty, with a plain reading of what the numbers mean.

## Reading the "inert gate" warning

The chi2 outlier gate is calibrated on the assumption that the filter is
consistent. At the default `outlier_threshold_gnss` of 16.27, the 99.9th
percentile of chi-squared with 3 degrees of freedom, a fix is rejected when its
NIS exceeds that.

That threshold only means "one in a thousand" while NIS actually averages 3. If
your run's NIS averages 0.03, the threshold sits roughly 500 times above the
typical value instead of 5 times above it, so the gate has far less sensitivity
than its nominal design point and a moderate multipath excursion will pass
straight through. The tool says so explicitly when the largest NIS in a whole run
never reaches the threshold.

## What this measures on real hardware today

Measured across three field runs on the development rover (a u-blox receiver, a
BNO085 IMU in UART-RVC mode, and wheel encoders):

| run | fixes | NIS median | filter position 1-sigma | heading 1-sigma |
|-----|-------|-----------|------------------------|-----------------|
| 2026-07-29 | 83 | 0.02 | 31.25 m | 131 deg |
| 2026-07-30 | 129 | 0.03 | 2.68 m | 57 deg |
| 2026-08-03 | 499 | 0.03 | 6.61 m | 143 deg |

All three are underconfident by roughly two orders of magnitude, and the middle
run is the useful control: every fix was accepted, no quality gate was firing,
and the result is the same. So this is a property of the filter on this hardware
and not a side effect of fixes being thrown away.

Two separate things push NIS down, and it is worth telling them apart because
only one of them is a filter problem.

### The receiver's covariance describes absolute accuracy, not innovation size

This is usually the larger effect and it surprises people. On the 2026-08-03 run
the receiver declared a horizontal 1-sigma of 4.84 m. If the fixes really carried
white noise of that size, consecutive fixes would disagree wildly: the median
second difference would be about 14 m. The measured value was 0.087 m, and the
largest anywhere in a 500 second run was 1.09 m. Consecutive fixes agree with
each other about 160 times more closely than the declared covariance implies.

Both statements are true at once. The receiver smooths internally, so its output
is strongly time-correlated: the absolute error really is metres, dominated by
multipath and ionosphere, while the fix-to-fix consistency is centimetres. A
Kalman filter assumes measurement noise is white. Hand it the declared number as
`R` and `S` becomes far larger than any innovation it will ever see, so NIS
collapses regardless of how well the filter is working.

**Do not respond by shrinking the covariance.** If you set `R` to the centimetre
scatter, the filter will track the receiver's multipath bias rigidly and report
centimetre confidence in a position that is metres off, which is the dangerous
direction. The correct treatment for a time-correlated measurement is to model
the correlation, for example by estimating a slowly varying GNSS bias, rather
than by relabelling it as white noise.

`tools/nis_from_bag.py` measures this for you and says so when it finds it. Note
the second difference also contains the robot's real acceleration, so it bounds
the measurement noise from above: the true white component is smaller still.

### Unobservable yaw inflates P

The second effect is the heading column above. With a 6-axis IMU, wheel encoders
and GNSS position, yaw is not observable: the gyro measures `wz + b_gz` and the
encoder measures `wz + b_ewz`, two equations for three unknowns. GPS track
heading only helps while the robot moves in a straight line fast enough for the
displacement bearing to beat the position noise. That uncertainty propagates into
the position covariance and inflates `S` further.

In simulation, where the measurement noise really is white and the receiver
effect is absent, this part is clearly visible. As GNSS noise grows, NEES without
an absolute heading source drifts from 2.0 to 7.0, meaning the filter becomes
steadily more overconfident. With a magnetometer supplying absolute yaw it stays
near 1.9 across the same range, and the heading 2-sigma drops from 8.8 to 25.5
degrees down to under one degree. That is measured in
`fusioncore_core/tests/test_consistency.cpp`.

So an absolute heading source (magnetometer or dual-antenna) is the fix for the
second effect, and it is worth having. It will not by itself bring NIS back to 3
on a receiver that smooths its output, because that part is not the filter's
doing.

## Simulated counterpart

`fusioncore_core/tests/test_consistency.cpp` runs the same measure in simulation,
where ground truth is available and NEES can be computed as well. It is a useful
reference for what these numbers look like when the filter is behaving.
