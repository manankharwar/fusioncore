# Send a bag, get a report

```
python3 tools/bag_report.py /path/to/your/bag
```

Nothing to build, no config to write, no ROS distro to match. It reads a rosbag and
tells you what is wrong with your estimation setup, in terms of your robot rather
than a benchmark dataset.

**Every check is computed from raw sensor topics with no ground truth.** That is the
design constraint that makes it work on a recording from a robot nobody has surveyed,
which is almost all of them.

Topics are matched **by message type**, not by name, because your topics are named
whatever they are named.

## What it checks

| | |
|---|---|
| Inventory and rates | achieved Hz, median against mean, which separates a uniformly slow stream from dropped runs |
| Clock skew | per-topic header-to-arrival, and offsets **between** sensors |
| Yaw-rate sign | whether your IMU and your wheels agree which way the robot turned |
| Yaw-rate scale | the magnitude ratio, which is a different bug from the sign |
| Stationary gyro bias | measured in the longest contiguous still interval |
| GNSS quality | declared sigma against how far consecutive fixes actually move, and blackout length |
| Observability | whether the bag contains a manoeuvre that makes heading and gyro bias observable at all |

Findings come back as `BLOCKER`, `WARNING`, `INFO` or `OK`. A **blocker** is something
that makes a whole class of estimation problem unfixable by tuning, so it is worth
doing before any parameter change.

`--json` emits the findings as structured data. Exit code is 1 if anything is a
blocker, so it can gate a CI job on a recorded regression bag.

## The two findings worth the most

### Your two rotation sources disagree

```
BLOCKER: IMU and wheel yaw rates disagree in sign on 100% of turns
127419 samples where both exceeded 0.08 rad/s. Median magnitude ratio 1.285.
```

One of them has a frame convention wrong. Under REP-103, turning the robot LEFT by
hand must produce a POSITIVE `angular_velocity.z`. A BNO085 in UART-RVC mode reports
yaw increasing clockwise and needs its sign flipped in the driver.

This does not crash anything, which is why it survives. A gyro usually has tighter
declared noise than wheel odometry, so the filter leans on the gyro and spends gain
rejecting the encoder on every single turn. A quiet tax, not a failure.

That exact output is from this project's own NCLT harness, where the IMU was converted
to ENU and the wheel odometry was not, for every benchmark number ever published here.
It took 137,509 samples of a raw dataset to notice. If it can hide in a harness that
is actively being measured, it can hide in yours.

The magnitude ratio is reported separately on purpose: a frame flip gives equal
magnitudes, so a 1.285x ratio is a **second** bug, a track-width or ticks-per-rev
error. Fixing the sign does not close it.

### Your bag cannot answer the question you are asking of it

```
BLOCKER: The robot never turned both ways
8431 samples turning left, 12 turning right.
```

A constant gyro bias and a constant yaw rate are indistinguishable until you turn
**both** directions. Without a reversal the pair slides along `WZ + B_GZ = const` at
zero measurement cost, measured at a correlation of -0.998 in this project's own
tests. No amount of tuning fixes a heading problem in that data, because the data does
not contain the information.

See [Observability](observability.md) for the manoeuvres and why each one works.

## What it will not tell you

No ground truth means **no accuracy number.** It cannot tell you your ATE, because
nothing in the bag knows where the robot actually was. It tells you whether your
inputs are self-consistent and whether your data contains the information your
estimator needs, which is the part that is wrong first and is almost never checked.
