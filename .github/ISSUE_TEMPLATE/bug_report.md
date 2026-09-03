---
name: Bug Report
about: Something isn't working correctly
labels: bug
---

## What happened

<!-- What did you expect? What happened instead? -->

## Environment

- ROS distro: <!-- e.g. Jazzy -->
- OS: <!-- e.g. Ubuntu 24.04 -->
- FusionCore version: <!-- ros2 param get /fusioncore version, or git log --oneline -1 -->
- Hardware: <!-- robot platform, IMU model, GPS receiver -->

## The startup log

<!--
The first ~30 lines after "Configuring FusionCore" are usually enough to find the
problem on their own. They show which topics were actually subscribed, whether the
bias window found an orientation, whether any sensor's timestamps disagree with the
node clock, and which lever arms resolved. Please paste them even if nothing in
them looks wrong.
-->

```

```

## What the filter says about itself

<!--
This is usually the fastest route to an answer, and most people do not know these
topics exist. Please run BOTH while the problem is happening:

    ros2 topic echo /fusion/debug/filter_health --once
    ros2 topic echo /fusion/debug/gnss_status --once

filter_health carries per-sensor innovation norms, heading uncertainty in degrees,
which source the heading came from, and separate counters for measurements dropped
because two drivers disagree about the clock rather than because the data was bad.
gnss_status gives the rejection reason for a fix plus its Mahalanobis distance next
to the threshold it was tested against.

If your estimate drifts and heading_sigma_deg is large, that is very likely the
answer and it saves us both a round trip.
-->

```

```

## Config

<!-- Your fusioncore yaml, or the relevant section -->

```yaml

```

## To reproduce

<!-- Minimal steps. Launch commands, topic echoes, what you drove. -->

## If you have a bag

<!--
Optional, but it turns guesswork into a measurement. If you can record even 60
seconds of the problem:

    python3 tools/nis_from_bag.py /path/to/your_bag

That checks whether the covariance the filter reports matches the errors it is
actually making. It needs no ground truth. Paste the output here.
-->

```

```
