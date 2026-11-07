# FusionCore

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![arXiv](https://img.shields.io/badge/arXiv-2605.25239-b31b1b)](https://arxiv.org/abs/2605.25239)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.20091053-blue)](https://doi.org/10.5281/zenodo.20091053)
[![Docs](https://img.shields.io/badge/docs-manankharwar.github.io%2Ffusioncore-blue)](https://manankharwar.github.io/fusioncore/)
[![Newsletter](https://img.shields.io/badge/newsletter-subscribe-orange)](https://manankharwar.substack.com)

**Sensor fusion for outdoor wheeled ground robots that lose GPS. IMU, wheel encoders, GPS and visual SLAM at 100 Hz in a 23-state UKF. Two numbers from your IMU datasheet instead of days of tuning, and when the estimate goes wrong it names the sensor and the reason instead of drifting silently. Apache 2.0, ROS 2 Jazzy and Humble, and the filter itself is a plain C++ library with no ROS dependency.**

<img width="1080" height="608" alt="586785007-e1e07cfb-74e0-48b9-9bfd-32b68ee5a6ef" src="https://github.com/user-attachments/assets/d59b74ec-af94-4cb1-ab19-e5310a5d138b" />

---

---

## What this is for

A **wheeled robot, outdoors, with wheel odometry, that loses GPS.** Differential drive,
skid steer, tracked or Ackermann. That is the platform every number on this page was
measured on: NCLT is a wheeled Segway and the hardware runs are a four-wheeled rover.

You tell the filter which platform it is, in one line, and that decides which states it
bothers to estimate:

```yaml
fusioncore:
  ros__parameters:
    platform.profile: ground_robot_flat   # general | ground_robot | ground_robot_flat
```

A 23-state filter can estimate more than a wheeled robot can observe. The gyro measures
`WZ + B_GZ` and the accelerometer measures `AX + B_AX + gravity`, so those pairs are not
separately observable from those sensors alone. On a robot with encoders,
`platform.profile: ground_robot` sets the bias process noises and priors to zero and
treats the encoder's yaw rate as bias-free, so the encoder carries the constraint. From
0.5.0 it also turns on `gnss.reanchor_enabled` (recovery after a GPS outage), which failed
its ship rule on median and jitter and is enabled there as a stated product decision; set
`gnss.reanchor_enabled: false` to opt out. It slows
the biases rather than holding them: the UKF's covariance regularisation still lets them
drift (a median sigma of 0.053 rad/s on the gyro bias on NCLT 2013-04-05). In replay on five NCLT sequences that
lowered ATE RMSE on all five, by 0.3% to 17.7% (table in the CHANGELOG). Declaring the
platform is what lets the filter stop guessing. See
[Declaring the platform](https://manankharwar.github.io/fusioncore/configuration/#declaring-the-platform).

**Not built for drones, boats, legged robots or mecanum platforms.** The library will run
on them: the core is full 3D, there is no planar assumption, and `platform.profile:
general` exists precisely so nothing is frozen that those platforms need. But nothing
here is tuned or benchmarked for them, the profiles do not apply, and the non-holonomic
constraint is wrong for anything that can move sideways on purpose.

**For those platforms, and for fusing an arbitrary set of sensors, use
[robot_localization](https://github.com/cra-ros-pkg/robot_localization).** It is mature,
it is general by design, and it is the right answer where this one has no evidence.

## Start without installing anything

If you have a rosbag, you can get something useful out of this before you build it.
Point `bag_report.py` at the bag and it tells you what is wrong with your estimation
setup, in terms of your robot rather than a benchmark dataset:

```bash
python3 tools/bag_report.py /path/to/your/bag
```

No config, no ROS distro to match, and every check works with no ground truth, which
is the point: almost nobody has a surveyed reference for their own robot. It reports
things like whether your IMU and your wheels disagree about which way the robot
turned, whether two sensors are on different clocks, and whether the bag even
contains a manoeuvre that makes heading observable. Details in
[docs/bag-report.md](https://manankharwar.github.io/fusioncore/bag-report/).

The first bag it was ever run on had an IMU and wheel encoders reporting opposite
yaw-rate signs on 100% of turns, over 127,419 samples. That bag was from this
project's own benchmark harness, and the bug had been there for every number it had
ever published.

---

## Answers to the things that stop people trying it

**Yes, you can run it beside robot_localization rather than instead of it.** Same bag,
same inputs, both filters, compare the two trajectories. That is how the numbers here
were produced, and `fusioncore_ros/tests/test_rl_interface_conformance.py` locks the
interface so it stays a drop-in comparison rather than a rewrite.

**Yes, your existing robot_localization config converts.** `tools/rl_to_fusioncore.py`
reads it and emits the FusionCore equivalent, because that config already encodes
everything you learned about your robot. It refuses to guess: anything that does not
map cleanly comes out as a commented `TODO` with the reason, repeated on stderr so it
is visible when you redirect stdout to a file. A config that looks complete but
quietly invented a number is worse than one that admits what it does not know.

**No, you do not need ROS 2.** The filter is a plain C++ library with Eigen and no ROS
dependency. The ROS 2 wrapper is a separate package you can ignore.

**No, you do not need ground truth to evaluate it.** `tools/bag_report.py` works
entirely on raw sensor topics, because almost nobody has a surveyed reference for
their own robot. It will not give you an accuracy number, and it says so rather than
inventing one.

**Yes, it runs on a Raspberry Pi 4.** Well under 1 ms per cycle in a Release build,
same source on ARM and x86. Build unoptimised and it is drastically slower, so do not
skip `CMAKE_BUILD_TYPE`.

**No, it does not beat raw GPS or robot_localization on NCLT.** All
twelve NCLT sequences were measured under a rule committed before the run: the shipped default
has higher error than raw GPS on 11 of 12 (the twelfth a 0.001 m tie), and with opt-in
re-anchoring it is lower than raw GPS on 2. Against robot_localization the result depends on the
sequence, in both directions and by large factors, and the table shows each one rather than a win
count. The old claim that FusionCore beat robot_localization on ten of twelve was measured through
a broken dataset player and is withdrawn.

**What is defensible today.** FusionCore rejects GPS fixes that are kinematically impossible,
and that one is provable from physics without any reference: a 60 m jump in one second cannot
happen to a 1.5 m/s robot. The large NCLT errors on both sides mostly start at the same moment,
the minutes after a GPS blackout, when a filter has to decide which returning fixes to trust:
robot_localization tends to accept a bad one and then lock out good ones, FusionCore's default
tends to reject good ones for too long.

---

## Quick start

```bash
sudo apt install ros-jazzy-fusioncore     # or ros-humble-fusioncore
```

Or from source:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to fusioncore_ros
source install/setup.bash
```

Check it works before wiring it to a robot. This starts the filter with fake sensors and verifies every output, in about 15 seconds:

```bash
bash tools/quick_test.sh
```

Then point it at your robot:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

The launch file brings the lifecycle node all the way up to `active` on its own. Pass `autoconfigure:=false` if a `nav2_lifecycle_manager` should own it instead.

Docker, if you would rather not install ROS 2: [docs/docker.md](https://manankharwar.github.io/fusioncore/docker/)

```bash
docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh
```

---

## When it goes wrong, it tells you why

Most localization debugging is not a mathematics problem. The filter drifts, and the hard part is working out which of six sensors caused it. FusionCore publishes what it is thinking while it runs, on real hardware:

```bash
ros2 topic echo /fusion/debug/gnss_status     # one message per GPS fix
ros2 topic echo /fusion/debug/filter_health   # filter state at 1 Hz
```

`gnss_status` answers "why was that fix dropped?" for every fix. A `rejection_reason` (`CHI2_FAILED`, `SIGMA_XY_HIGH`, `IMPLAUSIBLE_JUMP`, `DELAY_TOO_LARGE` and the rest), the Mahalanobis distance printed next to the threshold it was actually tested against, and the filter's own position sigma at that moment.

`filter_health` answers "does this filter even know which way it is pointing?" Per-sensor innovation norms, heading uncertainty in degrees, which source the heading came from (`GPS_TRACK`, `MAGNETOMETER`, `DUAL_ANTENNA`, `NONE`), and a separate count of measurements dropped because two drivers disagree about the clock rather than because the data was bad.

That last distinction matters more than it sounds. A sensor whose timestamps run behind the filter clock is not being fused at all, and from the outside that looks exactly like a badly tuned filter.

You can also ask, after the fact, whether the covariance the filter reported was honest. This needs no ground truth and works on any recorded bag:

```bash
python3 tools/nis_from_bag.py /path/to/your_bag
```

Details: [Is your filter's covariance honest?](https://manankharwar.github.io/fusioncore/guides/filter-consistency/)

---

## What FusionCore does not do

Every project has these. Most do not write them down.

**Yaw is not observable from a 6-axis IMU, wheel encoders and GPS position alone.** The gyro measures `wz + gyro_bias` and the encoder measures `wz + encoder_bias`, which is two equations for three unknowns. GPS track heading only helps while the robot moves in a straight line fast enough for the displacement bearing to beat the position noise. Add a magnetometer or dual-antenna GNSS heading and the problem goes away. Without one, expect heading uncertainty to grow during slow or twisty driving, and read `heading_sigma_deg` in `filter_health` rather than assuming.

**The chi-squared gate is less sensitive than its nominal threshold on a smoothing receiver.** Many GNSS receivers report their absolute accuracy, several metres dominated by multipath, while emitting fixes that agree with each other to centimetres because they filter internally. A Kalman filter assumes white measurement noise, so it gets handed a covariance far larger than any innovation it will see, and the gate then sits much further above typical than its 99.9% design point suggests. Measure yours with `nis_from_bag.py` before relying on the gate.

**Long GPS blackouts still accumulate heading error.** Beyond roughly five to seven minutes of dead reckoning, residual bias drift dominates. See [known limitations](https://manankharwar.github.io/fusioncore/known-limitations/).

---

## Built around the problems real robots have

| The problem | How FusionCore handles it |
|---|---|
| **IMU calibration is approximate** | Gyro and accel bias are filter states, estimated continuously. `init.stationary_window: 2.0` estimates startup bias before motion begins. |
| **Extrinsic calibration is never exact** | Reads `frame_id` from every IMU message and looks up the TF rotation to `base_link` automatically. Set `imu.frame_id` to override broken frame names from drivers. No manual rotation matrices. |
| **Sensors disagree about what time it is** | Stamps more than 1 s from the node clock warn at startup. A sensor lagging the filter clock is rejected as stale rather than being allowed to corrupt it, and the count is published so you can see it happening. |
| **GPS arrives late (50 to 200 ms)** | An IMU ring buffer replays the buffered updates when a delayed fix arrives, reconstructing the state at the GPS timestamp rather than approximating it. The buffer holds 100 IMU samples, one second at 100 Hz, but the usable window is `max_measurement_delay` (0.5 s by default): a fix older than that is dropped rather than replayed. |
| **Wheel odometry is noisy or slipping** | Adaptive noise covariance updates from the innovation sequence. Optional GPS velocity fusion compares GPS speed against wheel speed every cycle, so the innovation reveals slip and the gain down-weights it. |
| **Noise parameters need days of tuning** | Two numbers from your IMU datasheet, `imu.gyro_noise` and `imu.accel_noise`. The adaptive estimators handle the rest, though a bad frame or a bad timestamp will still need fixing by hand. |
| **The robot runs on a Raspberry Pi** | Well under 1 ms per cycle on a Pi 4 in a Release build. Same source on ARM and x86 via Eigen. Build unoptimised and it is drastically slower, so do not skip `CMAKE_BUILD_TYPE` (0.3.8 and later default to Release). |
| **Two IMUs on the platform** | Set `imu2.topic` to fuse a second IMU as an independent measurement. No pre-merging needed. |
| **GPS drops out under canopy** | Inertial coast mode holds position through sustained dropout, and the gate relaxes on re-acquisition after a genuine gap rather than after a sustained outlier. |
| **The robot sits still for minutes** | ZUPT fuses a zero-velocity pseudo-measurement when encoder speed and angular rate are both below threshold, so IMU noise does not integrate into drift while idle. |

<img width="1200" height="675" alt="fusioncore_demo_hmm" src="https://github.com/user-attachments/assets/89e9134d-3ec1-4cd9-898b-e3a9c62852dd" />

---

## What it costs you to run

Accuracy is the easy claim to make and the hard one to verify, especially on a robot
nobody has surveyed. These are the costs, which you can check against your own setup
in an afternoon:

| | |
|---|---|
| **Tuning** | Two numbers, `imu.gyro_noise` and `imu.accel_noise`, both off your IMU datasheet. The adaptive estimators handle the rest. A bad frame or a bad timestamp still needs fixing by hand, and the tools below find those. |
| **Finding out why it drifted** | Every sensor has a named rejection reason, and the gated ones carry their chi-squared value too, published on a debug topic while it runs. So "it drifts" becomes a count per reason rather than a guess about which of six sensors did it. |
| **Knowing whether to trust it** | The filter reports which quantities are actually observable and what to drive to fix the rest, rather than leaving you to infer it. See [Observability](https://manankharwar.github.io/fusioncore/observability/). |
| **Catching a config typo** | ROS 2 silently ignores an override for a parameter a node never declared, so a typo tunes nothing and warns nobody. `tools/check_config_params.py` fails on it. |
| **Evaluating it at all** | `tools/bag_report.py` on a recording, before you build anything. |

Three of those five exist because they were the thing that cost days here first. The
observability report exists because three separate issues in this project turned out
to be "the robot was never driven in a way that made this observable", and nothing
said so for weeks.

---

## Benchmark

FusionCore against robot_localization on the [NCLT dataset](http://robots.engin.umich.edu/nclt/): same IMU, wheel odometry and GPS, no manual tuning, twelve full-length sequences across all seasons. RL-EKF run with chi-squared-equivalent thresholds at 99.9% confidence.

> **Status, 2026-10-26, and this supersedes every dated note below it.**
>
> All twelve NCLT sequences were re-run under a reporting rule committed before the run
> started (`tools/report_rule_20261025.md`). The dataset player was corrected first: it had
> been declaring the IMU's accelerometer-derived tilt as a 0.57 degree attitude measurement.
> That declaration was ours, not the dataset's. It counted the accelerometer twice and made
> the ROS node lock onto a wrong position for about 800 s on 2012-01-08 (178.808 m full
> trajectory RMSE before the correction, 57.314 m in this run).
>
> Scored at GPS fix times, the shipped default does not beat raw GPS: raw GPS has the lower
> RMSE on 11 of 12, and the twelfth, 2012-05-11, is a tie at 9.186 m against 9.187 m. With
> opt-in re-anchoring (`gnss.reanchor_enabled`), FusionCore is lower than raw GPS on 2 of 12,
> 2012-03-31 by 22.0% and 2012-10-28 by 5.6%, both held-out sequences never used for tuning.
> Against the default on the same pipeline and scored over the full trajectory, re-anchoring
> lowers RMSE on 10 of 12 and raises it on none. It also raises the median on 4 (by 0.3% to
> 3.0%) and the output-rate jitter on 3 (by up to 0.124), which is why it is off by default.
>
> ATE RMSE (3D) at GPS fix times, every column on identical timestamps:
>
> | Sequence | Set | Raw GPS | Default, ROS node | Default, replay | Re-anchoring on, replay | Re-anchoring on, ratio to raw GPS |
> |---|---|---|---|---|---|---|
> | 2012-01-08 | held out | 9.883 m | 56.957 m | 50.065 m | 10.899 m | 1.103x |
> | 2012-02-04 | dev | 57.564 m | 163.830 m | 145.629 m | 59.340 m | 1.031x |
> | 2012-03-31 | held out | 13.825 m | 16.818 m | 16.937 m | **10.787 m** | 0.780x |
> | 2012-05-11 | held out | 9.187 m | 9.186 m | 9.187 m | 9.187 m | 1.000x |
> | 2012-06-15 | dev | 9.293 m | 24.363 m | 12.041 m | 12.041 m | 1.296x |
> | 2012-08-20 | dev | 34.250 m | 160.359 m | 172.961 m | 39.790 m | 1.162x |
> | 2012-09-28 | dev | 8.960 m | 18.883 m | 22.236 m | 10.176 m | 1.136x |
> | 2012-10-28 | held out | 13.923 m | 15.999 m | 16.763 m | **13.150 m** | 0.944x |
> | 2012-11-04 | held out | 20.267 m | 23.626 m | 24.512 m | 20.414 m | 1.007x |
> | 2012-12-01 | held out | 12.681 m | 16.160 m | 16.466 m | 12.741 m | 1.005x |
> | 2013-02-23 | held out | 15.724 m | 17.294 m | 16.951 m | 15.793 m | 1.004x |
> | 2013-04-05 | dev | 8.624 m | 9.994 m | 10.007 m | 8.798 m | 1.020x |
>
> How these were produced: the ROS node at 1x playback (achieved filter rate 99.8 to
> 103.3 Hz on every sequence) and the deterministic offline replay of the same shipped config
> on the same input, scored by `tools/score_nclt_suite.py` (version `803c158e99ff`, printed
> on every row) through `hardware/ate_tum.py`. Re-anchoring was run through replay only, so
> compare it with the replay default column. The node and the replay still disagree: node
> over replay is 0.70x to 1.14x over the full trajectory, and at fix times up to 2.0x on
> 2012-06-15 (24.363 m against 12.041 m). That gap is not yet explained. No sequence was
> excluded. Medians, path ratios, jitter and coverage for every arm and sequence are in
> `tools/eval_20261025_results.md`. "Held out" sequences were never used to develop or tune
> anything (`tools/ship_rule.md`).
>
> **robot_localization, re-run on all twelve sequences 2026-10-27 to 11-07.** Config
> `fusioncore_datasets/config/rl_ekf.yaml` as amended 2026-10-26, following robot_localization's own
> planar wheeled-robot example: wheel odometry fuses vx, vy and vyaw, the IMU fuses yaw rate only,
> GPS arrives through navsat_transform. Its GPS gate (`odom1_pose_rejection_threshold: 3.72`,
> chi-squared 2 DOF at 99.9%) has been in that file since April and was not changed. Same harness,
> player and scorer as above, node at 1x, one sequence at a time; that playback also ran FusionCore a
> second time, reported as a repeat. Filter rate 100.0 to 103.3 Hz on every sequence; robot_localization
> logged 9 update-rate misses on 2012-01-08 and none on the other eleven. Coverage 99.9% or above
> everywhere. No sequence excluded, none re-run.
>
> ATE RMSE / median (metres) at GPS fix times. The points are the timestamps common to every arm
> including robot_localization, so the raw GPS and FusionCore figures can differ from the table above
> in the third decimal (2012-01-08 raw GPS 9.878 m here, 9.883 m above):
>
> | Sequence | Set | Points scored | Raw GPS | robot_localization, node | FusionCore default, node | FusionCore default, node repeat | FusionCore default, replay | FusionCore re-anchoring, replay |
> |---|---|---|---|---|---|---|---|---|
> | 2012-01-08 | held out | 10167 | 9.878 / 6.768 | 31.647 / 6.176 | 56.969 / 13.800 | 57.666 / 13.837 | 50.075 / 12.352 | 10.895 / 6.816 |
> | 2012-02-04 | dev | 8905 | 57.564 / 13.600 | 145.976 / 36.984 | 163.830 / 42.020 | 150.519 / 36.416 | 145.629 / 35.612 | 59.340 / 13.519 |
> | 2012-03-31 | held out | 10046 | 13.824 / 7.469 | 160.874 / 42.830 | 16.816 / 7.357 | 15.886 / 7.336 | 16.934 / 7.376 | 10.785 / 7.413 |
> | 2012-05-11 | held out | 10384 | 9.187 / 5.210 | 8.387 / 4.425 | 9.186 / 5.267 | 9.184 / 5.251 | 9.187 / 5.217 | 9.187 / 5.217 |
> | 2012-06-15 | dev | 6526 | 9.293 / 4.891 | 45.941 / 16.375 | 24.363 / 7.846 | 23.366 / 7.931 | 12.041 / 4.900 | 12.041 / 4.900 |
> | 2012-08-20 | dev | 9799 | 34.250 / 7.319 | 10.400 / 4.451 | 160.359 / 35.210 | 159.581 / 35.923 | 172.961 / 38.082 | 39.790 / 7.495 |
> | 2012-09-28 | dev | 8404 | 8.960 / 6.430 | 95.140 / 26.585 | 18.883 / 6.859 | 18.875 / 6.863 | 22.236 / 7.140 | 10.176 / 6.554 |
> | 2012-10-28 | held out | 8901 | 13.923 / 8.251 | 25.929 / 6.679 | 15.999 / 8.399 | 15.997 / 8.386 | 16.763 / 8.436 | 13.150 / 8.335 |
> | 2012-11-04 | held out | 8200 | 20.267 / 9.657 | 24.170 / 7.834 | 23.626 / 10.220 | 24.733 / 10.268 | 24.512 / 10.301 | 20.414 / 9.806 |
> | 2012-12-01 | held out | 7838 | 12.681 / 7.842 | 12.528 / 5.421 | 16.160 / 7.910 | 16.135 / 7.884 | 16.466 / 7.859 | 12.741 / 7.948 |
> | 2013-02-23 | held out | 8569 | 15.724 / 9.451 | 77.986 / 24.301 | 17.294 / 9.650 | 17.300 / 9.633 | 16.951 / 9.630 | 15.793 / 9.441 |
> | 2013-04-05 | dev | 7458 | 8.624 / 6.319 | 8.329 / 4.449 | 9.994 / 6.457 | 10.020 / 6.462 | 10.007 / 6.447 | 8.798 / 6.443 |
>
> No aggregate and no winner, per the rule committed before the run: the spread is the result. Where
> robot_localization does badly, the one case examined in full (2012-03-31) is not the harness. A 262 s
> GPS blackout, then the first three fixes after it are 535 to 537 m wrong in the dataset. Its own
> covariance is so large after dead reckoning that the gate admits the first one, the covariance
> collapses onto it, and every good fix after that fails the gate for about 290 s while it drives
> about 600 m off. That window is 7% of the points and 99.7% of its squared error. 2012-09-28 and
> 2013-02-23 show the same pattern in a partial check. Where FusionCore does badly, 2012-08-20 is the
> clearest: robot_localization reads 10.400 m, below raw GPS, and FusionCore's default 160.359 m.
> That sequence has a cluster of fixes up to 727 m wrong at the end of a blackout; FusionCore's loss
> there was not re-diagnosed on this run. Full-trajectory RMSE, median,
> path ratio and jitter for both node arms are in `tools/eval_20261025_results.md`.
>
> Older dated notes, the May 2026 comparison table and every withdrawn figure are in
> [benchmarks/nclt_history.md](benchmarks/nclt_history.md), unchanged.

---

## Running on real hardware

> "The system was stable on real robot data and was relatively easy to configure. I was able to get reasonable behavior without spending excessive time on parameter tuning. The overall experience felt more deployment-oriented than research-demo-oriented."
>
> **Michał Bednarek** ([@mbed92](https://github.com/mbed92)), Robotics PhD
> Factory differential-drive robot, ROS 2 Humble: Cartographer (point-cloud localization, no preloaded map) + wheel odometry + IMU

> **Sam** ([@samuk](https://github.com/samuk)), [Agroecology Lab](https://github.com/Agroecology-Lab/feldfreund_devkit_ros)
> Outdoor agricultural robot, integration in progress

> **Russ Hall**, Andino robot (Raspberry Pi)
> OAK-D (stereo depth + IMU) + Velodyne VLP-16 + rtabmap: indoor SLAM mapping

Running FusionCore on your robot? Add yourself in [Discussions #22](https://github.com/manankharwar/fusioncore/discussions/22) and I will list you in [ADOPTERS.md](ADOPTERS.md).

<img width="1431" height="1127" alt="fig_bias_estimation" src="https://github.com/user-attachments/assets/da189e76-bb54-4d90-b0a7-48926060b86a" />

---

## Coming from robot_localization

FusionCore replaces the robot_localization + navsat_transform pair with one lifecycle node. The migration guide covers the YAML and launch changes: [migration guide](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)

<img width="1008" height="334" alt="demo_plot" src="https://github.com/user-attachments/assets/4fdec6e5-f827-4111-bd91-912992ab17fb" />

| Problem | How FusionCore handles it |
|---|---|
| A GPS outlier corrupts the state | Chi-squared gate per sensor DOF rejects bad fixes before they reach the filter, and reports which gate fired. Covariance is bounded at every step, so no NaN divergence. |
| UTM zone boundary near the operating area | GPS is fused directly in ECEF. No UTM projection and no zone boundary case. |
| A wheeled robot drifts sideways without GPS | Non-holonomic constraint zeros lateral and vertical velocity as a virtual measurement on every encoder update. |
| GPS fixes arrive 50 to 200 ms late | IMU ring buffer with retrodiction reconstructs the exact filter state at the GPS timestamp. |
| Bag replay gives different results each run | Message timestamps drive all updates under `use_sim_time: true`. Same bag, same config, same output. |
| An IMU mounted off-axis, or a driver with a wrong frame name | TF lookup on every message, with an `imu.frame_id` override. |
| navsat_transform startup ordering and CPU cost | No navsat_transform node. ECEF conversion is one matrix multiply per fix inside the filter. |

<img width="1109" height="1035" alt="fig_mahalanobis" src="https://github.com/user-attachments/assets/702427fe-9685-49f9-be2a-7c27db45691d" />
<img width="1967" height="1087" alt="fig3_two_outcomes" src="https://github.com/user-attachments/assets/792430dd-b86a-44e7-b51b-7a462cd8538d" />

---

## Using the filter without ROS 2

`fusioncore_core` is a plain C++17 library. Its only dependency is Eigen, and it contains no ROS headers, so it compiles and runs on its own:

```cpp
#include "fusioncore/fusioncore.hpp"

fusioncore::FusionCoreConfig cfg;
fusioncore::FusionCore fc(cfg);
fusioncore::State s;
fc.init(s, 0.0);
fc.update_imu(0.01, wx, wy, wz, ax, ay, az);
fc.update_encoder(0.02, vx, 0.0, wz);

fusioncore::sensors::GnssFix fix;      // ENU metres, plus hdop/vdop/satellites
fix.x = 12.0; fix.y = 3.0; fix.z = 0.0;
fc.update_gnss(0.20, fix);

const auto& out = fc.get_state();      // out.x is the 23-state vector, out.P its covariance
```

```bash
g++ -std=c++17 -O2 your_main.cpp fusioncore_core/src/*.cpp \
    -I fusioncore_core/include -I /usr/include/eigen3 -o your_app
```

Useful if you are on PX4, a custom middleware, or an embedded loop where ROS 2 is not the right fit. `fusioncore_ros` is a thin wrapper over exactly this API.

---

## In the ecosystem

**rtabmap_ros (merged):** included as a named demo in the official [rtabmap_ros](https://github.com/introlab/rtabmap_ros) repository. The demo runs FusionCore and `icp_odometry` in a feedback loop: FusionCore's stable odom frame seeds scan matching via `guess_frame_id`, and the ICP result returns as a second velocity source. [View the demo](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos)

**Stereolabs community:** a FusionCore + ZED integration guide is posted on the Stereolabs developer forum. [@privvyledge](https://github.com/privvyledge) set out to compare FusionCore against Wolf, TIER IV EagleEye and robot_localization in May 2026 on an F1/10 scale car and a full-size van; no results have been reported back.

**OpenMowerNext:** [PR #45](https://github.com/jkaflik/OpenMowerNext/pull/45) integrates FusionCore as the localization stack for a community ROS 2 mowing system, replacing robot_localization with a single lifecycle node fusing RTK GPS (u-blox F9P), IMU and wheel odometry.

---

## Documentation

**[manankharwar.github.io/fusioncore](https://manankharwar.github.io/fusioncore/)**

- [Getting Started](https://manankharwar.github.io/fusioncore/getting-started/)
- [Configuration reference](https://manankharwar.github.io/fusioncore/configuration/)
- [Hardware configs](https://manankharwar.github.io/fusioncore/hardware/)
- [Nav2 integration](https://manankharwar.github.io/fusioncore/nav2/)
- [Migrating from robot_localization](https://manankharwar.github.io/fusioncore/migration_from_robot_localization/)
- [Is your filter's covariance honest?](https://manankharwar.github.io/fusioncore/guides/filter-consistency/)
- [Known limitations](https://manankharwar.github.io/fusioncore/known-limitations/)
- [How it works](https://manankharwar.github.io/fusioncore/how-it-works/)
- [Troubleshooting](https://manankharwar.github.io/fusioncore/troubleshooting/)

---

## Contributing

Bug reports are genuinely useful here and several have changed the library. If
something misbehaves, the fastest route to an answer is usually the filter's own
diagnostics, which the [bug report template](.github/ISSUE_TEMPLATE/bug_report.md)
asks for:

```bash
ros2 topic echo /fusion/debug/filter_health --once
ros2 topic echo /fusion/debug/gnss_status --once
```

If you would rather write code, the
[good first issues](https://github.com/manankharwar/fusioncore/labels/good%20first%20issue)
are scoped so you do not need to understand the estimator to pick one up. They
point at the file and the line, and each one comes from a real failure rather than
a wishlist. [CONTRIBUTING.md](CONTRIBUTING.md) has the build and test steps.

---

## License

Apache 2.0, and it stays that way.

---

## Citation

```bibtex
@article{kharwar2026fusioncore,
  author  = {Kharwar, Manan},
  title   = {FusionCore: A 23-State Unscented Kalman Filter for
             IMU, Wheel Encoder, GPS, and Visual SLAM Fusion in ROS 2},
  journal = {arXiv preprint arXiv:2605.25239},
  year    = {2026},
  url     = {https://arxiv.org/abs/2605.25239}
}
```

To cite the software release itself:

```bibtex
@software{kharwar2026fusioncore_software,
  author    = {Kharwar, Manan},
  title     = {FusionCore: ROS 2 UKF Sensor Fusion},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.20091053},
  url       = {https://doi.org/10.5281/zenodo.20091053}
}
```

---

Bug reports with a log and a config get answered, usually the same day. If something behaves strangely, `/fusion/debug/filter_health` and `tools/nis_from_bag.py` will often say why before I can.

Need it working on your hardware with someone accountable for the result? [Commercial support and fixed-price integration](https://manankharwar.github.io/fusioncore/support/).
