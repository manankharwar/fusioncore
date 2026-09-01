# Known Limitations

These are limitations of the current implementation, not bugs. Each entry includes why it exists and what the workaround or planned fix is.

---

## The reported covariance is conservative without an absolute heading source

On a wheeled robot with a 6-axis IMU, wheel encoders and GNSS position, yaw is
not observable: the gyro measures `wz + b_gz` and the encoder measures
`wz + b_ewz`, two equations for three unknowns. GPS track heading only helps
while the robot moves in a straight line fast enough for the displacement bearing
to beat the position noise.

That heading uncertainty propagates into the position covariance. Measured across
three field runs on the development rover, the Normalized Innovation Squared for
GNSS position had a median of 0.02 to 0.03 where an honest filter averages 3.0,
with a filter heading 1-sigma of 57 to 143 degrees. One of those runs accepted
every fix with no quality gate firing and behaved the same, so this is not a side
effect of measurements being discarded.

A second and usually larger effect sits underneath it: a receiver that smooths
internally reports its absolute accuracy while emitting fixes that agree with
each other far more closely. On the same run the receiver declared 4.84 m
1-sigma, but the median second difference between consecutive fixes was 0.087 m
where white noise of that size would give about 14 m. The filter is handed the
declared figure as `R`, so `S` dwarfs any innovation it will see. That part is a
property of the measurement, not of the filter, and shrinking `R` is the wrong
response because the absolute error genuinely is metres.

Two consequences worth knowing. The Kalman gain is higher than optimal, so the
filter tracks GNSS noise more closely than it needs to. And the chi2 outlier gate
is calibrated for a consistent filter: at the default threshold of 16.27, the
99.9th percentile for 3 degrees of freedom, it is far less sensitive than that
nominal figure when NIS is running near 0.03, so a moderate multipath excursion
can pass through.

**Workaround:** add an absolute heading source, either a magnetometer
(`magnetometer.enabled`) or dual-antenna GNSS heading. Both make yaw observable
and shrink the position covariance that depends on it. Measure your own run first
with `python3 tools/nis_from_bag.py <bag>`, described in
[Is your filter's covariance honest?](guides/filter-consistency.md).

---

## GPS blackouts longer than 5-7 minutes cause heading drift

During GPS blackouts, FusionCore dead-reckons on IMU and wheel encoders. The encoder WZ bias (`B_EWZ`) is calibrated from GPS heading cross-covariance before the blackout and subtracted during it. For blackouts up to a few minutes, this works well. Beyond 5-7 minutes, the residual uncorrected heading rate error (thermal drift in `B_EWZ`, temperature-dependent gyro drift) accumulates into position error quadratically.

On the NCLT 2012-06-15 sequence (461-second blackout), FC accumulated significant heading error. See [issue #63](https://github.com/manankharwar/fusioncore/issues/63).

**Workaround:** Set `magnetometer.enabled: true` and point `magnetometer.topic` at your `sensor_msgs/MagneticField` publisher. FusionCore applies hard/soft iron correction and tilt compensation, then fuses the heading as a 1-DOF UKF update. This closes the heading observability gap during GPS outages: an absolute heading bounds the drift instead of letting it accumulate. Requires magnetometer calibration (imu_calib or magneto). For environments with transient magnetic disturbance (motors, steel structures, rebar), set `magnetometer.field_strength` to your local Earth field magnitude so FusionCore rejects readings whose magnitude is distorted, even when their direction would pass the chi2 gate. Note: the magnetometer relies on the wheel encoder as an independent turn-rate reference; on a platform with no wheel odometry it is less effective during very long outages.

---

## Adversarial GPS outlier clusters at blackout boundaries can corrupt the estimate

FusionCore's chi2 gate relaxes during GPS blackout recovery (`coast_q_factor`) to accept valid returning fixes after the predicted covariance inflates. If a cluster of corrupt GPS fixes arrives at exactly the re-acquisition moment, the relaxed gate may accept them before the filter's state recovers.

This pattern appears in NCLT 2012-08-20: 105 mode-3 GPS fixes 720-840 m off RTK arrive in a 24-second window at the end of a 211-second blackout. The filter accepts them, spikes to ~788 m error, and recovers within 2 minutes. See [issue #64](https://github.com/manankharwar/fusioncore/issues/64).

**Workaround:** Lower `gnss.coast_q_factor` to tighten the gate during recovery. This reduces the spike magnitude at the cost of slower re-acquisition after legitimate long blackouts. There is no single value that handles both cases optimally.

**Partial mitigation (implemented): gap-gated coast.** A closely related failure is a *sustained* spike during *continuous* GPS (no preceding gap): the chi2 gate rejects it at first, but after `coast_n` rejections coast inflation widens the gate until the spike is admitted. As of `gnss.coast_min_gap_s` (default 1.0 s), rejection-triggered coast only fires when the rejection streak began after a real GPS gap, so a continuously present outlier can no longer trigger the inflation that defeats the gate. This fully closes the continuous-spike variant (validated in simulation and unit tests). It does **not** close the case described above: the 2012-08-20 cluster arrives after a legitimate 211 s blackout, so a real gap precedes it and coast is correctly allowed. The physical-plausibility gate below now rejects that cluster, but (as noted there) the sequence's score is dominated by dead-reckoning drift during the blackout, not the cluster.

**Implemented: physical-plausibility gate (`gnss.max_speed`).** A GPS fix far from the dead-reckoned position implies an impossible amount of motion. FusionCore now rejects any fix farther from its predicted position than `gnss.max_speed * gap_seconds + gnss.max_speed_margin` (a kinematic bound, checked filter-vs-fix, off by default). On 2012-08-20 with `gnss.max_speed: 3.0` this rejects the cluster and cuts the peak error. Note: a naive GPS-to-GPS "implied speed" check does **not** catch this cluster (the first cluster fix sits only ~4 m/s away from the last pre-blackout fix over the long gap, which looks benign); the gate works because it measures the offset from the filter's own prediction, not fix-to-fix. **Important honesty caveat:** this gate is a robustness feature, not a fix for the #64 score. Rejecting the cluster lowers the peak spike, but the sequence's overall ATE stays high because the dominant error is the dead-reckoning drift accumulated *during* the 211 s blackout, not the cluster spike itself. The real lever for the underlying drift is an absolute heading source (see the heading-drift limitation above).

**Not yet implemented: cluster consistency gate.** A single outlier at 720 m is handled by chi2. Five consecutive fixes all landing 720-840 m from the predicted position with tight geometric consistency is a distinguishable pattern from noise. A secondary check on cluster coherence across multiple consecutive fixes would catch this without affecting single-fix rejection behavior. This is the planned architectural fix.

---

## IMU + encoder alone cannot separate gyro bias from encoder bias without GPS

When GPS is unavailable and both sensors are active, the system has three unknowns (WZ, B_GZ, B_EWZ) observable from two measurement equations. The system is rank-deficient.

FusionCore resolves this by treating `B_EWZ` as known from the GPS calibration phase (tight prior on `P(B_EWZ, B_EWZ)`). This works correctly during GPS blackouts when `B_EWZ` was calibrated before the blackout. It does not work if GPS was never available: `B_EWZ` remains unobservable, and the filter cannot separate the two biases.

**Workaround:** Ensure the robot operates with GPS for at least the first minute of each run to calibrate `B_EWZ`. The `init.stationary_window` setting helps calibrate `B_GZ` before motion begins.

---

## Non-holonomic constraint assumes flat ground

The NHC (non-holonomic constraint) zeros the lateral and vertical body-frame velocity as virtual measurements. On significant slopes or when traversing obstacles, the vertical velocity constraint briefly produces a non-zero innovation (the robot genuinely has body-frame VZ during a curb transition or a bump).

**This is handled automatically.** The VZ=0 and AZ=0 constraint noise adapts from the innovation sequence (controlled by `adaptive.ground_constraint: true`, enabled by default). When the robot hits rough terrain, large VZ or AZ innovations inflate the constraint noise within one sliding window (~1 second at encoder rate), loosening the assertion to match actual terrain behavior. When back on smooth ground, the noise relaxes back down to the configured floor values (`ground_constraint.vz_sigma` and `ground_constraint.az_sigma`). No config changes are needed when moving between terrain types.

The floor values control the minimum tightness on smooth ground:
- `ground_constraint.vz_sigma: 0.1` (default, flat floors and mild terrain)
- `ground_constraint.az_sigma: 0.5` (default, flat floors and mild terrain)

Increase these floors only if you want looser constraints even on smooth surfaces (rarely needed).

For lateral velocity: `adaptive.encoder: true` (enabled by default) adapts the VY encoder noise from innovations. For mecanum and omnidirectional robots that genuinely move laterally, set `encoder.nhc_vy_sigma: 10.0` or higher to effectively disable the lateral NHC constraint regardless of terrain.

---

## IMU ring buffer retrodiction assumes IMU is the highest-rate sensor

The delay compensation mechanism buffers up to 1 second of IMU measurements (at `imu.topic` rate) and replays them when a delayed GPS fix arrives. This assumes IMU arrives at the filter's nominal rate and GPS arrives late. If GPS arrives before IMU (e.g., low-frequency IMU with high-frequency GPS), the buffer does not help.

**Limitation:** The buffer covers GPS latency up to 500 ms by default (`gnss.max_delay_s`). Fixes arriving more than 500 ms late are processed at arrival time with no retrodiction.

---

## Single global config, no per-session adaptation

FusionCore's adaptive noise covariance updates within a session from the innovation sequence. It does not persist learned noise parameters across sessions or adapt its base parameters based on environment type (urban canyon, open field, indoor/outdoor).

**Workaround:** Maintain separate config files for meaningfully different operating environments and select the appropriate one at launch.

---

## VSLAM map frame assumed stationary

When fusing VSLAM pose (`vslam.topic`), FusionCore anchors the offset between the VSLAM map frame and the filter's odometry frame on first message. If the VSLAM system re-localizes to a different part of its map (loop closure, kidnapping recovery), the map origin shifts and the anchored offset becomes stale.

The reinitialization detector (`vslam.reinit_n`) catches large instantaneous jumps: after `vslam.reinit_n` consecutive chi2 rejections (default 10, approximately 2 seconds at 5 Hz), FusionCore re-anchors the map origin to the current filter position and resumes fusion. It does not catch slow drift due to map deformation.

**Workaround:** Trigger a filter reset (`ros2 service call /fusioncore/reset`) after VSLAM re-localization completes.
