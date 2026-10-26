PHASE 1 PREDICTION, written 2026-10-18 before measuring.

Question: why is var(WZ + B_GZ) = 2.94e-2 rad^2/s^2 when a gyro measures that sum at
100 Hz?

Config says imu.gyro_noise = 0.005 rad/s, so R_gz should be 0.005^2 = 2.5e-5.
Observed sum variance is 2.94e-2, which is 1176x larger.

  Prediction: right after an IMU update, var(WZ + B_GZ) should be about R_gz.
  If it is 2.94e-2 then either R_gz really is about 2.9e-2 (sigma 0.17 rad/s =
  9.8 deg/s, which would be a config or scaling bug, since no MEMS gyro used for
  navigation has 10 deg/s of white noise), or the updates are not being applied.

  My own leading suspect before looking: cfg.adaptive_imu is TRUE in the NCLT config
  and in every probe I have run today. If the adaptive path inflates R it would do
  exactly this, and it would explain why the number is stable across arms.
