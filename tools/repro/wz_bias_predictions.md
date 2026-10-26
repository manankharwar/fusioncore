PREDICTIONS, written 2026-10-18 BEFORE running, per the rule that a prediction
made after seeing the answer is not a prediction.

Hypothesis (expert): P_angvel = 0.2417 (rad/s)^2 means the filter claims not to know
its rotation rate to within ~0.49 rad/s = 28 deg/s, while a gyro measures that rate at
100 Hz. That is only consistent with the WZ / B_GZ pair at r = -0.998: each is
individually huge while their SUM is pinned by the gyro. Heading integrates WZ alone,
so yaw sigma inherits WZ's individual variance.

  P1. var(WZ + B_GZ) = P_ww + P_bb + 2*P_wb will be SMALL, of order the gyro noise
      variance (gyro_noise 0.005 rad/s, so ~2.5e-5), while var(WZ) alone is ~0.24.
      Ratio var(sum)/var(WZ) should be well under 1/100.

  P2. Correlation r(WZ, B_GZ) will be close to -1, consistent with the -0.998 already
      on record.

  P3. In the FROZEN arm, where B_GZ cannot move, yaw sigma will be single digit degrees
      and far below the free arm's 5-70 range.

  P4. This will hold with the attitude flag OFF as well as ON, since the pair is a
      state-vector property and not an attitude-representation one.

If P1 and P3 hold, IMU-as-input is the indicated fix: integrating gyro minus bias
directly removes the pair and its inflated yaw uncertainty by construction.
