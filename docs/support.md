# Support & Integration

FusionCore is Apache 2.0 and free to use forever. If your robot's localization is not
behaving and you want someone to look at it, start with the log review below. It costs
nothing and it tells both of us whether there is something to fix.

---

## Log review: free

Send one recorded rosbag from your robot and I will send back a written findings report.

**What I need:** a bag of the raw sensor topics (IMU, wheel odometry, GNSS fix, and a
heading topic if you have one), ideally 10 minutes or more, from a run where the problem
shows up. A stretch of a minute or so with the robot parked helps. No ground truth needed.

**What you get back:**

- The output of [`bag_report.py`](bag-report.md) on your bag: sensor rates and drops,
  clock offsets between sensors, IMU and wheel yaw-rate sign and scale, gyro bias at rest,
  declared GNSS sigma against measured fix-to-fix scatter, GPS blackouts, and whether your
  IMU orientation is really just accelerometer tilt.
- A short written note on which of those findings matter for your robot, in the order I
  would fix them, and what each one costs you if left alone.
- Where it is useful, FusionCore run on your bag next to whatever you run now, on the same
  input, with both trajectories.

**What it is not:** an accuracy claim. Without a surveyed reference I can tell you what is
inconsistent in your data and what each filter did with it, not which one was right.

**Turnaround:** typically 3 to 5 business days. Your bag is not shared or published
without your written permission.

You can also run the diagnostic yourself, with nothing to build:
`python3 tools/bag_report.py /path/to/your/bag`. See [Bag report](bag-report.md).

---

## Integration: $750 fixed price

If the review finds things worth fixing, this is the step after it. You provide your
sensor setup. I configure FusionCore, verify it on your hardware, and hand you a tested
launch file and YAML config.

**What's included:**

- Review of your sensor setup (IMU model, GPS, wheel odometry, VSLAM)
- Complete YAML configuration for your specific hardware
- Launch file ready to drop into your stack
- Verification against your bag files or live data
- 30 days of follow-up support for issues that arise from the integration

**What you get:** FusionCore running on your robot, not a starting point to figure out yourself.

**Turnaround:** typically 3 to 5 business days after receiving your sensor details and bag files.

---

## Production Support: $500/month

For teams running FusionCore on deployed hardware.

**What's included:**

- 24-hour response SLA for critical bugs
- Guaranteed fixes for reproducible issues
- Certified configuration for your hardware on file
- Direct email access (no GitHub queue)
- Monthly check-in call (optional)
- Priority on feature requests that affect your deployment

**What you get:** someone accountable when localization breaks at the wrong time.

---

## Get started

Email [manan.kharwar@outlook.com](mailto:manan.kharwar@outlook.com) with:

- What robot you're building
- Your sensor setup (IMU model, GPS model, ROS distro)
- What it is doing that it should not, and a link to the bag if you have one

Or [book a 30-minute call](https://calendly.com/manan-kharwar/new-meeting). Response within 24 hours.

---

!!! note "Not sure which option fits?"
    Start with the log review. For general technical questions, open a
    [GitHub Issue](https://github.com/manankharwar/fusioncore/issues): free support for
    genuine technical questions, always.
