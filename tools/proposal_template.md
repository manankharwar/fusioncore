# FusionCore Integration Proposal

**Date:** [DATE]
**Prepared for:** [COMPANY NAME] / [CONTACT NAME]
**Prepared by:** Manan Kharwar, FusionCore

---

## Your situation

[1-2 sentences describing their specific robot, their current localization stack, and the problem they are experiencing. Be specific: platform, sensors, environment, failure mode. This section should read back to them exactly what they told you.]

Example: "You are running a [platform] in [environment] using [current stack]. The problem you described is [specific failure: GPS spikes near buildings causing 10m position jumps, latency in navsat_transform startup causing Nav2 failures, etc.]."

---

## What I will deliver

A working FusionCore configuration for your platform, tested against your data, with written documentation for your team.

**Specifically:**

1. **Sensor configuration file** - FusionCore YAML tuned for your IMU model, GPS driver, and wheel odometry. Parameters derived from your IMU datasheet, not from trial-and-error.

2. **Launch file** - Drop-in replacement for your current ekf_node + navsat_transform_node launch. Same topic names as your current setup where possible to minimize integration work.

3. **Verification test** - A quick test script (similar to `tools/quick_test.sh`) that confirms FusionCore is receiving all your sensor topics and publishing correct output within 15 seconds. Pass/fail output, no manual inspection.

4. **Written handoff** - One page: what each parameter does for your platform, what to change if you swap sensors, and what the filter will do in your specific failure scenario.

---

## Scope and price

**Fixed-price integration: $750**

Includes everything listed above. One revision cycle after delivery.

Turnaround: [X] business days from receipt of sensor list and ROS bag.

**What I need from you:**
- IMU model (or datasheet ARW / VRW numbers)
- GPS driver and topic name it publishes
- ROS distro (Jazzy or Humble)
- A short rosbag (5-10 min, any conditions) for offline testing - or I can configure against synthetic data if no bag is available yet

**What is not included:**
- Hardware installation or on-site work
- Changes to your Nav2 or path planning stack
- Ongoing maintenance beyond the revision cycle

If you want ongoing support after delivery, that is available separately: Production Support at $500/month covers guaranteed 24-hour response to any FusionCore issue, plus one config update per month if your hardware changes.

---

## Why this is a fixed price

The scope is narrow and I have done this configuration for [N] platforms. The risk is on me, not you. If something in your setup is unusual and I underestimated the work, the price does not change.

---

## Next step

Reply to confirm the scope above and send me your sensor list. I will start the same day.

Questions? Email manan.kharwar@outlook.com or reply here.

---

*FusionCore: [github.com/manankharwar/fusioncore](https://github.com/manankharwar/fusioncore)*
*Documentation: [manankharwar.github.io/fusioncore](https://manankharwar.github.io/fusioncore)*
*arXiv: [arxiv.org/abs/2605.25239](https://arxiv.org/abs/2605.25239)*
