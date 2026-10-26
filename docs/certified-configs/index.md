# Certified Hardware Configs

Community-contributed configurations that have been validated on real hardware. Each entry includes the specific sensor models, the deployment context, and a copy-paste ready YAML.

---

## Certification levels

| Badge | Meaning |
|---|---|
| **Field validated** | Tested on a real robot outdoors with RTK or measured ground truth. Results are documented. |
| **Community contributed** | Contributed by a real deployer who ran it on their hardware. No RTK ground truth, but it runs without diverging. |
| **Validation in progress** | Configuration is correct per datasheet specs. A real user is integrating it and will update when validated. |
| **Untested** | The interface is verified by reading both sides, but nobody has run it. Published so the integration path is discoverable, and labelled honestly so nobody mistakes it for evidence. |

---

## Registry

| Config | IMU | GPS | Platform | Status |
|---|---|---|---|---|
| [Microstrain 3DM-GX3-45 + consumer GPS (NCLT)](microstrain-3dm-gx3-45-segway.md) | Microstrain 3DM-GX3-45 | Consumer-grade receiver (reports 3 m sigma) | Segway RMP campus rover | **Field validated** (12 sequences, 940 min, RTK ground truth; does not beat raw GPS at shipped defaults) |
| [Bosch BNO085 + u-blox ZED-F9P](bosch-bno085-ublox-f9p-outdoor.md) | Bosch BNO085 (9-axis) | u-blox ZED-F9P (standard GPS or RTK) | Differential drive outdoor | **Community contributed** (Oakland University IGVC robot, Agroecology Lab) |
| [Xsens MTi-680G](xsens-mti-680g-fsae.md) | Xsens MTi-680G (IMU + integrated GNSS) | Xsens MTi-680G GNSS (RTK capable) | Formula Student race car (Ackermann) | **Validation in progress** (UniNa Corse, FSAE Italy 2025) |
| [NVIDIA Isaac ROS Visual SLAM](nvidia-isaac-ros-vslam.md) | any (use the same one Isaac ROS uses) | optional | Jetson Orin or Thor with a camera | **Untested** (interface verified by inspection, never run) |

---

## Contribute your config

If FusionCore is running on your robot, contribute your config. It gets credited to you, indexed by Google, and helps the next person with your hardware skip 3 hours of tuning.

**What to submit:**

A pull request to `fusioncore_ros/config/` with:

1. A YAML file named `<imu-model>_<platform>.yaml` (e.g., `vectornav_vn200_husky.yaml`)
2. A doc page at `docs/certified-configs/<imu-model>-<platform>.md` using any existing page as a template
3. A one-line addition to the table above

**Minimum for a "community contributed" entry:**
- You ran FusionCore on a real robot (not simulation)
- The filter converged and tracked your position without diverging
- You can state your sensor models and ROS distro

**To reach "field validated":**
- Include a rosbag or trajectory plot alongside ground truth (RTK, measured loop, or GPS reference)

Open a [pull request](https://github.com/manankharwar/fusioncore/pulls) or [start a discussion](https://github.com/manankharwar/fusioncore/discussions) if you want help formatting the submission.

---

## Why model-specific configs matter

The generic hardware configs (`bno085_custom.yaml`, `clearpath_husky.yaml`) give you correct starting points but say "datasheet-tuned, not yet field-validated." A certified config gives you noise values that were actually verified against real GPS error statistics on a real robot. The difference in `imu.gyro_noise` between a 3DM-GX3-45 and a BNO085 is 6x. Using the wrong value adds 20-40% to your ATE on long runs.
