# Contributing to FusionCore

Thanks for your interest. Contributions are welcome: hardware configs, bug fixes, tests, and documentation all help.

## The fastest way to contribute

The most impactful contributions right now are **hardware configs**. If you have FusionCore running on a robot, platform, or IMU that isn't in the repo yet, open a PR adding a YAML under `fusioncore_ros/config/`. See the [hardware config section](#hardware-configs) below.

## Before you start

- Check [open issues](https://github.com/manankharwar/fusioncore/issues): the bug may already be reported
- Check [Discussions](https://github.com/manankharwar/fusioncore/discussions): the question may already be answered
- For anything bigger than a typo fix, open an issue or Discussion first so we can align before you write code

## Development setup

```bash
# Clone and build
git clone https://github.com/manankharwar/fusioncore.git
cd fusioncore

source /opt/ros/jazzy/setup.sh
rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
colcon build --packages-up-to compass_msgs fusioncore_core fusioncore_ros --cmake-args -DBUILD_TESTING=ON

# Run all tests before and after your change
colcon test --packages-select compass_msgs fusioncore_core fusioncore_ros
colcon test-result --verbose
```

All 49 tests must pass. CI will catch it if they don't.

## Hardware configs

A hardware config is a YAML file under `fusioncore_ros/config/` named after the platform (e.g. `clearpath_husky.yaml`, `ublox_f9p.yaml`).

Copy `fusioncore_ros/config/fusioncore.yaml` as the starting point and adjust:
- `imu.gyro_noise` / `imu.accel_noise`: pull from your IMU's datasheet
- `gnss.base_noise_xy`: your GPS receiver's CEP spec
- Any topic remaps specific to your platform

Add a comment at the top with: platform name, IMU model, GPS receiver model, and whether it was field-tested or tuned from datasheet only. Field-tested configs get merged faster.

## Pull request checklist

- [ ] All 49 tests pass (`colcon test-result --verbose` shows 0 failures)
- [ ] For new features: tests added in `fusioncore_core/tests/`
- [ ] For hardware configs: YAML includes a comment with platform + sensor details
- [ ] Commit message describes *why*, not just *what*

## Code style

C++17. Follow the style of the surrounding code: no reformatting unrelated lines. `clang-format` is not enforced but is appreciated.

## Reporting bugs

Use the [Bug Report](.github/ISSUE_TEMPLATE/bug_report.md) issue template. Include the output of `colcon test-result --verbose` if tests are involved.

## Questions

Open a [Discussion](https://github.com/manankharwar/fusioncore/discussions) rather than an issue. Issues are for bugs and tracked work; Discussions are for questions, configs, and ideas.

Response time: typically within 24 hours.
