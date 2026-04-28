# Environment Presets

Environment presets let you tune GPS noise thresholds for your operating conditions without editing your hardware config. They layer on top: hardware config first, then preset overrides the GPS-specific values.

| Preset | Use case |
|---|---|
| [`env_open.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/env_open.yaml) | Agricultural fields, open sky, minimal multipath |
| [`env_urban.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/env_urban.yaml) | Urban streets, buildings causing multipath |
| [`env_canopy.yaml`](https://github.com/manankharwar/fusioncore/blob/main/fusioncore_ros/config/env_canopy.yaml) | Forest, partial sky view |

---

## Using a preset

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  env_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/env_urban.yaml
```

The preset overrides only the GPS-related noise params. Everything else (IMU noise, encoder noise, UKF process noise) stays from your hardware config.
