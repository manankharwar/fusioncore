# Nav2 Integration

FusionCore is a drop-in odometry source for Nav2. It publishes everything Nav2 needs out of the box.

| FusionCore output | Nav2 uses it for |
|---|---|
| `/fusion/odom` | Set as `odom_topic` in nav2_params.yaml |
| `odom → base_link` TF | Costmaps and planners read this directly |
| `/fusion/pose` | AMCL initial pose, slam_toolbox pose input |
| `/diagnostics` | Nav2-compatible format |

---

## Outdoor GPS navigation (the main path)

One command starts the entire stack: FusionCore + Nav2, lifecycle managed automatically:

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

The bundled `nav2_params.yaml` is pre-wired to `/fusion/odom` and configured for outdoor GPS navigation: NavFn planner with `allow_unknown: true`, Regulated Pure Pursuit controller, no AMCL, no static map required.

**With an environment preset:**

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  env_config:=$(ros2 pkg prefix fusioncore_ros)/share/fusioncore_ros/config/env_urban.yaml
```

**With your own nav2_params.yaml:**

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=your_robot.yaml \
  nav2_params:=/path/to/your_nav2_params.yaml
```

---

## GPS waypoint navigation

Once FusionCore has a GPS fix, the `fromLL` service converts lat/lon to the local map frame for Nav2:

```bash
ros2 service call /fromLL fusioncore_ros/srv/FromLL \
  "{ll_point: {latitude: 43.2557, longitude: -79.8711, altitude: 0.0}}"
```

---

## What the launch file does under the hood

The lifecycle timing is important. `fusioncore_nav2.launch.py`:

1. Starts `fusioncore_node` as a lifecycle node
2. Waits 2 seconds, sends `configure`
3. On `configuring → inactive`, immediately sends `activate`
4. Waits 5 seconds after activation, then starts Nav2

This 5-second delay guarantees `odom → base_link` TF is publishing before Nav2's costmaps initialize. Without it, Nav2 fails silently at startup.

---

## Updating an existing nav2_params.yaml

If you have your own Nav2 config, find every `odom_topic` and update it:

```yaml
# before
bt_navigator:
  ros__parameters:
    odom_topic: /odometry/filtered

velocity_smoother:
  ros__parameters:
    odom_topic: /odometry/filtered

# after
bt_navigator:
  ros__parameters:
    odom_topic: /fusion/odom

velocity_smoother:
  ros__parameters:
    odom_topic: /fusion/odom
```

Also remove AMCL if you had it. FusionCore publishes the `map → odom` TF directly via GPS. Running AMCL alongside it will cause TF conflicts.

---

## Indoor navigation (no GPS)

FusionCore works without GPS. Set `reference.use_first_fix: false` in your config, and the filter starts at the origin on IMU + wheel odometry alone.

For indoor navigation with a map, run slam_toolbox or AMCL alongside FusionCore. FusionCore handles `odom → base_link`. Your SLAM system handles `map → odom`.

```bash
# FusionCore for odometry
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=your_robot.yaml

# slam_toolbox for map → odom
ros2 launch slam_toolbox online_async_launch.py
```

---

## Coming from robot_localization

See the [Migration Guide](migration_from_robot_localization.md) for a complete parameter mapping and step-by-step instructions.
