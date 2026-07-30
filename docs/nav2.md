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

FusionCore advertises `/fromLL`, which converts lat/lon/alt into the local map frame. This is the service `nav2_waypoint_follower` calls when you send a `FollowGPSWaypoints` goal, so GPS waypoint navigation works against FusionCore with no bridge node and no robot_localization instance running.

Convert a single point by hand:

```bash
ros2 service call /fromLL robot_localization/srv/FromLL \
  "{ll_point: {latitude: 43.2557, longitude: -79.8711, altitude: 0.0}}"
```

The service returns the point in the local ENU frame anchored at the first GPS fix (or at `reference.x/y/z` if you set an explicit origin). Until a fix arrives the reference is unset, so it returns zeros and logs a warning: wait for `/fusion/odom` before sending waypoints.

!!! note "The service type is `robot_localization/srv/FromLL`, deliberately"

    `nav2_waypoint_follower` has that type compiled into its header, and ROS 2 matches services on name **and** type. A service with identical fields under a different type name is invisible to Nav2's client, which then waits forever without ever reporting an error.

    So FusionCore advertises the type Nav2 expects. Only the interface definition is shared; no robot_localization code is linked or executed, and you do not run a robot_localization node. This is what makes FusionCore a drop-in replacement here rather than a migration that quietly breaks your waypoint following.

    **Versions before 0.3.5** advertised `/fromLL` as `fusioncore_ros/srv/FromLL`. Manual `ros2 service call` worked (you supply the type yourself) but `followGpsWaypoints` hung on `waiting for service to appear`. If you are on 0.3.4 or earlier, upgrade. `fusioncore_ros/srv/FromLL` still exists so old builds compile, but nothing serves it.

---

## What the launch file does under the hood

The lifecycle timing is important. `fusioncore_nav2.launch.py`:

1. Starts `fusioncore_node` as a lifecycle node
2. Waits 2 seconds, sends `configure`
3. On `configuring → inactive`, immediately sends `activate`
4. Waits 8 seconds from launch start, then starts Nav2

The 8-second delay gives FusionCore time to activate and publish `odom → base_link`, and gives Nav2's `bt_navigator` TF listener time to warm up before its configure step runs. Without this gap, `bt_navigator` can fail to configure because its TF buffer is empty on cold DDS startup.

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

Also remove AMCL if you are doing GPS-only outdoor navigation. AMCL publishes `map → odom` and needs a static map to localize against. Without a map it has nothing to do and will log constant warnings. The `global_frame` in bt_navigator, global_costmap, and behavior_server must be set to `odom` for GPS-only navigation.

If you are doing indoor navigation with a static map, keep AMCL. FusionCore publishes `odom → base_link`. AMCL publishes `map → odom`. These are different TF edges and are fully compatible: there is no conflict.

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
