# Nav2 Integration

FusionCore is a drop-in odometry source for Nav2.

| FusionCore output | Nav2 uses it for |
|---|---|
| `/fusion/odom` | Set as `odom_topic` in nav2_params.yaml |
| `odom → base_link` TF | Costmaps and planners read this directly |
| `/fusion/pose` | AMCL initial pose, slam_toolbox pose input |
| `/diagnostics` | Nav2-compatible format |

Two things it does **not** publish, so you know where the boundary is:

- **No `map → odom` TF.** FusionCore is an odometry source, not a global localizer. The bundled `nav2_params.yaml` therefore runs every costmap and planner with `global_frame: odom`, which is the correct setup for GPS navigation with no prior map. If you set `global_frame: map` you need something else publishing `map → odom`: AMCL against a static map, slam_toolbox, or a static identity transform.
- **No map.** There is no static map and no AMCL in the GPS path. Planning happens on the local rolling costmap with `allow_unknown: true`.

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

Only the single-point `FromLL` service is provided. `FromLLArray`, `ToLL` (map frame back to lat/lon) and `SetDatum` are not. Nav2's GPS waypoint follower on Jazzy uses `FromLL` only, so this covers that path; see the [migration guide](migration_from_robot_localization.md) for the full service comparison.

---

## Known issue: the collision monitor blocks bringup

**If you launch `fusioncore_nav2.launch.py` and the robot never moves, this is why.** Reported in [issue #73](https://github.com/manankharwar/fusioncore/issues/73).

`fusioncore_nav2.launch.py` includes nav2_bringup's `navigation_launch.py`, which starts `collision_monitor` unconditionally (there is no launch argument to turn it off on Jazzy) and includes it in the set of nodes its lifecycle manager brings up. The bundled `nav2_params.yaml` has no `collision_monitor` section, and that node refuses to configure without one:

```
[ERROR] [collision_monitor]: Error while getting parameters:
        parameter 'observation_sources' is not initialized
```

That matters more than a normal missing-config error, because the collision monitor sits **in the command path**: it subscribes to `cmd_vel_smoothed` and republishes to `cmd_vel`, which is the topic your base controller listens to. If it never activates, planning and control run fine and produce velocities that never reach the wheels.

`observation_sources` is mandatory and cannot be empty, so a lidar-less GPS robot has no valid pass-through configuration to fall back on. Until this ships fixed, the workaround is to bring Nav2 up without the collision monitor: copy `navigation_launch.py`, delete the `collision_monitor` Node and its entry in `lifecycle_nodes`, and point `fusioncore_nav2.launch.py` at your copy. If you do have a laser, the other option is to copy the `collision_monitor` block from `nav2_bringup`'s own `nav2_params.yaml` into yours and set `base_frame_id` to `base_link`.

This is a defect in the bundled Nav2 configuration, not in the filter. FusionCore itself is unaffected: `/fusion/odom` and the TF are published normally throughout.

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
