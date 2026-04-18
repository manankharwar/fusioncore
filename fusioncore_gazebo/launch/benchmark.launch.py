"""
FusionCore vs robot_localization benchmark launch file.

Starts:
  - Gazebo Harmonic simulation world
  - ROS-Gazebo bridge (IMU, wheel odom, cmd_vel, ground truth TF, clock)
  - TF publishers
  - FusionCore lifecycle node (auto-configured after 15s)
  - robot_localization ekf_local + navsat_transform + ekf_global

Does NOT start gz_pose_to_gps. The benchmark_runner publishes GPS so it can
control quality (good GPS, bad GPS, GPS outage) per scenario.

Usage:
  Terminal 1: ros2 launch fusioncore_gazebo benchmark.launch.py
  Terminal 2: ros2 run fusioncore_gazebo benchmark_runner
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    gz_pkg  = get_package_share_directory("fusioncore_gazebo")
    fc_pkg  = get_package_share_directory("fusioncore_ros")

    world      = os.path.join(gz_pkg, "worlds", "fusioncore_test.sdf")
    model_path = os.path.join(gz_pkg, "models")
    fc_config  = os.path.join(gz_pkg, "config", "fusioncore_gazebo.yaml")
    rl_config  = os.path.join(gz_pkg, "config", "robot_localization.yaml")

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        additional_env={"GZ_SIM_RESOURCE_PATH": model_path},
        output="screen"
    )

    # ── 2. ROS-Gazebo bridge ──────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "override_timestamps_with_wall_time": True,
            "expand_gz_topic_names": True,
        }],
        remappings=[
            ("/fusioncore_robot/imu_link/imu_sensor", "/imu/data"),
        ],
        arguments=[
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/fusioncore_test/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/odom/wheels@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # ── 3. TF: imu_link → base_link ──────────────────────────────────────────
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )

    imu_tf_gz = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_gz",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link",
                   "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"]
    )

    # Identity odom→base_link at start (FusionCore will take over)
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "odom", "--child-frame-id", "base_link"]
    )

    # Identity map→odom: required by ekf_global (world_frame: map)
    map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "odom"]
    )

    # gnss_link → base_link: the benchmark publishes GPS with frame_id="gnss_link".
    # navsat_transform looks up this transform to apply the GPS antenna offset.
    # Without it, navsat may drop fixes or emit TF errors, unfairly hurting RL.
    # The simulated robot has no physical antenna offset, so this is identity.
    gnss_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gnss_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "gnss_link"]
    )

    # ── 4. FusionCore lifecycle node ──────────────────────────────────────────
    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[fc_config],
    )

    # Auto-configure after 15s (bypasses DDS discovery issues on WSL2)
    configure_fc = TimerAction(
        period=15.0,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda action: action == fusioncore_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))
        ]
    )

    activate_fc = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda action: action == fusioncore_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )

    # ── 5. robot_localization: local EKF (IMU + encoder) ─────────────────────
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_rl",
        output="screen",
        parameters=[rl_config],
        remappings=[
            ("odometry/filtered", "/rl/odom_local"),
        ]
    )

    # ── 6. robot_localization: navsat_transform (GPS → odom frame) ────────────
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_rl",
        output="screen",
        parameters=[rl_config],
        remappings=[
            ("imu/data",          "/imu/data"),
            ("gps/fix",           "/gnss/fix"),
            ("odometry/filtered", "/rl/odom_local"),
            ("odometry/gps",      "/rl/gps_odom"),
        ]
    )

    # ── 7. robot_localization: global EKF (IMU + encoder + GPS odom) ──────────
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_rl",
        output="screen",
        parameters=[rl_config],
        remappings=[
            ("odometry/filtered", "/rl/odom"),
        ]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        imu_tf,
        imu_tf_gz,
        odom_tf,
        map_tf,
        gnss_tf,
        fusioncore_node,
        configure_fc,
        activate_fc,
        ekf_local,
        navsat,
        ekf_global,
    ])
