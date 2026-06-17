import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


# ── GPS disruption schedule ───────────────────────────────────────────────────
#
# Times are seconds after the GPS node publishes its first fix (~t=0 of launch).
# Robot starts moving at t=18 s (start_delay_s in scenario_driver).
#
#   t=18 s  robot starts lawnmower Row 1 (East)
#   t=40 s  Row 1 ends, U-turn
#   t=48 s  Row 2 starts (West)  ← Spike 1 at t=50: dramatic, robot going West,
#                                    spike pushes it 60 m East in GPS
#   t=70 s  Row 2 ends, U-turn
#   t=79 s  Row 3 starts (East)  ← Outage at t=82: 25 s blackout "under canopy"
#                                    RL drifts with wheels only; FC coasts
#  t=107 s  Outage ends, GPS resumes
#  t=109 s  Row 4 starts (West)  ← Spike 2 at t=110: GPS just returned, another
#                                    spike tests re-acquisition gating
#  t=131 s  Row 4 ends, stop
#
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():
    gz_pkg   = get_package_share_directory("fusioncore_gazebo")
    rl_yaml  = os.path.join(gz_pkg, "config", "rl_ekf_gazebo.yaml")
    fc_yaml  = os.path.join(gz_pkg, "config", "fusioncore_gazebo.yaml")
    rviz_cfg = os.path.join(gz_pkg, "config", "demo.rviz")
    world    = os.path.join(gz_pkg, "worlds", "fusioncore_outdoor.sdf")
    models   = os.path.join(gz_pkg, "models")

    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[fc_yaml],
    )

    configure_cmd = TimerAction(period=15.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action == fusioncore_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])

    activate_cmd = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda action: action == fusioncore_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([

        # ── User-adjustable args ───────────────────────────────────────
        DeclareLaunchArgument("rviz",             default_value="true"),
        DeclareLaunchArgument("spike_at_s",       default_value="50.0",
            description="Spike 1 time (s): mid Row 2, robot heading West"),
        DeclareLaunchArgument("spike_dx_m",       default_value="60.0",
            description="Spike 1 east offset (m)"),
        DeclareLaunchArgument("outage_at_s",      default_value="82.0",
            description="GPS blackout start (s): Row 3, simulates canopy"),
        DeclareLaunchArgument("outage_duration_s", default_value="25.0"),
        DeclareLaunchArgument("spike2_at_s",      default_value="110.0",
            description="Spike 2 time (s): GPS just resumed, Row 4"),
        DeclareLaunchArgument("spike2_dx_m",      default_value="-60.0",
            description="Spike 2 east offset (m, negative = West)"),

        # ── Gazebo ────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            additional_env={"GZ_SIM_RESOURCE_PATH": models},
            output="screen",
        ),

        # ── ROS-Gazebo bridge ─────────────────────────────────────────
        # override_timestamps_with_wall_time avoids sim-time / wall-time
        # mismatch that would prevent FusionCore from receiving any data.
        Node(
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
                "/world/fusioncore_outdoor/pose/info"
                    "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/odom/wheels@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
        ),

        # ── Static TFs ────────────────────────────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf",
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_gz",
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link",
                       "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"],
        ),

        # ── GPS publisher: spike / outage / marker ─────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="gz_pose_to_gps",
            name="gz_pose_to_gps",
            output="screen",
            parameters=[{
                "world_name":        "fusioncore_outdoor",
                "spike_at_s":        LaunchConfiguration("spike_at_s"),
                "spike_duration_s":  8.0,
                "spike_dx_m":        LaunchConfiguration("spike_dx_m"),
                "spike_dy_m":        0.0,
                "outage_at_s":       LaunchConfiguration("outage_at_s"),
                "outage_duration_s": LaunchConfiguration("outage_duration_s"),
                "spike2_at_s":       LaunchConfiguration("spike2_at_s"),
                "spike2_duration_s": 6.0,
                "spike2_dx_m":       LaunchConfiguration("spike2_dx_m"),
                "spike2_dy_m":       0.0,
            }],
        ),

        # ── robot_localization EKF ─────────────────────────────────────
        # No rejection threshold: RL accepts all spikes and diverges.
        # That divergence is the demo.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="rl_ekf",
            output="screen",
            parameters=[rl_yaml],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),

        # ── FusionCore ────────────────────────────────────────────────
        fusioncore_node,
        configure_cmd,
        activate_cmd,

        # ── Lawnmower scenario driver ──────────────────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="scenario_driver",
            name="scenario_driver",
            output="screen",
            parameters=[{"start_delay_s": 18.0}],
        ),

        # ── Path visualiser ────────────────────────────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="path_publisher",
            name="path_publisher",
            output="screen",
        ),

        # ── RViz ──────────────────────────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
