import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, EmitEvent,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
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
        parameters=[fc_yaml, {"use_sim_time": True}],
    )

    # Trigger the initial CONFIGURE once the node has spun up. With autostart
    # true (the default in fusioncore_gazebo.yaml) the node then activates
    # itself ~200ms later, so we do not emit ACTIVATE here. The 4s delay lets
    # DDS discovery and the gz bridge settle before the transition fires.
    configure_cmd = TimerAction(period=4.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action == fusioncore_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])

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
        DeclareLaunchArgument("headless",         default_value="false",
            description="Run Gazebo with no GUI (server only). Use for CI or "
                        "offscreen verification; the demo video needs GUI."),
        DeclareLaunchArgument("start_delay",      default_value="18.0",
            description="Seconds (sim time) before the robot starts driving. "
                        "Lower it (e.g. 2.0) to see motion immediately when "
                        "watching the GUI live."),

        # ── Gazebo (GUI) ──────────────────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            additional_env={"GZ_SIM_RESOURCE_PATH": models},
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("headless")),
        ),
        # ── Gazebo (headless server) ──────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", world],
            additional_env={"GZ_SIM_RESOURCE_PATH": models},
            output="screen",
            condition=IfCondition(LaunchConfiguration("headless")),
        ),

        # ── ROS-Gazebo bridge ─────────────────────────────────────────
        # The whole demo runs on sim time (use_sim_time:true everywhere) driven
        # by /clock from Gazebo. Bridged messages keep their Gazebo timestamps,
        # so every filter dt is sim time and is immune to wall-clock stalls
        # (which otherwise blow up the velocity estimate when running headless
        # on a slow filesystem or when the sim is not pinned to real time).
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
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
            parameters=[{"use_sim_time": True}],
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_gz",
            parameters=[{"use_sim_time": True}],
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link",
                       "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"],
        ),
        # GPS antenna frame. The NavSatFix messages are stamped gnss_link, and
        # FusionCore auto-resolves the lever arm from base_link to gnss_link.
        # Matches the antenna position on the robot model (rear-top).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gnss_tf",
            parameters=[{"use_sim_time": True}],
            arguments=["--x", "-0.12", "--y", "0", "--z", "0.12",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", "gnss_link"],
        ),

        # ── GPS publisher: spike / outage / marker ─────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="gz_pose_to_gps",
            name="gz_pose_to_gps",
            output="screen",
            parameters=[{
                "use_sim_time":      True,
                "world_name":        "fusioncore_outdoor",
                "spike_at_s":        LaunchConfiguration("spike_at_s"),
                "spike_duration_s":  3.0,
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
            parameters=[rl_yaml, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),

        # ── FusionCore (configure now, autostart handles activate) ────
        fusioncore_node,
        configure_cmd,

        # ── Lawnmower scenario driver ──────────────────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="scenario_driver",
            name="scenario_driver",
            output="screen",
            parameters=[{"use_sim_time": True,
                         "start_delay_s": LaunchConfiguration("start_delay")}],
        ),

        # ── Path visualiser ────────────────────────────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="path_publisher",
            name="path_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),

        # ── RViz ──────────────────────────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
