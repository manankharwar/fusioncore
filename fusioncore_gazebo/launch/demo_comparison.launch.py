"""
FusionCore vs robot_localization comparison demo.

What happens:
  - Robot drives a slow circle for 90 seconds.
  - At t=60s, a 20m GPS outlier spike is injected (simulates multipath).
  - robot_localization EKF has no outlier rejection: its path visibly jumps.
  - FusionCore gates the spike via chi-squared: its path stays on the true circle.
  - RViz2 shows three coloured trails:
      white  = ground truth (Gazebo pose)
      green  = FusionCore estimate
      red    = robot_localization estimate

Run:
  ros2 launch fusioncore_gazebo demo_comparison.launch.py
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
    gz_pkg     = get_package_share_directory("fusioncore_gazebo")
    world      = os.path.join(gz_pkg, "worlds",  "fusioncore_test.sdf")
    fc_config  = os.path.join(gz_pkg, "config",  "fusioncore_gazebo.yaml")
    rl_config  = os.path.join(gz_pkg, "config",  "robot_localization.yaml")
    rviz_cfg   = os.path.join(gz_pkg, "config",  "demo_comparison.rviz")
    model_path = os.path.join(gz_pkg, "models")

    # 1: Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        additional_env={"GZ_SIM_RESOURCE_PATH": model_path},
        output="screen"
    )

    # 2: ROS-Gazebo bridge
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

    # 3: Static TFs
    imu_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="imu_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )
    imu_tf_gz = Node(
        package="tf2_ros", executable="static_transform_publisher", name="imu_tf_gz",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link",
                   "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"]
    )
    odom_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="odom_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "odom", "--child-frame-id", "base_link"]
    )

    # 4: GPS publisher (with outlier injection at t=60s)
    gps_pub = Node(
        package="fusioncore_gazebo", executable="gz_pose_to_gps",
        name="gz_pose_to_gps", output="screen"
    )

    # 5: FusionCore lifecycle node
    fusioncore_node = LifecycleNode(
        package="fusioncore_ros", executable="fusioncore_node",
        name="fusioncore", namespace="", output="screen",
        parameters=[fc_config],
    )
    configure_cmd = TimerAction(
        period=15.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action == fusioncore_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))]
    )
    activate_cmd = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring", goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda action: action == fusioncore_node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )

    # 6: robot_localization EKF (comparison - no outlier rejection)
    rl_ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", namespace="rl", output="screen",
        parameters=[rl_config],
        remappings=[("/rl/odometry/filtered", "/rl/odometry/filtered")]
    )
    rl_navsat = Node(
        package="robot_localization", executable="navsat_transform_node",
        name="navsat_transform", namespace="rl", output="screen",
        parameters=[rl_config],
        remappings=[
            ("/rl/imu/data",          "/imu/data"),
            ("/rl/gps/fix",           "/gnss/fix"),
            ("/rl/odometry/filtered", "/rl/odometry/filtered"),
            ("/rl/odometry/gps",      "/rl/odometry/gps"),
        ]
    )

    # 7: Path recorder (accumulates both trajectories for RViz2 trail display)
    path_recorder = Node(
        package="fusioncore_gazebo", executable="path_recorder",
        name="path_recorder", output="screen"
    )

    # 8: Autonomous circle driver
    driver = Node(
        package="fusioncore_gazebo", executable="drive_circle",
        name="drive_circle", output="screen"
    )

    # 9: RViz2
    rviz = Node(
        package="rviz2", executable="rviz2",
        name="rviz2", output="screen",
        arguments=["-d", rviz_cfg]
    )

    return LaunchDescription([
        gazebo, bridge,
        imu_tf, imu_tf_gz, odom_tf,
        gps_pub,
        fusioncore_node, configure_cmd, activate_cmd,
        rl_ekf, rl_navsat,
        path_recorder, driver,
        rviz,
    ])
