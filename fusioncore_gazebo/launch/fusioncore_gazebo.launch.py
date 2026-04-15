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
    world   = os.path.join(gz_pkg, "worlds", "fusioncore_test.sdf")
    config  = os.path.join(gz_pkg, "config",  "fusioncore_gazebo.yaml")
    model_path = os.path.join(gz_pkg, "models")

    # 1: Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        additional_env={"GZ_SIM_RESOURCE_PATH": model_path},
        output="screen"
    )

    # 2: ROS-Gazebo bridge
    # override_timestamps_with_wall_time: true is the key fix.
    # It stamps all bridged messages with wall clock time so FusionCore's
    # wall clock filter and Gazebo's sim time never mismatch.
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
            # /gnss/fix handled by gz_pose_to_gps.py (avoids gz-sim #2163 NavSat bug)
            "/world/fusioncore_test/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/odom/wheels@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # 3: Static TF: imu_link -> base_link
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )

    # 3b: Static TF for Gazebo's namespaced IMU frame
    # Gazebo Harmonic publishes IMU with frame_id fusioncore_robot/imu_link/imu_sensor
    # alongside the clean imu_link frame. This TF entry covers both.
    imu_tf_gz = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_gz",
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link",
                   "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"]
    )

    # 4: Static TF: base_link -> odom (identity at start)
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "odom", "--child-frame-id", "base_link"]
    )

    # 5: FusionCore lifecycle node
    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[config],
    )

    # 6: Configure FusionCore via the launch system's internal event bus.
    # EmitEvent+ChangeState talks directly to the LifecycleNode action without
    # going through DDS, so it works even when 'ros2 lifecycle set' can't
    # discover the node (the root cause of "Node not found" on WSL2).
    configure_cmd = TimerAction(
        period=15.0,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda action: action == fusioncore_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))
        ]
    )

    # 7: Activate FusionCore once it reaches the 'inactive' state.
    # Event-driven: fires immediately after configure succeeds.
    activate_cmd = RegisterEventHandler(
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

    # GPS publisher: converts ground truth odom to NavSatFix
    # Replaces broken Gazebo NavSat sensor (gz-sim issue #2163)
    gps_pub = Node(
        package="fusioncore_gazebo",
        executable="gz_pose_to_gps",
        name="gz_pose_to_gps",
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        bridge,
        imu_tf,
        imu_tf_gz,
        odom_tf,
        fusioncore_node,
        gps_pub,
        configure_cmd,
        activate_cmd,
    ])
