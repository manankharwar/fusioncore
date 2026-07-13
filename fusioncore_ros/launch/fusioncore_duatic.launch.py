import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _make_node(context, *args, **kwargs):
    config = LaunchConfiguration("config").perform(context)
    autoconfigure = (
        LaunchConfiguration("autoconfigure").perform(context).lower() == "true"
    )

    params = [config]

    # When the launch file drives the lifecycle we turn off the node's own
    # self-activate, otherwise both fire and you get a double-activate.
    if autoconfigure:
        params.append({"autostart": False})

    node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=params,
        remappings=[
            # Marc's IMU topic to FusionCore's expected topic
            ("/imu/data",    "/sensor/imu/data"),
            # Marc's mecanum wheel odometry to FusionCore's expected topic
            ("/odom/wheels", "/mecanum_drive_controller/odom"),
        ],
    )

    # Someone else (a lifecycle manager, or you by hand) will bring it up.
    if not autoconfigure:
        return [node]

    configure = TimerAction(period=2.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])

    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        start_state="configuring",
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    return [node, configure, activate]


def generate_launch_description():
    pkg = get_package_share_directory("fusioncore_ros")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(pkg, "config", "duatic_mecanum.yaml"),
            description="Path to FusionCore config file"
        ),
        DeclareLaunchArgument(
            "autoconfigure",
            default_value="true",
            description=(
                "Bring the lifecycle node up to active automatically. "
                "Set false if a lifecycle manager drives it."
            )
        ),
        OpaqueFunction(function=_make_node),
    ])
