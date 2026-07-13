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
    config = LaunchConfiguration("fusioncore_config").perform(context)
    env = LaunchConfiguration("env_config").perform(context)
    autoconfigure = (
        LaunchConfiguration("autoconfigure").perform(context).lower() == "true"
    )

    params = [config]
    if env:
        params.append(env)

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
    )

    # Someone else (a lifecycle manager, or you by hand) will bring it up.
    if not autoconfigure:
        return [node]

    # Give the node a couple of seconds to spin up and for DDS discovery to
    # settle, then configure it.
    configure = TimerAction(period=2.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])

    # As soon as it lands in inactive, activate it.
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
            "fusioncore_config",
            default_value=os.path.join(pkg, "config", "fusioncore.yaml"),
            description="Path to hardware/robot config YAML"
        ),
        DeclareLaunchArgument(
            "env_config",
            default_value="",
            description=(
                "Optional environment preset YAML (overrides GPS noise and outlier thresholds). "
                "Choices: env_open.yaml, env_urban.yaml, env_canopy.yaml"
            )
        ),
        DeclareLaunchArgument(
            "autoconfigure",
            default_value="true",
            description=(
                "Bring the lifecycle node up to active automatically. "
                "Set false if a lifecycle manager (e.g. nav2_lifecycle_manager) drives it."
            )
        ),
        OpaqueFunction(function=_make_node),
    ])
