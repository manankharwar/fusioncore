import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


# ── FusionCore lifecycle node + automated configure → activate ────────────────

def _launch_fusioncore(context, *args, **kwargs):
    pkg = get_package_share_directory("fusioncore_ros")
    config = LaunchConfiguration("fusioncore_config").perform(context)
    env    = LaunchConfiguration("env_config").perform(context)
    sim    = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    params = [config]
    if env:
        params.append(env)
    params.append({"use_sim_time": sim})

    fc = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=params,
    )

    # Wait 2 s for the node to spin up, then send configure
    configure = TimerAction(
        period=2.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is fc,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))],
    )

    # As soon as configuring → inactive, send activate
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=fc,
        start_state="configuring",
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is fc,
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    return [fc, configure, activate]


# ── Nav2: delayed until FusionCore is publishing TF ─────────────────────────

def _launch_nav2(context, *args, **kwargs):
    nav2_params = LaunchConfiguration("nav2_params").perform(context)
    sim         = LaunchConfiguration("use_sim_time").perform(context)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch", "navigation_launch.py",
        )),
        launch_arguments={
            "params_file":    nav2_params,
            "use_sim_time":   sim,
        }.items(),
    )

    # 5 s gives FusionCore time to reach active and start publishing odom → base_link TF
    return [TimerAction(period=5.0, actions=[nav2])]


# ── Launch description ────────────────────────────────────────────────────────

def generate_launch_description():
    pkg = get_package_share_directory("fusioncore_ros")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fusioncore_config",
            default_value=os.path.join(pkg, "config", "fusioncore.yaml"),
            description="Path to FusionCore hardware/robot config YAML",
        ),
        DeclareLaunchArgument(
            "env_config",
            default_value="",
            description=(
                "Optional environment preset YAML: overrides GPS noise and outlier thresholds. "
                "Choices: env_open.yaml, env_urban.yaml, env_canopy.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(pkg, "config", "nav2_params.yaml"),
            description=(
                "Path to Nav2 params YAML. Default is the bundled reference config "
                "(outdoor GPS, differential drive, Regulated Pure Pursuit). "
                "Override with your own file for custom robots or indoor lidar nav."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (set true for Gazebo)",
        ),

        OpaqueFunction(function=_launch_fusioncore),
        OpaqueFunction(function=_launch_nav2),
    ])
