import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context, *args, **kwargs):
    config = LaunchConfiguration("fusioncore_config").perform(context)
    env = LaunchConfiguration("env_config").perform(context)

    params = [config]
    if env:
        params.append(env)

    return [Node(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        output="screen",
        parameters=params,
    )]


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
        OpaqueFunction(function=_make_node),
    ])
