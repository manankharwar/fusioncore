import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("fusioncore_ros")

    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(pkg, "config", "fusioncore.yaml"),
            description="Path to FusionCore config file"
        ),

        Node(
            package="fusioncore_ros",
            executable="fusioncore_node",
            name="fusioncore",
            output="screen",
            parameters=[config],
        ),
    ])
