from pathlib import Path

from clearpath_config.clearpath_config import ClearpathConfig
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
    ),
    DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Path containing robot.yaml.',
    ),
]


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')

    cc = ClearpathConfig(str(Path(setup_path.perform(context)) / 'robot.yaml'))
    namespace = cc.system.namespace
    ns = f'/{namespace}'

    config = PathJoinSubstitution([
        FindPackageShare('fusioncore_husky'),
        'config',
        'husky_fusioncore.yaml',
    ])

    fc = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace=namespace,
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data', f'{ns}/sensors/imu_0/data'),
            ('/odom/wheels', f'{ns}/platform/odom'),
            ('/gnss/fix', f'{ns}/sensors/gps_0/fix'),
        ],
    )

    lm = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_fusioncore',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['fusioncore'],
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return [fc, lm]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
