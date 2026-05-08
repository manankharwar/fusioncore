"""
FusionCore live demo -- replays the NCLT demo bag through FusionCore
and records the output trajectory for comparison with ground truth.

Does NOT require robot_localization. The comparison plot uses pre-baked
RL results from benchmarks/nclt/2012-01-08/rl_ekf.tum.

Prerequisites:
  1. Build FusionCore and source install/setup.bash
  2. Download the demo bag from GitHub Releases:
       wget -O demo/nclt_demo_120s.mcap \\
         https://github.com/manankharwar/fusioncore/releases/download/demo-data/nclt_demo_120s.mcap

Usage:
  ros2 launch fusioncore demo/nclt_demo.launch.py
  ros2 launch fusioncore demo/nclt_demo.launch.py bag:=/path/to/other.mcap

After the bag finishes (Ctrl+C to stop recording), run:
  bash demo/run_demo.sh --plot-only
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             LogInfo, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

REPO_ROOT    = Path(__file__).resolve().parent.parent
DEFAULT_BAG  = str(REPO_ROOT / 'demo' / 'nclt_demo_120s.mcap')
DEFAULT_OUT  = '/tmp/fc_demo_out'


def generate_launch_description():
    pkg = get_package_share_directory('fusioncore_datasets')
    fc_config = os.path.join(pkg, 'config', 'nclt_fusioncore.yaml')

    bag_path = LaunchConfiguration('bag')
    out_bag  = LaunchConfiguration('output')
    rate     = LaunchConfiguration('rate')

    args = [
        DeclareLaunchArgument('bag',    default_value=DEFAULT_BAG,
                              description='Path to demo MCAP bag file'),
        DeclareLaunchArgument('output', default_value=DEFAULT_OUT,
                              description='Output bag directory for recorded trajectory'),
        DeclareLaunchArgument('rate',   default_value='1.0',
                              description='Playback speed (1.0 = real-time)'),
    ]

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': True}],
    )

    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf',
        arguments=['--z', '0.3',
                   '--frame-id', 'base_link', '--child-frame-id', 'gnss_link'],
        parameters=[{'use_sim_time': True}],
    )

    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fc_config, {'use_sim_time': True}],
    )

    configure_fc = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda a: a == fusioncore_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))
        ],
    )

    activate_fc = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda a: a == fusioncore_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )

    recorder = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record',
                     '-o', out_bag,
                     '/fusion/odom', '/gnss/fix', '/clock'],
                output='screen',
            )
        ],
    )

    player = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', '--rate', rate, '--clock', '100', bag_path],
                output='screen',
            )
        ],
    )

    hint = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg=(
                '\n'
                '====================================================\n'
                ' FusionCore demo running. Wait for bag to finish,\n'
                ' then Ctrl+C and run:  bash demo/run_demo.sh --plot-only\n'
                '====================================================\n'
            )),
        ],
    )

    return LaunchDescription(args + [
        imu_tf,
        gps_tf,
        fusioncore_node,
        configure_fc,
        activate_fc,
        recorder,
        player,
        hint,
    ])
