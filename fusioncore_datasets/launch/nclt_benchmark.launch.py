"""
NCLT benchmark launch: plays NCLT CSV data through FusionCore and
robot_localization EKF simultaneously, then records both outputs.

Usage:
  ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:=/path/to/nclt/2012-01-08 \
    output_bag:=./results/nclt_2012_01_08

All nodes use simulated time from /clock (published by nclt_player).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, LogInfo)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('fusioncore_datasets')
    fc_config  = os.path.join(pkg, 'config', 'nclt_fusioncore.yaml')
    rl_config  = os.path.join(pkg, 'config', 'rl_ekf.yaml')
    nav_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    data_dir   = LaunchConfiguration('data_dir')
    output_bag = LaunchConfiguration('output_bag')
    rate       = LaunchConfiguration('playback_rate')
    duration   = LaunchConfiguration('duration_s')

    # ── args ──────────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('data_dir',      description='Path to NCLT sequence directory'),
        DeclareLaunchArgument('output_bag',     default_value='./benchmarks/nclt/2012-01-08/bag',
                              description='Output bag path for results'),
        DeclareLaunchArgument('playback_rate',  default_value='1.0',
                              description='Playback speed multiplier'),
        DeclareLaunchArgument('duration_s',     default_value='0.0',
                              description='Seconds of data to play (0 = all)'),
    ]

    # ── NCLT data player ──────────────────────────────────────────────────────
    nclt_player = Node(
        package='fusioncore_datasets',
        executable='nclt_player.py',
        name='nclt_player',
        output='screen',
        parameters=[{
            'data_dir':      data_dir,
            'playback_rate': rate,
            'duration_s':    duration,
            'use_sim_time':  True,
        }],
    )

    # ── static TFs ────────────────────────────────────────────────────────────
    # NCLT: IMU co-located with base_link (Segway RMP center)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': True}],
    )

    # GPS antenna approximately 30cm above base_link
    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf',
        arguments=['--z', '0.3',
                   '--frame-id', 'base_link', '--child-frame-id', 'gnss_link'],
        parameters=[{'use_sim_time': True}],
    )

    # ── FusionCore ────────────────────────────────────────────────────────────
    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fc_config, {'use_sim_time': True}],
    )

    configure_fc = TimerAction(
        period=4.0,
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

    # ── robot_localization EKF + navsat_transform ─────────────────────────────
    rl_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='rl_ekf',
        output='screen',
        remappings=[
            ('odometry/filtered', '/rl/odometry'),   # avoid clashing with FC
        ],
        parameters=[rl_config, {'use_sim_time': True}],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        remappings=[
            ('imu/data',            '/imu/data'),
            ('gps/fix',             '/gnss/fix'),
            ('odometry/filtered',   '/rl/odometry'),
            ('gps/filtered',        '/rl/gps/filtered'),
            ('odometry/gps',        '/gps/odometry'),
        ],
        parameters=[nav_config, {'use_sim_time': True}],
    )

    # ── bag recorder ─────────────────────────────────────────────────────────
    # Records both filter outputs for offline evaluation with tools/evaluate.py
    recorder = TimerAction(
        period=6.0,   # wait for FC configure(4s) + activate before recording
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', output_bag,
                    '/fusion/odom',       # FusionCore output
                    '/rl/odometry',       # robot_localization EKF output
                    '/gnss/fix',          # raw GPS (needed to establish ENU origin)
                    '/clock',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg='Starting NCLT benchmark. Waiting 2s for player to initialize...'),
        nclt_player,
        imu_tf,
        gps_tf,
        fusioncore_node,
        configure_fc,
        activate_fc,
        rl_ekf,
        navsat,
        recorder,
    ])
