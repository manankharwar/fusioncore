"""
NCLT benchmark launch: plays NCLT CSV data through FusionCore, robot_localization EKF,
and robot_localization UKF simultaneously, then records all outputs.

Usage:
  # Normal benchmark
  ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:=/path/to/nclt/2012-01-08 \
    output_bag:=./benchmarks/nclt/2012-01-08/bag \
    playback_rate:=3.0 duration_s:=600.0

  # GPS spike test (500m spike at t=120s)
  ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:=/path/to/nclt/2012-01-08 \
    output_bag:=./benchmarks/nclt/2012-01-08/bag_spike \
    playback_rate:=3.0 duration_s:=300.0 \
    gps_spike_time_s:=120.0 gps_spike_magnitude_m:=500.0

  # GPS outage test (45s outage starting at t=120s)
  ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:=/path/to/nclt/2012-01-08 \
    output_bag:=./benchmarks/nclt/2012-01-08/bag_outage \
    playback_rate:=3.0 duration_s:=300.0 \
    gps_outage_start_s:=120.0 gps_outage_duration_s:=45.0

All nodes use simulated time from /clock (published by nclt_player).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('fusioncore_datasets')
    fc_config  = os.path.join(pkg, 'config', 'nclt_fusioncore.yaml')
    rl_config  = os.path.join(pkg, 'config', 'rl_ekf.yaml')
    rl_ukf_cfg = os.path.join(pkg, 'config', 'rl_ukf.yaml')
    nav_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    data_dir     = LaunchConfiguration('data_dir')
    output_bag   = LaunchConfiguration('output_bag')
    rate         = LaunchConfiguration('playback_rate')
    duration     = LaunchConfiguration('duration_s')
    spike_time   = LaunchConfiguration('gps_spike_time_s')
    spike_mag    = LaunchConfiguration('gps_spike_magnitude_m')
    outage_start = LaunchConfiguration('gps_outage_start_s')
    outage_dur   = LaunchConfiguration('gps_outage_duration_s')

    # ── args ──────────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('data_dir',      description='Path to NCLT sequence directory'),
        DeclareLaunchArgument('output_bag',     default_value='./benchmarks/nclt/2012-01-08/bag',
                              description='Output bag path'),
        DeclareLaunchArgument('playback_rate',  default_value='1.0',
                              description='Playback speed multiplier'),
        DeclareLaunchArgument('duration_s',     default_value='0.0',
                              description='Seconds of data to play (0 = all)'),
        DeclareLaunchArgument('gps_spike_time_s',      default_value='-1.0',
                              description='Sim-time seconds to inject GPS spike (-1 = off)'),
        DeclareLaunchArgument('gps_spike_magnitude_m', default_value='500.0',
                              description='GPS spike magnitude in meters'),
        DeclareLaunchArgument('gps_outage_start_s',    default_value='-1.0',
                              description='Sim-time seconds to begin GPS outage (-1 = off)'),
        DeclareLaunchArgument('gps_outage_duration_s', default_value='45.0',
                              description='GPS outage duration in seconds'),
    ]

    # ── NCLT data player ──────────────────────────────────────────────────────
    nclt_player = Node(
        package='fusioncore_datasets',
        executable='nclt_player.py',
        name='nclt_player',
        output='screen',
        parameters=[{
            'data_dir':               data_dir,
            'playback_rate':          rate,
            'duration_s':             duration,
            'use_sim_time':           True,
            'gps_spike_time_s':       spike_time,
            'gps_spike_magnitude_m':  spike_mag,
            'gps_outage_start_s':     outage_start,
            'gps_outage_duration_s':  outage_dur,
        }],
    )

    # ── static TFs ────────────────────────────────────────────────────────────
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
        remappings=[('odometry/filtered', '/rl/odometry')],
        parameters=[rl_config, {'use_sim_time': True}],
    )

    # ── robot_localization UKF ────────────────────────────────────────────────
    rl_ukf = Node(
        package='robot_localization',
        executable='ukf_node',
        name='rl_ukf',
        output='screen',
        remappings=[('odometry/filtered', '/rl_ukf/odometry')],
        parameters=[rl_ukf_cfg, {'use_sim_time': True}],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        remappings=[
            ('imu/data',          '/imu/data'),
            ('gps/fix',           '/gnss/fix'),
            ('odometry/filtered', '/rl/odometry'),
            ('gps/filtered',      '/rl/gps/filtered'),
            ('odometry/gps',      '/gps/odometry'),
        ],
        parameters=[nav_config, {'use_sim_time': True}],
    )

    # ── bag recorder ─────────────────────────────────────────────────────────
    recorder = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', output_bag,
                    '/fusion/odom',
                    '/rl/odometry',
                    '/rl_ukf/odometry',
                    '/gnss/fix',
                    '/clock',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg='Starting NCLT benchmark...'),
        nclt_player,
        imu_tf,
        gps_tf,
        fusioncore_node,
        configure_fc,
        activate_fc,
        rl_ekf,
        rl_ukf,
        navsat,
        recorder,
    ])
