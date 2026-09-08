#!/usr/bin/env python3
"""一键启动：世界 + Nav2 + 多点导航节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    waypoints = os.path.join(pkg_stage1, 'config', 'waypoints.yaml')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stage1, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 留 25 秒给 Nav2 起来 + 你 "手动设初始位姿"(因为 Nav2 起来后会自动清除初始位姿)，再启动多点导航节点
    navigator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tb3_stage1',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints_file': waypoints,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        nav,
        navigator,
    ])