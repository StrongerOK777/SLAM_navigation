#!/usr/bin/env python3
"""加载已有地图 + 启动 Nav2 导航栈。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_stage1 = get_package_share_directory('tb3_stage1')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 先算出普通字符串路径
    default_map = os.path.join(pkg_stage1, 'maps', 'stage1_map.yaml')
    default_params = os.path.join(pkg_stage1, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_stage1, 'launch', 'stage1_world.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),

        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': 'true',
                }.items(),
            )
        ]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(
                pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])