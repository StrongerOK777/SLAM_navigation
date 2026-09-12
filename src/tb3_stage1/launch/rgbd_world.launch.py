#!/usr/bin/env python3
"""启动自定义世界 + 带 RGB-D 相机的 TurtleBot3。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_stage1 = get_package_share_directory('tb3_stage1')

    # URDF 和模型都在自己的包里 —— 别人 clone 仓库即可复现
    world = os.path.join(pkg_stage1, 'worlds', 'stage1_world.world')
    urdf = os.path.join(pkg_stage1, 'urdf', 'turtlebot3_burger_rgbd.urdf')
    sdf = os.path.join(pkg_stage1, 'models', 'turtlebot3_burger_rgbd', 'model.sdf')

    # 让 Gazebo 能解析 model://turtlebot3_common/...（网格来自 apt 装的包）
    # 以及自己包里的模型。这样别人不改 .bashrc 也能跑。
    model_path = ':'.join([
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(pkg_tb3_gazebo, 'models'),
        os.path.join(pkg_stage1, 'models'),
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # 读 URDF 文本，交给 robot_state_publisher
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),

        # robot_state_publisher：读 URDF 发 TF（包含相机坐标系）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
        ),

        # spawn：把带相机的 SDF 塞进 Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'burger_rgbd',
                '-file', sdf,
                '-x', '-3.0', '-y', '-2.0', '-z', '0.01',
            ],
            output='screen',
        ),
    ])