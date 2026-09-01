#!/usr/bin/env python3
"""启动自定义世界 + 生成 TurtleBot3。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_stage1 = get_package_share_directory('tb3_stage1')

    world_path = os.path.join(pkg_stage1, 'worlds', 'stage1_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')

    # 1) gzserver：物理引擎本体。必须挂 init/factory/force_system 插件，
    #    否则 /clock 不发布、spawn_entity 无法生成模型。
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    # 2) gzclient：3D 图形界面。可以关掉以节省资源。
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # 3) robot_state_publisher：读 URDF，发布机器人自身的 TF
    #    （base_footprint → base_link → base_scan ...）
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4) spawn：把 TurtleBot3 的 model.sdf 塞进已经在跑的 Gazebo 里
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用 Gazebo 仿真时间'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 图形界面'),
        DeclareLaunchArgument('x_pose', default_value='-3.0',
                              description='机器人初始 x'),
        DeclareLaunchArgument('y_pose', default_value='-2.0',
                              description='机器人初始 y'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
    ])