#!/usr/bin/env python3
"""显示车体 URDF、TF、激光和地图的基础可视化 launch。

这个文件的作用：
1. 通过 xacro 生成 robot_description；
2. 启动 robot_state_publisher / joint_state_publisher；
3. 按需拉起 RViz，用于单独检查模型、TF 和基础显示配置。

它更偏“看模型/看显示”的轻量入口，不负责底盘串口、雷达驱动或导航栈。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('car_description'), 'urdf', 'car.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('car_description'), 'rviz', 'car.rviz']
    )

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
