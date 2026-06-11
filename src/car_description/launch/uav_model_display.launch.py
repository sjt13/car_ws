"""Publish a UAV model for RViz visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    uav_urdf = PathJoinSubstitution([
        FindPackageShare('car_description'),
        'urdf',
        'uav.urdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('uav_base_frame', default_value='uav/base_link'),
        DeclareLaunchArgument('uav_map_frame', default_value='uav_map'),
        DeclareLaunchArgument('uav_x', default_value='0.0'),
        DeclareLaunchArgument('uav_y', default_value='0.0'),
        DeclareLaunchArgument('uav_z', default_value='1.0'),
        DeclareLaunchArgument('uav_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_static_pose', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='uav_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', uav_urdf]),
                'frame_prefix': '',
            }],
            remappings=[
                ('/robot_description', '/uav/robot_description'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='uav_static_pose_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_static_pose')),
            arguments=[
                '--x', LaunchConfiguration('uav_x'),
                '--y', LaunchConfiguration('uav_y'),
                '--z', LaunchConfiguration('uav_z'),
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', LaunchConfiguration('uav_yaw'),
                '--frame-id', LaunchConfiguration('uav_map_frame'),
                '--child-frame-id', LaunchConfiguration('uav_base_frame'),
            ],
        ),
    ])
