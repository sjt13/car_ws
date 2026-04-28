from launch import LaunchDescription
"""启动“底盘 + 雷达 + AMCL 定位”的定位入口。

这个文件的作用：
1. 启动车体模型、底盘驱动和雷达驱动；
2. include 官方 `nav2_bringup/localization_launch.py`；
3. 拉起 `map_server + amcl + lifecycle_manager_localization`；
4. 可选启动 RViz，做地图匹配与初始位姿设置。

它是当前项目里“稳定可用”的定位底座，不包含完整导航控制链。
"""
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    base_port = LaunchConfiguration('base_port')
    base_baudrate = LaunchConfiguration('base_baudrate')
    cmd_timeout = LaunchConfiguration('cmd_timeout')
    publish_rate = LaunchConfiguration('publish_rate')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_yaw_scale = LaunchConfiguration('odom_yaw_scale')

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('car_description'), 'urdf', 'car.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('car_description'), 'rviz', 'car.rviz']
    )
    localization_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']
    )

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/elf/maps/car_map_v1.yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('car_description'), 'rviz', 'nav2_params.yaml']
            ),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_port', default_value='/dev/ttyS9'),
        DeclareLaunchArgument('base_baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('tf_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_yaw_scale', default_value='0.53'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser_link'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Standard'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='car_driver',
            executable='base_driver_node',
            name='base_driver_node',
            output='screen',
            parameters=[{
                'port': base_port,
                'baudrate': base_baudrate,
                'cmd_timeout': cmd_timeout,
                'publish_rate': publish_rate,
                'tf_publish_rate': tf_publish_rate,
                'imu_frame_id': imu_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'odom_yaw_scale': odom_yaw_scale,
            }],
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baudrate,
                'frame_id': lidar_frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
