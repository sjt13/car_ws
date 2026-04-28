from launch import LaunchDescription
"""只启动底盘驱动 + 机器人模型发布的最小 bringup。

这个文件的作用：
1. 发布 URDF 对应的 robot_description；
2. 启动 `base_driver_node`，把 /cmd_vel 下发到底盘；
3. 为后续 odom、IMU、TF、串口联调提供最小入口。

它不包含雷达、手柄和导航，适合单独排查底盘串口链路。
"""
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    cmd_timeout = LaunchConfiguration('cmd_timeout')
    publish_rate = LaunchConfiguration('publish_rate')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('car_description'), 'urdf', 'car.urdf.xacro']
    )

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    driver_parameters = [{
        'port': port,
        'baudrate': baudrate,
        'cmd_timeout': cmd_timeout,
        'publish_rate': publish_rate,
        'tf_publish_rate': tf_publish_rate,
        'imu_frame_id': imu_frame_id,
        'odom_frame_id': odom_frame_id,
        'base_frame_id': base_frame_id,
    }]

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS9'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('tf_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
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
            parameters=driver_parameters,
        ),
    ])
