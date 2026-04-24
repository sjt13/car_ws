from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_port = LaunchConfiguration('base_port')
    base_baudrate = LaunchConfiguration('base_baudrate')
    cmd_timeout = LaunchConfiguration('cmd_timeout')
    publish_rate = LaunchConfiguration('publish_rate')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    use_rviz = LaunchConfiguration('use_rviz')
    use_joy = LaunchConfiguration('use_joy')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    linear_scale = LaunchConfiguration('linear_scale')
    lateral_scale = LaunchConfiguration('lateral_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    smoothing_alpha = LaunchConfiguration('smoothing_alpha')
    max_linear_accel = LaunchConfiguration('max_linear_accel')
    max_lateral_accel = LaunchConfiguration('max_lateral_accel')
    max_angular_accel = LaunchConfiguration('max_angular_accel')
    max_linear_decel = LaunchConfiguration('max_linear_decel')
    max_lateral_decel = LaunchConfiguration('max_lateral_decel')
    max_angular_decel = LaunchConfiguration('max_angular_decel')
    target_linear_epsilon = LaunchConfiguration('target_linear_epsilon')
    target_lateral_epsilon = LaunchConfiguration('target_lateral_epsilon')
    target_angular_epsilon = LaunchConfiguration('target_angular_epsilon')

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
        DeclareLaunchArgument('base_port', default_value='/dev/ttyS9'),
        DeclareLaunchArgument('base_baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('tf_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser_link'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Standard'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.1'),
        DeclareLaunchArgument('linear_scale', default_value='0.4'),
        DeclareLaunchArgument('lateral_scale', default_value='0.4'),
        DeclareLaunchArgument('angular_scale', default_value='1.0'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.25'),
        DeclareLaunchArgument('max_linear_accel', default_value='0.6'),
        DeclareLaunchArgument('max_lateral_accel', default_value='0.6'),
        DeclareLaunchArgument('max_angular_accel', default_value='1.5'),
        DeclareLaunchArgument('max_linear_decel', default_value='1.2'),
        DeclareLaunchArgument('max_lateral_decel', default_value='1.2'),
        DeclareLaunchArgument('max_angular_decel', default_value='3.0'),
        DeclareLaunchArgument('target_linear_epsilon', default_value='0.015'),
        DeclareLaunchArgument('target_lateral_epsilon', default_value='0.015'),
        DeclareLaunchArgument('target_angular_epsilon', default_value='0.03'),
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
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(use_joy),
        ),
        Node(
            package='car_teleop',
            executable='joy_to_cmdvel_node',
            name='joy_to_cmdvel_node',
            output='screen',
            parameters=[{
                'deadzone': joy_deadzone,
                'linear_scale': linear_scale,
                'lateral_scale': lateral_scale,
                'angular_scale': angular_scale,
                'smoothing_alpha': smoothing_alpha,
                'max_linear_accel': max_linear_accel,
                'max_lateral_accel': max_lateral_accel,
                'max_angular_accel': max_angular_accel,
                'max_linear_decel': max_linear_decel,
                'max_lateral_decel': max_lateral_decel,
                'max_angular_decel': max_angular_decel,
                'target_linear_epsilon': target_linear_epsilon,
                'target_lateral_epsilon': target_lateral_epsilon,
                'target_angular_epsilon': target_angular_epsilon,
            }],
            condition=IfCondition(use_joy),
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
