"""Start online SLAM with EKF owning odom -> base_footprint TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    odom_yaw_scale = LaunchConfiguration('odom_yaw_scale')

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    use_rviz = LaunchConfiguration('use_rviz')
    use_game_controller = LaunchConfiguration('use_game_controller')
    use_joy_to_cmdvel = LaunchConfiguration('use_joy_to_cmdvel')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    linear_scale = LaunchConfiguration('linear_scale')
    lateral_scale = LaunchConfiguration('lateral_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    smoothing_alpha = LaunchConfiguration('smoothing_alpha')
    max_linear_accel = LaunchConfiguration('max_linear_accel')
    max_lateral_accel = LaunchConfiguration('max_lateral_accel')
    max_angular_accel = LaunchConfiguration('max_angular_accel')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    output_odom_topic = LaunchConfiguration('output_odom_topic')

    full_bringup_launch = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'launch',
        'full_bringup.launch.py',
    ])
    ekf_launch = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'launch',
        'ekf_odom_imu.launch.py',
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    ])
    default_ekf_params = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'config',
        'ekf_odom_imu.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('base_port', default_value='/dev/ttyS9'),
        DeclareLaunchArgument('base_baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('tf_publish_rate', default_value='20.0'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_yaw_scale', default_value='0.91'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser_link'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Standard'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_game_controller', default_value='true'),
        DeclareLaunchArgument('use_joy_to_cmdvel', default_value='true'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.1'),
        DeclareLaunchArgument('linear_scale', default_value='0.25'),
        DeclareLaunchArgument('lateral_scale', default_value='0.20'),
        DeclareLaunchArgument('angular_scale', default_value='0.65'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.25'),
        DeclareLaunchArgument('max_linear_accel', default_value='0.35'),
        DeclareLaunchArgument('max_lateral_accel', default_value='0.35'),
        DeclareLaunchArgument('max_angular_accel', default_value='0.9'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params),
        DeclareLaunchArgument('output_odom_topic', default_value='/odometry/filtered'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_bringup_launch),
            launch_arguments={
                'base_port': base_port,
                'base_baudrate': base_baudrate,
                'cmd_timeout': cmd_timeout,
                'publish_rate': publish_rate,
                'tf_publish_rate': tf_publish_rate,
                'publish_odom_tf': 'false',
                'imu_frame_id': imu_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'odom_yaw_scale': odom_yaw_scale,
                'lidar_port': lidar_port,
                'lidar_baudrate': lidar_baudrate,
                'lidar_frame_id': lidar_frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'use_rviz': use_rviz,
                'use_joy': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                'params_file': ekf_params_file,
                'output_odom_topic': output_odom_topic,
                'use_sim_time': use_sim_time,
                'publish_tf': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            condition=IfCondition(use_game_controller),
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
            }],
            condition=IfCondition(use_joy_to_cmdvel),
        ),
    ])
