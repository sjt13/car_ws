"""Start a parallel wheel-odom + MPU6050 EKF for observation only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    output_odom_topic = LaunchConfiguration('output_odom_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'config',
        'ekf_odom_imu.yaml',
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'publish_tf': publish_tf},
        ],
        remappings=[
            ('odometry/filtered', output_odom_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('output_odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        ekf_node,
    ])
