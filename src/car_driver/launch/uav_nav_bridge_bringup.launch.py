"""Start car Nav2, the UAV map alignment TF, and the UAV target bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf_odom = LaunchConfiguration('use_ekf_odom')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    uav_map_x = LaunchConfiguration('uav_map_x')
    uav_map_y = LaunchConfiguration('uav_map_y')
    uav_map_z = LaunchConfiguration('uav_map_z')
    uav_map_roll = LaunchConfiguration('uav_map_roll')
    uav_map_pitch = LaunchConfiguration('uav_map_pitch')
    uav_map_yaw = LaunchConfiguration('uav_map_yaw')

    nav_bringup_launch = PathJoinSubstitution(
        [FindPackageShare('car_driver'), 'launch', 'nav_bringup.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/elf/car/car_ws/maps/214map.yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/elf/car/car_ws/src/car_description/rviz/nav2_params_natural.yaml',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_ekf_odom', default_value='true'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        DeclareLaunchArgument('uav_map_x', default_value='-0.78'),
        DeclareLaunchArgument('uav_map_y', default_value='-0.61'),
        DeclareLaunchArgument('uav_map_z', default_value='0.0'),
        DeclareLaunchArgument('uav_map_roll', default_value='0.0'),
        DeclareLaunchArgument('uav_map_pitch', default_value='0.0'),
        DeclareLaunchArgument('uav_map_yaw', default_value='0.0'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('source_target_topic', default_value='/uav/shared/target_pose'),
        DeclareLaunchArgument('target_points_topic', default_value='/uav/target_points_map'),
        DeclareLaunchArgument('target_markers_topic', default_value='/uav/target_markers'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_bringup_launch),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_rviz': use_rviz,
                'publish_odom_tf': publish_odom_tf,
                'use_ekf_odom': use_ekf_odom,
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_uav_map_static_tf',
            output='screen',
            arguments=[
                '--x',
                uav_map_x,
                '--y',
                uav_map_y,
                '--z',
                uav_map_z,
                '--roll',
                uav_map_roll,
                '--pitch',
                uav_map_pitch,
                '--yaw',
                uav_map_yaw,
                '--frame-id',
                'map',
                '--child-frame-id',
                'uav_map',
            ],
        ),
        Node(
            package='car_yolo',
            executable='uav_target_bridge_node',
            name='uav_target_bridge_node',
            output='screen',
            parameters=[{
                'pose_stamped_topic': LaunchConfiguration('source_target_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'points_topic': LaunchConfiguration('target_points_topic'),
                'markers_topic': LaunchConfiguration('target_markers_topic'),
                'marker_republish_hz': 1.0,
                'marker_scale': 0.35,
            }],
        ),
    ])
