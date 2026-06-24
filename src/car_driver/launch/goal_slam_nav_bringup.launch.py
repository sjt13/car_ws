"""Start goal-driven online SLAM navigation without AMCL."""

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
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav_params_file = LaunchConfiguration('nav_params_file')
    mapper_params_file = LaunchConfiguration('mapper_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    output_odom_topic = LaunchConfiguration('output_odom_topic')

    uav_map_x = LaunchConfiguration('uav_map_x')
    uav_map_y = LaunchConfiguration('uav_map_y')
    uav_map_z = LaunchConfiguration('uav_map_z')
    uav_map_roll = LaunchConfiguration('uav_map_roll')
    uav_map_pitch = LaunchConfiguration('uav_map_pitch')
    uav_map_yaw = LaunchConfiguration('uav_map_yaw')
    use_uav_target_bridge = LaunchConfiguration('use_uav_target_bridge')
    use_openclaw_goal_decision = LaunchConfiguration('use_openclaw_goal_decision')
    use_uav_model = LaunchConfiguration('use_uav_model')

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
    navigation_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py',
    ])
    uav_model_launch = PathJoinSubstitution([
        FindPackageShare('car_description'),
        'launch',
        'uav_model_display.launch.py',
    ])
    default_ekf_params = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'config',
        'ekf_odom_imu.yaml',
    ])
    default_mapper_params = PathJoinSubstitution([
        FindPackageShare('car_driver'),
        'config',
        'mapper_params_online_async.yaml',
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value='/home/elf/car/car_ws/src/car_description/rviz/nav2_params_goal_slam.yaml',
        ),
        DeclareLaunchArgument('mapper_params_file', default_value=default_mapper_params),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params),
        DeclareLaunchArgument('output_odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('uav_map_x', default_value='0.0'),
        DeclareLaunchArgument('uav_map_y', default_value='0.0'),
        DeclareLaunchArgument('uav_map_z', default_value='0.0'),
        DeclareLaunchArgument('uav_map_roll', default_value='0.0'),
        DeclareLaunchArgument('uav_map_pitch', default_value='0.0'),
        DeclareLaunchArgument('uav_map_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_uav_target_bridge', default_value='true'),
        DeclareLaunchArgument('use_openclaw_goal_decision', default_value='true'),
        DeclareLaunchArgument('use_uav_model', default_value='true'),
        DeclareLaunchArgument('publish_uav_static_pose', default_value='false'),
        DeclareLaunchArgument('pose_array_topic', default_value='/uav/target_points'),
        DeclareLaunchArgument('source_target_topic', default_value=''),
        DeclareLaunchArgument('detections_topic', default_value='/uav/target_detections'),
        DeclareLaunchArgument('target_points_topic', default_value='/uav/target_points_map'),
        DeclareLaunchArgument(
            'goal_topics',
            default_value='/goal_slam_nav/goal,/task_goal,/uav/task_goal',
        ),
        DeclareLaunchArgument('pose_array_topics', default_value=''),

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
                'slam_params_file': mapper_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_params_file,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': use_respawn,
                'log_level': log_level,
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_uav_map_static_tf_goal_slam',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uav_model_launch),
            condition=IfCondition(use_uav_model),
            launch_arguments={
                'uav_map_frame': 'uav_map',
                'uav_base_frame': 'uav/base_link',
                'uav_x': '0.0',
                'uav_y': '0.0',
                'uav_z': '1.0',
                'uav_yaw': '0.0',
                'publish_static_pose': LaunchConfiguration('publish_uav_static_pose'),
            }.items(),
        ),
        Node(
            package='car_yolo',
            executable='uav_target_bridge_node',
            name='uav_goal_slam_target_bridge_node',
            output='screen',
            condition=IfCondition(use_uav_target_bridge),
            parameters=[{
                'pose_array_topic': LaunchConfiguration('pose_array_topic'),
                'pose_stamped_topic': LaunchConfiguration('source_target_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'target_frame': 'map',
                'points_topic': LaunchConfiguration('target_points_topic'),
                'markers_topic': '/uav/target_markers',
                'marker_republish_hz': 0.5,
                'detections_max_publish_hz': 2.0,
                'marker_scale': 0.35,
            }],
        ),
        Node(
            package='car_yolo',
            executable='openclaw_goal_decision_node',
            name='openclaw_goal_decision_node',
            output='screen',
            condition=IfCondition(use_openclaw_goal_decision),
            parameters=[{
                'target_points_topic': LaunchConfiguration('target_points_topic'),
                'target_markers_topic': '/uav/target_markers',
                'control_topic': '/openclaw_goal/control',
                'selected_goal_topic': '/openclaw_goal/selected_goal',
                'task_goal_topic': '/uav/task_goal',
                'target_order_topic': '/openclaw_goal/target_order',
                'decision_topic': '/openclaw_goal/decision',
                'decision_text_topic': '/openclaw_goal/decision_text',
                'reply_text_topic': '/openclaw_goal/reply_text',
                'mission_status_topic': '/openclaw_goal/mission_status',
                'reached_goal_topic': '/goal_slam_nav/reached_goal',
                'nav_status_topic': '/goal_slam_nav/status',
                'target_frame': 'map',
                'base_frames': 'base_footprint,base_link',
                'class_priority': 'red_ball,red_cube',
                'request_timeout_sec': 60.0,
                'send_goal_on_fallback': False,
                'multi_goal_auto': True,
                'continuous_auto': True,
                'continuous_decision_period_sec': 5.0,
                'goal_reached_match_radius_m': 0.60,
                'visited_radius_m': 0.45,
            }],
        ),
        Node(
            package='car_driver',
            executable='goal_slam_navigator_node.py',
            name='goal_slam_navigator_node',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': base_frame_id,
                'goal_topics': LaunchConfiguration('goal_topics'),
                'pose_array_topics': LaunchConfiguration('pose_array_topics'),
                'map_topic': '/map',
                'navigate_action': '/navigate_to_pose',
                'minimum_temporary_goal_distance_m': 0.60,
            }],
        ),
    ])
