"""Launch the UAV-to-car target bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_array_topic', default_value='/uav/target_points'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='/uav/shared/target_pose'),
        DeclareLaunchArgument('pose_stamped_source_frame_override', default_value=''),
        DeclareLaunchArgument('detections_topic', default_value='/uav/target_detections'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('points_topic', default_value='/uav/target_points_map'),
        DeclareLaunchArgument('markers_topic', default_value='/uav/target_markers'),
        DeclareLaunchArgument('class_filter', default_value=''),
        DeclareLaunchArgument('min_score', default_value='0.0'),
        DeclareLaunchArgument('max_targets', default_value='20'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('marker_lifetime_sec', default_value='0.0'),
        DeclareLaunchArgument('marker_republish_hz', default_value='1.0'),
        DeclareLaunchArgument('marker_scale', default_value='0.35'),
        DeclareLaunchArgument('force_ground_z', default_value='true'),
        DeclareLaunchArgument('ground_z', default_value='0.0'),
        DeclareLaunchArgument('debug_log', default_value='false'),
        Node(
            package='car_yolo',
            executable='uav_target_bridge_node',
            name='uav_target_bridge_node',
            output='screen',
            parameters=[{
                'pose_array_topic': LaunchConfiguration('pose_array_topic'),
                'pose_stamped_topic': LaunchConfiguration('pose_stamped_topic'),
                'pose_stamped_source_frame_override': LaunchConfiguration('pose_stamped_source_frame_override'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'points_topic': LaunchConfiguration('points_topic'),
                'markers_topic': LaunchConfiguration('markers_topic'),
                'class_filter': LaunchConfiguration('class_filter'),
                'min_score': LaunchConfiguration('min_score'),
                'max_targets': LaunchConfiguration('max_targets'),
                'tf_timeout_sec': LaunchConfiguration('tf_timeout_sec'),
                'marker_lifetime_sec': LaunchConfiguration('marker_lifetime_sec'),
                'marker_republish_hz': LaunchConfiguration('marker_republish_hz'),
                'marker_scale': LaunchConfiguration('marker_scale'),
                'force_ground_z': LaunchConfiguration('force_ground_z'),
                'ground_z': LaunchConfiguration('ground_z'),
                'debug_log': LaunchConfiguration('debug_log'),
            }],
        ),
    ])
