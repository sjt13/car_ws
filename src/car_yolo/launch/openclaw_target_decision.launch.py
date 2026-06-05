"""Launch the OpenClaw target decision node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_points_topic', default_value='/uav/target_points_map'),
        DeclareLaunchArgument('selected_goal_topic', default_value='/openclaw/selected_goal'),
        DeclareLaunchArgument('target_order_topic', default_value='/openclaw/target_order'),
        DeclareLaunchArgument('decision_topic', default_value='/openclaw/target_decision'),
        DeclareLaunchArgument('decision_text_topic', default_value='/openclaw/target_decision_text'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('base_frames', default_value='base_footprint,base_link'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('openclaw_base_url', default_value='http://127.0.0.1:18789'),
        DeclareLaunchArgument('openclaw_config_path', default_value='/home/elf/.openclaw/openclaw.json'),
        DeclareLaunchArgument('openclaw_token', default_value=''),
        DeclareLaunchArgument('openclaw_agent_id', default_value='main'),
        DeclareLaunchArgument('openclaw_model', default_value='openclaw'),
        DeclareLaunchArgument('decision_language', default_value='zh_CN'),
        DeclareLaunchArgument('request_timeout_sec', default_value='25.0'),
        DeclareLaunchArgument('min_decision_interval_sec', default_value='2.0'),
        DeclareLaunchArgument('max_targets', default_value='20'),
        DeclareLaunchArgument('target_labels', default_value=''),
        DeclareLaunchArgument('class_priority', default_value='red_ball,red_cube'),
        DeclareLaunchArgument('ignore_duplicate_targets', default_value='true'),
        DeclareLaunchArgument('duplicate_position_epsilon_m', default_value='0.02'),
        DeclareLaunchArgument('fallback_on_error', default_value='true'),
        DeclareLaunchArgument('visited_radius_m', default_value='0.45'),
        DeclareLaunchArgument('debug_log', default_value='false'),
        Node(
            package='car_yolo',
            executable='openclaw_target_decision_node',
            name='openclaw_target_decision_node',
            output='screen',
            parameters=[{
                'target_points_topic': LaunchConfiguration('target_points_topic'),
                'selected_goal_topic': LaunchConfiguration('selected_goal_topic'),
                'target_order_topic': LaunchConfiguration('target_order_topic'),
                'decision_topic': LaunchConfiguration('decision_topic'),
                'decision_text_topic': LaunchConfiguration('decision_text_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'base_frames': LaunchConfiguration('base_frames'),
                'tf_timeout_sec': LaunchConfiguration('tf_timeout_sec'),
                'openclaw_base_url': LaunchConfiguration('openclaw_base_url'),
                'openclaw_config_path': LaunchConfiguration('openclaw_config_path'),
                'openclaw_token': LaunchConfiguration('openclaw_token'),
                'openclaw_agent_id': LaunchConfiguration('openclaw_agent_id'),
                'openclaw_model': LaunchConfiguration('openclaw_model'),
                'decision_language': LaunchConfiguration('decision_language'),
                'request_timeout_sec': LaunchConfiguration('request_timeout_sec'),
                'min_decision_interval_sec': LaunchConfiguration('min_decision_interval_sec'),
                'max_targets': LaunchConfiguration('max_targets'),
                'target_labels': LaunchConfiguration('target_labels'),
                'class_priority': LaunchConfiguration('class_priority'),
                'ignore_duplicate_targets': LaunchConfiguration('ignore_duplicate_targets'),
                'duplicate_position_epsilon_m': LaunchConfiguration('duplicate_position_epsilon_m'),
                'fallback_on_error': LaunchConfiguration('fallback_on_error'),
                'visited_radius_m': LaunchConfiguration('visited_radius_m'),
                'debug_log': LaunchConfiguration('debug_log'),
            }],
        ),
    ])
