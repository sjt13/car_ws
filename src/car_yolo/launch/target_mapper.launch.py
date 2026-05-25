"""启动 YOLO 目标落图节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('target_frame', default_value='odom'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('max_depth_age_sec', default_value='0.5'),
        DeclareLaunchArgument('fallback_fx', default_value='554.0'),
        DeclareLaunchArgument('fallback_fy', default_value='554.0'),
        DeclareLaunchArgument('fallback_cx', default_value='320.0'),
        DeclareLaunchArgument('fallback_cy', default_value='240.0'),
        DeclareLaunchArgument('debug_log', default_value='false'),
        Node(
            package='car_yolo',
            executable='target_mapper_node',
            name='target_mapper_node',
            output='screen',
            parameters=[{
                'detections_topic': LaunchConfiguration('detections_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'max_depth_age_sec': LaunchConfiguration('max_depth_age_sec'),
                'fallback_fx': LaunchConfiguration('fallback_fx'),
                'fallback_fy': LaunchConfiguration('fallback_fy'),
                'fallback_cx': LaunchConfiguration('fallback_cx'),
                'fallback_cy': LaunchConfiguration('fallback_cy'),
                'debug_log': LaunchConfiguration('debug_log'),
            }],
        ),
    ])
