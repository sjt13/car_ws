"""Launch the car-side YOLO target mapping node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('color_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('target_frame', default_value='odom'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('projection_mode', default_value='depth'),
        DeclareLaunchArgument('sample_point', default_value='bottom_center'),
        DeclareLaunchArgument('depth_scale', default_value='0.01'),
        DeclareLaunchArgument('min_depth_m', default_value='0.03'),
        DeclareLaunchArgument('max_depth_m', default_value='8.0'),
        DeclareLaunchArgument('depth_patch_radius', default_value='4'),
        DeclareLaunchArgument('depth_source', default_value='bbox_region'),
        DeclareLaunchArgument('bbox_depth_quantile', default_value='0.5'),
        DeclareLaunchArgument('max_depth_age_sec', default_value='0.5'),
        DeclareLaunchArgument('rgb_to_depth_scale_x', default_value='0.0'),
        DeclareLaunchArgument('rgb_to_depth_scale_y', default_value='0.0'),
        DeclareLaunchArgument('rgb_to_depth_offset_x', default_value='0.0'),
        DeclareLaunchArgument('rgb_to_depth_offset_y', default_value='0.0'),
        DeclareLaunchArgument('ground_plane_z', default_value='0.0'),
        DeclareLaunchArgument('class_filter', default_value='red_ball,red_cube'),
        DeclareLaunchArgument('min_score', default_value='0.35'),
        DeclareLaunchArgument('max_targets', default_value='1'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.4'),
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
                'color_image_topic': LaunchConfiguration('color_image_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'projection_mode': LaunchConfiguration('projection_mode'),
                'sample_point': LaunchConfiguration('sample_point'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'min_depth_m': LaunchConfiguration('min_depth_m'),
                'max_depth_m': LaunchConfiguration('max_depth_m'),
                'depth_patch_radius': LaunchConfiguration('depth_patch_radius'),
                'depth_source': LaunchConfiguration('depth_source'),
                'bbox_depth_quantile': LaunchConfiguration('bbox_depth_quantile'),
                'max_depth_age_sec': LaunchConfiguration('max_depth_age_sec'),
                'rgb_to_depth_scale_x': LaunchConfiguration('rgb_to_depth_scale_x'),
                'rgb_to_depth_scale_y': LaunchConfiguration('rgb_to_depth_scale_y'),
                'rgb_to_depth_offset_x': LaunchConfiguration('rgb_to_depth_offset_x'),
                'rgb_to_depth_offset_y': LaunchConfiguration('rgb_to_depth_offset_y'),
                'ground_plane_z': LaunchConfiguration('ground_plane_z'),
                'class_filter': LaunchConfiguration('class_filter'),
                'min_score': LaunchConfiguration('min_score'),
                'max_targets': LaunchConfiguration('max_targets'),
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'debug_log': LaunchConfiguration('debug_log'),
            }],
        ),
    ])
