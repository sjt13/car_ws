"""Try Orbbec native RGB-D registration for target mapping validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def str_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('orbbec_product_id', default_value='0x0403'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('color_format', default_value='YUYV'),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('depth_format', default_value='Y16'),
        DeclareLaunchArgument('depth_registration', default_value='true'),
        DeclareLaunchArgument('align_mode', default_value='HW'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='true'),
        DeclareLaunchArgument('enable_point_cloud', default_value='false'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'product_id': str_param('orbbec_product_id'),
                'camera_name': 'camera',
                'enable_color': True,
                'enable_depth': True,
                'camera_color_frame_id': 'camera_color_frame',
                'color_optical_frame_id': 'camera_color_optical_frame',
                'camera_depth_frame_id': 'camera_depth_frame',
                'depth_optical_frame_id': 'camera_depth_optical_frame',
                'color_width': int_param('color_width'),
                'color_height': int_param('color_height'),
                'color_fps': int_param('color_fps'),
                'color_format': str_param('color_format'),
                'depth_width': int_param('depth_width'),
                'depth_height': int_param('depth_height'),
                'depth_fps': int_param('depth_fps'),
                'depth_format': str_param('depth_format'),
                'depth_registration': bool_param('depth_registration'),
                'align_mode': str_param('align_mode'),
                'enable_publish_extrinsic': bool_param('enable_publish_extrinsic'),
                'enable_point_cloud': bool_param('enable_point_cloud'),
                'enable_colored_point_cloud': bool_param('enable_colored_point_cloud'),
                'enable_ir': False,
            }],
        ),
    ])
