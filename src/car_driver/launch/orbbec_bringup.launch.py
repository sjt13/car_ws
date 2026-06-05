"""启动 Astra Pro 的 USB2 兼容 RGB + Depth 输入链路。

这台 Astra Pro 在 ELF2 上会拆成两个设备：
1. Orbbec SDK 设备负责 depth；
2. 普通 UVC 设备负责 RGB。

不要用 `orbbec_camera` 单独同时开彩色和深度，当前实测会在彩色流启动时报
`OB_SENSOR_COLOR Match openni video mode failed`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def str_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('orbbec_product_id', default_value='0x0403'),
        DeclareLaunchArgument('depth_width', default_value='320'),
        DeclareLaunchArgument('depth_height', default_value='240'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('depth_format', default_value='Y12'),

        DeclareLaunchArgument('rgb_device', default_value='/dev/video21'),
        DeclareLaunchArgument('rgb_width', default_value='640'),
        DeclareLaunchArgument('rgb_height', default_value='480'),
        DeclareLaunchArgument('rgb_fps_num', default_value='1'),
        DeclareLaunchArgument('rgb_fps_den', default_value='30'),
        DeclareLaunchArgument('rgb_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('rgb_frame_id', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument(
            'rgb_camera_info_url',
            default_value='file:///home/elf/car/car_ws/src/car_driver/config/camera_color_factory_640x480.yaml',
        ),

        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'product_id': str_param('orbbec_product_id'),
                'camera_name': 'camera',
                'enable_color': False,
                'enable_depth': True,
                'camera_depth_frame_id': 'camera_depth_frame',
                'depth_optical_frame_id': 'camera_depth_optical_frame',
                'depth_width': int_param('depth_width'),
                'depth_height': int_param('depth_height'),
                'depth_fps': int_param('depth_fps'),
                'depth_format': str_param('depth_format'),
                'enable_ir': False,
                'enable_point_cloud': False,
            }],
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='color_camera',
            output='screen',
            parameters=[{
                'video_device': str_param('rgb_device'),
                # v4l2_camera 严格要求 integer_array；LaunchConfiguration 放进数组会被 ROS2 参数系统误判类型。
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
                'pixel_format': str_param('rgb_pixel_format'),
                'camera_frame_id': str_param('rgb_frame_id'),
                'camera_info_url': str_param('rgb_camera_info_url'),
            }],
            remappings=[
                ('image_raw', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
        ),
    ])
