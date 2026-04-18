from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/elf/rknn/yolo11/model/yolo11n.rknn'),
        DeclareLaunchArgument('target', default_value='rk3588'),
        DeclareLaunchArgument('device_id', default_value=''),
        DeclareLaunchArgument('camera_device', default_value='/dev/video21'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_link'),
        DeclareLaunchArgument('timer_hz', default_value='15.0'),
        Node(
            package='car_yolo',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'target': LaunchConfiguration('target'),
                'device_id': LaunchConfiguration('device_id'),
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'timer_hz': LaunchConfiguration('timer_hz'),
            }],
        ),
    ])
