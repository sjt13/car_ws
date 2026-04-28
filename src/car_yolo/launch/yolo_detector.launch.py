from launch import LaunchDescription
"""启动车端 RKNN YOLO 检测节点的 launch 入口。

这个文件的作用：
1. 给 `yolo_detector_node` 提供模型路径、相机设备、目标平台等参数；
2. 启动车端目标检测链，发布 `/yolo/detections`、标注图和 FPS；
3. 方便在不同相机或不同 RKNN 模型之间切换测试。
"""
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
