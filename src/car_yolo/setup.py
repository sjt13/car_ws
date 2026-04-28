"""car_yolo 包的 setuptools 安装入口。

这个文件的作用：
1. 定义 `car_yolo` 包的安装信息；
2. 把 launch 文件一起安装到 share 目录；
3. 注册 `yolo_detector_node` 控制台入口，供 `ros2 run` 调用。

它不是推理逻辑本体，真正的检测逻辑在 `car_yolo/yolo_detector_node.py`。
"""

from setuptools import find_packages, setup

package_name = 'car_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elf',
    maintainer_email='sjt03i@163.com',
    description='YOLO detector node for the car-side RKNN validation pipeline.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = car_yolo.yolo_detector_node:main',
        ],
    },
)
