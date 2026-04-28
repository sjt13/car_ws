"""car_teleop 包的 setuptools 安装入口。

这个文件的作用：
1. 定义包名、资源文件和安装方式；
2. 注册 `joy_to_cmdvel_node` 命令行入口；
3. 让 ROS2 能通过 `ros2 run car_teleop joy_to_cmdvel_node` 启动手柄节点。

它不是运行逻辑本体，真正的控制逻辑在 `car_teleop/joy_to_cmdvel_node.py`。
"""

from setuptools import find_packages, setup

package_name = 'car_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='sjt03i@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_to_cmdvel_node = car_teleop.joy_to_cmdvel_node:main',
        ],
    },
)
