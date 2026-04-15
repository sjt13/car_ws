#!/usr/bin/env python3
"""将 /joy 手柄输入转换为麦克纳姆底盘的 /cmd_vel 速度指令。"""

from typing import Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToCmdVelNode(Node):
    """订阅手柄数据并发布底盘速度指令。"""

    def __init__(self) -> None:
        super().__init__('joy_to_cmdvel_node')

        # 声明可调参数。
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('linear_scale', 0.4)
        self.declare_parameter('lateral_scale', 0.4)
        self.declare_parameter('angular_scale', 1.0)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        self.get_logger().info('joy_to_cmdvel_node 已启动，订阅 /joy，发布 /cmd_vel。')

    @staticmethod
    def apply_deadzone(value: float, deadzone: float) -> float:
        """对摇杆值做死区处理：在死区内返回 0.0，否则返回原值。"""
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def read_axis(axes: Sequence[float], index: int) -> float:
        """安全读取轴值：若索引不存在则返回 0.0。"""
        if index >= len(axes):
            return 0.0
        return float(axes[index])

    def joy_callback(self, msg: Joy) -> None:
        # 每次回调都读取参数，便于运行时动态调参。
        deadzone = float(self.get_parameter('deadzone').value)
        linear_scale = float(self.get_parameter('linear_scale').value)
        lateral_scale = float(self.get_parameter('lateral_scale').value)
        angular_scale = float(self.get_parameter('angular_scale').value)

        # 1) 读取原始轴值。
        raw_forward = self.read_axis(msg.axes, 1)  # axes[1]：左摇杆上下
        raw_lateral = self.read_axis(msg.axes, 0)  # axes[0]：左摇杆左右
        raw_yaw = self.read_axis(msg.axes, 2)      # axes[2]：右摇杆左右

        # 2) 死区处理：小幅抖动直接视为 0，避免底盘漂移。
        forward_after_deadzone = self.apply_deadzone(raw_forward, deadzone)
        lateral_after_deadzone = self.apply_deadzone(raw_lateral, deadzone)
        yaw_after_deadzone = self.apply_deadzone(raw_yaw, deadzone)

        # 3) 方向约定 / 方向修正。
        # 若实际控制方向相反，把对应符号改为 -1.0 即可。
        forward_sign = 1.0
        lateral_sign = 1.0
        yaw_sign = 1.0
        #
        # linear.x：约定左摇杆上推为正（前进）。
        forward_direction_fixed = forward_after_deadzone * forward_sign
        #
        # linear.y（横移）：
        # ROS base_link 坐标中，+Y 指向车体左侧。
        # 当前映射下：
        # - 摇杆左推  -> linear.y > 0（向左平移）
        # - 摇杆右推  -> linear.y < 0（向右平移）
        lateral_direction_fixed = lateral_after_deadzone * lateral_sign
        #
        # angular.z（旋转）：
        # +angular.z 表示逆时针（左转）。
        # 右推摇杆对应正负，取决于手柄驱动返回的 axes[2] 符号：
        # - 若右推为负值：右推 -> angular.z < 0（右转）
        # - 若右推为正值：右推 -> angular.z > 0（左转）
        yaw_direction_fixed = yaw_after_deadzone * yaw_sign

        # 4) 缩放：把处理后的归一化轴值转换为最终速度值。
        cmd = Twist()
        cmd.linear.x = forward_direction_fixed * linear_scale
        cmd.linear.y = lateral_direction_fixed * lateral_scale
        cmd.angular.z = yaw_direction_fixed * angular_scale

        # 5) 发布 /cmd_vel。
        self.cmd_vel_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # timeout 或 launch 停止时，ROS 上下文可能已提前关闭。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
