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
        self.declare_parameter('smoothing_alpha', 0.25)
        self.declare_parameter('max_linear_accel', 0.6)
        self.declare_parameter('max_lateral_accel', 0.6)
        self.declare_parameter('max_angular_accel', 1.5)
        self.declare_parameter('max_linear_decel', 1.2)
        self.declare_parameter('max_lateral_decel', 1.2)
        self.declare_parameter('max_angular_decel', 3.0)
        self.declare_parameter('output_linear_epsilon', 0.015)
        self.declare_parameter('output_lateral_epsilon', 0.015)
        self.declare_parameter('output_angular_epsilon', 0.03)
        self.declare_parameter('publish_rate', 30.0)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        self.target_cmd = Twist()
        self.current_cmd = Twist()
        self.last_tick_time = self.get_clock().now()

        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            publish_rate = 30.0
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_smoothed_cmd)

        self.get_logger().info(
            'joy_to_cmdvel_node 已启动，订阅 /joy，发布 /cmd_vel，已启用输出平滑和加速度限幅。'
        )

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

    @staticmethod
    def first_order_filter(current: float, target: float, alpha: float) -> float:
        """一阶低通：用 alpha 控制当前值向目标值靠近的速度。"""
        alpha = max(0.0, min(1.0, alpha))
        return current + alpha * (target - current)

    @staticmethod
    def apply_rate_limit(
        current: float,
        target: float,
        max_accel: float,
        max_decel: float,
        dt: float,
    ) -> float:
        """非对称斜坡限幅：加速慢一点，收杆减速快一点。"""
        if dt <= 0.0:
            return target

        delta = target - current
        if delta == 0.0:
            return target

        accelerating_same_direction = current == 0.0 or (current * target > 0.0 and abs(target) > abs(current))
        limit = max_accel if accelerating_same_direction else max_decel
        if limit <= 0.0:
            return target

        max_delta = limit * dt
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    @staticmethod
    def suppress_small_output(value: float, epsilon: float) -> float:
        """把很小的残余输出压成 0，减少底盘被细碎速度反复拨动。"""
        if abs(value) < max(0.0, epsilon):
            return 0.0
        return value

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

        # 4) 先算目标速度，不在回调里立刻发布；交给定时器做平滑和限幅。
        self.target_cmd.linear.x = forward_direction_fixed * linear_scale
        self.target_cmd.linear.y = lateral_direction_fixed * lateral_scale
        self.target_cmd.angular.z = yaw_direction_fixed * angular_scale

    def publish_smoothed_cmd(self) -> None:
        """按固定频率输出平滑后的 /cmd_vel，避免输入抖动直接打到底盘。"""
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds / 1e9
        self.last_tick_time = now
        if dt <= 0.0:
            dt = 1.0 / 30.0

        smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        max_lateral_accel = float(self.get_parameter('max_lateral_accel').value)
        max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        max_linear_decel = float(self.get_parameter('max_linear_decel').value)
        max_lateral_decel = float(self.get_parameter('max_lateral_decel').value)
        max_angular_decel = float(self.get_parameter('max_angular_decel').value)
        output_linear_epsilon = float(self.get_parameter('output_linear_epsilon').value)
        output_lateral_epsilon = float(self.get_parameter('output_lateral_epsilon').value)
        output_angular_epsilon = float(self.get_parameter('output_angular_epsilon').value)

        filtered_linear_x = self.first_order_filter(
            self.current_cmd.linear.x,
            self.target_cmd.linear.x,
            smoothing_alpha,
        )
        filtered_linear_y = self.first_order_filter(
            self.current_cmd.linear.y,
            self.target_cmd.linear.y,
            smoothing_alpha,
        )
        filtered_angular_z = self.first_order_filter(
            self.current_cmd.angular.z,
            self.target_cmd.angular.z,
            smoothing_alpha,
        )

        self.current_cmd.linear.x = self.apply_rate_limit(
            self.current_cmd.linear.x,
            filtered_linear_x,
            max_linear_accel,
            max_linear_decel,
            dt,
        )
        self.current_cmd.linear.y = self.apply_rate_limit(
            self.current_cmd.linear.y,
            filtered_linear_y,
            max_lateral_accel,
            max_lateral_decel,
            dt,
        )
        self.current_cmd.angular.z = self.apply_rate_limit(
            self.current_cmd.angular.z,
            filtered_angular_z,
            max_angular_accel,
            max_angular_decel,
            dt,
        )

        cmd = Twist()
        cmd.linear.x = self.suppress_small_output(self.current_cmd.linear.x, output_linear_epsilon)
        cmd.linear.y = self.suppress_small_output(self.current_cmd.linear.y, output_lateral_epsilon)
        cmd.angular.z = self.suppress_small_output(self.current_cmd.angular.z, output_angular_epsilon)
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
