#!/usr/bin/env python3
import math
import sys
from typing import List

import rclpy
from geometry_msgs.msg import PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class UavTargetPromptNavigator(Node):
    def __init__(self) -> None:
        super().__init__('uav_target_prompt_navigator')
        self._target_msg = None
        self._sub = self.create_subscription(
            PoseArray,
            '/uav/target_points_map',
            self._target_callback,
            10,
        )
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _target_callback(self, msg: PoseArray) -> None:
        self._target_msg = msg

    def wait_for_targets(self, timeout_sec: float) -> PoseArray:
        end_time = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self._target_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds / 1e9 >= end_time:
                raise TimeoutError('No /uav/target_points_map message received.')
        return self._target_msg

    def send_goal(self, pose_array: PoseArray, index: int, yaw: float) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print('ERROR: /navigate_to_pose action server is not available.')
            return False

        pose = pose_array.poses[index]
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = pose_array.header.frame_id or 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = pose

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('Goal rejected by Nav2.')
            return False

        print('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        print(f'Navigation finished with status: {result.status}')
        return result.status == 4

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            print(f'  distance_remaining: {distance:.2f} m')


def _read_index(max_count: int) -> int:
    if max_count == 1:
        return 0
    while True:
        raw = input(f'Select target index [0-{max_count - 1}] or q to quit: ').strip()
        if raw.lower() in ('q', 'quit', 'exit'):
            raise KeyboardInterrupt
        try:
            index = int(raw)
        except ValueError:
            print('Please input a number.')
            continue
        if 0 <= index < max_count:
            return index
        print('Index out of range.')


def _read_yaw() -> float:
    raw = input('Target yaw in degrees [default 0]: ').strip()
    if not raw:
        return 0.0
    return math.radians(float(raw))


def main() -> int:
    rclpy.init()
    node = UavTargetPromptNavigator()
    try:
        print('Waiting for /uav/target_points_map ...')
        targets = node.wait_for_targets(timeout_sec=8.0)
        frame_id = targets.header.frame_id or 'map'
        if not targets.poses:
            print('No target poses in /uav/target_points_map.')
            return 1

        print(f'Current targets in frame: {frame_id}')
        for i, pose in enumerate(targets.poses):
            p = pose.position
            print(f'  [{i}] x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')

        index = _read_index(len(targets.poses))
        yaw = _read_yaw()
        p = targets.poses[index].position
        answer = input(
            f'Navigate to target [{index}] x={p.x:.3f}, y={p.y:.3f}? [y/N]: '
        ).strip().lower()
        if answer not in ('y', 'yes'):
            print('Cancelled. No goal was sent.')
            return 0

        return 0 if node.send_goal(targets, index, yaw) else 1
    except KeyboardInterrupt:
        print('\nCancelled. No goal was sent.')
        return 0
    except Exception as exc:
        print(f'ERROR: {exc}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
