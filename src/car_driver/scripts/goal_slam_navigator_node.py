#!/usr/bin/env python3
"""Goal-driven online SLAM navigation helper.

This node receives a task goal in the active SLAM map frame, or transforms it
into that frame, then sends Nav2 either the final goal or a short known-free
temporary goal in the final goal direction.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, LookupException, TransformException, TransformListener


class GoalSlamNavigator(Node):
    def __init__(self) -> None:
        super().__init__('goal_slam_navigator_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topics', '/goal_slam_nav/goal,/task_goal,/uav/task_goal')
        self.declare_parameter('pose_array_topics', '')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('status_topic', '/goal_slam_nav/status')
        self.declare_parameter('final_goal_topic', '/goal_slam_nav/final_goal')
        self.declare_parameter('temporary_goal_topic', '/goal_slam_nav/temporary_goal')
        self.declare_parameter('reached_goal_topic', '/goal_slam_nav/reached_goal')
        self.declare_parameter('tf_timeout_sec', 0.3)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('clearance_radius_m', 0.18)
        self.declare_parameter('goal_tolerance_m', 0.30)
        self.declare_parameter('temporary_goal_distances', '1.5,1.2,1.0,0.8,0.6,0.4')
        self.declare_parameter('temporary_goal_angle_offsets_deg', '0,-10,10,-20,20,-35,35,-50,50')
        self.declare_parameter('retry_period_sec', 2.0)
        self.declare_parameter('max_temporary_goals', 10)
        self.declare_parameter('max_nav_failures', 5)
        self.declare_parameter('debug_log', False)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.navigate_action = str(self.get_parameter('navigate_action').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.clearance_radius_m = float(self.get_parameter('clearance_radius_m').value)
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.temporary_goal_distances = self._parse_float_list(
            str(self.get_parameter('temporary_goal_distances').value)
        )
        self.temporary_goal_angle_offsets = [
            math.radians(value) for value in self._parse_float_list(
                str(self.get_parameter('temporary_goal_angle_offsets_deg').value)
            )
        ]
        self.retry_period_sec = float(self.get_parameter('retry_period_sec').value)
        self.max_temporary_goals = int(self.get_parameter('max_temporary_goals').value)
        self.max_nav_failures = int(self.get_parameter('max_nav_failures').value)
        self.debug_log = self._as_bool(self.get_parameter('debug_log').value)

        self.tf_buffer = Buffer()
        self.tf_listener_node = Node('goal_slam_tf_listener_node')
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.tf_listener_node,
            spin_thread=True,
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

        self.map_msg: Optional[OccupancyGrid] = None
        self.final_goal: Optional[PoseStamped] = None
        self.state = 'IDLE'
        self.active_goal_is_final = False
        self.nav_goal_handle = None
        self.nav_goal_pending = False
        self.temporary_goal_count = 0
        self.nav_failure_count = 0
        self.next_retry_time = self.get_clock().now()
        self.last_status_text = ''
        self.last_tf_error = ''

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.final_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('final_goal_topic').value),
            10,
        )
        self.temporary_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('temporary_goal_topic').value),
            10,
        )
        self.reached_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('reached_goal_topic').value),
            10,
        )

        self._subscriptions = [
            self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 10)
        ]

        for topic in self._parse_topic_list(str(self.get_parameter('goal_topics').value)):
            self._subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, source_topic=topic: self._pose_goal_callback(msg, source_topic),
                    10,
                )
            )

        for topic in self._parse_topic_list(str(self.get_parameter('pose_array_topics').value)):
            self._subscriptions.append(
                self.create_subscription(
                    PoseArray,
                    topic,
                    lambda msg, source_topic=topic: self._pose_array_callback(msg, source_topic),
                    10,
                )
            )

        self.create_timer(0.5, self._tick)
        self._publish_status('IDLE waiting for task goal')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _pose_goal_callback(self, msg: PoseStamped, source_topic: str) -> None:
        transformed = self._transform_pose_to_map(msg)
        if transformed is None:
            self._publish_status(f'RECEIVE_GOAL ignored {source_topic}: transform unavailable')
            return
        self._set_new_goal(transformed, source_topic)

    def _pose_array_callback(self, msg: PoseArray, source_topic: str) -> None:
        if not msg.poses:
            return
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.poses[0]
        self._pose_goal_callback(goal, source_topic)

    def _set_new_goal(self, goal: PoseStamped, source_topic: str) -> None:
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        self.nav_goal_pending = False

        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        self.final_goal = goal
        self.temporary_goal_count = 0
        self.nav_failure_count = 0
        self.state = 'RECEIVE_GOAL'
        self.final_goal_pub.publish(goal)
        p = goal.pose.position
        self._publish_status(
            f'RECEIVE_GOAL from {source_topic}: x={p.x:.3f}, y={p.y:.3f}, frame={self.map_frame}'
        )

    def _tick(self) -> None:
        if self.final_goal is None:
            return
        if self.nav_goal_pending:
            return
        if self.nav_goal_handle is not None:
            self._finish_if_final_goal_is_close(cancel_active_goal=True)
            return
        if self.map_msg is None:
            self._publish_status('WAIT_MAP waiting for slam_toolbox /map')
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self._publish_status('WAIT_NAV2 waiting for /navigate_to_pose')
            return
        if self.get_clock().now() < self.next_retry_time:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._publish_status(f'WAIT_TF waiting for {self.map_frame} -> {self.base_frame}')
            return

        if self._finish_if_final_goal_is_close(robot_pose=robot_pose):
            return

        self.state = 'TRY_PLAN_TO_GOAL'
        if self._pose_is_known_free(self.final_goal):
            self._send_nav_goal(self.final_goal, is_final=True)
            return

        temp_goal = self._choose_temporary_goal(robot_pose, self.final_goal)
        if temp_goal is None:
            self.nav_failure_count += 1
            if self.nav_failure_count >= self.max_nav_failures:
                self.state = 'FAILED_OR_FALLBACK'
                self._publish_status('FAILED_OR_FALLBACK no known-free temporary goal found')
                self.final_goal = None
            else:
                self.state = 'RETRY_PLAN'
                self.next_retry_time = self._time_after(self.retry_period_sec)
                self._publish_status('RETRY_PLAN no temporary goal yet; waiting for map growth')
            return

        self.temporary_goal_count += 1
        if self.temporary_goal_count > self.max_temporary_goals:
            self.state = 'FAILED_OR_FALLBACK'
            self._publish_status('FAILED_OR_FALLBACK temporary goal limit reached')
            self.final_goal = None
            return

        self.temporary_goal_pub.publish(temp_goal)
        self._send_nav_goal(temp_goal, is_final=False)

    def _finish_if_final_goal_is_close(
        self,
        robot_pose: Optional[PoseStamped] = None,
        cancel_active_goal: bool = False,
    ) -> bool:
        if self.final_goal is None:
            return False
        if robot_pose is None:
            robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        if self._distance(robot_pose.pose.position, self.final_goal.pose.position) > self.goal_tolerance_m:
            return False

        if cancel_active_goal and self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

        self.state = 'DONE'
        self._publish_status('DONE final goal is within tolerance')
        self._publish_reached_goal(self.final_goal)
        self.final_goal = None
        return True

    def _publish_reached_goal(self, goal: Optional[PoseStamped]) -> None:
        if goal is None:
            return
        reached = PoseStamped()
        reached.header.stamp = self.get_clock().now().to_msg()
        reached.header.frame_id = self.map_frame
        reached.pose = goal.pose
        self.reached_goal_pub.publish(reached)

    def _send_nav_goal(self, pose: PoseStamped, is_final: bool) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.active_goal_is_final = is_final
        self.state = 'NAV_TO_GOAL' if is_final else 'NAV_TO_FRONTIER_NEAR_GOAL'
        label = 'final' if is_final else 'temporary'
        p = pose.pose.position
        self._publish_status(f'{self.state} sending {label} goal x={p.x:.3f}, y={p.y:.3f}')

        self.nav_goal_pending = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self.nav_goal_pending = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.nav_goal_handle = None
            self.nav_failure_count += 1
            self.state = 'RETRY_PLAN'
            self.next_retry_time = self._time_after(self.retry_period_sec)
            self._publish_status('RETRY_PLAN Nav2 rejected goal')
            return

        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self.nav_goal_handle = None

        if self.final_goal is None and self.state == 'DONE':
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.active_goal_is_final:
                self.state = 'DONE'
                self._publish_status('DONE final goal reached')
                self._publish_reached_goal(self.final_goal)
                self.final_goal = None
            else:
                self.state = 'RETRY_PLAN'
                self.next_retry_time = self._time_after(0.5)
                self._publish_status('RETRY_PLAN temporary goal reached; retrying final goal')
            return

        self.nav_failure_count += 1
        if self.nav_failure_count >= self.max_nav_failures:
            self.state = 'FAILED_OR_FALLBACK'
            self._publish_status(f'FAILED_OR_FALLBACK Nav2 status={result.status}')
            self.final_goal = None
            return

        self.state = 'RETRY_PLAN'
        self.next_retry_time = self._time_after(self.retry_period_sec)
        self._publish_status(f'RETRY_PLAN Nav2 status={result.status}')

    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, TransformException) as exc:
            error_text = str(exc)
            if error_text != self.last_tf_error:
                self.last_tf_error = error_text
                self.get_logger().warning(f'robot pose TF unavailable: {error_text}')
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position = Point(
            x=tf_msg.transform.translation.x,
            y=tf_msg.transform.translation.y,
            z=tf_msg.transform.translation.z,
        )
        pose.pose.orientation = tf_msg.transform.rotation
        self.last_tf_error = ''
        return pose

    def _transform_pose_to_map(self, pose: PoseStamped) -> Optional[PoseStamped]:
        source_frame = pose.header.frame_id or self.map_frame
        if source_frame == self.map_frame:
            out = PoseStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.map_frame
            out.pose = pose.pose
            return out

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time.from_msg(pose.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, TransformException):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except (LookupException, TransformException) as exc:
                self.get_logger().warning(
                    f'cannot transform goal {source_frame} -> {self.map_frame}: {exc}'
                )
                return None

        point = self._transform_point(pose.pose.position, tf_msg)
        yaw = self._yaw_from_quaternion(tf_msg.transform.rotation) + self._yaw_from_quaternion(
            pose.pose.orientation
        )

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.pose.position = point
        out.pose.orientation = self._quaternion_from_yaw(yaw)
        return out

    def _pose_is_known_free(self, pose: PoseStamped) -> bool:
        return self._world_is_known_free(pose.pose.position.x, pose.pose.position.y)

    def _choose_temporary_goal(self, robot_pose: PoseStamped, final_goal: PoseStamped) -> Optional[PoseStamped]:
        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y
        tx = final_goal.pose.position.x
        ty = final_goal.pose.position.y
        dx = tx - rx
        dy = ty - ry
        distance_to_goal = math.hypot(dx, dy)
        if distance_to_goal < 1e-6:
            return None

        heading = math.atan2(dy, dx)
        for dist in self.temporary_goal_distances:
            if dist >= distance_to_goal:
                dist = max(0.3, distance_to_goal - self.goal_tolerance_m)
            for offset in self.temporary_goal_angle_offsets:
                candidate_heading = heading + offset
                x = rx + math.cos(candidate_heading) * dist
                y = ry + math.sin(candidate_heading) * dist
                if self._world_is_known_free(x, y):
                    goal = PoseStamped()
                    goal.header.frame_id = self.map_frame
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position = Point(x=x, y=y, z=0.0)
                    goal.pose.orientation = self._quaternion_from_yaw(heading)
                    return goal
        return None

    def _world_is_known_free(self, x: float, y: float) -> bool:
        if self.map_msg is None:
            return False
        cell = self._world_to_cell(x, y)
        if cell is None:
            return False

        resolution = self.map_msg.info.resolution
        radius_cells = max(0, int(math.ceil(self.clearance_radius_m / resolution)))
        cx, cy = cell
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                if math.hypot(xx - cx, yy - cy) * resolution > self.clearance_radius_m:
                    continue
                value = self._cell_value(xx, yy)
                if value is None or value < 0 or value > self.free_threshold:
                    return False
        return True

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        yaw = self._yaw_from_quaternion(info.origin.orientation)
        dx = x - ox
        dy = y - oy
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        mx = cos_yaw * dx - sin_yaw * dy
        my = sin_yaw * dx + cos_yaw * dy
        cx = int(math.floor(mx / info.resolution))
        cy = int(math.floor(my / info.resolution))
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return cx, cy

    def _cell_value(self, x: int, y: int) -> Optional[int]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return None
        return int(self.map_msg.data[y * info.width + x])

    def _publish_status(self, text: str) -> None:
        if text == self.last_status_text:
            return
        self.last_status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _time_after(self, seconds: float):
        return self.get_clock().now() + Duration(seconds=seconds)

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def _parse_topic_list(value: str) -> List[str]:
        return [item.strip() for item in value.split(',') if item.strip()]

    @staticmethod
    def _parse_float_list(value: str) -> List[float]:
        return [float(item.strip()) for item in value.split(',') if item.strip()]

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    @staticmethod
    def _transform_point(point: Point, tf_msg: TransformStamped) -> Point:
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rot = GoalSlamNavigator._quat_to_matrix(q.x, q.y, q.z, q.w)
        x = rot[0][0] * point.x + rot[0][1] * point.y + rot[0][2] * point.z + t.x
        y = rot[1][0] * point.x + rot[1][1] * point.y + rot[1][2] * point.z + t.y
        z = rot[2][0] * point.x + rot[2][1] * point.y + rot[2][2] * point.z + t.z
        return Point(x=x, y=y, z=z)

    @staticmethod
    def _quat_to_matrix(x: float, y: float, z: float, w: float) -> Tuple[Tuple[float, float, float], ...]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return (
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSlamNavigator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
