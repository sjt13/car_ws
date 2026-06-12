"""Triggered OpenClaw goal decision node for the goal-SLAM navigation chain."""

from __future__ import annotations

import json
import math
import os
import re
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, LookupException, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


RobotPose = Tuple[float, float, float]


class OpenClawGoalDecisionNode(Node):
    """Cache mapped UAV targets and ask OpenClaw only when an operator command arrives."""

    def __init__(self):
        super().__init__('openclaw_goal_decision_node')

        self.declare_parameter('target_points_topic', '/uav/target_points_map')
        self.declare_parameter('target_markers_topic', '/uav/target_markers')
        self.declare_parameter('control_topic', '/openclaw_goal/control')
        self.declare_parameter('selected_goal_topic', '/openclaw_goal/selected_goal')
        self.declare_parameter('task_goal_topic', '/uav/task_goal')
        self.declare_parameter('target_order_topic', '/openclaw_goal/target_order')
        self.declare_parameter('decision_topic', '/openclaw_goal/decision')
        self.declare_parameter('decision_text_topic', '/openclaw_goal/decision_text')
        self.declare_parameter('reply_text_topic', '/openclaw_goal/reply_text')
        self.declare_parameter('mission_status_topic', '/openclaw_goal/mission_status')
        self.declare_parameter('reached_goal_topic', '/goal_slam_nav/reached_goal')
        self.declare_parameter('nav_status_topic', '/goal_slam_nav/status')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frames', 'base_footprint,base_link')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('openclaw_base_url', 'http://127.0.0.1:18789')
        self.declare_parameter('openclaw_config_path', '/home/elf/.openclaw/openclaw.json')
        self.declare_parameter('openclaw_token', '')
        self.declare_parameter('openclaw_agent_id', 'main')
        self.declare_parameter('openclaw_model', 'openclaw')
        self.declare_parameter('decision_language', 'zh_CN')
        self.declare_parameter('request_timeout_sec', 60.0)
        self.declare_parameter('min_decision_interval_sec', 1.0)
        self.declare_parameter('max_targets', 20)
        self.declare_parameter('class_priority', 'red_ball,red_cube')
        self.declare_parameter('fallback_on_error', True)
        self.declare_parameter('send_goal_on_fallback', False)
        self.declare_parameter('multi_goal_auto', True)
        self.declare_parameter('continuous_auto', True)
        self.declare_parameter('continuous_decision_period_sec', 5.0)
        self.declare_parameter('goal_reached_match_radius_m', 0.60)
        self.declare_parameter('visited_radius_m', 0.45)
        self.declare_parameter('debug_log', False)

        self.target_points_topic = str(self.get_parameter('target_points_topic').value)
        self.target_markers_topic = str(self.get_parameter('target_markers_topic').value)
        self.control_topic = str(self.get_parameter('control_topic').value)
        self.selected_goal_topic = str(self.get_parameter('selected_goal_topic').value)
        self.task_goal_topic = str(self.get_parameter('task_goal_topic').value)
        self.target_order_topic = str(self.get_parameter('target_order_topic').value)
        self.decision_topic = str(self.get_parameter('decision_topic').value)
        self.decision_text_topic = str(self.get_parameter('decision_text_topic').value)
        self.reply_text_topic = str(self.get_parameter('reply_text_topic').value)
        self.mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self.reached_goal_topic = str(self.get_parameter('reached_goal_topic').value)
        self.nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.base_frames = self._parse_csv(str(self.get_parameter('base_frames').value))
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.openclaw_base_url = str(self.get_parameter('openclaw_base_url').value).rstrip('/')
        self.openclaw_config_path = str(self.get_parameter('openclaw_config_path').value)
        self.openclaw_token = str(self.get_parameter('openclaw_token').value).strip()
        self.openclaw_agent_id = str(self.get_parameter('openclaw_agent_id').value).strip()
        self.openclaw_model = str(self.get_parameter('openclaw_model').value).strip()
        self.decision_language = str(self.get_parameter('decision_language').value).strip()
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.min_decision_interval_sec = float(self.get_parameter('min_decision_interval_sec').value)
        self.max_targets = int(self.get_parameter('max_targets').value)
        self.class_priority = self._parse_csv(str(self.get_parameter('class_priority').value))
        self.class_priority_map = {
            label: rank for rank, label in enumerate(label.lower() for label in self.class_priority)
        }
        self.fallback_on_error = self._as_bool(self.get_parameter('fallback_on_error').value)
        self.send_goal_on_fallback = self._as_bool(self.get_parameter('send_goal_on_fallback').value)
        self.multi_goal_auto = self._as_bool(self.get_parameter('multi_goal_auto').value)
        self.continuous_auto = self._as_bool(self.get_parameter('continuous_auto').value)
        self.continuous_decision_period_sec = float(
            self.get_parameter('continuous_decision_period_sec').value
        )
        self.goal_reached_match_radius_m = float(self.get_parameter('goal_reached_match_radius_m').value)
        self.visited_radius_m = float(self.get_parameter('visited_radius_m').value)
        self.debug_log = self._as_bool(self.get_parameter('debug_log').value)

        if self.max_targets < 1:
            raise ValueError('max_targets must be >= 1')
        if not self.base_frames:
            raise ValueError('base_frames must contain at least one frame')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_decision_time = self.get_clock().now() - Duration(seconds=9999.0)
        self._last_targets: Optional[PoseArray] = None
        self._last_marker_labels: List[Dict[str, Any]] = []
        self._mission_active = False
        self._mission_queue: List[PoseStamped] = []
        self._mission_current: Optional[PoseStamped] = None
        self._mission_total = 0
        self._mission_done = 0
        self._failure_handled_for_current = False
        self._continuous_auto_active = False
        self._last_continuous_signature: Tuple[Tuple[Any, ...], ...] = tuple()
        self._next_continuous_decision_time = self.get_clock().now()

        self.selected_goal_pub = self.create_publisher(PoseStamped, self.selected_goal_topic, 10)
        self.task_goal_pub = self.create_publisher(PoseStamped, self.task_goal_topic, 10)
        self.target_order_pub = self.create_publisher(PoseArray, self.target_order_topic, 10)
        self.decision_pub = self.create_publisher(String, self.decision_topic, 10)
        self.decision_text_pub = self.create_publisher(String, self.decision_text_topic, 10)
        self.reply_text_pub = self.create_publisher(String, self.reply_text_topic, 10)
        self.mission_status_pub = self.create_publisher(String, self.mission_status_topic, 10)
        self.create_subscription(PoseArray, self.target_points_topic, self.targets_callback, 10)
        self.create_subscription(MarkerArray, self.target_markers_topic, self.markers_callback, 10)
        self.create_subscription(String, self.control_topic, self.control_callback, 10)
        self.create_subscription(PoseStamped, self.reached_goal_topic, self.reached_goal_callback, 10)
        self.create_subscription(String, self.nav_status_topic, self.nav_status_callback, 10)
        self.create_timer(1.0, self.continuous_auto_tick)

        self.get_logger().info(
            'openclaw_goal_decision_node started: '
            f'target_points_topic={self.target_points_topic}, target_markers_topic={self.target_markers_topic}, '
            f'control_topic={self.control_topic}, task_goal_topic={self.task_goal_topic}, '
            f'class_priority={",".join(self.class_priority) or "none"}, '
            f'multi_goal_auto={self.multi_goal_auto}, continuous_auto={self.continuous_auto}'
        )

    def targets_callback(self, msg: PoseArray) -> None:
        self._last_targets = msg

    def markers_callback(self, msg: MarkerArray) -> None:
        labels = []
        for marker in msg.markers:
            if (
                marker.action == Marker.ADD
                and marker.type == Marker.TEXT_VIEW_FACING
                and marker.ns == 'uav_target_labels'
                and marker.text
            ):
                labels.append((marker.id, self._parse_marker_text(marker.text)))
        labels.sort(key=lambda item: item[0])
        self._last_marker_labels = [item[1] for item in labels]

    def control_callback(self, msg: String) -> None:
        mode = self._parse_control_mode(msg.data)
        if mode is None:
            self._publish_text_only(f'无法识别决策命令：{msg.data}')
            return
        if mode == 'stop':
            self._continuous_auto_active = False
            self._reset_mission()
            self._publish_mission_status('MISSION_STOPPED 操作员手动退出，停止持续自动模式')
            return
        if mode == 'auto':
            self._continuous_auto_active = self.continuous_auto
            self._last_continuous_signature = tuple()
            self._run_decision(auto_send_goal=True, force=True)
            return
        self._continuous_auto_active = False
        self._run_decision(auto_send_goal=False, force=True)

    def continuous_auto_tick(self) -> None:
        if not self._continuous_auto_active:
            return
        if self._mission_active:
            return
        if self.get_clock().now() < self._next_continuous_decision_time:
            return
        self._run_decision(auto_send_goal=True, force=False)

    def reached_goal_callback(self, msg: PoseStamped) -> None:
        if not self._mission_active or self._mission_current is None:
            return
        distance = self._pose_distance_2d(msg, self._mission_current)
        if distance > self.goal_reached_match_radius_m:
            self._publish_mission_status(
                f'MISSION_IGNORE_REACHED 距离当前目标 {distance:.2f}m，忽略非当前目标到达反馈'
            )
            return
        self._mission_done += 1
        p = self._mission_current.pose.position
        self._publish_mission_status(
            f'MISSION_GOAL_REACHED {self._mission_done}/{self._mission_total} '
            f'x={p.x:.3f}, y={p.y:.3f}'
        )
        self._mission_current = None
        self._reorder_remaining_mission_goals()
        self._publish_next_mission_goal()

    def nav_status_callback(self, msg: String) -> None:
        if not self._mission_active:
            return
        if 'FAILED_OR_FALLBACK' not in msg.data:
            self._failure_handled_for_current = False
            return
        if self._failure_handled_for_current or self._mission_current is None:
            return
        self._failure_handled_for_current = True
        self._mission_done += 1
        p = self._mission_current.pose.position
        self._publish_mission_status(
            f'MISSION_GOAL_FAILED {self._mission_done}/{self._mission_total} '
            f'x={p.x:.3f}, y={p.y:.3f}，跳过当前目标并继续：{msg.data}'
        )
        self._mission_current = None
        self._reorder_remaining_mission_goals()
        self._publish_next_mission_goal()

    def _run_decision(self, auto_send_goal: bool, force: bool = False) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_decision_time).nanoseconds / 1e9
        if elapsed < self.min_decision_interval_sec:
            self._publish_text_only(f'距离上次决策只有 {elapsed:.2f}s，稍等后再试。')
            return
        self._next_continuous_decision_time = (
            now + Duration(seconds=self.continuous_decision_period_sec)
        )
        self._last_decision_time = now
        self._reset_mission()

        if self._last_targets is None:
            self._publish_empty_decision('当前还没有收到 /uav/target_points_map。', auto_send_goal)
            return

        source_msg = self._last_targets
        targets = list(source_msg.poses[:self.max_targets])
        if not targets:
            self._publish_empty_decision('当前没有可用目标点。', auto_send_goal)
            return

        frame_id = source_msg.header.frame_id or self.target_frame
        robot_pose = self._lookup_robot_pose(frame_id)
        metadata = self._target_metadata(targets, robot_pose)
        candidate_indices = [item['index'] for item in metadata if not item['visited']]
        candidate_signature = self._candidate_signature(metadata)
        if auto_send_goal and self._continuous_auto_active and not force:
            if candidate_signature == self._last_continuous_signature:
                return
            self._last_continuous_signature = candidate_signature
        if not candidate_indices:
            if auto_send_goal and self._continuous_auto_active:
                self._publish_mission_status('MISSION_WAITING_FOR_TARGETS 当前没有新的未到达目标，继续等待')
            self._publish_decision(
                source_msg,
                targets=targets,
                metadata=metadata,
                robot_pose=robot_pose,
                selected_index=None,
                ordered_indices=[],
                reason='所有目标均已到达或被标记为已到达。',
                decision_text='所有目标均已到达或被标记为已到达，本次不再发送新的导航目标。',
                reply_text='这些目标都已经处理过了，我不会重复派车。',
                raw_response='',
                fallback_used=False,
                error='',
                auto_send_goal=auto_send_goal,
            )
            return
        if auto_send_goal and self._continuous_auto_active:
            self._last_continuous_signature = candidate_signature

        selected_index: Optional[int] = None
        ordered_indices: List[int] = []
        reason = ''
        decision_text = ''
        reply_text = ''
        raw_response = ''
        fallback_used = False
        error = ''

        try:
            payload = self._build_openclaw_payload(metadata, frame_id, robot_pose, auto_send_goal)
            raw_response = self._ask_openclaw(payload)
            parsed = self._parse_decision(raw_response, candidate_indices)
            selected_index = parsed['selected_index']
            ordered_indices = parsed['ordered_indices']
            reason = parsed['reason']
            decision_text = parsed['decision_text']
            reply_text = parsed['openclaw_reply_text']
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
            if not self.fallback_on_error:
                self._publish_decision(
                    source_msg,
                    targets=targets,
                    metadata=metadata,
                    robot_pose=robot_pose,
                    selected_index=None,
                    ordered_indices=[],
                    reason='OpenClaw decision failed',
                    decision_text='OpenClaw 没有返回有效决策。',
                    reply_text='OpenClaw 没有返回有效决策。',
                    raw_response=raw_response,
                    fallback_used=False,
                    error=error,
                    auto_send_goal=auto_send_goal,
                )
                return
            selected_index, ordered_indices, reason = self._fallback_decision(metadata)
            if auto_send_goal and not self.send_goal_on_fallback:
                decision_text = (
                    f'OpenClaw 请求失败：{error}。已生成回退推荐目标 {selected_index}，'
                    f'但安全起见没有自动发车。回退原因：{reason}'
                )
            else:
                decision_text = f'回退策略选择目标 {selected_index}：{reason}'
            reply_text = decision_text
            fallback_used = True
            self.get_logger().warning(f'OpenClaw decision failed, using fallback: {error}')

        self._publish_decision(
            source_msg,
            targets=targets,
            metadata=metadata,
            robot_pose=robot_pose,
            selected_index=selected_index,
            ordered_indices=ordered_indices,
            reason=reason,
            decision_text=decision_text,
            reply_text=reply_text,
            raw_response=raw_response,
            fallback_used=fallback_used,
            error=error,
            auto_send_goal=auto_send_goal,
        )

    def _target_metadata(self, targets: Sequence[Pose], robot_pose: Optional[RobotPose]) -> List[Dict[str, Any]]:
        items = []
        for index, pose in enumerate(targets):
            marker = self._last_marker_labels[index] if index < len(self._last_marker_labels) else {}
            label = str(marker.get('label') or 'unknown')
            score = marker.get('score')
            distance = self._distance_to_robot(pose, robot_pose)
            visited = bool(marker.get('visited')) or bool(
                distance is not None and distance <= self.visited_radius_m
            )
            items.append({
                'index': index,
                'label': label,
                'score': None if score is None else float(score),
                'class_priority_rank': self._priority_rank(label),
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
                'distance_m': distance,
                'visited': visited,
            })
        return items

    def _build_openclaw_payload(
        self,
        metadata: Sequence[Dict[str, Any]],
        frame_id: str,
        robot_pose: Optional[RobotPose],
        auto_send_goal: bool,
    ) -> Dict[str, Any]:
        targets = []
        for item in metadata:
            targets.append({
                'index': item['index'],
                'label': item['label'],
                'score': None if item['score'] is None else round(item['score'], 4),
                'class_priority_rank': item['class_priority_rank'],
                'x': round(item['x'], 4),
                'y': round(item['y'], 4),
                'z': round(item['z'], 4),
                'distance_m': None if item['distance_m'] is None else round(item['distance_m'], 4),
                'visited': item['visited'],
            })
        return {
            'frame_id': frame_id,
            'robot_pose': None if robot_pose is None else {
                'x': round(robot_pose[0], 4),
                'y': round(robot_pose[1], 4),
                'yaw': round(robot_pose[2], 4),
            },
            'auto_send_goal': auto_send_goal,
            'visited_radius_m': self.visited_radius_m,
            'decision_language': self.decision_language,
            'class_priority': self.class_priority,
            'targets': targets,
        }

    def _ask_openclaw(self, decision_payload: Dict[str, Any]) -> str:
        token = self._load_openclaw_token()
        if not token:
            raise RuntimeError('OpenClaw token is not configured')

        prompt = (
            'You are choosing the next ground-vehicle target from UAV-provided map targets.\n'
            'Return only a JSON object. Do not include markdown fences.\n'
            'Use observable factors only: label, confidence score, class_priority_rank, distance_m, '
            'visited status, and simple route priority. Do not expose hidden chain-of-thought.\n'
            'Put a complete machine-auditable decision explanation in decision_text. '
            'Put a natural normal OpenClaw-style reply in openclaw_reply_text. '
            'These are explicit task-level explanations, not hidden chain-of-thought.\n'
            'Write reason, decision_text, openclaw_reply_text, and scores[].note in Simplified Chinese.\n'
            'Schema:\n'
            '{"selected_index": int|null, "ordered_indices": [int], "reason": string, '
            '"decision_text": string, "openclaw_reply_text": string, '
            '"scores": [{"index": int, "score": number, "note": string}]}\n'
            'Rules:\n'
            '- Never select targets where visited=true.\n'
            '- Minimize the next driving distance first. Prefer the nearest valid target.\n'
            '- Use class_priority_rank and confidence only as tie-breakers when candidate distances '
            'differ by no more than 0.5 m.\n'
            '- selected_index must be one of the provided target indices or null if no target is valid.\n'
            '- ordered_indices should contain valid unvisited target indices in recommended visit order.\n'
            '- decision_text must be detailed. Include input target list, class priority rule, robot pose '
            'and distances, per-target evaluation, visited-target rejection reasons, final ordered_indices, '
            'and why selected_index was chosen.\n'
            '- openclaw_reply_text should sound like a normal direct reply to the operator.\n\n'
            f'Data:\n{json.dumps(decision_payload, ensure_ascii=False)}'
        )
        body = {'model': self.openclaw_model or 'openclaw', 'input': prompt, 'stream': False}
        data = json.dumps(body).encode('utf-8')
        request = urllib.request.Request(
            f'{self.openclaw_base_url}/v1/responses',
            data=data,
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': 'application/json',
                'x-openclaw-agent-id': self.openclaw_agent_id or 'main',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                response_data = response.read().decode('utf-8', errors='replace')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'OpenClaw HTTP {exc.code}: {detail[:500]}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'OpenClaw request failed: {exc}') from exc

        try:
            parsed = json.loads(response_data)
            return self._extract_response_text(parsed) or response_data
        except json.JSONDecodeError:
            return response_data

    def _parse_decision(self, raw_response: str, valid_indices: Sequence[int]) -> Dict[str, Any]:
        decision = self._extract_json_object(raw_response)
        valid = set(valid_indices)
        selected_index = decision.get('selected_index')
        if selected_index is not None:
            selected_index = int(selected_index)
            if selected_index not in valid:
                raise ValueError(f'selected_index is not a valid unvisited target: {selected_index}')

        ordered_indices = []
        for item in decision.get('ordered_indices', []):
            index = int(item)
            if index in valid and index not in ordered_indices:
                ordered_indices.append(index)
        if selected_index is not None and selected_index not in ordered_indices:
            ordered_indices.insert(0, selected_index)
        for index in valid_indices:
            if index not in ordered_indices:
                ordered_indices.append(index)
        if selected_index is None and ordered_indices:
            selected_index = ordered_indices[0]

        return {
            'selected_index': selected_index,
            'ordered_indices': ordered_indices,
            'reason': str(decision.get('reason', '')).strip() or 'OpenClaw returned a valid decision.',
            'decision_text': str(
                decision.get('decision_text')
                or decision.get('thinking_text')
                or decision.get('visible_thinking')
                or ''
            ).strip(),
            'openclaw_reply_text': str(
                decision.get('openclaw_reply_text')
                or decision.get('reply_text')
                or decision.get('normal_reply')
                or decision.get('reason')
                or ''
            ).strip(),
        }

    def _publish_decision(
        self,
        source_msg: PoseArray,
        targets: Sequence[Pose],
        metadata: Sequence[Dict[str, Any]],
        robot_pose: Optional[RobotPose],
        selected_index: Optional[int],
        ordered_indices: Sequence[int],
        reason: str,
        decision_text: str,
        reply_text: str,
        raw_response: str,
        fallback_used: bool,
        error: str,
        auto_send_goal: bool,
    ) -> None:
        frame_id = source_msg.header.frame_id or self.target_frame
        stamp = self.get_clock().now().to_msg()
        goal_sent = bool(
            auto_send_goal
            and selected_index is not None
            and (not fallback_used or self.send_goal_on_fallback)
        )
        mission_started = False

        if selected_index is not None and 0 <= selected_index < len(targets):
            selected = PoseStamped()
            selected.header.stamp = stamp
            selected.header.frame_id = frame_id
            selected.pose = targets[selected_index]
            self.selected_goal_pub.publish(selected)
            if goal_sent:
                if self.multi_goal_auto:
                    mission_started = self._start_mission(source_msg, targets, ordered_indices)
                else:
                    self.task_goal_pub.publish(selected)

        ordered = PoseArray()
        ordered.header.stamp = stamp
        ordered.header.frame_id = frame_id
        for index in ordered_indices:
            if 0 <= index < len(targets):
                ordered.poses.append(targets[index])
        self.target_order_pub.publish(ordered)

        decision_doc = {
            'selected_index': selected_index,
            'ordered_indices': list(ordered_indices),
            'auto_send_goal': auto_send_goal,
            'goal_sent': goal_sent,
            'mission_started': mission_started,
            'mission_total': self._mission_total if mission_started else 0,
            'send_goal_on_fallback': self.send_goal_on_fallback,
            'reason': reason,
            'decision_text': decision_text,
            'openclaw_reply_text': reply_text,
            'raw_response': raw_response,
            'fallback_used': fallback_used,
            'error': error,
            'frame_id': frame_id,
            'targets': list(metadata),
            'robot_pose': None if robot_pose is None else {
                'x': robot_pose[0],
                'y': robot_pose[1],
                'yaw': robot_pose[2],
            },
            'target_count': len(targets),
        }
        decision_msg = String()
        decision_msg.data = json.dumps(decision_doc, ensure_ascii=False)
        self.decision_pub.publish(decision_msg)

        text_msg = String()
        text_msg.data = self._format_decision_text(decision_doc)
        self.decision_text_pub.publish(text_msg)

        reply_msg = String()
        reply_msg.data = reply_text
        self.reply_text_pub.publish(reply_msg)

        if self.debug_log:
            self.get_logger().info(text_msg.data)

    def _start_mission(
        self,
        source_msg: PoseArray,
        targets: Sequence[Pose],
        ordered_indices: Sequence[int],
    ) -> bool:
        frame_id = source_msg.header.frame_id or self.target_frame
        stamp = self.get_clock().now().to_msg()
        queue = []
        for index in ordered_indices:
            if 0 <= index < len(targets):
                goal = PoseStamped()
                goal.header.stamp = stamp
                goal.header.frame_id = frame_id
                goal.pose = targets[index]
                queue.append(goal)
        if not queue:
            return False
        self._mission_active = True
        self._mission_queue = queue
        self._mission_current = None
        self._mission_total = len(queue)
        self._mission_done = 0
        self._failure_handled_for_current = False
        self._publish_mission_status(f'MISSION_STARTED total={self._mission_total}')
        self._publish_next_mission_goal()
        return True

    def _publish_next_mission_goal(self) -> None:
        if not self._mission_active:
            return
        if not self._mission_queue:
            self._mission_active = False
            self._mission_current = None
            self._publish_mission_status(f'MISSION_COMPLETE total={self._mission_total}')
            done_text = f'多目标任务完成，共处理 {self._mission_total} 个目标。'
            self._publish_text_only(done_text)
            reply = String()
            reply.data = done_text
            self.reply_text_pub.publish(reply)
            if self._continuous_auto_active:
                self._publish_mission_status('MISSION_CONTINUOUS_WAITING 本轮目标完成，继续等待无人机新目标')
                self._next_continuous_decision_time = (
                    self.get_clock().now() + Duration(seconds=self.continuous_decision_period_sec)
                )
            return

        self._mission_current = self._mission_queue.pop(0)
        self._mission_current.header.stamp = self.get_clock().now().to_msg()
        self.selected_goal_pub.publish(self._mission_current)
        self.task_goal_pub.publish(self._mission_current)
        p = self._mission_current.pose.position
        self._publish_mission_status(
            f'MISSION_GOAL_SENT {self._mission_done + 1}/{self._mission_total} '
            f'x={p.x:.3f}, y={p.y:.3f}, remaining={len(self._mission_queue)}'
        )

    def _reorder_remaining_mission_goals(self) -> None:
        if len(self._mission_queue) < 2:
            return
        frame_id = self._mission_queue[0].header.frame_id or self.target_frame
        robot_pose = self._lookup_robot_pose(frame_id)
        if robot_pose is None:
            self._publish_mission_status(
                'MISSION_REPLAN_SKIPPED 无法获取实时车位姿，保留当前剩余目标顺序'
            )
            return

        self._mission_queue.sort(
            key=lambda goal: self._distance_to_robot(goal.pose, robot_pose)
        )
        self._publish_mission_status(
            f'MISSION_REPLANNED robot_x={robot_pose[0]:.3f}, robot_y={robot_pose[1]:.3f}, '
            f'remaining={len(self._mission_queue)}'
        )

    def _reset_mission(self) -> None:
        if self._mission_active:
            self._publish_mission_status('MISSION_REPLACED 收到新的 OpenClaw 决策命令，替换当前多目标任务')
        self._mission_active = False
        self._mission_queue = []
        self._mission_current = None
        self._mission_total = 0
        self._mission_done = 0
        self._failure_handled_for_current = False

    def _publish_mission_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.mission_status_pub.publish(msg)
        self.get_logger().info(text)

    def _candidate_signature(self, metadata: Sequence[Dict[str, Any]]) -> Tuple[Tuple[Any, ...], ...]:
        signature = []
        for item in metadata:
            if item['visited']:
                continue
            signature.append((
                int(item['index']),
                str(item['label']),
                round(float(item['x']), 2),
                round(float(item['y']), 2),
            ))
        return tuple(signature)

    def _publish_empty_decision(self, text: str, auto_send_goal: bool) -> None:
        source = PoseArray()
        source.header.stamp = self.get_clock().now().to_msg()
        source.header.frame_id = self.target_frame
        self._publish_decision(
            source,
            targets=[],
            metadata=[],
            robot_pose=None,
            selected_index=None,
            ordered_indices=[],
            reason=text,
            decision_text=text,
            reply_text=text,
            raw_response='',
            fallback_used=False,
            error='',
            auto_send_goal=auto_send_goal,
        )

    def _publish_text_only(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.decision_text_pub.publish(msg)

    def _fallback_decision(self, metadata: Sequence[Dict[str, Any]]) -> Tuple[Optional[int], List[int], str]:
        candidates = [item for item in metadata if not item['visited']]
        if not candidates:
            return None, [], 'no unvisited targets available'

        def score(item: Dict[str, Any]) -> Tuple[float, float, int]:
            distance = item['distance_m'] if item['distance_m'] is not None else 999.0
            confidence_penalty = 0.0 if item['score'] is None else 1.0 - max(0.0, min(float(item['score']), 1.0))
            return (
                distance
                + float(item['class_priority_rank']) * 0.25
                + confidence_penalty * 0.10,
                distance,
                int(item['index']),
            )

        ordered_items = sorted(candidates, key=score)
        ordered = [int(item['index']) for item in ordered_items]
        selected = ordered[0]
        selected_item = ordered_items[0]
        return selected, ordered, (
            f'类别优先级与距离回退选择 {selected_item["label"]}，'
            f'index={selected}，distance={selected_item["distance_m"]}'
        )

    def _priority_rank(self, label: str) -> int:
        return self.class_priority_map.get(label.strip().lower(), len(self.class_priority_map))

    def _lookup_robot_pose(self, frame_id: str) -> Optional[RobotPose]:
        timeout = Duration(seconds=self.tf_timeout_sec)
        for base_frame in self.base_frames:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    frame_id,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=timeout,
                )
                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
                return float(t.x), float(t.y), yaw
            except (LookupException, TransformException):
                continue
        self.get_logger().warning(f'cannot lookup robot pose in {frame_id} using {self.base_frames}')
        return None

    def _load_openclaw_token(self) -> str:
        if self.openclaw_token:
            return self.openclaw_token
        env_token = os.environ.get('OPENCLAW_GATEWAY_TOKEN') or os.environ.get('OPENCLAW_TOKEN')
        if env_token:
            return env_token.strip()
        try:
            with open(self.openclaw_config_path, 'r', encoding='utf-8') as config_file:
                config = json.load(config_file)
            return str(config.get('gateway', {}).get('auth', {}).get('token', '')).strip()
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'cannot read OpenClaw token from {self.openclaw_config_path}: {exc}') from exc

    @staticmethod
    def _parse_control_mode(text: str) -> Optional[str]:
        stripped = text.strip()
        try:
            data = json.loads(stripped)
            mode = str(data.get('mode') or data.get('command') or '').strip().lower()
        except json.JSONDecodeError:
            mode = stripped.lower()
        if mode in ('auto', 'automatic', 'send', 'go', '自动', '自动发车'):
            return 'auto'
        if mode in ('stop', 'cancel', 'quit', 'exit', '停止', '退出', '停止自动'):
            return 'stop'
        if mode in ('dry_run', 'decision_only', 'manual', 'decide', '只决策', '只决策不发车'):
            return 'dry_run'
        return None

    @staticmethod
    def _parse_marker_text(text: str) -> Dict[str, Any]:
        visited = text.startswith('已到达')
        clean = text.replace('已到达', '', 1).strip() if visited else text.strip()
        parts = clean.split()
        label = parts[0] if parts else 'unknown'
        score = None
        if len(parts) >= 2:
            try:
                score = float(parts[-1])
            except ValueError:
                score = None
        return {'label': label, 'score': score, 'visited': visited}

    @staticmethod
    def _extract_response_text(parsed: Dict[str, Any]) -> str:
        if isinstance(parsed.get('output_text'), str):
            return parsed['output_text']
        output = parsed.get('output')
        if isinstance(output, list):
            chunks = []
            for item in output:
                content = item.get('content') if isinstance(item, dict) else None
                if isinstance(content, list):
                    for part in content:
                        if isinstance(part, dict) and isinstance(part.get('text'), str):
                            chunks.append(part['text'])
                elif isinstance(content, str):
                    chunks.append(content)
            if chunks:
                return '\n'.join(chunks)
        choices = parsed.get('choices')
        if isinstance(choices, list) and choices:
            message = choices[0].get('message', {}) if isinstance(choices[0], dict) else {}
            if isinstance(message.get('content'), str):
                return message['content']
        return ''

    @staticmethod
    def _extract_json_object(text: str) -> Dict[str, Any]:
        stripped = text.strip()
        if stripped.startswith('```'):
            stripped = re.sub(r'^```(?:json)?\s*', '', stripped)
            stripped = re.sub(r'\s*```$', '', stripped)
        try:
            parsed = json.loads(stripped)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass

        start = stripped.find('{')
        end = stripped.rfind('}')
        if start == -1 or end == -1 or end <= start:
            raise ValueError(f'OpenClaw response does not contain JSON: {text[:300]}')
        parsed = json.loads(stripped[start:end + 1])
        if not isinstance(parsed, dict):
            raise ValueError('OpenClaw JSON response is not an object')
        return parsed

    @staticmethod
    def _format_decision_text(decision: Dict[str, Any]) -> str:
        lines = [
            f'模式={"自动发车" if decision["auto_send_goal"] else "只决策不发车"}',
            f'selected_index={decision["selected_index"]}',
            f'ordered_indices={decision["ordered_indices"]}',
            f'goal_sent={decision["goal_sent"]}',
            f'mission_started={decision.get("mission_started", False)}',
            f'mission_total={decision.get("mission_total", 0)}',
            f'fallback_used={decision["fallback_used"]}',
            f'reason={decision["reason"]}',
        ]
        decision_text = str(decision.get('decision_text') or '').strip()
        if decision_text:
            lines.append(f'decision_text={decision_text}')
        reply_text = str(decision.get('openclaw_reply_text') or '').strip()
        if reply_text:
            lines.append(f'openclaw_reply_text={reply_text}')
        error = str(decision.get('error') or '').strip()
        if error:
            lines.append(f'error={error}')
        raw_response = str(decision.get('raw_response') or '').strip()
        if raw_response:
            lines.append(f'raw_response={raw_response}')
        return '\n'.join(lines)

    @staticmethod
    def _distance_to_robot(pose: Pose, robot_pose: Optional[RobotPose]) -> Optional[float]:
        if robot_pose is None:
            return None
        dx = float(pose.position.x) - robot_pose[0]
        dy = float(pose.position.y) - robot_pose[1]
        return math.hypot(dx, dy)

    @staticmethod
    def _pose_distance_2d(a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(
            a.pose.position.x - b.pose.position.x,
            a.pose.position.y - b.pose.position.y,
        )

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _parse_csv(value: str) -> List[str]:
        return [item.strip() for item in value.split(',') if item.strip()]

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = OpenClawGoalDecisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
