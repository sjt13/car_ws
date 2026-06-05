"""Ask OpenClaw to choose a UAV target while keeping ROS2 in control."""

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


RobotPose = Tuple[float, float, float]


class OpenClawTargetDecisionNode(Node):
    """Subscribe to mapped targets and publish an audited OpenClaw decision."""

    def __init__(self):
        super().__init__('openclaw_target_decision_node')

        self.declare_parameter('target_points_topic', '/uav/target_points_map')
        self.declare_parameter('selected_goal_topic', '/openclaw/selected_goal')
        self.declare_parameter('target_order_topic', '/openclaw/target_order')
        self.declare_parameter('decision_topic', '/openclaw/target_decision')
        self.declare_parameter('decision_text_topic', '/openclaw/target_decision_text')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frames', 'base_footprint,base_link')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('openclaw_base_url', 'http://127.0.0.1:18789')
        self.declare_parameter('openclaw_config_path', '/home/elf/.openclaw/openclaw.json')
        self.declare_parameter('openclaw_token', '')
        self.declare_parameter('openclaw_agent_id', 'main')
        self.declare_parameter('openclaw_model', 'openclaw')
        self.declare_parameter('decision_language', 'zh_CN')
        self.declare_parameter('request_timeout_sec', 25.0)
        self.declare_parameter('min_decision_interval_sec', 2.0)
        self.declare_parameter('max_targets', 20)
        self.declare_parameter('target_labels', '')
        self.declare_parameter('class_priority', 'red_ball,red_cube')
        self.declare_parameter('ignore_duplicate_targets', True)
        self.declare_parameter('duplicate_position_epsilon_m', 0.02)
        self.declare_parameter('fallback_on_error', True)
        self.declare_parameter('visited_radius_m', 0.45)
        self.declare_parameter('debug_log', False)

        self.target_points_topic = str(self.get_parameter('target_points_topic').value)
        self.selected_goal_topic = str(self.get_parameter('selected_goal_topic').value)
        self.target_order_topic = str(self.get_parameter('target_order_topic').value)
        self.decision_topic = str(self.get_parameter('decision_topic').value)
        self.decision_text_topic = str(self.get_parameter('decision_text_topic').value)
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
        self.target_labels = self._parse_csv(str(self.get_parameter('target_labels').value))
        self.class_priority = self._parse_csv(str(self.get_parameter('class_priority').value))
        self.class_priority_map = {
            label: rank for rank, label in enumerate(label.lower() for label in self.class_priority)
        }
        self.ignore_duplicate_targets = self._as_bool(self.get_parameter('ignore_duplicate_targets').value)
        self.duplicate_position_epsilon_m = float(self.get_parameter('duplicate_position_epsilon_m').value)
        self.fallback_on_error = self._as_bool(self.get_parameter('fallback_on_error').value)
        self.visited_radius_m = float(self.get_parameter('visited_radius_m').value)
        self.debug_log = self._as_bool(self.get_parameter('debug_log').value)

        if self.max_targets < 1:
            raise ValueError('max_targets must be >= 1')
        if not self.base_frames:
            raise ValueError('base_frames must contain at least one frame')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_decision_time = self.get_clock().now() - Duration(seconds=9999.0)
        self._last_target_signature: Optional[Tuple[Any, ...]] = None

        self.selected_goal_pub = self.create_publisher(PoseStamped, self.selected_goal_topic, 10)
        self.target_order_pub = self.create_publisher(PoseArray, self.target_order_topic, 10)
        self.decision_pub = self.create_publisher(String, self.decision_topic, 10)
        self.decision_text_pub = self.create_publisher(String, self.decision_text_topic, 10)
        self.create_subscription(PoseArray, self.target_points_topic, self.targets_callback, 10)

        token_source = 'parameter' if self.openclaw_token else 'config/env'
        self.get_logger().info(
            'openclaw_target_decision_node started: '
            f'target_points_topic={self.target_points_topic}, target_frame={self.target_frame}, '
            f'base_frames={",".join(self.base_frames)}, openclaw_base_url={self.openclaw_base_url}, '
            f'agent_id={self.openclaw_agent_id or "default"}, token_source={token_source}, '
            f'decision_language={self.decision_language or "default"}, '
            f'class_priority={",".join(self.class_priority) or "none"}, '
            f'fallback_on_error={self.fallback_on_error}, '
            f'ignore_duplicate_targets={self.ignore_duplicate_targets}'
        )

    def targets_callback(self, msg: PoseArray):
        now = self.get_clock().now()
        elapsed = (now - self._last_decision_time).nanoseconds / 1e9
        if elapsed < self.min_decision_interval_sec:
            if self.debug_log:
                self.get_logger().info(f'skip decision: only {elapsed:.2f}s since last request')
            return
        self._last_decision_time = now

        targets = list(msg.poses[:self.max_targets])
        if not targets:
            self._publish_decision(
                msg,
                targets=[],
                robot_pose=None,
                selected_index=None,
                ordered_indices=[],
                reason='no targets received',
                decision_text=(
                    '当前没有可用目标点。'
                    if self._use_chinese_decision_text()
                    else 'No target points are available.'
                ),
                openclaw_reply_text=(
                    '当前没有可用目标点。'
                    if self._use_chinese_decision_text()
                    else 'No target points are available.'
                ),
                raw_response='',
                fallback_used=False,
                error='',
            )
            return

        frame_id = msg.header.frame_id or self.target_frame
        signature = self._target_signature(frame_id, targets)
        if self.ignore_duplicate_targets and signature == self._last_target_signature:
            if self.debug_log:
                self.get_logger().info('skip decision: duplicate target set')
            return
        self._last_target_signature = signature

        robot_pose = self._lookup_robot_pose(frame_id)
        fallback = self._fallback_decision(targets, robot_pose)

        selected_index: Optional[int] = None
        ordered_indices: List[int] = []
        reason = ''
        decision_text = ''
        openclaw_reply_text = ''
        raw_response = ''
        fallback_used = False
        error = ''

        try:
            payload = self._build_openclaw_payload(targets, frame_id, robot_pose)
            raw_response = self._ask_openclaw(payload)
            parsed = self._parse_decision(raw_response, len(targets))
            selected_index = parsed['selected_index']
            ordered_indices = parsed['ordered_indices']
            reason = parsed['reason']
            decision_text = parsed['decision_text']
            openclaw_reply_text = parsed['openclaw_reply_text']
        except Exception as exc:  # noqa: BLE001 - publish the failure reason for field debugging.
            error = str(exc)
            if not self.fallback_on_error:
                self.get_logger().warning(f'OpenClaw decision failed and fallback is disabled: {error}')
                self._publish_decision(
                    msg,
                    targets=targets,
                    robot_pose=robot_pose,
                    selected_index=None,
                    ordered_indices=[],
                    reason='OpenClaw decision failed',
                    decision_text='OpenClaw did not return a valid decision.',
                    openclaw_reply_text='OpenClaw did not return a valid decision.',
                    raw_response=raw_response,
                    fallback_used=False,
                    error=error,
                )
                return
            selected_index, ordered_indices, reason = fallback
            decision_text = (
                f'回退策略选择目标 {selected_index}：{reason}'
                if self._use_chinese_decision_text()
                else f'Fallback selected target {selected_index}: {reason}'
            )
            openclaw_reply_text = decision_text
            fallback_used = True
            self.get_logger().warning(f'OpenClaw decision failed, using fallback: {error}')

        self._publish_decision(
            msg,
            targets=targets,
            robot_pose=robot_pose,
            selected_index=selected_index,
            ordered_indices=ordered_indices,
            reason=reason,
            decision_text=decision_text,
            openclaw_reply_text=openclaw_reply_text,
            raw_response=raw_response,
            fallback_used=fallback_used,
            error=error,
        )

    def _build_openclaw_payload(
        self,
        targets: Sequence[Pose],
        frame_id: str,
        robot_pose: Optional[RobotPose],
    ) -> Dict[str, Any]:
        target_items = []
        for index, pose in enumerate(targets):
            distance = self._distance_to_robot(pose, robot_pose)
            label = self._target_label(index)
            target_items.append({
                'index': index,
                'label': label,
                'class_priority_rank': self._priority_rank(label),
                'x': round(float(pose.position.x), 4),
                'y': round(float(pose.position.y), 4),
                'z': round(float(pose.position.z), 4),
                'distance_m': None if distance is None else round(distance, 4),
                'already_near_robot': bool(distance is not None and distance <= self.visited_radius_m),
            })

        return {
            'frame_id': frame_id,
            'robot_pose': None if robot_pose is None else {
                'x': round(robot_pose[0], 4),
                'y': round(robot_pose[1], 4),
                'yaw': round(robot_pose[2], 4),
            },
            'visited_radius_m': self.visited_radius_m,
            'decision_language': self.decision_language,
            'class_priority': self.class_priority,
            'targets': target_items,
        }

    def _ask_openclaw(self, decision_payload: Dict[str, Any]) -> str:
        token = self._load_openclaw_token()
        if not token:
            raise RuntimeError('OpenClaw token is not configured')

        prompt = (
            'You are choosing the next ground-vehicle target from UAV-provided map points.\n'
            'Return only a JSON object. Do not include markdown fences.\n'
            'Use observable factors only: label, class_priority_rank, distance_m, '
            'whether the robot is already near a target, target validity, and simple route priority. '
            'Do not expose hidden chain-of-thought.\n'
            'Put a complete machine-auditable decision report in decision_text. '
            'Put a natural normal OpenClaw-style reply in openclaw_reply_text. '
            'These are explicit task-level explanations, not hidden chain-of-thought.\n'
            'Write reason, decision_text, openclaw_reply_text, and scores[].note in Simplified Chinese '
            'when decision_language is zh_CN.\n'
            'Schema:\n'
            '{"selected_index": int|null, "ordered_indices": [int], "reason": string, '
            '"decision_text": string, "openclaw_reply_text": string, '
            '"scores": [{"index": int, "score": number, "note": string}]}\n'
            'Rules:\n'
            '- Prefer unvisited targets, meaning already_near_robot=false.\n'
            '- Lower class_priority_rank is higher priority; red_ball has priority over red_cube.\n'
            '- If robot_pose is available, prefer shorter distance unless another target has a clear reason.\n'
            '- selected_index must be one of the provided target indices or null if no target is valid.\n'
            '- ordered_indices must contain valid target indices in recommended visit order.\n'
            '- decision_text must be detailed, not a summary. Include: input target list, class priority rule, '
            'robot pose and distances, per-target evaluation, rejection/penalty reasons, final ordered_indices, '
            'and why selected_index was chosen.\n'
            '- openclaw_reply_text should sound like a normal direct reply to the operator, not a rigid report. '
            'It should mention the selected target and the practical reason in a conversational style.\n'
            '- Use clear numbered Chinese steps in decision_text when decision_language is zh_CN.\n\n'
            f'Data:\n{json.dumps(decision_payload, ensure_ascii=False)}'
        )
        body = {
            'model': self.openclaw_model or 'openclaw',
            'input': prompt,
            'stream': False,
        }
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

    def _parse_decision(self, raw_response: str, target_count: int) -> Dict[str, Any]:
        decision = self._extract_json_object(raw_response)
        selected_index = decision.get('selected_index')
        if selected_index is not None:
            selected_index = int(selected_index)
            if selected_index < 0 or selected_index >= target_count:
                raise ValueError(f'selected_index out of range: {selected_index}')

        ordered_indices = []
        for item in decision.get('ordered_indices', []):
            index = int(item)
            if 0 <= index < target_count and index not in ordered_indices:
                ordered_indices.append(index)
        if selected_index is not None and selected_index not in ordered_indices:
            ordered_indices.insert(0, selected_index)
        for index in range(target_count):
            if index not in ordered_indices:
                ordered_indices.append(index)

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
        robot_pose: Optional[RobotPose],
        selected_index: Optional[int],
        ordered_indices: Sequence[int],
        reason: str,
        decision_text: str,
        openclaw_reply_text: str,
        raw_response: str,
        fallback_used: bool,
        error: str,
    ):
        frame_id = source_msg.header.frame_id or self.target_frame
        stamp = source_msg.header.stamp

        if selected_index is not None and 0 <= selected_index < len(targets):
            selected = PoseStamped()
            selected.header.stamp = stamp
            selected.header.frame_id = frame_id
            selected.pose = targets[selected_index]
            self.selected_goal_pub.publish(selected)

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
            'reason': reason,
            'decision_text': decision_text,
            'openclaw_reply_text': openclaw_reply_text,
            'raw_response': raw_response,
            'fallback_used': fallback_used,
            'error': error,
            'frame_id': frame_id,
            'target_labels': [self._target_label(index) for index in range(len(targets))],
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

        if self.debug_log:
            self.get_logger().info(text_msg.data)

    def _fallback_decision(
        self,
        targets: Sequence[Pose],
        robot_pose: Optional[RobotPose],
    ) -> Tuple[Optional[int], List[int], str]:
        if not targets:
            return None, [], 'no targets available'

        scored = []
        for index, pose in enumerate(targets):
            distance = self._distance_to_robot(pose, robot_pose)
            visited_penalty = 10000.0 if distance is not None and distance <= self.visited_radius_m else 0.0
            priority_score = float(self._priority_rank(self._target_label(index)) * 100.0)
            score = priority_score + (distance if distance is not None else float(index)) + visited_penalty
            scored.append((score, index, distance))

        scored.sort(key=lambda item: item[0])
        ordered = [item[1] for item in scored]
        selected = ordered[0]
        distance_text = 'unknown distance' if scored[0][2] is None else f'distance={scored[0][2]:.2f}m'
        label = self._target_label(selected)
        if self._use_chinese_decision_text():
            return selected, ordered, f'类别优先级回退选择 {label}（{distance_text}）'
        return selected, ordered, f'class-priority fallback selected {label} ({distance_text})'

    def _target_label(self, index: int) -> str:
        if 0 <= index < len(self.target_labels):
            return self.target_labels[index]
        return 'unknown'

    def _priority_rank(self, label: str) -> int:
        return self.class_priority_map.get(label.strip().lower(), len(self.class_priority_map))

    def _use_chinese_decision_text(self) -> bool:
        return self.decision_language.strip().lower() in ('zh', 'zh_cn', 'zh-cn', 'chinese')

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
            f'selected_index={decision["selected_index"]}',
            f'ordered_indices={decision["ordered_indices"]}',
            f'fallback_used={decision["fallback_used"]}',
            f'reason={decision["reason"]}',
        ]
        decision_text = str(decision.get('decision_text') or '').strip()
        if decision_text:
            lines.append(f'decision_text={decision_text}')
        openclaw_reply_text = str(decision.get('openclaw_reply_text') or '').strip()
        if openclaw_reply_text:
            lines.append(f'openclaw_reply_text={openclaw_reply_text}')
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

    def _target_signature(self, frame_id: str, targets: Sequence[Pose]) -> Tuple[Any, ...]:
        epsilon = max(self.duplicate_position_epsilon_m, 1e-6)
        points = []
        for pose in targets:
            points.append((
                round(float(pose.position.x) / epsilon),
                round(float(pose.position.y) / epsilon),
                round(float(pose.position.z) / epsilon),
            ))
        return frame_id, tuple(points)

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
    node = OpenClawTargetDecisionNode()
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
