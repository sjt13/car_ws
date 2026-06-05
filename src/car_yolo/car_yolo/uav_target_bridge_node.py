"""Bridge UAV target reports into the car-side map visualization topics."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, LookupException, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


Target = Tuple[Tuple[float, float, float], str, float]


class UavTargetBridgeNode(Node):
    """Convert UAV target messages into map-frame PoseArray and RViz markers."""

    def __init__(self):
        super().__init__('uav_target_bridge_node')

        self.declare_parameter('pose_array_topic', '/uav/target_points')
        self.declare_parameter('pose_stamped_topic', '/uav/shared/target_pose')
        self.declare_parameter('pose_stamped_source_frame_override', '')
        self.declare_parameter('detections_topic', '/uav/target_detections')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('points_topic', '/uav/target_points_map')
        self.declare_parameter('markers_topic', '/uav/target_markers')
        self.declare_parameter('class_filter', '')
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('max_targets', 20)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('marker_lifetime_sec', 0.0)
        self.declare_parameter('marker_republish_hz', 1.0)
        self.declare_parameter('marker_scale', 0.35)
        self.declare_parameter('force_ground_z', True)
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('debug_log', False)

        self.pose_array_topic = str(self.get_parameter('pose_array_topic').value)
        self.pose_stamped_topic = str(self.get_parameter('pose_stamped_topic').value)
        self.pose_stamped_source_frame_override = str(
            self.get_parameter('pose_stamped_source_frame_override').value
        ).strip()
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.points_topic = str(self.get_parameter('points_topic').value)
        self.markers_topic = str(self.get_parameter('markers_topic').value)
        self.class_filter = self._parse_class_filter(str(self.get_parameter('class_filter').value))
        self.min_score = float(self.get_parameter('min_score').value)
        self.max_targets = int(self.get_parameter('max_targets').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self.marker_republish_hz = float(self.get_parameter('marker_republish_hz').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.force_ground_z = self._as_bool(self.get_parameter('force_ground_z').value)
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.debug_log = self._as_bool(self.get_parameter('debug_log').value)

        if self.max_targets < 1:
            raise ValueError('max_targets must be >= 1')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.points_pub = self.create_publisher(PoseArray, self.points_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self._last_pose_array: Optional[PoseArray] = None
        self._last_markers: Optional[MarkerArray] = None

        if self.pose_array_topic:
            self.create_subscription(PoseArray, self.pose_array_topic, self.pose_array_callback, 10)
        if self.pose_stamped_topic:
            self.create_subscription(PoseStamped, self.pose_stamped_topic, self.pose_stamped_callback, 10)
        if self.detections_topic:
            self.create_subscription(Detection3DArray, self.detections_topic, self.detections_callback, 10)
        if self.marker_republish_hz > 0.0:
            self.create_timer(1.0 / self.marker_republish_hz, self._republish_last)

        class_filter_text = ','.join(sorted(self.class_filter)) if self.class_filter else 'all'
        self.get_logger().info(
            'uav_target_bridge_node started: '
            f'pose_array_topic={self.pose_array_topic}, pose_stamped_topic={self.pose_stamped_topic}, '
            f'pose_stamped_source_frame_override={self.pose_stamped_source_frame_override or "none"}, '
            f'detections_topic={self.detections_topic}, '
            f'target_frame={self.target_frame}, class_filter={class_filter_text}, '
            f'min_score={self.min_score:.2f}, max_targets={self.max_targets}, '
            f'marker_republish_hz={self.marker_republish_hz:.2f}'
        )

    def pose_array_callback(self, msg: PoseArray):
        source_frame = msg.header.frame_id or self.target_frame
        targets = [((pose.position.x, pose.position.y, pose.position.z), 'uav_target', 1.0) for pose in msg.poses]
        self._publish_targets(targets, source_frame, msg.header.stamp)

    def pose_stamped_callback(self, msg: PoseStamped):
        source_frame = self.pose_stamped_source_frame_override or msg.header.frame_id or self.target_frame
        p = msg.pose.position
        self._publish_targets([((p.x, p.y, p.z), 'shared_target', 1.0)], source_frame, msg.header.stamp)

    def detections_callback(self, msg: Detection3DArray):
        source_frame = msg.header.frame_id or self.target_frame
        targets = []
        for det in msg.detections:
            label, score = self._best_label(det)
            if score < self.min_score:
                continue
            if self.class_filter and label.strip().lower() not in self.class_filter:
                continue
            p = det.bbox.center.position
            targets.append(((p.x, p.y, p.z), label or 'uav_target', score))

        targets.sort(key=lambda item: item[2], reverse=True)
        self._publish_targets(targets[:self.max_targets], source_frame, msg.header.stamp)

    def _publish_targets(self, targets: List[Target], source_frame: str, stamp):
        if not targets:
            self._publish_empty(stamp)
            return

        tf_msg = self._lookup_transform(source_frame, stamp)
        if tf_msg is None:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.target_frame
        markers = MarkerArray()

        for point_source, label, score in targets:
            point_target = self._transform_point(point_source, tf_msg)
            if self.force_ground_z:
                point_target = (point_target[0], point_target[1], self.ground_z)

            pose = Pose()
            pose.position = Point(x=point_target[0], y=point_target[1], z=point_target[2])
            pose.orientation = Quaternion(w=1.0)
            pose_array.poses.append(pose)

            marker_id = len(markers.markers)
            markers.markers.append(self._make_sphere_marker(marker_id, stamp, pose.position, score))
            markers.markers.append(self._make_text_marker(marker_id + 1, stamp, pose.position, label, score))

        self.points_pub.publish(pose_array)
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.insert(0, delete_all)
        self.markers_pub.publish(markers)
        self._last_pose_array = pose_array
        self._last_markers = markers

        if self.debug_log:
            self.get_logger().info(
                f'published {len(pose_array.poses)} UAV targets in {self.target_frame}'
            )

    def _lookup_transform(self, source_frame: str, stamp) -> Optional[TransformStamped]:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = self.target_frame
        tf_msg.child_frame_id = source_frame
        tf_msg.transform.rotation.w = 1.0

        if source_frame == self.target_frame:
            return tf_msg

        timeout = Duration(seconds=self.tf_timeout_sec)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=timeout,
            )
        except (LookupException, TransformException) as first_exc:
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=timeout,
                )
            except (LookupException, TransformException) as latest_exc:
                self.get_logger().warning(
                    f'cannot transform {source_frame} -> {self.target_frame}: '
                    f'{latest_exc}; stamped lookup was: {first_exc}'
                )
                return None

    def _transform_point(self, point: Tuple[float, float, float], tf_msg: TransformStamped) -> Tuple[float, float, float]:
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rot = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        p = np.array(point, dtype=np.float64)
        out = rot @ p + np.array([t.x, t.y, t.z], dtype=np.float64)
        return float(out[0]), float(out[1]), float(out[2])

    def _publish_empty(self, stamp):
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.target_frame
        self.points_pub.publish(pose_array)

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers = MarkerArray(markers=[delete_all])
        self.markers_pub.publish(markers)
        self._last_pose_array = pose_array
        self._last_markers = markers

    def _republish_last(self):
        if self._last_pose_array is None or self._last_markers is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._last_pose_array.header.stamp = stamp
        for marker in self._last_markers.markers:
            marker.header.stamp = stamp
        self.points_pub.publish(self._last_pose_array)
        self.markers_pub.publish(self._last_markers)

    def _make_sphere_marker(self, marker_id: int, stamp, position: Point, score: float) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = 'uav_targets'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color = ColorRGBA(r=0.1, g=0.55 + 0.45 * min(max(score, 0.0), 1.0), b=1.0, a=0.9)
        marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        return marker

    def _make_text_marker(self, marker_id: int, stamp, position: Point, label: str, score: float) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = 'uav_target_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=position.x, y=position.y, z=position.z + self.marker_scale * 1.3)
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.marker_scale * 0.6
        marker.color = ColorRGBA(r=0.8, g=0.95, b=1.0, a=0.95)
        marker.text = f'{label} {score:.2f}' if label else f'{score:.2f}'
        marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        return marker

    @staticmethod
    def _best_label(det: Detection3D) -> Tuple[str, float]:
        if not det.results:
            return '', 1.0
        best = max(det.results, key=lambda item: item.hypothesis.score)
        return str(best.hypothesis.class_id), float(best.hypothesis.score)

    @staticmethod
    def _parse_class_filter(value: str) -> set:
        return {item.strip().lower() for item in value.split(',') if item.strip()}

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    @staticmethod
    def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])


def main(args=None):
    rclpy.init(args=args)
    node = UavTargetBridgeNode()
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
