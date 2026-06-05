"""YOLO 2D detections -> target points in odom/map/base frame.

This is the new v1 target mapping node.  It supports two projection modes:

- depth: use the selected RGB bbox pixel, sample depth around the mapped depth
  pixel, back-project with RGB camera intrinsics, then transform through TF.
- ground_plane: ignore depth and intersect the selected RGB pixel ray with a
  horizontal plane in the target frame.

The default mode is depth because the next validation step is RGB-depth
registration.  If split RGB/depth alignment is not trustworthy yet, switch to
projection_mode:=ground_plane for a geometry-only first pass.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, LookupException, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


class TargetMapperNode(Node):
    """Map YOLO detections to target points using depth or ground-plane geometry."""

    def __init__(self):
        super().__init__('target_mapper_node')

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('points_topic', '/yolo/target_points')
        self.declare_parameter('markers_topic', '/yolo/target_markers')
        self.declare_parameter('projection_mode', 'depth')
        self.declare_parameter('sample_point', 'bottom_center')
        self.declare_parameter('depth_scale', 0.01)
        self.declare_parameter('min_depth_m', 0.03)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('depth_patch_radius', 4)
        self.declare_parameter('depth_source', 'bbox_region')
        self.declare_parameter('use_bbox_depth_fallback', True)
        self.declare_parameter('bbox_depth_quantile', 0.5)
        self.declare_parameter('max_depth_age_sec', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.15)
        self.declare_parameter('marker_lifetime_sec', 1.0)
        self.declare_parameter('ground_plane_z', 0.0)
        self.declare_parameter('min_ground_range_m', 0.10)
        self.declare_parameter('max_ground_range_m', 8.0)
        self.declare_parameter('rgb_to_depth_scale_x', 0.0)
        self.declare_parameter('rgb_to_depth_scale_y', 0.0)
        self.declare_parameter('rgb_to_depth_offset_x', 0.0)
        self.declare_parameter('rgb_to_depth_offset_y', 0.0)
        self.declare_parameter('fallback_color_width', 640)
        self.declare_parameter('fallback_color_height', 480)
        self.declare_parameter('fallback_fx', 554.0)
        self.declare_parameter('fallback_fy', 554.0)
        self.declare_parameter('fallback_cx', 320.0)
        self.declare_parameter('fallback_cy', 240.0)
        self.declare_parameter('class_filter', 'red_ball,red_cube')
        self.declare_parameter('min_score', 0.35)
        self.declare_parameter('max_targets', 1)
        self.declare_parameter('smoothing_alpha', 0.4)
        self.declare_parameter('debug_log', False)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.projection_mode = str(self.get_parameter('projection_mode').value)
        self.sample_point = str(self.get_parameter('sample_point').value)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_patch_radius = int(self.get_parameter('depth_patch_radius').value)
        self.depth_source = str(self.get_parameter('depth_source').value)
        self.use_bbox_depth_fallback = self._as_bool(self.get_parameter('use_bbox_depth_fallback').value)
        self.bbox_depth_quantile = float(self.get_parameter('bbox_depth_quantile').value)
        self.max_depth_age_sec = float(self.get_parameter('max_depth_age_sec').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self.ground_plane_z = float(self.get_parameter('ground_plane_z').value)
        self.min_ground_range_m = float(self.get_parameter('min_ground_range_m').value)
        self.max_ground_range_m = float(self.get_parameter('max_ground_range_m').value)
        self.rgb_to_depth_scale_x = float(self.get_parameter('rgb_to_depth_scale_x').value)
        self.rgb_to_depth_scale_y = float(self.get_parameter('rgb_to_depth_scale_y').value)
        self.rgb_to_depth_offset_x = float(self.get_parameter('rgb_to_depth_offset_x').value)
        self.rgb_to_depth_offset_y = float(self.get_parameter('rgb_to_depth_offset_y').value)
        self.fallback_color_width = int(self.get_parameter('fallback_color_width').value)
        self.fallback_color_height = int(self.get_parameter('fallback_color_height').value)
        self.fallback_fx = float(self.get_parameter('fallback_fx').value)
        self.fallback_fy = float(self.get_parameter('fallback_fy').value)
        self.fallback_cx = float(self.get_parameter('fallback_cx').value)
        self.fallback_cy = float(self.get_parameter('fallback_cy').value)
        self.class_filter = self._parse_class_filter(str(self.get_parameter('class_filter').value))
        self.min_score = float(self.get_parameter('min_score').value)
        self.max_targets = int(self.get_parameter('max_targets').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.debug_log = self._as_bool(self.get_parameter('debug_log').value)

        if self.projection_mode not in ('depth', 'ground_plane'):
            raise ValueError('projection_mode must be depth or ground_plane')
        if self.sample_point not in ('bottom_center', 'center'):
            raise ValueError('sample_point must be bottom_center or center')
        if self.depth_source not in ('bbox_region', 'sample_patch', 'sample_then_bbox'):
            raise ValueError('depth_source must be bbox_region, sample_patch, or sample_then_bbox')
        if self.max_targets < 1:
            raise ValueError('max_targets must be >= 1')
        if self.smoothing_alpha < 0.0 or self.smoothing_alpha > 1.0:
            raise ValueError('smoothing_alpha must be between 0.0 and 1.0')
        class_filter_text = ','.join(sorted(self.class_filter)) if self.class_filter else 'all'

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_depth_msg: Optional[Image] = None
        self.last_depth_image: Optional[np.ndarray] = None
        self.last_camera_info: Optional[CameraInfo] = None
        self.last_color_size: Optional[Tuple[int, int]] = None
        self.filtered_points: List[Tuple[float, float, float]] = []
        self.warned_keys = set()

        self.points_pub = self.create_publisher(PoseArray, str(self.get_parameter('points_topic').value), 10)
        self.markers_pub = self.create_publisher(MarkerArray, str(self.get_parameter('markers_topic').value), 10)

        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            self.detections_callback,
            10,
        )
        self.create_subscription(Image, str(self.get_parameter('depth_topic').value), self.depth_callback, 10)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback,
            10,
        )
        color_topic = str(self.get_parameter('color_image_topic').value)
        if color_topic:
            self.create_subscription(Image, color_topic, self.color_image_callback, 10)

        self.get_logger().info(
            'target_mapper_node started: '
            f'mode={self.projection_mode}, sample_point={self.sample_point}, depth_source={self.depth_source}, '
            f'class_filter={class_filter_text}, min_score={self.min_score:.2f}, '
            f'max_targets={self.max_targets}, smoothing_alpha={self.smoothing_alpha:.2f}, '
            f'target_frame={self.target_frame}, camera_frame={self.camera_frame}'
        )

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv2.error as exc:
            self.get_logger().warning(f'depth image conversion failed: {exc}')
            return

        if depth.ndim == 3:
            depth = depth[:, :, 0]
        self.last_depth_msg = msg
        self.last_depth_image = np.asarray(depth)

    def camera_info_callback(self, msg: CameraInfo):
        self.last_camera_info = msg

    def color_image_callback(self, msg: Image):
        if msg.width > 0 and msg.height > 0:
            self.last_color_size = (int(msg.width), int(msg.height))

    def detections_callback(self, msg: Detection2DArray):
        if self.last_camera_info is None:
            self.get_logger().debug('no camera_info yet, skip detections')
            return
        if self.projection_mode == 'depth' and (self.last_depth_msg is None or self.last_depth_image is None):
            self.get_logger().debug('no depth image yet, skip detections')
            return
        if not msg.detections:
            self._publish_empty(msg.header.stamp)
            return

        selected_detections = self._select_detections(msg.detections)
        if not selected_detections:
            self._publish_empty(msg.header.stamp)
            return

        tf_msg = self._lookup_transform(msg.header.stamp)
        if tf_msg is None:
            return

        if self.projection_mode == 'depth':
            age = self._stamp_diff_sec(msg.header.stamp, self.last_depth_msg.header.stamp)
            if age > self.max_depth_age_sec:
                self.get_logger().warning(
                    f'depth/detection age {age:.3f}s exceeds {self.max_depth_age_sec:.3f}s, skip frame'
                )
                return

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.target_frame
        markers = MarkerArray()
        raw_points = []
        labels_scores = []

        for det in selected_detections:
            point_target = self._project_detection(det, tf_msg)
            if point_target is None:
                continue
            raw_points.append(point_target)
            labels_scores.append(self._best_label(det))

        if not raw_points:
            self._publish_empty(msg.header.stamp)
            return

        points = self._smooth_points(raw_points)

        for point_target, (label, score) in zip(points, labels_scores):
            pose = Pose()
            pose.position = Point(x=point_target[0], y=point_target[1], z=point_target[2])
            pose.orientation = Quaternion(w=1.0)
            pose_array.poses.append(pose)

            marker_id = len(markers.markers)
            markers.markers.append(self._make_sphere_marker(marker_id, msg.header.stamp, pose.position, score))
            markers.markers.append(self._make_text_marker(marker_id + 1, msg.header.stamp, pose.position, label, score))

        self.points_pub.publish(pose_array)
        if markers.markers:
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            markers.markers.insert(0, delete_all)
        self.markers_pub.publish(markers)

        if self.debug_log:
            self.get_logger().info(
                f'published {len(pose_array.poses)} target points in {self.target_frame}'
            )

    def _project_detection(self, det: Detection2D, tf_msg: TransformStamped) -> Optional[Tuple[float, float, float]]:
        pixel = self._selected_pixel(det)
        if self.projection_mode == 'ground_plane':
            return self._project_ground_plane(pixel, tf_msg)
        return self._project_depth(det, pixel, tf_msg)

    def _project_depth(
        self,
        det: Detection2D,
        pixel: Tuple[float, float],
        tf_msg: TransformStamped,
    ) -> Optional[Tuple[float, float, float]]:
        assert self.last_depth_image is not None
        u_color, v_color = pixel
        color_width, color_height = self._color_size()
        depth_height, depth_width = self.last_depth_image.shape[:2]

        u_depth, v_depth = self._rgb_to_depth_pixel(
            u_color,
            v_color,
            color_width,
            color_height,
            depth_width,
            depth_height,
        )
        z = None
        if self.depth_source == 'bbox_region':
            z = self._sample_bbox_depth_m(det, color_width, color_height, depth_width, depth_height)
            if z is None:
                z = self._sample_depth_m(u_depth, v_depth)
        elif self.depth_source == 'sample_patch':
            z = self._sample_depth_m(u_depth, v_depth)
        else:
            z = self._sample_depth_m(u_depth, v_depth)
            if z is None and self.use_bbox_depth_fallback:
                z = self._sample_bbox_depth_m(det, color_width, color_height, depth_width, depth_height)
        if z is None:
            if self.debug_log:
                label, _ = self._best_label(det)
                self.get_logger().info(f'{label} skipped: no valid depth')
            return None

        intr = self._camera_intrinsics()
        if intr is None:
            return None
        fx, fy, cx, cy = intr
        point_camera = ((u_color - cx) * z / fx, (v_color - cy) * z / fy, z)
        return self._transform_point(point_camera, tf_msg)

    def _project_ground_plane(
        self,
        pixel: Tuple[float, float],
        tf_msg: TransformStamped,
    ) -> Optional[Tuple[float, float, float]]:
        intr = self._camera_intrinsics()
        if intr is None:
            return None
        fx, fy, cx, cy = intr
        u, v = pixel
        ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        ray_camera /= max(np.linalg.norm(ray_camera), 1e-9)

        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rot = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        origin = np.array([t.x, t.y, t.z], dtype=np.float64)
        ray_target = rot @ ray_camera
        if abs(ray_target[2]) < 1e-6:
            return None

        scale = (self.ground_plane_z - origin[2]) / ray_target[2]
        if scale <= 0.0:
            return None

        point = origin + scale * ray_target
        distance = float(np.linalg.norm(point - origin))
        if distance < self.min_ground_range_m or distance > self.max_ground_range_m:
            return None
        return float(point[0]), float(point[1]), float(point[2])

    def _lookup_transform(self, stamp) -> Optional[TransformStamped]:
        timeout = Duration(seconds=self.tf_timeout_sec)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=timeout,
            )
        except (LookupException, TransformException) as first_exc:
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=timeout,
                )
            except (LookupException, TransformException) as latest_exc:
                self.get_logger().warning(
                    f'cannot transform {self.camera_frame} -> {self.target_frame}: '
                    f'{latest_exc}; stamped lookup was: {first_exc}'
                )
                return None

    def _selected_pixel(self, det: Detection2D) -> Tuple[float, float]:
        u = float(det.bbox.center.position.x)
        v = float(det.bbox.center.position.y)
        if self.sample_point == 'bottom_center':
            v += float(det.bbox.size_y) * 0.5
        return u, v

    def _color_size(self) -> Tuple[int, int]:
        if self.last_camera_info is not None and self.last_camera_info.width > 0 and self.last_camera_info.height > 0:
            return int(self.last_camera_info.width), int(self.last_camera_info.height)
        if self.last_color_size is not None:
            return self.last_color_size
        return max(self.fallback_color_width, 1), max(self.fallback_color_height, 1)

    def _camera_intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        assert self.last_camera_info is not None
        fx = float(self.last_camera_info.k[0])
        fy = float(self.last_camera_info.k[4])
        cx = float(self.last_camera_info.k[2])
        cy = float(self.last_camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            fx = self.fallback_fx
            fy = self.fallback_fy
            cx = self.fallback_cx
            cy = self.fallback_cy
            self._warn_once(
                'empty_intrinsics',
                'camera_info intrinsics are empty; using fallback intrinsics for rough validation',
            )
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning('invalid camera intrinsics, skip projection')
            return None
        return fx, fy, cx, cy

    def _rgb_to_depth_pixel(
        self,
        u: float,
        v: float,
        color_width: int,
        color_height: int,
        depth_width: int,
        depth_height: int,
    ) -> Tuple[int, int]:
        sx = self.rgb_to_depth_scale_x if self.rgb_to_depth_scale_x > 0.0 else depth_width / max(color_width, 1)
        sy = self.rgb_to_depth_scale_y if self.rgb_to_depth_scale_y > 0.0 else depth_height / max(color_height, 1)
        u_depth = int(round(u * sx + self.rgb_to_depth_offset_x))
        v_depth = int(round(v * sy + self.rgb_to_depth_offset_y))
        u_depth = min(max(u_depth, 0), depth_width - 1)
        v_depth = min(max(v_depth, 0), depth_height - 1)
        return u_depth, v_depth

    def _sample_depth_m(self, u: int, v: int) -> Optional[float]:
        assert self.last_depth_image is not None
        h, w = self.last_depth_image.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        radius = max(self.depth_patch_radius, 0)
        x1 = max(0, u - radius)
        x2 = min(w, u + radius + 1)
        y1 = max(0, v - radius)
        y2 = min(h, v + radius + 1)
        return self._valid_depth_stat(self.last_depth_image[y1:y2, x1:x2])

    def _sample_bbox_depth_m(
        self,
        det: Detection2D,
        color_width: int,
        color_height: int,
        depth_width: int,
        depth_height: int,
    ) -> Optional[float]:
        assert self.last_depth_image is not None
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
        half_w = max(float(det.bbox.size_x) * 0.5, 1.0)
        half_h = max(float(det.bbox.size_y) * 0.5, 1.0)
        x1, y1 = self._rgb_to_depth_pixel(cx - half_w, cy - half_h, color_width, color_height, depth_width, depth_height)
        x2, y2 = self._rgb_to_depth_pixel(cx + half_w, cy + half_h, color_width, color_height, depth_width, depth_height)
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        if x1 == x2:
            x2 = min(depth_width - 1, x2 + 1)
        if y1 == y2:
            y2 = min(depth_height - 1, y2 + 1)
        patch = self.last_depth_image[y1:y2 + 1, x1:x2 + 1]
        return self._valid_depth_stat(patch, quantile=self.bbox_depth_quantile)

    def _valid_depth_stat(self, patch: np.ndarray, quantile: float = 0.5) -> Optional[float]:
        if patch.size == 0:
            return None
        values = patch.astype(np.float32)
        if np.issubdtype(patch.dtype, np.integer):
            values = values * self.depth_scale
        values = values[np.isfinite(values)]
        values = values[(values >= self.min_depth_m) & (values <= self.max_depth_m)]
        if values.size == 0:
            return None
        q = min(max(float(quantile), 0.0), 1.0)
        return float(np.quantile(values, q))

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
        self.filtered_points = []

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        self.markers_pub.publish(MarkerArray(markers=[delete_all]))

    def _make_sphere_marker(self, marker_id: int, stamp, position: Point, score: float) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = 'yolo_targets'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color = ColorRGBA(r=1.0 - score, g=score, b=0.1, a=0.85)
        marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        return marker

    def _make_text_marker(self, marker_id: int, stamp, position: Point, label: str, score: float) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = 'yolo_target_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=position.x, y=position.y, z=position.z + 0.18)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker.text = f'{label} {score:.2f}' if label else f'{score:.2f}'
        marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
        return marker

    @staticmethod
    def _best_label(det: Detection2D) -> Tuple[str, float]:
        if not det.results:
            return '', 0.0
        best = max(det.results, key=lambda item: item.hypothesis.score)
        return str(best.hypothesis.class_id), float(best.hypothesis.score)

    def _select_detections(self, detections: List[Detection2D]) -> List[Detection2D]:
        selected = []
        for det in detections:
            label, score = self._best_label(det)
            if score < self.min_score:
                continue
            if self.class_filter and label.strip().lower() not in self.class_filter:
                continue
            selected.append((score, det))
        selected.sort(key=lambda item: item[0], reverse=True)
        return [det for _, det in selected[:self.max_targets]]

    def _smooth_points(self, points: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        if not self.filtered_points or len(self.filtered_points) != len(points) or self.smoothing_alpha >= 1.0:
            self.filtered_points = list(points)
            return list(points)

        alpha = self.smoothing_alpha
        smoothed = []
        for previous, current in zip(self.filtered_points, points):
            smoothed.append((
                previous[0] * (1.0 - alpha) + current[0] * alpha,
                previous[1] * (1.0 - alpha) + current[1] * alpha,
                previous[2] * (1.0 - alpha) + current[2] * alpha,
            ))
        self.filtered_points = smoothed
        return smoothed

    @staticmethod
    def _parse_class_filter(value: str) -> set:
        return {item.strip().lower() for item in value.split(',') if item.strip()}

    @staticmethod
    def _stamp_diff_sec(a, b) -> float:
        a_sec = float(a.sec) + float(a.nanosec) * 1e-9
        b_sec = float(b.sec) + float(b.nanosec) * 1e-9
        return abs(a_sec - b_sec)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _warn_once(self, key: str, message: str):
        if key in self.warned_keys:
            return
        self.warned_keys.add(key)
        self.get_logger().warning(message)

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
    node = TargetMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
