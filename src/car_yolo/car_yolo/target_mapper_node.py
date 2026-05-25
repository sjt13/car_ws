"""把 YOLO 2D 检测结果粗略投到 odom/map 坐标系。

当前定位：MVP 工程验证版。

输入：
- /yolo/detections：vision_msgs/Detection2DArray，目标 2D 框；
- /camera/depth/image_raw：深度图；
- /camera/color/camera_info：RGB 相机内参；
- TF：camera_color_optical_frame -> odom/map。

输出：
- /yolo/target_points：geometry_msgs/PoseArray，目标点；
- /yolo/target_markers：visualization_msgs/MarkerArray，RViz 标记。

注意：Astra Pro 当前 RGB 与 depth 是拆开的链路，depth 没有做严格 registration。
这里用“RGB 像素按分辨率比例映射到 depth 像素”的近似方法，够先看目标能不能落到图上，
但不要把它当标定级坐标。后面要做准，需要补 RGB-depth 外参/配准。
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

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
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


class TargetMapperNode(Node):
    """YOLO 检测框 + 深度 + TF -> odom/map 目标点。"""

    def __init__(self):
        super().__init__('target_mapper_node')

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('points_topic', '/yolo/target_points')
        self.declare_parameter('markers_topic', '/yolo/target_markers')
        self.declare_parameter('depth_scale', 0.001)  # 常见 16UC1 深度单位：mm -> m
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('depth_patch_radius', 3)
        self.declare_parameter('max_depth_age_sec', 0.5)
        self.declare_parameter('marker_lifetime_sec', 1.0)
        self.declare_parameter('fallback_fx', 554.0)
        self.declare_parameter('fallback_fy', 554.0)
        self.declare_parameter('fallback_cx', 320.0)
        self.declare_parameter('fallback_cy', 240.0)
        self.declare_parameter('debug_log', False)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_patch_radius = int(self.get_parameter('depth_patch_radius').value)
        self.max_depth_age_sec = float(self.get_parameter('max_depth_age_sec').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self.fallback_fx = float(self.get_parameter('fallback_fx').value)
        self.fallback_fy = float(self.get_parameter('fallback_fy').value)
        self.fallback_cx = float(self.get_parameter('fallback_cx').value)
        self.fallback_cy = float(self.get_parameter('fallback_cy').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_depth_msg: Optional[Image] = None
        self.last_depth_image: Optional[np.ndarray] = None
        self.last_camera_info: Optional[CameraInfo] = None

        self.points_pub = self.create_publisher(PoseArray, str(self.get_parameter('points_topic').value), 10)
        self.markers_pub = self.create_publisher(MarkerArray, str(self.get_parameter('markers_topic').value), 10)

        self.create_subscription(Image, str(self.get_parameter('depth_topic').value), self.depth_callback, 10)
        self.create_subscription(CameraInfo, str(self.get_parameter('camera_info_topic').value), self.camera_info_callback, 10)
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            'target_mapper_node 已启动: '
            f'detections={self.get_parameter("detections_topic").value}, '
            f'depth={self.get_parameter("depth_topic").value}, '
            f'camera_info={self.get_parameter("camera_info_topic").value}, '
            f'target_frame={self.target_frame}'
        )

    def depth_callback(self, msg: Image):
        """缓存最新深度图。"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv2.error as exc:
            self.get_logger().warning(f'深度图转换失败: {exc}')
            return

        if depth.ndim == 3:
            depth = depth[:, :, 0]
        self.last_depth_msg = msg
        self.last_depth_image = np.asarray(depth)

    def camera_info_callback(self, msg: CameraInfo):
        """缓存 RGB 相机内参。"""
        self.last_camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        """收到检测框后，查深度并投到目标坐标系。"""
        if self.last_depth_msg is None or self.last_depth_image is None:
            self.get_logger().debug('还没有 depth，跳过本轮检测')
            return
        if self.last_camera_info is None:
            self.get_logger().debug('还没有 camera_info，跳过本轮检测')
            return
        if not msg.detections:
            self._publish_empty(msg)
            return

        age = self._stamp_diff_sec(msg.header.stamp, self.last_depth_msg.header.stamp)
        if age > self.max_depth_age_sec:
            self.get_logger().warning(
                f'depth 与 detection 时间差 {age:.3f}s 超过阈值 {self.max_depth_age_sec:.3f}s，跳过'
            )
            return
        if self.debug_log:
            self.get_logger().info(f'收到 {len(msg.detections)} 个检测，depth_age={age:.3f}s')

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, TransformException) as exc:
            # 有些链路没有历史 TF，退一步用最新 TF，先保证工程验证能跑。
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except (LookupException, TransformException) as latest_exc:
                self.get_logger().warning(f'找不到 {self.camera_frame} -> {self.target_frame} TF: {latest_exc}; first={exc}')
                return

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.target_frame
        markers = MarkerArray()

        marker_id = 0
        for det in msg.detections:
            point_camera = self._detection_to_camera_point(det)
            if point_camera is None:
                if self.debug_log:
                    label, _ = self._best_label(det)
                    self.get_logger().info(f'检测 {label} 没有取到有效深度，跳过')
                continue

            point_target = self._transform_point(point_camera, tf_msg)
            pose = Pose()
            pose.position = Point(x=point_target[0], y=point_target[1], z=point_target[2])
            pose.orientation = Quaternion(w=1.0)
            pose_array.poses.append(pose)

            label, score = self._best_label(det)
            markers.markers.append(self._make_sphere_marker(marker_id, msg.header.stamp, pose.position, score))
            marker_id += 1
            markers.markers.append(self._make_text_marker(marker_id, msg.header.stamp, pose.position, label, score))
            marker_id += 1

        self.points_pub.publish(pose_array)
        self.markers_pub.publish(markers)
        if self.debug_log:
            self.get_logger().info(f'发布 {len(pose_array.poses)} 个目标点到 {self.target_frame}')

    def _publish_empty(self, msg: Detection2DArray):
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.target_frame
        self.points_pub.publish(pose_array)
        self.markers_pub.publish(MarkerArray())

    def _detection_to_camera_point(self, det) -> Optional[Tuple[float, float, float]]:
        """检测框中心像素 + 深度 -> camera optical frame 下 3D 点。"""
        assert self.last_depth_image is not None
        assert self.last_camera_info is not None

        color_width = max(int(self.last_camera_info.width), 1)
        color_height = max(int(self.last_camera_info.height), 1)
        depth_height, depth_width = self.last_depth_image.shape[:2]

        u_color = float(det.bbox.center.position.x)
        v_color = float(det.bbox.center.position.y)
        u_depth = int(round(u_color * depth_width / color_width))
        v_depth = int(round(v_color * depth_height / color_height))

        half_w_depth = max(1, int(round(float(det.bbox.size_x) * depth_width / color_width / 2.0)))
        half_h_depth = max(1, int(round(float(det.bbox.size_y) * depth_height / color_height / 2.0)))
        z = self._sample_depth_m(u_depth, v_depth)
        if z is None:
            z = self._sample_depth_region_m(u_depth, v_depth, half_w_depth, half_h_depth)
        if z is None:
            return None

        fx = float(self.last_camera_info.k[0])
        fy = float(self.last_camera_info.k[4])
        cx = float(self.last_camera_info.k[2])
        cy = float(self.last_camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            # v4l2_camera 没加载标定文件时 camera_info 会全 0。
            # MVP 阶段先用近似内参兜底，保证目标能落图；后面再换真实标定。
            fx = self.fallback_fx
            fy = self.fallback_fy
            cx = self.fallback_cx
            cy = self.fallback_cy
            self.get_logger().warn_once(
                'camera_info 内参为空，使用 fallback_fx/fy/cx/cy 近似内参；目标点只适合粗验证'
            )
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning('相机内参 fx/fy 非法，无法投影')
            return None

        # 用 RGB 内参反投影。这里 point 是 optical frame: x 向右, y 向下, z 向前。
        x = (u_color - cx) * z / fx
        y = (v_color - cy) * z / fy
        return x, y, z

    def _sample_depth_m(self, u: int, v: int) -> Optional[float]:
        """从深度图中心小窗口取有效深度中位数。"""
        assert self.last_depth_image is not None
        h, w = self.last_depth_image.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        r = max(self.depth_patch_radius, 0)
        x1 = max(0, u - r)
        x2 = min(w, u + r + 1)
        y1 = max(0, v - r)
        y2 = min(h, v + r + 1)
        patch = self.last_depth_image[y1:y2, x1:x2].astype(np.float32)

        if np.issubdtype(self.last_depth_image.dtype, np.integer):
            patch = patch * self.depth_scale

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _sample_depth_region_m(self, u: int, v: int, half_w: int, half_h: int) -> Optional[float]:
        """中心小窗口没深度时，在整个检测框对应区域里找有效深度。

        Astra Pro 当前 RGB/depth 没有严格配准，框中心常常落到 depth 空洞上。
        MVP 先取检测框区域内较近的一组有效深度中位数，保证能先落图。
        """
        assert self.last_depth_image is not None
        h, w = self.last_depth_image.shape[:2]
        x1 = max(0, u - half_w)
        x2 = min(w, u + half_w + 1)
        y1 = max(0, v - half_h)
        y2 = min(h, v + half_h + 1)
        if x1 >= x2 or y1 >= y2:
            return None

        patch = self.last_depth_image[y1:y2, x1:x2].astype(np.float32)
        if np.issubdtype(self.last_depth_image.dtype, np.integer):
            patch = patch * self.depth_scale

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None

        # 取偏近的有效深度，避免框背景把目标距离拉远。
        valid.sort()
        near_count = max(1, int(valid.size * 0.25))
        return float(np.median(valid[:near_count]))

    def _transform_point(self, point: Tuple[float, float, float], tf_msg: TransformStamped) -> Tuple[float, float, float]:
        """手动应用 TransformStamped，避免额外依赖 tf2_geometry_msgs。"""
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rot = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        p = np.array(point, dtype=np.float64)
        out = rot @ p + np.array([t.x, t.y, t.z], dtype=np.float64)
        return float(out[0]), float(out[1]), float(out[2])

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
    def _best_label(det) -> Tuple[str, float]:
        if not det.results:
            return '', 0.0
        best = max(det.results, key=lambda item: item.hypothesis.score)
        return str(best.hypothesis.class_id), float(best.hypothesis.score)

    @staticmethod
    def _stamp_diff_sec(a, b) -> float:
        a_sec = float(a.sec) + float(a.nanosec) * 1e-9
        b_sec = float(b.sec) + float(b.nanosec) * 1e-9
        return abs(a_sec - b_sec)


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
