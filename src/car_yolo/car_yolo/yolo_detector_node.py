import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# 兼容当前从 rknn_model_zoo-self 搬出来的目录结构。
RKNN_ZOO_ROOT = '/home/elf/rknn/rknn_model_zoo-self'
if RKNN_ZOO_ROOT not in sys.path:
    sys.path.append(RKNN_ZOO_ROOT)

from py_utils.coco_utils import COCO_test_helper  # noqa: E402
from py_utils.rknn_executor import RKNN_model_container  # noqa: E402


OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = (640, 640)

CLASSES = (
    'person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'sofa',
    'pottedplant', 'bed', 'diningtable', 'toilet', 'tvmonitor', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
    'teddy bear', 'hair drier', 'toothbrush'
)


def filter_boxes(boxes, box_confidences, box_class_probs):
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[class_pos]
    boxes = boxes[class_pos]
    classes = classes[class_pos]
    return boxes, classes, scores


def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
    areas = w * h
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
        w1 = np.maximum(0.0, xx2 - xx1 + 1e-5)
        h1 = np.maximum(0.0, yy2 - yy1 + 1e-5)
        inter = w1 * h1
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    return np.array(keep)


def dfl(position):
    import torch
    x = torch.tensor(position)
    n, c, h, w = x.shape
    p_num = 4
    mc = c // p_num
    y = x.reshape(n, p_num, mc, h, w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y.numpy()


def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)
    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
    return xyxy


def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    default_branch = 3
    pair_per_branch = len(input_data) // default_branch
    for i in range(default_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(input_data[pair_per_branch * i + 2])

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)
    if boxes is None or len(boxes) == 0:
        return None, None, None

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c_vals = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)
        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c_vals[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    return boxes, classes, scores


def draw(image, boxes, scores, classes):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(
            image,
            f'{CLASSES[int(cl)]} {float(score):.2f}',
            (top, max(20, left + 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.declare_parameter('model_path', '/home/elf/rknn/yolo11/model/yolo11s.rknn')
        self.declare_parameter('target', 'rk3588')
        self.declare_parameter('device_id', '')
        self.declare_parameter('camera_device', '/dev/video21')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('annotated_image_topic', '/yolo/image_annotated')
        self.declare_parameter('publish_fps_topic', '/yolo/debug_fps')
        self.declare_parameter('timer_hz', 15.0)

        self.model_path = self.get_parameter('model_path').value
        self.target = self.get_parameter('target').value
        self.device_id = self.get_parameter('device_id').value or None
        self.camera_device = self.get_parameter('camera_device').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.timer_hz = float(self.get_parameter('timer_hz').value)

        self.bridge = CvBridge()
        self.co_helper = COCO_test_helper(enable_letter_box=True)
        self.model = RKNN_model_container(self.model_path, self.target, self.device_id)

        self.detections_pub = self.create_publisher(
            Detection2DArray, self.get_parameter('detections_topic').value, 10
        )
        self.annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_image_topic').value, 10
        )
        self.fps_pub = self.create_publisher(
            Float32, self.get_parameter('publish_fps_topic').value, 10
        )

        self.cap = self._open_capture(self.camera_device)
        self.timer = self.create_timer(1.0 / max(self.timer_hz, 1.0), self.timer_callback)
        self.get_logger().info(
            f'yolo_detector_node 已启动: model={self.model_path}, camera={self.camera_device}, target={self.target}'
        )

    def _open_capture(self, camera_device: str):
        if camera_device.startswith('/dev/video'):
            cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        else:
            try:
                cap = cv2.VideoCapture(int(camera_device), cv2.CAP_V4L2)
            except ValueError:
                cap = cv2.VideoCapture(camera_device)
        if not cap.isOpened():
            raise RuntimeError(f'无法打开摄像头: {camera_device}')
        return cap

    def _build_detection_array(self, stamp, boxes, classes, scores):
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id

        if boxes is None:
            return msg

        for index, (box, score, cl) in enumerate(zip(boxes, scores, classes)):
            x1, y1, x2, y2 = [float(v) for v in box]
            detection = Detection2D()
            detection.header.stamp = stamp
            detection.header.frame_id = self.camera_frame_id
            detection.id = str(index)
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = CLASSES[int(cl)]
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)
            msg.detections.append(detection)

        return msg

    def timer_callback(self):
        begin = time.time()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('摄像头读帧失败，跳过本轮')
            return

        stamp = self.get_clock().now().to_msg()
        img = self.co_helper.letter_box(im=frame.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=(0, 0, 0))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        outputs = self.model.run([img])
        boxes, classes, scores = post_process(outputs)

        annotated = frame.copy()
        real_boxes = None
        if boxes is not None:
            real_boxes = self.co_helper.get_real_box(boxes)
            draw(annotated, real_boxes, scores, classes)

        fps = 1.0 / max(time.time() - begin, 1e-6)
        cv2.putText(annotated, f'{fps:.2f} fps', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        det_msg = self._build_detection_array(
            stamp=stamp,
            boxes=real_boxes,
            classes=classes,
            scores=scores,
        )
        self.detections_pub.publish(det_msg)

        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.camera_frame_id
        self.annotated_pub.publish(img_msg)

        fps_msg = Float32()
        fps_msg.data = float(fps)
        self.fps_pub.publish(fps_msg)

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            if hasattr(self, 'model') and self.model is not None:
                self.model.release()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
