# car_yolo

`car_yolo` 维护车端视觉检测、RGB-D 目标落图、UAV 共享目标桥接和 OpenClaw 目标决策相关节点。

## 主要功能

该包当前包含五个可执行节点：

- `yolo_detector_node`：读取摄像头或 ROS 图像，运行 RKNN YOLO 推理，发布 2D 检测结果。
- `target_mapper_node`：订阅 2D 检测、深度图、相机内参和 TF，输出目标点和 RViz marker。
- `uav_target_bridge_node`：订阅 UAV 侧目标，转换到车端目标 frame 后发布 PoseArray 和 marker。
- `openclaw_target_decision_node`：对目标点做一次性目标排序/选择，输出决策结果。
- `openclaw_goal_decision_node`：面向目标驱动导航的决策节点，可结合导航到达状态连续更新任务目标。

这些节点不负责底盘控制。导航动作由 `car_driver` 的 Nav2/goal-slam 链路处理。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `car_yolo/yolo_detector_node.py` | RKNN YOLO 推理和检测话题发布。 |
| `car_yolo/target_mapper_node.py` | 车端 RGB-D 目标落图。 |
| `car_yolo/uav_target_bridge_node.py` | UAV 目标转换到车端坐标系。 |
| `car_yolo/openclaw_target_decision_node.py` | OpenClaw 目标选择/排序。 |
| `car_yolo/openclaw_goal_decision_node.py` | OpenClaw 目标导航决策。 |
| `launch/yolo_detector.launch.py` | 启动检测节点。 |
| `launch/target_mapper.launch.py` | 启动车端目标落图节点。 |
| `launch/uav_target_bridge.launch.py` | 启动 UAV 目标桥接节点。 |
| `launch/openclaw_target_decision.launch.py` | 启动一次性目标决策节点。 |
| `launch/openclaw_goal_decision.launch.py` | 启动目标导航决策节点。 |
| `setup.py` | 注册 5 个 console entry。 |

## 节点

| 节点名称 | 入口 | 主要职责 |
| --- | --- | --- |
| `yolo_detector_node` | `car_yolo/yolo_detector_node.py` | 图像输入、RKNN 推理、检测框和标注图发布。 |
| `target_mapper_node` | `car_yolo/target_mapper_node.py` | 将 `/yolo/detections` 与深度/相机内参/TF 结合，发布目标点。 |
| `uav_target_bridge_node` | `car_yolo/uav_target_bridge_node.py` | 将 UAV PoseArray、PoseStamped 或 Detection3DArray 转到 `target_frame`。 |
| `openclaw_target_decision_node` | `car_yolo/openclaw_target_decision_node.py` | 调用 OpenClaw `/v1/responses` 或 fallback 逻辑，选择目标。 |
| `openclaw_goal_decision_node` | `car_yolo/openclaw_goal_decision_node.py` | 结合目标、marker、导航状态和到达记录，发布任务目标和决策文本。 |

## 接口

### `yolo_detector_node`

订阅：

| 名称 | 类型 | 条件 | 用途 |
| --- | --- | --- | --- |
| `image_topic` 参数指定的话题 | `sensor_msgs/msg/Image` | `image_topic` 非空 | 使用 ROS 图像作为输入。 |

发布：

| 名称 | 类型 | 默认话题 | 用途 |
| --- | --- | --- | --- |
| `detections_topic` | `vision_msgs/msg/Detection2DArray` | `/yolo/detections` | 2D 检测框、类别和置信度。 |
| `annotated_image_topic` | `sensor_msgs/msg/Image` | `/yolo/image_annotated` | 画框后的调试图像。 |
| `publish_fps_topic` | `std_msgs/msg/Float32` | `/yolo/debug_fps` | 推理帧率。 |

如果 `image_topic` 为空，节点直接打开 `camera_device`，默认 `/dev/video21`。

### `target_mapper_node`

订阅：

| 名称 | 类型 | 默认话题 | 用途 |
| --- | --- | --- | --- |
| `detections_topic` | `vision_msgs/msg/Detection2DArray` | `/yolo/detections` | YOLO 2D 检测结果。 |
| `depth_topic` | `sensor_msgs/msg/Image` | `/camera/depth/image_raw` | 深度图。 |
| `camera_info_topic` | `sensor_msgs/msg/CameraInfo` | `/camera/color/camera_info` | RGB 相机内参。 |
| `color_image_topic` | `sensor_msgs/msg/Image` | `/camera/color/image_raw` | 彩色图像，主要用于时间戳/调试关联。 |
| TF | - | `camera_frame` -> `target_frame` | 将相机系点转换到目标 frame。 |

发布：

| 名称 | 类型 | 默认话题 | 用途 |
| --- | --- | --- | --- |
| `points_topic` | `geometry_msgs/msg/PoseArray` | `/yolo/target_points` | 目标点数组。 |
| `markers_topic` | `visualization_msgs/msg/MarkerArray` | `/yolo/target_markers` | RViz 目标显示。 |

`points_topic` 和 `markers_topic` 是源码中声明的参数；当前 `target_mapper.launch.py` 没有暴露这两个 launch 参数，使用源码默认值。

### `uav_target_bridge_node`

订阅：

| 名称 | 类型 | 默认话题 | 用途 |
| --- | --- | --- | --- |
| `pose_array_topic` | `geometry_msgs/msg/PoseArray` | `/uav/target_points` | UAV 目标数组。 |
| `pose_stamped_topic` | `geometry_msgs/msg/PoseStamped` | `/uav/shared/target_pose` | 单个 UAV 共享目标。 |
| `detections_topic` | `vision_msgs/msg/Detection3DArray` | `/uav/target_detections` | 3D 检测目标。 |
| `reached_goal_topic` | `geometry_msgs/msg/PoseStamped` | `/goal_slam_nav/reached_goal` | 已到达目标，用于过滤已访问点。 |
| TF | - | source frame -> `target_frame` | 坐标转换。 |

发布：

| 名称 | 类型 | 默认话题 | 用途 |
| --- | --- | --- | --- |
| `points_topic` | `geometry_msgs/msg/PoseArray` | `/uav/target_points_map` | 转换到 `target_frame` 后的目标点。 |
| `markers_topic` | `visualization_msgs/msg/MarkerArray` | `/uav/target_markers` | RViz marker。 |

### OpenClaw 决策节点

`openclaw_target_decision_node` 默认订阅 `/uav/target_points_map`，发布：

| 名称 | 类型 | 默认话题 |
| --- | --- | --- |
| `selected_goal_topic` | `geometry_msgs/msg/PoseStamped` | `/openclaw/selected_goal` |
| `target_order_topic` | `geometry_msgs/msg/PoseArray` | `/openclaw/target_order` |
| `decision_topic` | `std_msgs/msg/String` | `/openclaw/target_decision` |
| `decision_text_topic` | `std_msgs/msg/String` | `/openclaw/target_decision_text` |

`openclaw_goal_decision_node` 默认订阅 `/uav/target_points_map`、`/uav/target_markers`、`/openclaw_goal/control`、`/goal_slam_nav/reached_goal`、`/goal_slam_nav/status`，发布：

| 名称 | 类型 | 默认话题 |
| --- | --- | --- |
| `selected_goal_topic` | `geometry_msgs/msg/PoseStamped` | `/openclaw_goal/selected_goal` |
| `task_goal_topic` | `geometry_msgs/msg/PoseStamped` | `/uav/task_goal` |
| `target_order_topic` | `geometry_msgs/msg/PoseArray` | `/openclaw_goal/target_order` |
| `decision_topic` | `std_msgs/msg/String` | `/openclaw_goal/decision` |
| `decision_text_topic` | `std_msgs/msg/String` | `/openclaw_goal/decision_text` |
| `reply_text_topic` | `std_msgs/msg/String` | `/openclaw_goal/reply_text` |
| `mission_status_topic` | `std_msgs/msg/String` | `/openclaw_goal/mission_status` |

两个 OpenClaw 节点都会向 `openclaw_base_url + /v1/responses` 发送 HTTP 请求；默认 `openclaw_base_url` 为 `http://127.0.0.1:18789`。

## 主要参数

### `yolo_detector_node`

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model_path` | `/home/elf/rknn/yolo11/model/yolo11n.rknn` | RKNN 模型路径。 |
| `target` | `rk3588` | RKNN 运行目标。 |
| `device_id` | 空 | RKNN 设备 ID。 |
| `camera_device` | `/dev/video21` | OpenCV 直接打开的相机设备。 |
| `camera_width` / `camera_height` | `1280` / `720` | 直接打开相机时设置的分辨率。 |
| `camera_fps` | `30.0` | 相机 FPS。 |
| `camera_fourcc` | `MJPG` | 相机像素格式。 |
| `image_topic` | 空 | ROS 图像输入话题；非空时不直接打开相机设备。 |
| `camera_frame_id` | `camera_color_optical_frame` | 检测消息 frame。 |
| `timer_hz` | `15.0` | 推理定时器频率。 |
| `obj_thresh` | `0.25` | 置信度阈值。 |

### `target_mapper_node`

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `target_frame` | `odom` | 输出目标点 frame。 |
| `camera_frame` | `camera_color_optical_frame` | 相机光学 frame。 |
| `projection_mode` | `depth` | 使用深度图；源码也包含 `ground_plane` 分支。 |
| `sample_point` | `bottom_center` | 从检测框取样的位置。 |
| `depth_scale` | `0.01` | 深度值转米比例。 |
| `min_depth_m` / `max_depth_m` | `0.03` / `8.0` | 有效深度范围。 |
| `depth_source` | `bbox_region` | 深度采样来源。 |
| `bbox_depth_quantile` | `0.5` | bbox 区域深度统计分位数。 |
| `class_filter` | `red_ball,red_cube` | 只处理这些类别；空字符串表示不过滤。 |
| `min_score` | `0.35` | 最低置信度。 |
| `max_targets` | `1` | 单帧最多输出目标数。 |
| `smoothing_alpha` | `0.4` | 目标点平滑系数。 |

### `uav_target_bridge_node`

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `target_frame` | `map` | 输出目标 frame。 |
| `class_filter` | 空 | 类别过滤。 |
| `min_score` | `0.0` | 最低置信度。 |
| `max_targets` | `20` | 最多输出目标数。 |
| `visited_match_radius_m` | `0.45` | 已到达目标过滤半径。 |
| `tf_timeout_sec` | `0.2` | TF 查询超时。 |
| `marker_republish_hz` | `1.0` | marker 重发频率。 |
| `detections_max_publish_hz` | `2.0` | 3D 检测输入最大处理频率。 |
| `force_ground_z` | `true` | 是否强制目标 z 为 `ground_z`。 |
| `ground_z` | `0.0` | 地面 z 值。 |

## 依赖

### ROS2 包依赖

- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `vision_msgs`
- `cv_bridge`
- `tf2_ros`

### Python/系统依赖

源码中使用 `cv2`、`numpy`、`urllib` 等模块。`yolo_detector_node` 还依赖 RKNN 运行环境和 RKNN 模型文件；这些依赖不由 `package.xml` 自动安装。

### 硬件依赖

- RK3588/RKNN 运行环境。
- RGB 相机或 ROS 图像输入。
- `target_mapper_node` 使用 `depth` 模式时需要深度图和相机内参。

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_yolo
source install/setup.bash
```

## 常用启动

直接打开相机跑检测：

```bash
ros2 launch car_yolo yolo_detector.launch.py
```

订阅 Orbbec RGB 图像跑检测：

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_frame_id:=camera_color_optical_frame
```

启动 RGB-D 目标落图：

```bash
ros2 launch car_yolo target_mapper.launch.py
```

UAV 目标桥接：

```bash
ros2 launch car_yolo uav_target_bridge.launch.py
```

已知地图导航链中更常用的启动方式：

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py
```

目标驱动在线 SLAM 中更常用的启动方式：

```bash
ros2 launch car_driver goal_slam_nav_bringup.launch.py
```

OpenClaw 目标导航决策单独启动：

```bash
ros2 launch car_yolo openclaw_goal_decision.launch.py
```

## 常用检查

检测输出：

```bash
ros2 topic echo /yolo/detections --once
ros2 topic echo /yolo/debug_fps
rqt_image_view /yolo/image_annotated
```

目标落图：

```bash
ros2 topic echo /yolo/target_points --once
ros2 topic echo /yolo/target_markers --once
ros2 run tf2_ros tf2_echo odom camera_color_optical_frame
```

UAV 桥接：

```bash
ros2 topic echo /uav/target_points_map --once
ros2 topic echo /uav/target_markers --once
ros2 run tf2_ros tf2_echo map uav_map
```

OpenClaw 目标决策：

```bash
ros2 topic echo /openclaw_goal/decision_text
ros2 topic echo /openclaw_goal/mission_status
```

## 排查

### 检测节点无法打开相机

如果同时运行 `orbbec_bringup.launch.py`，RGB 设备可能已由 `v4l2_camera` 占用。此时让 YOLO 订阅 ROS 图像：

```bash
ros2 launch car_yolo yolo_detector.launch.py image_topic:=/camera/color/image_raw
```

### `target_mapper_node` 没有目标点

按顺序检查：

```bash
ros2 topic echo /yolo/detections --once
ros2 topic echo /camera/depth/image_raw --once
ros2 topic echo /camera/color/camera_info --once
ros2 run tf2_ros tf2_echo odom camera_color_optical_frame
```

还需要确认 `class_filter` 和 `min_score` 没有过滤掉当前检测结果。

### UAV 目标无法转换到 `map`

检查输入消息的 `header.frame_id`，以及该 frame 到 `map` 是否存在 TF。已知地图桥接链通常依赖 `map -> uav_map` 静态 TF。

### OpenClaw 请求失败

先检查本机 OpenClaw gateway 是否在 `openclaw_base_url` 对应地址提供 `/v1/responses`。如果请求失败且 `fallback_on_error:=true`，节点会走源码中的 fallback 选择逻辑；是否自动发布任务目标还受 `send_goal_on_fallback`、`multi_goal_auto`、`continuous_auto` 等参数影响。
