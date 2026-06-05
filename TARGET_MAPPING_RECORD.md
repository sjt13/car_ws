# 车端目标落图记录

更新时间：2026-06-05

## 当前状态

旧目标落图 MVP 已删除；新版目标落图 v1 已恢复入口，默认使用 depth 模式，也支持 `ground_plane` fallback。当前还补齐了 UAV 目标桥接链路，可以把无人机 `/uav/shared/target_pose` 转成车端 `map` 下的目标点和 RViz Marker。

已新增：
- `src/car_yolo/car_yolo/target_mapper_node.py`
- `src/car_yolo/launch/target_mapper.launch.py`
- `src/car_driver/launch/orbbec_registered_bringup.launch.py`
- `src/car_driver/launch/uav_nav_bridge_bringup.launch.py`
- `src/car_yolo/car_yolo/uav_target_bridge_node.py`
- `src/car_yolo/launch/uav_target_bridge.launch.py`
- `setup.py` 中的 `target_mapper_node` console script 和 `target_mapper.launch.py` 安装入口

当前 `car_yolo` 保留 YOLO 检测链，并恢复 `/yolo/target_points` 与 `/yolo/target_markers` 车端目标落图输出；UAV 桥接输出 `/uav/target_points_map` 与 `/uav/target_markers`。

## 来源文件

- `src/car_driver/README.md`
- `src/car_driver/launch/orbbec_bringup.launch.py`
- `src/car_yolo/README.md`
- `src/car_yolo/launch/yolo_detector.launch.py`
- `src/car_yolo/launch/target_mapper.launch.py`
- `src/car_yolo/car_yolo/target_mapper_node.py`
- `src/car_yolo/launch/uav_target_bridge.launch.py`
- `src/car_yolo/car_yolo/uav_target_bridge_node.py`
- `src/car_driver/launch/uav_nav_bridge_bringup.launch.py`

## 已保留链路

目标落图 v1 复用三部分输入链路：

1. `car_driver/orbbec_bringup.launch.py`
   - 启动 Astra Pro 的 RGB + Depth 输入链路。
   - Depth 由 `orbbec_camera` 发布，只启用 depth，不启用 color。
   - RGB 由 `v4l2_camera` 读取 `/dev/video21` 后发布。
   - RGB 输出话题为 `/camera/color/image_raw` 和 `/camera/color/camera_info`。
   - Depth 输出话题为 `/camera/depth/image_raw` 和 `/camera/depth/camera_info`。

2. `car_yolo/yolo_detector.launch.py`
   - 启动 `yolo_detector_node`。
   - 推荐订阅 `/camera/color/image_raw`，不要直接抢占 `/dev/video21`。
   - 输出 `/yolo/detections`、`/yolo/image_annotated`、`/yolo/debug_fps`。

3. `car_yolo/target_mapper.launch.py`
   - 启动 `target_mapper_node`。
   - 订阅 `/yolo/detections`、`/camera/depth/image_raw`、`/camera/color/camera_info`。
   - 默认 `projection_mode:=depth`、`sample_point:=bottom_center`、`target_frame:=odom`。
   - 输出 `/yolo/target_points`、`/yolo/target_markers`。

## 检测链启动顺序

先启动 RGB-D 输入：

```bash
ros2 launch car_driver orbbec_bringup.launch.py
```

再让 YOLO 订阅 RGB 图像话题，不再直接占用 `/dev/video21`：

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_frame_id:=camera_color_optical_frame
```

最后启动目标落图：

```bash
ros2 launch car_yolo target_mapper.launch.py
```

如果 RGB-depth 配准暂时不稳，先切地面交点模式：

```bash
ros2 launch car_yolo target_mapper.launch.py projection_mode:=ground_plane
```

## Orbbec 原生配准验证

优先尝试同源 RGB-D 对齐：

```bash
ros2 launch car_driver orbbec_registered_bringup.launch.py
```

该入口同时启用 color/depth，并打开：
- `depth_registration:=true`
- `align_mode:=HW`
- `enable_publish_extrinsic:=true`

如果仍出现 `OB_SENSOR_COLOR Match openni video mode failed`，继续使用当前 split 链路 `orbbec_bringup.launch.py`，并通过 `target_mapper_node` 的 `rgb_to_depth_scale_*` / `rgb_to_depth_offset_*` 或 `projection_mode:=ground_plane` 调试。

## 可观察输出

- `/yolo/detections`：`vision_msgs/msg/Detection2DArray`，YOLO 2D 检测框。
- `/yolo/image_annotated`：`sensor_msgs/msg/Image`，带框调试图。
- `/yolo/debug_fps`：`std_msgs/msg/Float32`，推理帧率。
- `/yolo/target_points`：`geometry_msgs/msg/PoseArray`，目标点数组。
- `/yolo/target_markers`：`visualization_msgs/msg/MarkerArray`，RViz 目标显示。
- `/uav/target_points_map`：`geometry_msgs/msg/PoseArray`，无人机共享目标桥接到车端 `map` 后的目标点。
- `/uav/target_markers`：`visualization_msgs/msg/MarkerArray`，无人机目标在车端 RViz 中的显示。

## UAV 目标桥接链路

当前 UAV 侧业务话题采用 `/uav/...` 命名，避免与车端话题冲突。车端桥接默认订阅：

```text
/uav/shared/target_pose
```

该话题应表示目标在 `uav_map` 或共享地图中的 `PoseStamped`。车端通过 `map -> uav_map` 静态 TF 转换到车端 `map`，默认输出：

```text
/uav/target_points_map
/uav/target_markers
```

车端一键启动入口：

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py
```

当前默认对齐参数：

```text
map -> uav_map: x=-0.78, y=-0.61, z=0.0, yaw=0.0
```

如果后续无人机起点、T265 / UAV odom 原点或车端建图原点变化，需要重新校准这组参数。

人工确认导航推荐使用 Nav2 action：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 目标x, y: 目标y, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}" --feedback
```

## 重新开发前的约束

新落图节点调试时需要关注：

- Astra Pro 当前 RGB 与 depth 是拆分链路，尚未完成严格 RGB-depth 配准。
- `/camera/color/camera_info` 在没有正式标定文件时可能为空内参。
- 当前检测 ID 是单帧临时编号，不是稳定跨帧跟踪 ID。
- 目标落到 `odom` / `map` 的精度仍依赖相机外参、TF 链路、深度质量和检测时间戳对齐。
- UAV 目标落到车端 `map` 的精度依赖 `map -> uav_map` 对齐参数。

## 后续建议

- 在 `car_description` 中正式补充相机安装 link / optical frame 及外参。
- 补充 RGB 相机内参和畸变参数，避免长期依赖 fallback 内参。
- 优先验证“地面目标落点”：用检测框底边中点或目标接地点近似，再结合地面平面或 depth 转换到 `base_link` / `odom`。
- 做多帧融合、置信度过滤和重复目标去重，否则地图上容易出现抖动目标点。
