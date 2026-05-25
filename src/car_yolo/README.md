# car_yolo 使用说明（车端检测链）

`car_yolo` 是当前无人车侧的视觉检测包，负责在 RK3588 平台上读取车端 RGB 图像，运行 RKNN YOLO11 推理，并将结果以 ROS2 话题形式发布。

当前节点支持两种输入方式：
- 直接打开本机 `/dev/videoX` 摄像头；
- 订阅 ROS 图像话题，例如 `/camera/color/image_raw`。

先把丑话说前面：**当前 `car_yolo` 已经完成的是“目标检测链”，还没有完成“目标空间定位链”。**

也就是说，现在已经能回答：
- 图像里有什么目标
- 目标出现在图像的哪里
- 置信度大概多少

但空间定位链目前是 **MVP 粗验证版**：已经有 `target_mapper_node` 可以尝试把检测框结合 depth 和 TF 落到 `odom` / `map`，但 Astra Pro 当前 RGB/depth 是拆分链路且未精确配准，目标区域可能取不到有效 depth，所以不要把输出当成标定级坐标。

---

## 1. 功能概述

当前节点可以：
- 从本机摄像头读取图像，或订阅 ROS 图像话题
- 在 RK3588 上跑 RKNN 推理
- 发布标准检测结果 `vision_msgs/Detection2DArray`
- 发布带框可视化图像 `sensor_msgs/Image`
- 发布当前推理帧率 `std_msgs/Float32`

另外，`target_mapper_node` 可订阅检测框、深度图、相机内参和 TF，输出目标点与 RViz Marker，用于第一版目标落图验证。

---

## 2. 启动方式

### 2.1 编译

在 `car_ws` 下：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_yolo
source install/setup.bash
```

### 2.2 启动 YOLO 检测

#### 单独验证 YOLO，直接打开摄像头

```bash
ros2 launch car_yolo yolo_detector.launch.py
```

#### 和 `orbbec_bringup` 同跑，订阅 RGB 图像话题

这时不要再让 YOLO 直接抢 `/dev/video21`，应订阅 `v4l2_camera` 已发布的 RGB 图像：

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_frame_id:=camera_color_optical_frame
```

### 2.3 启动目标落图 MVP

先确保已启动：
- `ros2 launch car_driver orbbec_bringup.launch.py`
- `ros2 launch car_yolo yolo_detector.launch.py image_topic:=/camera/color/image_raw camera_frame_id:=camera_color_optical_frame`
- 车体 TF / 里程计链，例如 `car_description display.launch.py`、`car_driver bringup.launch.py` 或定位/导航 launch。

然后启动目标落图：

```bash
ros2 launch car_yolo target_mapper.launch.py
```

调试阶段建议先落到 `base_link`，排除 `odom/map` 链路影响：

```bash
ros2 launch car_yolo target_mapper.launch.py \
  target_frame:=base_link \
  max_depth_age_sec:=2.0
```

RViz 中可添加：
- `PoseArray`：`/yolo/target_points`
- `MarkerArray`：`/yolo/target_markers`

### 2.4 常用可改参数

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  camera_device:=/dev/video21 \
  image_topic:=/camera/color/image_raw \
  camera_frame_id:=camera_color_optical_frame \
  model_path:=/home/elf/rknn/yolo11/model/yolo11n.rknn \
  target:=rk3588 \
  timer_hz:=15.0
```

参数说明：
- `model_path`：RKNN 模型路径
- `target`：推理目标平台，当前默认 `rk3588`
- `device_id`：RKNN 设备 ID，一般留空
- `camera_device`：摄像头设备节点，例如 `/dev/video21`；仅在 `image_topic` 为空时使用
- `image_topic`：ROS 图像输入话题；设置后 YOLO 不再直接打开 `/dev/video21`
- `camera_frame_id`：检测结果所属坐标系，当前默认 `camera_color_optical_frame`
- `timer_hz`：节点处理频率，默认 `15 Hz`

---

## 3. 当前输出话题

### 3.1 `/yolo/detections`
- **消息类型：** `vision_msgs/msg/Detection2DArray`
- **作用：** 发布当前这一帧的所有检测框
- **这是后续做目标定位时最核心的话题**

查看方式：

```bash
ros2 topic echo /yolo/detections
```

每个检测当前主要包含：
- `header.stamp`：检测时间戳
- `header.frame_id`：检测所属坐标系，默认 `camera_link`
- `id`：当前帧内临时编号，不是跨帧跟踪 ID
- `bbox.center.position.x / y`：检测框中心像素坐标
- `bbox.size_x / size_y`：检测框宽高（像素）
- `results[0].hypothesis.class_id`：类别名
- `results[0].hypothesis.score`：置信度

### 3.2 `/yolo/image_annotated`
- **消息类型：** `sensor_msgs/msg/Image`
- **作用：** 发布带检测框和类别标注的图像
- **用途：** 主要用于调试和可视化

查看方式：

```bash
rqt_image_view /yolo/image_annotated
```

### 3.3 `/yolo/debug_fps`
- **消息类型：** `std_msgs/msg/Float32`
- **作用：** 当前推理帧率
- **用途：** 调试性能和实时性

查看方式：

```bash
ros2 topic echo /yolo/debug_fps
```

### 3.4 `/yolo/target_points`
- **消息类型：** `geometry_msgs/msg/PoseArray`
- **作用：** `target_mapper_node` 输出的目标点数组
- **默认坐标系：** `odom`

查看方式：

```bash
ros2 topic echo /yolo/target_points
```

### 3.5 `/yolo/target_markers`
- **消息类型：** `visualization_msgs/msg/MarkerArray`
- **作用：** 给 RViz 显示目标球和文字标签

查看方式：

```bash
ros2 topic echo /yolo/target_markers
```

---

## 4. 当前输入方式与限制

### 4.1 当前输入方式
`yolo_detector_node` 支持两种输入方式：

1. `image_topic` 为空时，直接打开本机 `/dev/videoX` 摄像头设备；
2. `image_topic` 非空时，订阅 ROS 图像话题。

与 Astra Pro RGB-D 链路同跑时，推荐使用：

```bash
image_topic:=/camera/color/image_raw
```

这样 `v4l2_camera` 负责独占 `/dev/video21`，YOLO 只消费 ROS 图像，避免重复抢设备。

### 4.2 当前限制
当前版本仍有这些限制：

1. **目标落图仍是 MVP 粗验证版，不是标定级定位**
2. **没有稳定跨帧跟踪 ID**，`detection.id` 只是单帧编号
3. **没有输出目标速度、朝向、空间尺寸估计**
4. **Astra Pro 当前 RGB 与 depth 是拆分链路，未做严格 RGB-depth 配准**
5. **RGB camera_info 没有标定文件时会是空内参，`target_mapper_node` 只能用 fallback 内参粗算**
6. **目标区域如果没有有效 depth，`/yolo/target_points` 会发布空数组**

所以它现在的准确定位是：

> **检测链已打通，目标落图 MVP 已有入口，但精度和稳定性还依赖 RGB-depth 配准、相机标定、有效深度和 TF 链。**

---

## 5. 与整车导航 / 坐标系的衔接关系

这部分是最容易被说糊的，我直接给你写明白。

### 5.1 现在已经接上的部分
当前整车侧已经有：
- `/odom`
- `odom -> base_footprint`
- `base_footprint -> base_link`
- `laser_link`
- Nav2 的 `map / odom / base_footprint` 定位导航链

而 `car_yolo` 当前提供的是：
- 检测时间戳
- `camera_frame_id`
- 图像里的 2D 框
- 类别与置信度

也就是说，视觉链已经把“图像观测”准备好了，但还没把它真正变成“地图中的目标点”。

### 5.2 现在还缺什么
如果后续要把视觉检测结果落到 `odom` 或 `map`，至少还需要补下面这些条件：

#### 1）相机外参
需要在整车 TF 树中明确：
- `base_link -> camera_link`
- 必要时再补 `camera_optical_frame`

如果这条 TF 没有，检测框就永远只是“某个相机视角里的框”，没法严肃地变成车体系/地图系坐标。

#### 2）相机内参
需要知道：
- 焦距 `fx, fy`
- 主点 `cx, cy`
- 畸变参数

否则只能看像素框，不能做严格反投影。

#### 3）距离来源
只靠一个 2D 框，通常没法唯一确定目标 3D 位置。

后续至少要有一种距离信息来源：
- 深度相机 / RGB-D
- 与激光雷达或点云做关联
- 已知目标尺寸的单目估距
- 默认目标位于地面平面的几何求交

#### 4）检测时刻的载体位姿
还需要拿检测消息时间戳，到 TF/位姿缓存里取对应时刻的：
- `odom -> base_link`
- 或 `map -> base_link`

别拿“当前时刻姿态”去对“几百毫秒前的检测框”，那种做法会把目标点甩得很离谱。

---

## 6. 推荐的后续落图流程

如果后面要继续扩展，我建议按下面流程做，而不是拍脑袋硬算。

### 步骤 1：从检测框得到目标视线
输入：
- `/yolo/detections`
- 相机内参

处理：
- 取检测框中心或底边中点像素
- 反投影成相机坐标系下一条视线方向

### 步骤 2：补距离，得到相机系 3D 点
可选方案：
- 深度相机
- 雷达/点云关联
- 地面平面求交
- 已知尺寸单目估距

输出：
- `camera_link` 下的目标三维点

### 步骤 3：变换到车体系
利用相机外参：

```text
P_base = T_base_camera * P_camera
```

### 步骤 4：再变换到 `odom` 或 `map`
利用检测时刻的车体位姿：

```text
P_odom = T_odom_base * P_base
```
或
```text
P_map = T_map_base * P_base
```

### 步骤 5：做多帧融合
单帧检测不稳，后续建议至少做：
- 时间窗聚类
- 多帧平均/滤波
- 置信度门限
- 重复观测去重

不然地图上会长出一堆“目标星云”，看着热闹，实际没法用。

---

## 7. 当前最实用的工程建议

如果后续要尽快让视觉链和导航链接起来，我建议优先做这几步：

### 7.1 先在 `car_description` 里补 `camera_link`
把相机挂点正式写进 URDF / TF，而不是只在 `car_yolo` 参数里口头写 `camera_link`。

### 7.2 再补相机标定信息
至少把：
- 相机内参
- 畸变参数
- 相机外参

沉淀成固定配置，而不是临时靠人记。

### 7.3 第一阶段优先做“地面目标落点”
如果比赛目标默认在地面上，优先考虑：
- 用检测框底边中点作为近似接地点
- 视线与地面平面求交
- 转到 `odom/map`

这条路比一上来做复杂三维融合更省时间，也更适合先把链路跑通。

---

## 8. 相关包协作关系

当前和 `car_yolo` 最直接相关的包有：

- `car_description`
  - 负责整车 TF、后续相机挂点、车体坐标框架
- `car_driver`
  - 提供 `/odom`、`odom -> base_footprint` 等底层位姿链
- `rplidar_ros`
  - 可作为后续视觉-激光关联的距离来源之一
- `nav2`
  - 提供 `map / odom / base` 导航定位主链

所以 `car_yolo` 当前不是孤立包，它已经处在整车系统里，只是**它和地图坐标之间那一段融合逻辑还没补完**。
