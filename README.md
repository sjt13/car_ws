# car_ws 使用说明

这个工作区当前主要用于电赛无人车侧 ROS2 开发，重点包括：

- 底盘驱动与串口通信：`car_driver`
- 车体模型与 TF 挂点：`car_description`
- 车端目标检测节点：`car_yolo`

当前默认 ROS 版本：`humble`

> 不要再偷懒写 `source /opt/ros/*/setup.bash`，后面踩坑了别装无辜。

---

## 目录说明

```text
car_ws/
├── src/
│   ├── car_description/   # URDF/xacro、RViz 配置、模型显示 launch
│   ├── car_driver/        # 底盘驱动、串口通信、odom/TF 发布
│   └── car_yolo/          # RKNN YOLO11 检测节点
├── build/
├── install/
└── log/
```

---

## 1. 环境初始化

进入工作区后先执行：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
```

如果刚编译过，还需要：

```bash
source /home/elf/car/car_ws/install/setup.bash
```

---

## 2. 编译

全量编译：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build
```

只编译某个包：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_driver car_description car_yolo
```

---

## 3. 车体模型与底盘启动

当前推荐 TF 关系：

```text
odom -> base_footprint -> base_link
```

其中：

- `car_driver` 负责发布：`odom -> base_footprint`
- `robot_state_publisher` + URDF 负责发布：`base_footprint -> base_link` 以及其他传感器/结构挂点

启动整车 bringup：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_driver bringup.launch.py
```

如果要指定串口：

```bash
ros2 launch car_driver bringup.launch.py port:=/dev/ttyS9
```

---

## 4. 车模单独显示

用于只检查 URDF / TF / RViz 展示：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_description display.launch.py
```

---

## 5. YOLO 检测节点使用

`car_yolo` 当前是第一版车端检测节点，直接读取本机摄像头，调用 RKNN YOLO11 推理，并发布：

- `/yolo/detections` → `vision_msgs/msg/Detection2DArray`
- `/yolo/image_annotated` → `sensor_msgs/msg/Image`
- `/yolo/debug_fps` → `std_msgs/msg/Float32`

### 5.0 yolo_detector_node 输入/输出说明

#### 输入

当前节点不是订阅 ROS 图像话题，而是**直接打开本机摄像头设备**。

主要输入参数：

- `model_path`：RKNN 模型文件路径
- `camera_device`：摄像头设备节点，例如 `/dev/video21`
- `target`：RKNN 目标平台，例如 `rk3588`
- `camera_frame_id`：检测结果和图像消息使用的 frame id

当前默认输入来源是：

- 本机 V4L2 摄像头设备
- RKNN 模型文件

#### 输出

节点当前会输出 3 个话题：

1. `/yolo/detections`  
   类型：`vision_msgs/msg/Detection2DArray`

   每帧输出检测结果，包含：
   - `header.stamp`
   - `header.frame_id`
   - `detections[].bbox.center.position.x/y`
   - `detections[].bbox.size_x/size_y`
   - `detections[].results[].hypothesis.class_id`
   - `detections[].results[].hypothesis.score`
   - `detections[].id`

   说明：
   - `class_id` 当前填的是类别名字符串，例如 `person`、`car`
   - `id` 当前只是该帧中的检测序号，不是跨帧跟踪 ID

2. `/yolo/image_annotated`  
   类型：`sensor_msgs/msg/Image`

   输出带检测框的图像，用于调试和肉眼确认检测结果。

3. `/yolo/debug_fps`  
   类型：`std_msgs/msg/Float32`

   输出当前节点的粗略处理帧率，用于快速判断性能。

#### 当前不输出的内容

这个节点目前只是 **2D 检测节点**，不会直接输出：

- 目标在 `map` 坐标系中的位置
- 目标在 `base_link` 坐标系中的位置
- 目标距离
- 稳定跟踪 ID
- 跨帧目标关联结果

这些需要后续单独的定位 / 融合 / 跟踪节点处理。

### 5.1 启动节点

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_yolo yolo_detector.launch.py
```

### 5.2 常用参数

默认参数位于：

- `src/car_yolo/launch/yolo_detector.launch.py`
- `src/car_yolo/car_yolo/yolo_detector_node.py`

常用启动覆盖方式：

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  model_path:=/home/elf/rknn/yolo11/model/yolo11s.rknn \
  camera_device:=/dev/video21 \
  camera_frame_id:=camera_link \
  target:=rk3588
```

### 5.3 查看检测结果

看检测消息：

```bash
ros2 topic echo /yolo/detections
```

看带框图像：

```bash
rqt_image_view
```

然后选择：

- `/yolo/image_annotated`

如果未安装：

```bash
sudo apt install ros-humble-rqt-image-view
```

### 5.4 模型文件放哪里

当前建议模型统一放外部目录，不要塞进 `car_yolo` 包里复制多份。

当前已用路径示例：

```text
/home/elf/rknn/yolo11/model/yolo11s.rknn
```

后续切模型，优先通过 launch 参数改：

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  model_path:=/home/elf/rknn/yolo11/model/你的模型.rknn
```

---

## 6. 已知说明

### 6.1 `car_yolo` 依赖外部 `py_utils`

当前节点为了兼容现有 RKNN 工程，代码里仍依赖：

```text
/home/elf/rknn/rknn_model_zoo-self
```

也就是说，`car_yolo` 暂时不是完全自包含包。

如果后续要迁到无人机或别的机器，至少要保证以下内容一起同步：

- `car_ws/src/car_yolo`
- 模型文件目录
- `rknn_model_zoo-self` 中实际需要的 `py_utils`
- 目标机上的 ROS2 humble + RKNN 运行环境

### 6.2 当前检测节点输入方式

当前 `car_yolo` 是**直接开摄像头设备节点**，还不是订阅 ROS 图像话题。

这适合当前快速验证，但后面如果要更规整地接入系统，建议改成：

- 订阅 `/camera/image_raw`
- 后续再接 `/camera/camera_info`

---

## 7. 更新维护约定

后续每次对 `car_ws` 做出有意义更新时，应该同步更新本 README，至少补充：

- 新增了什么包/节点
- 启动方式有没有变化
- 关键话题/参数有没有变化
- 已知限制是否改变

并按当前仓库规则：

1. 先做最小自检
2. 再 `git status`
3. 提交中英混合 commit message
4. `pull --rebase`
5. `push` 到仓库实际远端分支（当前是 `master`）

别把 README 写成墓碑，更新了代码却不更新说明，那玩意儿比没有还烦。
