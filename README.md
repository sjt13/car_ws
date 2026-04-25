# car_ws 使用说明

这个工作区当前主要用于空地协同项目无人车侧 ROS2 开发，重点包括：

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
│   ├── car_yolo/          # RKNN YOLO11 检测节点
│   └── rplidar_ros/       # 思岚 RPLIDAR ROS2 驱动（已包含 sdk 源码）
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
- 若现场发现“实际转 90°，RViz 转得更大”，现在可直接通过 `odom_yaw_scale` 做 yaw 比例标定，先从 `0.53` 附近试。

启动整车底盘（不含雷达、不含手柄）：

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

ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  use_joy:=true \
  odom_yaw_scale:=0.53
```

如果现场观察仍偏大/偏小，直接按比例微调：

- RViz 转得偏大 → 继续把 `odom_yaw_scale` 往下压，比如 `0.50`
- RViz 转得偏小 → 把 `odom_yaw_scale` 往上加，比如 `0.56`

别一上来又去乱改 teleop，那是在给自己找事。

---

## 4. 手柄控制启动

当前手柄控制由两个节点组成：

- `joy` 包的 `joy_node`：读取手柄输入，发布 `/joy`
- `car_teleop` 的 `joy_to_cmdvel_node`：把 `/joy` 转成 `/cmd_vel`

### 4.1 分开启动

先启动手柄驱动：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 run joy joy_node
```

再启动手柄转速度节点：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 run car_teleop joy_to_cmdvel_node
```

### 4.2 常见联调用法

如果底盘驱动已经在跑，可以额外开一个终端启动上面两个节点，然后检查：

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

### 4.3 手柄设备不在默认节点时

如果不是默认设备，可手动指定：

```bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

---

## 5. 车模单独显示

用于只检查 URDF / TF / RViz 展示：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_description display.launch.py
```

---

## 6. YOLO 检测节点使用


`car_yolo` 当前是第一版车端检测节点，直接读取本机摄像头，调用 RKNN YOLO11 推理，并发布：

- `/yolo/detections` → `vision_msgs/msg/Detection2DArray`
- `/yolo/image_annotated` → `sensor_msgs/msg/Image`
- `/yolo/debug_fps` → `std_msgs/msg/Float32`

### 6.0 yolo_detector_node 输入/输出说明


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

### 6.1 启动节点


```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_yolo yolo_detector.launch.py
```

### 6.2 常用参数


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

### 6.3 查看检测结果


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

### 6.4 模型文件放哪里


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

## 7. 激光雷达（思岚 C1）接入


当前工作区已接入：

- `src/rplidar_ros/` → Slamtec 官方 ROS2 驱动

### 7.1 关于 SDK 和 ROS2 包的关系


别被官网那句提示绕进去。

当前 `ros2` 分支的 `rplidar_ros` 仓库**已经自带 `sdk/` 目录和对应源码**，`CMakeLists.txt` 也是直接从包内的 `sdk/src` 编译进节点，不要求你先单独安装一份外部 SDK。

也就是说，当前这套接法是：

- 克隆 `rplidar_ros`
- 直接在 `car_ws` 里 `colcon build`

不是“先装 SDK 再编 ROS 包”这套。

只有在下面两种情况，你才需要单独碰 `rplidar_sdk`：

- 你想写**不依赖 ROS** 的底层测试程序
- 你想单独研究 SDK API / demo

### 7.2 编译雷达驱动


```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rplidar_ros
source /home/elf/car/car_ws/install/setup.bash
```

### 7.3 单独启动 C1 雷达


官方自带 launch：

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

如果想连 RViz 一起开：

```bash
ros2 launch rplidar_ros view_rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

### 7.4 结合当前车体模型启动


工作区里额外补了一个**雷达联调入口**：

```bash
ros2 launch car_driver lidar_bringup.launch.py serial_port:=/dev/ttyUSB0
```

这个 launch 会同时启动：

- `robot_state_publisher`
- `rplidar_node`
- RViz

默认参数：

- `frame_id:=laser_link`
- `serial_baudrate:=460800`

### 7.5 整车总启动（底盘 + 雷达 + 可选手柄）

现在工作区还补了一个**总 bringup**：

```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0
```

如果还要顺手把手柄也一起起：

```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  use_joy:=true
```

这个 launch 会按需启动：

- `robot_state_publisher`
- `base_driver_node`
- `rplidar_node`
- `rviz2`
- `joy_node`（当 `use_joy:=true`）
- `joy_to_cmdvel_node`（当 `use_joy:=true`）

常用参数：

- `base_port`：底盘串口，默认 `/dev/ttyS9`
- `lidar_port`：雷达串口，默认 `/dev/ttyUSB0`
- `use_rviz`：是否启动 RViz，默认 `true`
- `use_joy`：是否启动手柄链路，默认 `false`
- `joy_dev`：手柄设备，默认 `/dev/input/js0`

### 7.6 上车前先确认设备节点

雷达插上后先查：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

再看内核日志：

```bash
dmesg | tail -n 50
```

如果不是 `/dev/ttyUSB0`，启动参数记得改，别拿默认值硬怼。

### 7.7 权限问题

如果节点起不来，先怀疑串口权限，不要先怀疑玄学。

临时放权：

```bash
sudo chmod 666 /dev/ttyUSB0
```

官方仓库里也带了 udev 规则脚本：

```bash
cd /home/elf/car/car_ws/src/rplidar_ros
source scripts/create_udev_rules.sh
```

---

## 8. 已知说明

### 8.1 `car_yolo` 依赖外部 `py_utils`


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

### 8.2 当前检测节点输入方式


当前 `car_yolo` 是**直接开摄像头设备节点**，还不是订阅 ROS 图像话题。

这适合当前快速验证，但后面如果要更规整地接入系统，建议改成：

- 订阅 `/camera/image_raw`
- 后续再接 `/camera/camera_info`

---

## 9. 定位优先启动（先确保地图能出来）

如果你遇到 `Fixed Frame` 里根本没有 `map`，先不要继续硬开完整导航。先走这条**定位优先**入口，把地图和 `map` frame 先抬起来。

### 9.1 启动定位链路

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash

ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  odom_yaw_scale:=0.53
```

这条只先拉起：

- `robot_state_publisher`
- `base_driver_node`
- `rplidar_node`
- `map_server`
- `amcl`
- `rviz2`

### 9.2 确认地图是否已出来

开新终端：

```bash
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 topic echo /map --once
```

只要能看到 `/map`，说明地图链路已经起来了。

### 9.3 手动激活 lifecycle 节点

当前这台机器上，Nav2 lifecycle 自动 bringup 不稳定，所以保留一个手动激活脚本：

```bash
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
/home/elf/car/car_ws/install/car_driver/share/car_driver/scripts/nav_lifecycle_activate.sh map_server
/home/elf/car/car_ws/install/car_driver/share/car_driver/scripts/nav_lifecycle_activate.sh amcl
```

如果后面再接完整导航节点，再继续按需激活：

```bash
/home/elf/car/car_ws/install/car_driver/share/car_driver/scripts/nav_lifecycle_activate.sh all
```

### 9.4 当前已知风险

- `rplidar_node` 仍偶发 `SL_RESULT_OPERATION_TIMEOUT`
- 这会直接影响 AMCL 和导航，不是地图文件坏
- 如果 `/map` 有了但定位还是起不来，先看 `/scan` 是否还在

---

## 10. 最小导航启动（载入已保存地图）

当你已经用 `slam_toolbox` 存好了地图，例如：

- `/home/elf/maps/car_map_v1.yaml`

就不要继续开建图模式了，直接切到导航定位模式。

### 9.1 启动最小导航链路

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash

ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  odom_yaw_scale:=0.53
```

这个启动会一起拉起：

- `robot_state_publisher`
- `base_driver_node`
- `rplidar_node`
- `map_server`
- `amcl`
- `nav2_bringup` 的最小导航栈
- `rviz2`

### 9.2 第一次启动后要做什么

1. 在 RViz 里确认地图已经载入
2. 用 **2D Pose Estimate** 给车一个初始位姿
3. 看激光是否能贴住墙
4. 再用 **Nav2 Goal** 发近距离目标点测试

### 9.3 这套默认参数的取向

当前 `nav2_params.yaml` 是按这台小麦轮车的“先跑通、先稳住”思路给的：

- `amcl` 使用 `OmniMotionModel`
- `base_frame_id = base_footprint`
- 速度、加速度、膨胀半径都偏保守
- 目标是先完成定位与近距离导航闭环，不是先卷极限性能

如果后面出现：

- 走得太怂
- 横移不积极
- 贴障太保守
- 到点角度收不紧

再单独调 `nav2_params.yaml`，别一上来乱改三坨。

---

## 10. 更新维护约定


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
