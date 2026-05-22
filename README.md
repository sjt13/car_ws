# car_ws

`car_ws` 是当前空地协同项目中**无人车侧 ROS2 工作区**，主要承载底盘驱动、车体模型、激光雷达接入、定位导航以及车端视觉检测。

当前默认 ROS 版本：`humble`

这个工作区的目标不是做零散单节点实验，而是逐步形成一套能用于实车联调的完整车端系统。

---

## 1. 工作区当前包含哪些核心包

```text
car_ws/
├── src/
│   ├── car_description/   # 车体模型、TF 挂点、RViz 配置、Nav2 参数
│   ├── car_driver/        # 底盘驱动、串口通信、odom/TF 发布、整车 bringup
│   ├── car_teleop/        # 手柄转 /cmd_vel
│   ├── car_yolo/          # RKNN YOLO11 车端检测节点
│   └── rplidar_ros/       # RPLidar ROS2 驱动
├── build/
├── install/
└── log/
```

各包职责可以简单理解为：

- `car_driver`：负责“车怎么动”
- `car_description`：负责“车长什么样、TF 怎么挂”
- `rplidar_ros`：负责“雷达怎么接进来”
- `car_teleop`：负责“手柄怎么变成 `/cmd_vel`”
- `car_yolo`：负责“车端怎么做目标检测”

---

## 2. 环境初始化

进入工作区后，先做环境初始化：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
```

如果已经编译过，还要继续 source 工作区环境：

```bash
source /home/elf/car/car_ws/install/setup.bash
```

推荐日常顺序：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
```

别再写那种 `source /opt/ros/*/setup.bash` 的懒狗命令了，后面踩坑只会回旋镖打自己。

---

## 3. 编译方式

### 全量编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 编译核心自有包

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_driver car_description car_teleop car_yolo
```

### 单独编译某个包

例如只编译 `car_yolo`：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_yolo
```

编译完成后记得：

```bash
source /home/elf/car/car_ws/install/setup.bash
```

---

## 4. 当前系统的基础坐标关系

当前推荐 TF 主链为：

```text
odom -> base_footprint -> base_link
```

其中：
- `car_driver` 负责发布 `odom -> base_footprint`
- `robot_state_publisher + car_description` 负责发布 `base_footprint -> base_link` 及各传感器挂点

当前模型中已经包含：
- `imu_link`
- `laser_link`

后续如果要让视觉链真正接入地图和导航，推荐继续在 `car_description` 中补：
- `camera_link`
- `camera_optical_frame`

---

## 5. 最常用的几个启动入口

当前最常用的整车 launch 都在 `car_driver` 包里。

### 5.1 基础整车 bringup
用于底盘 + 雷达 + 可选手柄 + 可选 RViz 联调：

```bash
ros2 launch car_driver full_bringup.launch.py
```

常见覆盖方式：

```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  use_joy:=true
```

适合：
- 底盘串口联调
- 雷达是否正常出 `/scan`
- TF / 车模检查
- 手柄遥控测试

### 5.2 定位 bringup
用于底盘 + 雷达 + 地图 + AMCL 定位：

```bash
ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

适合：
- 地图加载验证
- AMCL 定位测试
- 检查 `/map`、`/scan`、`/odom` 是否协同正常
- 在 RViz 中设置初始位姿

### 5.3 导航 bringup
用于底盘 + 雷达 + 定位 + Nav2 导航：

```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

适合：
- 目标点导航
- planner / controller 联调
- 全局路径和局部路径观察
- 避障与导航参数测试

> 三个 launch 的完整参数、常用参数和常用命令速查，请直接看：
>
> `src/car_driver/README.md`

---

## 6. 车体模型与 RViz 显示

如果只是想单独检查 URDF、TF 和 RViz 显示，不必起整车链路，直接启动：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_description display.launch.py
```

当前 `car_description` 已包含：
- 车体主体结构
- 四个轮子
- IMU 挂点
- 激光雷达挂点
- Nav2 参数文件

更详细的模型与参数说明见：

- `src/car_description/README.md`

---

## 7. 手柄控制

当前手柄控制由两部分组成：
- `joy` 包的 `joy_node`
- `car_teleop` 的 `joy_to_cmdvel_node`

如果不想通过总 bringup 自动起，也可以单独启动。

### 启动手柄驱动

```bash
ros2 run joy joy_node
```

### 启动手柄转速度节点

```bash
ros2 run car_teleop joy_to_cmdvel_node
```

### 指定手柄设备

```bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

联调时可检查：

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

---

## 8. 激光雷达接入

当前工作区已接入：
- `src/rplidar_ros/` → RPLidar ROS2 驱动

### 单独启动 C1 雷达

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

### 连 RViz 一起开

```bash
ros2 launch rplidar_ros view_rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

### 结合当前车体模型做雷达联调

```bash
ros2 launch car_driver lidar_bringup.launch.py serial_port:=/dev/ttyUSB0
```

### 上车前先确认设备节点

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

再看内核日志：

```bash
dmesg | tail -n 50
```

如果不是 `/dev/ttyUSB0`，启动参数记得改，别拿默认值硬怼。

---

## 9. 车端视觉检测

`car_yolo` 当前负责：
- 从本机摄像头读取图像
- 在 RK3588 上跑 RKNN YOLO11 推理
- 发布检测结果、带框图像和 FPS

### 启动方式

```bash
ros2 launch car_yolo yolo_detector.launch.py
```

### 常用覆盖方式

```bash
ros2 launch car_yolo yolo_detector.launch.py \
  camera_device:=/dev/video21 \
  camera_frame_id:=camera_link \
  model_path:=/home/elf/rknn/yolo11/model/yolo11n.rknn \
  target:=rk3588 \
  timer_hz:=15.0
```

### 当前输出的话题
- `/yolo/detections`
- `/yolo/image_annotated`
- `/yolo/debug_fps`

### 当前能力边界
当前已经能稳定解决的是：
- 目标类别识别
- 图像中目标框位置输出
- 时间戳与 frame_id 输出

当前还没有直接解决的是：
- 目标在 `odom` / `map` 中的位置
- 跨帧稳定跟踪 ID
- 与导航地图的自动融合

更完整说明见：
- `src/car_yolo/README.md`

---

## 10. 当前推荐的联调顺序

如果现场要排查问题，推荐按这个顺序来，别一上来就全开然后怀疑人生。

### 第一步：只查底层链路
```bash
ros2 launch car_driver full_bringup.launch.py use_rviz:=false
```
确认：
- 串口能不能通
- `/odom` 有没有
- `/scan` 有没有

### 第二步：查 TF 和车模
```bash
ros2 launch car_description display.launch.py
```
确认：
- 车模结构是否正常
- `laser_link`、`imu_link` 是否合理

### 第三步：查定位
```bash
ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```
确认：
- 地图能否正常加载
- `map` frame 是否存在
- AMCL 是否工作

### 第四步：查完整导航
```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```
确认：
- planner / controller 是否正常
- 全局路径、局部路径、局部代价地图是否合理

### 第五步：再接视觉链
```bash
ros2 launch car_yolo yolo_detector.launch.py
```
确认：
- 相机是否正常
- 推理是否正常
- `/yolo/detections` 是否稳定输出

这套顺序的核心原则就一句话：

> **先把底盘、雷达、定位、导航站稳，再谈视觉融合。**

别把所有锅同时背在身上，那样只会把自己查麻。

---

## 11. 当前几个最值得看的文档

- `src/car_driver/README.md`
  - 整车 bringup、定位、导航的参数和启动方法
- `src/car_description/README.md`
  - 车体模型、TF 挂点、Nav2 参数说明
- `src/car_yolo/README.md`
  - 检测链输出语义、与坐标系/导航的衔接关系

如果你要快速上手，不想在一堆 launch 和参数里乱翻，先看这三份，能省不少时间。
