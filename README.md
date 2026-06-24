# car_ws

`car_ws` 是无人车侧 ROS2 Humble 工作区，包含底盘串口驱动、车体模型、雷达接入、定位导航、RGB-D 目标落图和 UAV 共享目标桥接相关代码。

当前实车工作区通常位于 ELF2：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 包清单

### 项目本地功能包

| 包 | 类型 | 主要职责 |
| --- | --- | --- |
| `car_description` | `ament_cmake` | 维护无人车 URDF/xacro、UAV 简化模型、RViz 配置和 Nav2 参数文件。 |
| `car_driver` | `ament_cmake` | 维护底盘串口节点、整车 bringup、雷达/定位/导航/SLAM/UAV 桥接启动入口。 |
| `car_teleop` | `ament_python` | 将 `/joy` 手柄输入平滑转换为 `/cmd_vel`。 |
| `car_yolo` | `ament_python` | 维护 RKNN YOLO 检测、车端 RGB-D 目标落图、UAV 目标桥接和 OpenClaw 目标决策节点。 |

### 第三方驱动和模型包

| 包 | 类型 | 主要职责 |
| --- | --- | --- |
| `orbbec_camera` | `ament_cmake` | Orbbec RGB-D 相机 ROS2 驱动，提供 `orbbec_camera_node`。 |
| `orbbec_camera_msgs` | `ament_cmake` | Orbbec 驱动使用的消息和服务定义。 |
| `orbbec_description` | `ament_cmake` | Orbbec 相机 URDF、mesh 和显示 launch。 |
| `rplidar_ros` | `ament_cmake` | Slamtec RPLidar ROS2 驱动，发布 `/scan`。 |

### 公共目录

| 路径 | 说明 |
| --- | --- |
| `maps/` | 当前仓库内保存的地图 yaml 和配套地图文件。 |
| `analysis/` | 检查、截图或临时分析输出，不是 ROS2 功能包。 |
| `TARGET_MAPPING_RECORD.md` | 目标落图和 UAV 目标桥接的阶段记录。 |

## 常用编译

全量编译：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只编译项目本地包：

```bash
colcon build --packages-select car_description car_driver car_teleop car_yolo
source install/setup.bash
```

只编译某个包：

```bash
colcon build --packages-select car_driver
source install/setup.bash
```

## 坐标关系

当前车端主 TF 链由以下几部分组成：

```text
map -> odom -> base_footprint -> base_link
```

常见发布者：

| TF | 发布者 |
| --- | --- |
| `map -> odom` | AMCL 或 `slam_toolbox`，取决于启动入口。 |
| `odom -> base_footprint` | `base_driver_node` 或 `robot_localization/ekf_node`，取决于 `publish_odom_tf`/EKF 配置。 |
| `base_footprint -> base_link` 及传感器固定 frame | `robot_state_publisher` 读取 `car_description/urdf/car.urdf.xacro`。 |
| `map -> uav_map` | `uav_nav_bridge_bringup.launch.py` 或 `goal_slam_nav_bringup.launch.py` 中的静态 TF。 |

车体模型中已经定义 `imu_link`、`laser_link`、`camera_link`、`camera_depth_optical_frame`、`camera_color_optical_frame` 等 frame。相机和 UAV 地图外参需要按现场安装继续校准，README 只记录当前代码默认值。

## 常用启动入口

### 底盘最小 bringup

只启动车体模型和 `base_driver_node`：

```bash
ros2 launch car_driver bringup.launch.py
```

默认底盘串口为 `/dev/ttyS9`，波特率为 `115200`。

### 基础整车 bringup

启动底盘、雷达、可选手柄和可选 RViz：

```bash
ros2 launch car_driver full_bringup.launch.py
```

常用覆盖：

```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  use_joy:=true \
  use_rviz:=false
```

### 定位与导航

AMCL 定位：

```bash
ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

Nav2 导航：

```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/car/car_ws/maps/my_map.yaml
```

`nav_bringup.launch.py` 默认使用 `car_description/rviz/nav2_params.yaml`，也支持 `use_ekf_odom:=true` 切换为 EKF 发布 `odom -> base_footprint`。

### 在线 SLAM 与目标驱动导航

只做在线 SLAM/EKF TF 验证：

```bash
ros2 launch car_driver mapping_ekf_tf.launch.py
```

目标驱动在线 SLAM 导航：

```bash
ros2 launch car_driver goal_slam_nav_bringup.launch.py
```

该入口组合了基础 bringup、EKF、`slam_toolbox`、Nav2、UAV 目标桥接、OpenClaw 目标决策和 `goal_slam_navigator_node.py`。它不启动 AMCL。

### UAV 目标桥接

已知地图导航链 + UAV 共享目标桥接：

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py
```

当前默认：

| 参数 | 默认值 |
| --- | --- |
| `map` | `/home/elf/car/car_ws/maps/214map.yaml` |
| `params_file` | `/home/elf/car/car_ws/src/car_description/rviz/nav2_params_natural.yaml` |
| `use_ekf_odom` | `true` |
| `publish_odom_tf` | `false` |
| `uav_map_x` / `uav_map_y` / `uav_map_yaw` | `-0.78` / `-0.61` / `0.0` |
| `source_target_topic` | `/uav/shared/target_pose` |
| `target_points_topic` | `/uav/target_points_map` |
| `target_markers_topic` | `/uav/target_markers` |

## 主要话题

| 话题 | 类型 | 来源 | 用途 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `car_teleop`、Nav2 或其他控制节点 | 底盘速度指令。 |
| `/odom` | `nav_msgs/msg/Odometry` | `base_driver_node` | STM32 上行速度积分后的里程计。 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | `base_driver_node` | STM32 上行 IMU 原始量换算结果。 |
| `/wheel_ticks` | `std_msgs/msg/Int32MultiArray` | `base_driver_node` | 上行编码器/速度整型数据。 |
| `/stm32_debug` | `std_msgs/msg/Int32MultiArray` | `base_driver_node` | `$DBG` 调试帧。 |
| `/scan` | `sensor_msgs/msg/LaserScan` | `rplidar_node` | Nav2、AMCL、SLAM 使用的激光数据。 |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `yolo_detector_node` | 车端目标检测框。 |
| `/yolo/target_points` | `geometry_msgs/msg/PoseArray` | `target_mapper_node` | RGB-D 目标落图结果。 |
| `/uav/target_points_map` | `geometry_msgs/msg/PoseArray` | `uav_target_bridge_node` | UAV 共享目标转换到车端 `map` 后的点。 |

## 常用检查指令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /tf --once
ros2 topic echo /uav/target_points_map --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map uav_map
```

检查串口设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null
dmesg | tail -n 50
```

## 阅读顺序

接手时建议按下面顺序看：

1. `src/car_driver/README.md`：整车启动、底盘串口、导航、SLAM、UAV 桥接。
2. `src/car_description/README.md`：车体 TF、URDF、Nav2 参数文件。
3. `src/car_yolo/README.md`：视觉检测、目标落图、OpenClaw 决策。
4. `src/car_teleop/README.md`：手柄到 `/cmd_vel`。
5. `src/rplidar_ros/README.md`、`src/orbbec_camera/README.md`：第三方传感器驱动在本仓库中的使用方式。
