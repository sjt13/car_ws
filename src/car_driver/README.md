# car_driver

`car_driver` 是当前车端 ROS2 控制链的核心包，主要负责：

- 订阅 `/cmd_vel` 并通过串口下发到底盘 STM32
- 解析 STM32 上行遥测并发布 `/odom`、`/imu/data_raw`、`/wheel_ticks` 等话题
- 提供整车常用 bringup 入口，包括：
  - `full_bringup.launch.py`
  - `localization_bringup.launch.py`
  - `nav_bringup.launch.py`

下面重点整理这 3 个 launch 文件的用途、包含参数、常用参数和常用启动指令，作为当前项目里的**最终版速查文档**。

---

## 1. `full_bringup.launch.py`

### 功能
启动底盘驱动、雷达驱动、车体模型、可选手柄控制链，以及可选 RViz。

适合用于：
- 底盘串口联调
- 雷达接入验证
- TF / 车模显示检查
- 手柄遥控测试

### 启动的节点
- `robot_state_publisher`
- `car_driver/base_driver_node`
- `rplidar_ros/rplidar_node`
- 可选：
  - `joy/joy_node`
  - `car_teleop/joy_to_cmdvel_node`
  - `rviz2`

### 包含的参数

#### 底盘驱动参数
- `base_port`，默认 `/dev/ttyS9`
- `base_baudrate`，默认 `115200`
- `cmd_timeout`，默认 `0.5`
- `publish_rate`，默认 `30.0`
- `tf_publish_rate`，默认 `20.0`
- `imu_frame_id`，默认 `imu_link`
- `odom_frame_id`，默认 `odom`
- `base_frame_id`，默认 `base_footprint`
- `odom_yaw_scale`，默认 `0.91`

#### 雷达参数
- `lidar_port`，默认 `/dev/ttyUSB0`
- `lidar_baudrate`，默认 `460800`
- `lidar_frame_id`，默认 `laser_link`
- `inverted`，默认 `false`
- `angle_compensate`，默认 `true`
- `scan_mode`，默认 `Standard`

#### 显示与手柄参数
- `use_rviz`，默认 `true`
- `use_joy`，默认 `false`
- `joy_dev`，默认 `/dev/input/js0`

#### 手柄转 `/cmd_vel` 参数
- `joy_deadzone`，默认 `0.1`
- `linear_scale`，默认 `0.4`
- `lateral_scale`，默认 `0.4`
- `angular_scale`，默认 `1.0`
- `smoothing_alpha`，默认 `0.25`
- `max_linear_accel`，默认 `0.6`
- `max_lateral_accel`，默认 `0.6`
- `max_angular_accel`，默认 `1.5`
- `max_linear_decel`，默认 `1.2`
- `max_lateral_decel`，默认 `1.2`
- `max_angular_decel`，默认 `3.0`
- `target_linear_epsilon`，默认 `0.015`
- `target_lateral_epsilon`，默认 `0.015`
- `target_angular_epsilon`，默认 `0.03`

### 常用参数
日常最常改的是：

- 底盘串口：`base_port`、`base_baudrate`
- 雷达串口：`lidar_port`、`lidar_baudrate`
- 是否显示 RViz：`use_rviz`
- 是否开启手柄：`use_joy`
- 手柄设备：`joy_dev`
- 速度倍率：`linear_scale`、`lateral_scale`、`angular_scale`
- 平滑程度：`smoothing_alpha`
- 航向修正：`odom_yaw_scale`

### 常用启动指令

#### 默认启动
```bash
ros2 launch car_driver full_bringup.launch.py
```

#### 不开 RViz
```bash
ros2 launch car_driver full_bringup.launch.py use_rviz:=false
```

#### 开启手柄控制
```bash
ros2 launch car_driver full_bringup.launch.py use_joy:=true
```

#### 指定手柄设备
```bash
ros2 launch car_driver full_bringup.launch.py \
  use_joy:=true \
  joy_dev:=/dev/input/js0
```

#### 指定底盘串口和雷达串口
```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0
```

#### 调整手柄速度倍率
```bash
ros2 launch car_driver full_bringup.launch.py \
  use_joy:=true \
  linear_scale:=0.3 \
  lateral_scale:=0.3 \
  angular_scale:=0.8
```

---

## 2. `localization_bringup.launch.py`

### 功能
启动底盘驱动、雷达驱动、车体模型，并叠加地图与 AMCL 定位链。

适合用于：
- 地图加载验证
- AMCL 定位测试
- `/map`、`/scan`、`/odom` 联调
- RViz 中设置初始位姿

### 启动的节点 / 模块
- `robot_state_publisher`
- `car_driver/base_driver_node`
- `rplidar_ros/rplidar_node`
- include 官方 `nav2_bringup/localization_launch.py`
  - `map_server`
  - `amcl`
  - `lifecycle_manager_localization`
- 可选：`rviz2`

### 包含的参数

#### 地图与 Nav2 参数
- `map`，默认 `/home/elf/maps/car_map_v1.yaml`
- `params_file`，默认 `car_description/rviz/nav2_params.yaml`
- `use_rviz`，默认 `true`

#### 底盘驱动参数
- `base_port`，默认 `/dev/ttyS9`
- `base_baudrate`，默认 `115200`
- `cmd_timeout`，默认 `0.5`
- `publish_rate`，默认 `30.0`
- `tf_publish_rate`，默认 `20.0`
- `imu_frame_id`，默认 `imu_link`
- `odom_frame_id`，默认 `odom`
- `base_frame_id`，默认 `base_footprint`
- `odom_yaw_scale`，默认 `0.91`

#### 雷达参数
- `lidar_port`，默认 `/dev/ttyUSB0`
- `lidar_baudrate`，默认 `460800`
- `lidar_frame_id`，默认 `laser_link`
- `inverted`，默认 `false`
- `angle_compensate`，默认 `true`
- `scan_mode`，默认 `Standard`

### 常用参数
日常最常改的是：

- 地图文件：`map`
- Nav2 参数文件：`params_file`
- 底盘串口：`base_port`
- 雷达串口：`lidar_port`
- 是否显示 RViz：`use_rviz`
- 航向修正：`odom_yaw_scale`

### 常用启动指令

#### 默认定位启动
```bash
ros2 launch car_driver localization_bringup.launch.py
```

#### 指定地图文件
```bash
ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

#### 不开 RViz
```bash
ros2 launch car_driver localization_bringup.launch.py use_rviz:=false
```

#### 指定底盘串口和雷达串口
```bash
ros2 launch car_driver localization_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0
```

#### 指定 Nav2 参数文件
```bash
ros2 launch car_driver localization_bringup.launch.py \
  params_file:=/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml
```

#### 调整 odom 航向比例
```bash
ros2 launch car_driver localization_bringup.launch.py \
  odom_yaw_scale:=0.91
```

---

## 3. `nav_bringup.launch.py`

### 功能
在定位链基础上继续拉起完整 Nav2 导航模块，形成“底盘 + 雷达 + 地图定位 + 路径规划 + 局部控制”的完整导航入口。

适合用于：
- 目标点导航
- 路径规划验证
- 局部避障测试
- Nav2 参数联调

### 启动的节点 / 模块
- `robot_state_publisher`
- `car_driver/base_driver_node`
- `rplidar_ros/rplidar_node`
- include 官方 `nav2_bringup/localization_launch.py`
- include 官方 `nav2_bringup/navigation_launch.py`
- 可选：`rviz2`

实际会拉起的导航相关模块包括：
- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `waypoint_follower`
- 对应 lifecycle manager

### 包含的参数

#### 地图与 Nav2 启动参数
- `map`，默认 `/home/elf/maps/car_map_v1.yaml`
- `params_file`，默认 `car_description/rviz/nav2_params.yaml`
- `use_rviz`，默认 `true`
- `autostart`，默认 `true`
- `use_respawn`，默认 `False`
- `log_level`，默认 `info`

#### 底盘驱动参数
- `base_port`，默认 `/dev/ttyS9`
- `base_baudrate`，默认 `115200`
- `cmd_timeout`，默认 `0.5`
- `publish_rate`，默认 `30.0`
- `tf_publish_rate`，默认 `20.0`
- `imu_frame_id`，默认 `imu_link`
- `odom_frame_id`，默认 `odom`
- `base_frame_id`，默认 `base_footprint`
- `odom_yaw_scale`，默认 `0.91`

#### 雷达参数
- `lidar_port`，默认 `/dev/ttyUSB0`
- `lidar_baudrate`，默认 `460800`
- `lidar_frame_id`，默认 `laser_link`
- `inverted`，默认 `false`
- `angle_compensate`，默认 `true`
- `scan_mode`，默认 `Standard`

### 常用参数
日常最常改的是：

- 地图文件：`map`
- Nav2 参数文件：`params_file`
- 底盘串口：`base_port`
- 雷达串口：`lidar_port`
- 是否显示 RViz：`use_rviz`
- 自动启动：`autostart`
- 调试日志级别：`log_level`
- 航向修正：`odom_yaw_scale`

### 常用启动指令

#### 默认导航启动
```bash
ros2 launch car_driver nav_bringup.launch.py
```

#### 指定地图文件
```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

#### 不开 RViz
```bash
ros2 launch car_driver nav_bringup.launch.py use_rviz:=false
```

#### 指定底盘串口和雷达串口
```bash
ros2 launch car_driver nav_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0
```

#### 指定 Nav2 参数文件
```bash
ros2 launch car_driver nav_bringup.launch.py \
  params_file:=/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml
```

#### 打开更详细日志
```bash
ros2 launch car_driver nav_bringup.launch.py log_level:=debug
```

#### 禁止自动 lifecycle 启动，便于调试
```bash
ros2 launch car_driver nav_bringup.launch.py autostart:=false
```

---

## 4. 三个 launch 的使用区别总结

### `full_bringup.launch.py`
适合：
- 验证底盘串口通信
- 验证雷达是否正常出 `/scan`
- 检查 TF 和车模显示
- 做手柄遥控联调

一句话总结：**底层联调用它。**

### `localization_bringup.launch.py`
适合：
- 验证地图加载
- 做 AMCL 定位
- 检查 `/map`、`/scan`、`/odom` 是否协同正常
- 在 RViz 中设置初始位姿

一句话总结：**定位联调用它。**

### `nav_bringup.launch.py`
适合：
- 做目标点导航
- 验证 planner / controller
- 观察全局路径与局部路径
- 调导航和避障参数

一句话总结：**完整导航测试用它。**
