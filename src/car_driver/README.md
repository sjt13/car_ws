# car_driver

`car_driver` 是当前车端 ROS2 控制链的核心包，主要负责：

- 订阅 `/cmd_vel` 并通过串口下发到底盘 STM32
- 解析 STM32 上行遥测并发布 `/odom`、`/imu/data_raw`、`/wheel_ticks` 等话题
- 提供整车常用 bringup 入口，包括：
  - `full_bringup.launch.py`
  - `localization_bringup.launch.py`
  - `nav_bringup.launch.py`
  - `uav_nav_bridge_bringup.launch.py`

下面重点整理这些 launch 文件的用途、包含参数、常用参数和常用启动指令，作为当前项目里的**最终版速查文档**。

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
- `map`，默认 `/home/elf/car/car_ws/maps/my_map.yaml`
- `params_file`，默认 `/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml`
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

当前常用完整写法（现场优先用这个，路径更明确）：

```bash
ros2 launch car_driver nav_bringup.launch.py \
  params_file:=/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml \
  map:=/home/elf/car/car_ws/maps/my_map.yaml
```

#### 指定地图文件
```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/car/car_ws/maps/my_map.yaml
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

---

## 5. `orbbec_bringup.launch.py`

### 功能
启动 Astra Pro 当前在 ELF2 上可用的 RGB + Depth 拆分链路：

- RGB：通过 UVC `/dev/video21` 由 `v4l2_camera` 发布；
- Depth：通过 `orbbec_camera` depth-only 发布；
- 不启用 Orbbec/OpenNI 的 color stream，因为当前实测会报 `OB_SENSOR_COLOR Match openni video mode failed`。

### 输出话题
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

### 常用启动指令
```bash
ros2 launch car_driver orbbec_bringup.launch.py
```

如果现场 depth 默认格式不稳，优先试上午验证过的 `Y11`：

```bash
ros2 launch car_driver orbbec_bringup.launch.py depth_format:=Y11
```

### 常用参数
- `rgb_device`，默认 `/dev/video21`
- `rgb_width` / `rgb_height`，默认 `640` / `480`
- `rgb_pixel_format`，默认 `YUYV`
- `rgb_frame_id`，默认 `camera_color_optical_frame`
- `depth_width` / `depth_height`，默认 `320` / `240`
- `depth_fps`，默认 `30`
- `depth_format`，默认 `Y12`

注意：RGB 的 `/camera/color/camera_info` 已支持加载出厂内参配置；目标落图仍是工程验证链路，精度取决于 RGB-depth 配准、相机外参和 TF。

---

## 6. `uav_nav_bridge_bringup.launch.py`

### 功能
一键启动车端空地协同目标落图链路：

- include `nav_bringup.launch.py`，启动车端 Nav2；
- 发布 `map -> uav_map` 静态 TF，用于把 UAV 共享地图坐标对齐到车端地图；
- 启动 `car_yolo/uav_target_bridge_node`，把无人机目标转换成车端 `map` 下的 PoseArray 和 RViz Marker。

### 当前默认参数
- `map`：`/home/elf/car/car_ws/maps/214map.yaml`
- `params_file`：`/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml`
- `use_rviz`：`false`
- `uav_map_x`：`-0.78`
- `uav_map_y`：`-0.61`
- `uav_map_z`：`0.0`
- `uav_map_roll`：`0.0`
- `uav_map_pitch`：`0.0`
- `uav_map_yaw`：`0.0`
- `source_target_topic`：`/uav/shared/target_pose`
- `target_points_topic`：`/uav/target_points_map`
- `target_markers_topic`：`/uav/target_markers`

### 常用启动指令

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py
```

如果需要临时调整 UAV 地图对齐：

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py \
  uav_map_x:=-0.78 \
  uav_map_y:=-0.61 \
  uav_map_yaw:=0.0
```

### 手动确认导航到目标
桥接后先查看当前目标：

```bash
ros2 topic echo /uav/target_points_map --once
```

确认要过去后，用 Nav2 action 发送目标，不建议只发 `/goal_pose`：

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

### 桌面脚本
ELF2 桌面保留了两个常用入口：

- `UAV Car Bringup`：启动 Nav2 + `map -> uav_map` TF + UAV target bridge；
- `Kill ROS Launches`：清理当前用户下的 ROS2 launch 和常见节点进程。

---

## 7. `mapping_ekf_tf.launch.py`

### 功能
用于在线建图时启用 MPU6050 辅助的 EKF odom TF 链路：

- `base_driver_node` 继续发布 `/odom`、`/imu/data_raw`、`/wheel_ticks`、`/stm32_debug`
- `base_driver_node` 不发布 `odom -> base_footprint`
- `robot_localization/ekf_node` 发布 `/odometry/filtered`，并接管 `odom -> base_footprint`
- `slam_toolbox` 发布 `map -> odom`
- 不启动 AMCL / Nav2，避免多个节点同时发布 `map -> odom`

这个 launch 是在保留原始 wheel odom 链路的基础上做 EKF TF 建图验证；如果需要回退，继续使用 `full_bringup.launch.py` 默认参数即可。

### 启动命令

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_driver mapping_ekf_tf.launch.py
```

默认会使用 `joy/game_controller_node` 读取 ELF2 上的 Twin USB 手柄接收器，不依赖 `/dev/input/js0`。

### 常用参数

- `base_port`，默认 `/dev/ttyS9`
- `lidar_port`，默认 `/dev/ttyUSB0`
- `odom_yaw_scale`，默认 `0.91`
- `ekf_params_file`，默认 `car_driver/config/ekf_odom_imu.yaml`
- `output_odom_topic`，默认 `/odometry/filtered`
- `use_game_controller`，默认 `true`
- `use_joy_to_cmdvel`，默认 `true`
- `linear_scale`，默认 `0.25`
- `lateral_scale`，默认 `0.20`
- `angular_scale`，默认 `0.65`
- `use_rviz`，默认 `false`

### TF 所有权

EKF TF 建图模式下只能有一个 `odom -> base_footprint` 发布者：

```text
map -> odom              slam_toolbox
odom -> base_footprint   ekf_node
base_footprint -> ...    robot_state_publisher
```

不要同时手动启动 `full_bringup.launch.py publish_odom_tf:=true` 和 `ekf_odom_imu.launch.py publish_tf:=true`，否则会造成 TF 冲突。

### 录包建议

```bash
ros2 bag record -o slam_ekf_tf_debug \
  /cmd_vel /joy /odom /odometry/filtered /imu/data_raw \
  /scan /map /tf /tf_static /wheel_ticks /stm32_debug
```

验证时建议先做 2 到 4 分钟小闭环：慢速直行、几次慢速转弯、回到起点附近；不要后退，不要快速原地猛转。
