# car_driver

`car_driver` 维护无人车底盘串口驱动和整车启动入口，是车端 ROS2 系统的主要 bringup 包。

## 主要功能

- `base_driver_node` 订阅 `/cmd_vel`，按 STM32 串口协议下发速度指令。
- 解析 STM32 上行 `$TEL` 和 `$DBG` 帧，发布 `/odom`、`/imu/data_raw`、`/wheel_ticks`、`/stm32_debug`。
- 按参数决定是否发布 `odom -> base_footprint`。
- 提供底盘、雷达、定位、Nav2、EKF、在线 SLAM、UAV 目标桥接等 launch 入口。
- 安装少量运维脚本，例如启动 goal-slam 和清理 ROS launch 进程的脚本。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/base_driver_node.cpp` | 底盘串口、遥测解析、里程计和 IMU 发布。 |
| `include/car_driver/base_driver_node.hpp` | `base_driver_node` 类型和遥测结构定义。 |
| `launch/` | 整车 bringup、定位、导航、SLAM、UAV 目标桥接入口。 |
| `config/ekf_odom_imu.yaml` | `robot_localization/ekf_node` 参数。 |
| `config/mapper_params_online_async.yaml` | `slam_toolbox` 在线建图参数。 |
| `config/camera_color_factory_640x480.yaml` | Astra Pro RGB 出厂内参文件，供 `v4l2_camera` 加载。 |
| `scripts/goal_slam_navigator_node.py` | 目标驱动在线 SLAM 导航状态机。 |
| `scripts/start_goal_slam_nav.sh` | ELF2 上启动 goal-slam 链路的脚本。 |

## 节点

| 节点名称 | 入口 | 主要职责 | 常见启动方式 |
| --- | --- | --- | --- |
| `base_driver_node` | `src/base_driver_node.cpp` | `/cmd_vel` 到 STM32，下行 `$CAR`；解析 `$TEL`/`$DBG`，发布 odom/IMU/调试数据。 | `ros2 launch car_driver bringup.launch.py` |
| `goal_slam_navigator_node` | `scripts/goal_slam_navigator_node.py` | 接收目标点或目标数组，通过 `/navigate_to_pose` action 驱动 Nav2，并发布到达/状态话题。 | `ros2 launch car_driver goal_slam_nav_bringup.launch.py` |

## `base_driver_node` 接口

### 订阅话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令，节点会按 `max_vx/max_vy/max_wz` 裁剪。 |

### 发布话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 根据 STM32 上行 `ix/iy/iw` 积分得到的里程计。 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 根据 `$TEL` 中 IMU 原始值换算得到的线加速度和角速度。 |
| `/wheel_ticks` | `std_msgs/msg/Int32MultiArray` | `{ix, iy, iw, wa, wb, wc, wd}`。 |
| `/stm32_debug` | `std_msgs/msg/Int32MultiArray` | `{tgx, tgy, tgw, rta, rtb, rtc, rtd, pwma, pwmb, pwmc, pwmd, dt}`。 |

### TF

| TF | 默认发布者 | 说明 |
| --- | --- | --- |
| `odom -> base_footprint` | `base_driver_node` | 由参数 `publish_odom_tf` 控制，默认 `true`。 |

在 EKF 模式下，应让 `base_driver_node` 只发布 `/odom`，由 `robot_localization/ekf_node` 发布 `odom -> base_footprint`。

### 串口协议

| 方向 | 格式 | 说明 |
| --- | --- | --- |
| ROS -> STM32 | `$CAR:x,y,z!` | `x/y/z` 为 `vx/vy/wz * 1000` 后的整数。 |
| STM32 -> ROS | `$TEL:...!` | 上行遥测，源码解析为编码器速度和 IMU 原始量。 |
| STM32 -> ROS | `$DBG:...!` | 调试帧，发布到 `/stm32_debug`。 |

`$TEL` 和 `$DBG` 的字段顺序以 `base_driver_node.cpp` 中解析函数和发布数组为准。

## 主要参数

`base_driver_node` 参数：

| 参数 | 默认值 | 含义 | 单位 |
| --- | --- | --- | --- |
| `port` | `/dev/ttyS9` | 底盘串口设备。 | - |
| `baudrate` | `115200` | 串口波特率。 | baud |
| `cmd_timeout` | `0.5` | `/cmd_vel` 超时后自动下发零速度。 | s |
| `publish_rate` | `30.0` | 串口轮询和速度下发周期。 | Hz |
| `tf_publish_rate` | `20.0` | odom TF 发布频率。 | Hz |
| `publish_odom_tf` | `true` | 是否发布 `odom -> base_footprint`。 | bool |
| `odom_linear_deadband` | `0.05` | 里程计线速度死区。 | m/s |
| `odom_angular_deadband` | `0.10` | 里程计角速度死区。 | rad/s |
| `odom_yaw_scale` | `0.91` | 上行 yaw 角速度比例修正。 | 无 |
| `max_vx` | `1.5` | 下发前后速度上限。 | m/s |
| `max_vy` | `1.2` | 下发横移速度上限。 | m/s |
| `max_wz` | `6.28` | 下发角速度上限。 | rad/s |
| `reconnect_interval` | `1.0` | 串口重连间隔。 | s |
| `imu_frame_id` | `imu_link` | IMU 消息 frame。 | - |
| `odom_frame_id` | `odom` | odom 父 frame。 | - |
| `base_frame_id` | `base_footprint` | odom 子 frame。 | - |
| `imu_auto_gyro_bias` | `true` | 静止时自动估计 gyro bias。 | bool |
| `imu_auto_gyro_bias_samples` | `120` | 自动估计 bias 的静止样本数。 | samples |

IMU 原始量换算参数还包括 `imu_accel_lsb_per_g`、`imu_gyro_lsb_per_dps`、各轴 bias 和 covariance，默认值见源码参数声明。

## Launch 入口

| launch | 主要内容 | 典型用途 |
| --- | --- | --- |
| `bringup.launch.py` | `robot_state_publisher` + `base_driver_node` | 底盘串口最小联调。 |
| `full_bringup.launch.py` | 底盘 + RPLidar + 可选手柄 + 可选 RViz | 基础整车联调。 |
| `lidar_bringup.launch.py` | 车体模型 + RPLidar + 可选 RViz | 单独检查雷达和 TF。 |
| `localization_bringup.launch.py` | 底盘 + 雷达 + `nav2_bringup/localization_launch.py` | 已知地图 AMCL 定位。 |
| `nav_bringup.launch.py` | 定位链 + `nav2_bringup/navigation_launch.py` + 可选 EKF | 已知地图 Nav2 导航。 |
| `ekf_odom_imu.launch.py` | `robot_localization/ekf_node` | 融合 `/odom` 和 `/imu/data_raw`。 |
| `mapping_ekf_tf.launch.py` | full bringup + EKF + `slam_toolbox` | 在线建图，EKF 发布 odom TF。 |
| `goal_slam_nav_bringup.launch.py` | full bringup + EKF + SLAM + Nav2 + UAV 桥接 + OpenClaw + goal navigator | 目标驱动在线 SLAM 导航。 |
| `orbbec_bringup.launch.py` | Orbbec depth-only + `v4l2_camera` RGB | Astra Pro 当前拆分 RGB-D 输入链。 |
| `orbbec_registered_bringup.launch.py` | Orbbec 原生 color/depth/registration 尝试 | RGB-D 注册验证。 |
| `uav_nav_bridge_bringup.launch.py` | `nav_bringup` + `map -> uav_map` + `uav_target_bridge_node` | 已知地图下接入 UAV 共享目标。 |

## 常用启动

底盘最小链：

```bash
ros2 launch car_driver bringup.launch.py port:=/dev/ttyS9
```

底盘 + 雷达：

```bash
ros2 launch car_driver full_bringup.launch.py \
  base_port:=/dev/ttyS9 \
  lidar_port:=/dev/ttyUSB0 \
  use_rviz:=false
```

带手柄：

```bash
ros2 launch car_driver full_bringup.launch.py \
  use_joy:=true \
  joy_dev:=/dev/input/js0
```

AMCL 定位：

```bash
ros2 launch car_driver localization_bringup.launch.py \
  map:=/home/elf/maps/car_map_v1.yaml
```

Nav2 导航：

```bash
ros2 launch car_driver nav_bringup.launch.py \
  map:=/home/elf/car/car_ws/maps/my_map.yaml \
  params_file:=/home/elf/car/car_ws/src/car_description/rviz/nav2_params.yaml
```

EKF 里程计：

```bash
ros2 launch car_driver nav_bringup.launch.py \
  use_ekf_odom:=true \
  publish_odom_tf:=false \
  params_file:=/home/elf/car/car_ws/src/car_description/rviz/nav2_params_natural.yaml
```

在线 SLAM：

```bash
ros2 launch car_driver mapping_ekf_tf.launch.py
```

目标驱动在线 SLAM：

```bash
ros2 launch car_driver goal_slam_nav_bringup.launch.py
```

UAV 已知地图桥接：

```bash
ros2 launch car_driver uav_nav_bridge_bringup.launch.py
```

## 传感器和硬件依赖

- 底盘 STM32 串口，默认 `/dev/ttyS9`。
- RPLidar 串口，常用 launch 默认 `/dev/ttyUSB0`。
- Astra Pro RGB-D 相机，`orbbec_bringup.launch.py` 默认 depth 走 Orbbec SDK，RGB 走 `/dev/video21`。
- 可选手柄设备，例如 `/dev/input/js0`。

## ROS2 依赖

`package.xml` 中直接声明了 `rclcpp`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`nav_msgs`、`nav2_msgs`、`action_msgs`、`tf2`、`tf2_ros`、`robot_localization`、`slam_toolbox`、`joy`、`car_teleop`、`car_description` 和 Nav2 相关包。

`orbbec_bringup.launch.py` 还调用 `v4l2_camera`，该依赖不在 `car_driver/package.xml` 中声明，现场使用前需要确认环境已安装。

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_driver
source install/setup.bash
```

如果改动涉及 `car_description`、`car_teleop` 或 `car_yolo`，按依赖一起编译：

```bash
colcon build --packages-select car_description car_teleop car_yolo car_driver
source install/setup.bash
```

## 常用检查

检查底盘链：

```bash
ros2 topic echo /odom --once
ros2 topic echo /imu/data_raw --once
ros2 topic echo /wheel_ticks --once
ros2 topic echo /stm32_debug --once
```

检查 TF 所有权：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

检查 Nav2 action：

```bash
ros2 action list | grep navigate_to_pose
```

发送手动导航目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.5, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}" --feedback
```

## 排查

### `/cmd_vel` 有值但车不动

检查 `base_driver_node` 是否启动、串口是否存在、STM32 是否有上行帧：

```bash
ros2 node list | grep base_driver_node
ls /dev/ttyS9 /dev/ttyUSB* 2>/dev/null
ros2 topic echo /wheel_ticks --once
```

### `/odom` 有跳变或方向不对

先确认 `$TEL` 上行字段是否合理，再检查 `odom_yaw_scale`、`odom_linear_deadband`、`odom_angular_deadband`。如果启用了 EKF，还要确认只有一个节点发布 `odom -> base_footprint`。

### Nav2 或 SLAM TF 冲突

已知地图 AMCL 链中通常由 AMCL 发布 `map -> odom`。在线 SLAM 链中由 `slam_toolbox` 发布 `map -> odom`。不要同时启动 AMCL 和 `slam_toolbox` 去发布同一条 TF。

EKF 模式下推荐：

```text
map -> odom              AMCL 或 slam_toolbox
odom -> base_footprint   ekf_node
base_footprint -> ...    robot_state_publisher
```

### Orbbec RGB-D 输入异常

当前 `orbbec_bringup.launch.py` 使用拆分链路：Orbbec SDK 只开 depth，RGB 由 `v4l2_camera` 打开 `/dev/video21`。如果 RGB 设备号变化，需要覆盖 `rgb_device`。如果 depth 格式不匹配，可尝试覆盖 `depth_format`。
