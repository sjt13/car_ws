# car_description

`car_description` 维护无人车和 UAV 可视化模型、固定 TF 结构、RViz 配置以及 Nav2 参数文件。

## 主要功能

该包不启动底盘、雷达或导航节点。它提供以下基础资源，供 `car_driver` 的 bringup 和 Nav2 启动入口复用：

- `urdf/car.urdf.xacro`：无人车车体、轮子、IMU、雷达和相机 frame。
- `urdf/uav.urdf`：UAV 简化显示模型。
- `launch/display.launch.py`：单独查看无人车 URDF、TF 和 RViz。
- `launch/uav_model_display.launch.py`：发布 UAV 模型和可选静态位姿。
- `rviz/car.rviz`：车端 RViz 显示配置。
- `rviz/nav2_params*.yaml`：AMCL、Nav2、costmap、controller 等参数文件。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `urdf/car.urdf.xacro` | 无人车模型和固定传感器 frame。 |
| `urdf/uav.urdf` | UAV 简化模型，用于 RViz 显示。 |
| `launch/display.launch.py` | 启动 `robot_state_publisher`、`joint_state_publisher` 和可选 RViz。 |
| `launch/uav_model_display.launch.py` | 启动 UAV 的 `robot_state_publisher` 和可选静态 TF。 |
| `rviz/car.rviz` | 整车常用 RViz 配置。 |
| `rviz/nav2_params.yaml` | 已知地图 AMCL/Nav2 参数，`odom_topic` 使用 `/odom`。 |
| `rviz/nav2_params_natural.yaml` | 已知地图自然导航参数，Nav2 `odom_topic` 使用 `/odometry/filtered`。 |
| `rviz/nav2_params_goal_slam.yaml` | 在线 SLAM 目标导航参数，Nav2 `odom_topic` 使用 `/odometry/filtered`。 |

## 启动入口

| launch | 作用 |
| --- | --- |
| `display.launch.py` | 单独查看无人车模型、TF 和 RViz。 |
| `uav_model_display.launch.py` | 发布 UAV 模型，可选发布 `uav_map -> uav/base_link` 静态 TF。 |

单独显示无人车模型：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_description display.launch.py
```

显示 UAV 模型：

```bash
ros2 launch car_description uav_model_display.launch.py \
  uav_map_frame:=uav_map \
  uav_base_frame:=uav/base_link \
  uav_z:=1.0
```

## TF 和模型

`car.urdf.xacro` 中的主链路为：

```text
base_footprint -> base_link -> sensor links
```

主要 link：

| link | 来源 | 说明 |
| --- | --- | --- |
| `base_footprint` | `car.urdf.xacro` | 底盘平面参考 frame。 |
| `base_link` | `car.urdf.xacro` | 车体主 frame。 |
| `imu_link` | `car.urdf.xacro` | IMU 固定挂点。 |
| `laser_link` | `car.urdf.xacro` | RPLidar 固定挂点，当前 yaw 为 `pi`。 |
| `camera_link` | `car.urdf.xacro` | Astra Pro 相机外壳挂点。 |
| `camera_depth_frame` / `camera_depth_optical_frame` | `car.urdf.xacro` | depth frame 和 optical frame。 |
| `camera_color_frame` / `camera_color_optical_frame` | `car.urdf.xacro` | color frame 和 optical frame。 |

`odom -> base_footprint` 不由本包发布，通常由 `car_driver/base_driver_node` 或 `robot_localization/ekf_node` 发布。

## 主要几何参数

参数直接来自 `urdf/car.urdf.xacro`：

| 参数 | 当前值 | 单位 | 说明 |
| --- | --- | --- | --- |
| `body_length` | `0.280` | m | 车体长度。 |
| `body_width` | `0.150` | m | 车体宽度。 |
| `body_height` | `0.049` | m | 车体高度。 |
| `wheel_radius` | `0.040` | m | 轮半径。 |
| `wheel_width` | `0.035` | m | 轮宽。 |
| `wheel_track` | `0.212` | m | 左右轮距。 |
| `wheel_base` | `0.200` | m | 前后轴距。 |
| `imu_x/y/z` | `-0.090 / 0.0 / 0.025` | m | IMU 相对 `base_link` 的位置。 |
| `laser_x/y/z` | `0.000 / 0.000 / 0.080` | m | 雷达相对 `base_link` 的位置。 |
| `camera_x/y/z` | 由 xacro 表达式计算 | m | 相机相对 `base_link` 的位置。 |

## Nav2 参数文件

| 文件 | 主要使用场景 | 关键 frame/topic |
| --- | --- | --- |
| `rviz/nav2_params.yaml` | 已知地图 AMCL/Nav2 | `map`、`odom`、`base_footprint`、`/odom`、`/scan` |
| `rviz/nav2_params_natural.yaml` | `uav_nav_bridge_bringup.launch.py` 默认参数 | `map`、`odom`、`base_footprint`、`/odometry/filtered`、`/scan` |
| `rviz/nav2_params_goal_slam.yaml` | `goal_slam_nav_bringup.launch.py` 默认参数 | `map`、`odom`、`base_footprint`、`/odometry/filtered`、`/scan` |

三个参数文件都包含 controller、planner、local/global costmap、behavior、velocity smoother 等 Nav2 配置。`nav2_params.yaml` 中 AMCL 使用 `base_frame_id: base_footprint`、`global_frame_id: map`、`odom_frame_id: odom`、`scan_topic: scan`。

当前 costmap footprint 在这些文件中保持为：

```text
[[0.17, 0.11], [0.17, -0.11], [-0.17, -0.11], [-0.17, 0.11]]
```

## 依赖

### ROS2 包依赖

- `ament_cmake`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `tf2_ros`
- `rviz2`
- `xacro`

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_description
source install/setup.bash
```

## 常用检查

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

检查 robot description：

```bash
ros2 param get /robot_state_publisher robot_description
```

## 排查

RViz 中没有模型时，先确认 `robot_state_publisher` 是否启动，并检查是否 source 了工作区：

```bash
ros2 node list | grep robot_state_publisher
echo $AMENT_PREFIX_PATH
```

导航中激光或相机位置不对时，先检查本包 URDF 中的固定 frame，再检查对应传感器驱动发布的 `frame_id` 是否一致。
