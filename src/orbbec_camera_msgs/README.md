# orbbec_camera_msgs

`orbbec_camera_msgs` 是 Orbbec 驱动使用的 ROS2 消息和服务定义包，当前仓库把它作为第三方接口包使用。

## 主要功能

该包只生成接口，不包含运行节点。`orbbec_camera` 依赖这些消息和服务来发布设备信息、外参、IMU 信息、RGBD 组合消息，并提供设备查询和参数设置服务。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `msg/` | Orbbec 自定义消息定义。 |
| `srv/` | Orbbec 自定义服务定义。 |
| `CMakeLists.txt` | 使用 `rosidl_generate_interfaces` 生成接口。 |
| `package.xml` | 声明 `rosidl_default_generators`、`sensor_msgs`、`std_msgs` 等依赖。 |

## 消息

| 名称 | 主要字段 | 用途 |
| --- | --- | --- |
| `DeviceInfo` | `header`、`name`、`serial_number`、`firmware_version`、`supported_min_sdk_version`、`hardware_version` | 设备信息。 |
| `Extrinsics` | `header`、`rotation[9]`、`translation[3]` | 相机不同传感器之间的外参。 |
| `IMUInfo` | `header`、`noise_density`、`random_walk`、`bias`、`gravity` 等 | IMU 标定和噪声信息。 |
| `Metadata` | `header`、`json_data` | JSON 形式的元数据。 |
| `RGBD` | `rgb_camera_info`、`depth_camera_info`、`rgb`、`depth` | RGB-D 组合消息。 |

## 服务

| 名称 | 请求 | 响应 | 用途 |
| --- | --- | --- | --- |
| `GetBool` | 空 | `data`、`success`、`message` | 查询 bool 状态。 |
| `GetCameraInfo` | 空 | `sensor_msgs/msg/CameraInfo info` | 查询相机内参。 |
| `GetDeviceInfo` | 空 | `DeviceInfo info`、`success`、`message` | 查询设备信息。 |
| `GetInt32` | 空 | `data`、`success`、`message` | 查询整型参数。 |
| `GetString` | 空 | `data`、`success`、`message` | 查询字符串参数。 |
| `SetInt32` | `int32 data` | `success`、`message` | 设置整型参数。 |
| `SetString` | `string data` | `success`、`message` | 设置字符串参数。 |

## 依赖

- `ament_cmake`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `sensor_msgs`
- `std_msgs`

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_camera_msgs
source install/setup.bash
```

如果同时编译驱动：

```bash
colcon build --packages-select orbbec_camera_msgs orbbec_camera
source install/setup.bash
```

## 常用检查

```bash
ros2 interface show orbbec_camera_msgs/msg/DeviceInfo
ros2 interface show orbbec_camera_msgs/msg/Extrinsics
ros2 interface show orbbec_camera_msgs/srv/GetDeviceInfo
```

## 维护说明

该包是接口包。修改 `.msg` 或 `.srv` 会影响 `orbbec_camera` 的编译和运行接口，也会要求所有依赖该接口的工作区重新编译。
