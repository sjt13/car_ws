# orbbec_camera

`orbbec_camera` 是 Orbbec RGB-D 相机 ROS2 驱动包，当前仓库把它作为第三方驱动使用。

## 主要功能

该包编译 `orbbec_camera_node`，通过 Orbbec SDK 连接相机并发布图像、相机内参、点云、TF 和设备服务。当前车端常用入口不是直接运行本包 launch，而是由 `car_driver/orbbec_bringup.launch.py` 调用 `orbbec_camera_node` 的 depth-only 模式。

本项目当前对 Astra Pro 的常用链路为：

- depth：`orbbec_camera_node` 发布；
- RGB：`v4l2_camera` 打开 `/dev/video21` 发布；
- 目标落图：`car_yolo/target_mapper_node` 订阅 RGB、depth、CameraInfo 和 TF。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/` | Orbbec ROS2 驱动源码。 |
| `include/` | 驱动头文件。 |
| `SDK/` | Orbbec SDK 头文件和库。 |
| `launch/` | 官方多型号启动文件。 |
| `config/camera_params.yaml` | 官方相机参数示例。 |
| `scripts/99-obsensor-libusb.rules` | udev 规则文件。 |
| `tools/` | 附加工具节点源码。 |

## 节点和可执行文件

| 名称 | 来源 | 说明 |
| --- | --- | --- |
| `orbbec_camera_node` | `rclcpp_components_register_node` 注册的 `orbbec_camera::OBCameraNodeDriver` | 主相机驱动节点。 |
| `frame_latency` | `tools/frame_latency.cpp` | frame latency 工具节点。 |
| `list_devices_node` 等工具 | `CMakeLists.txt` 安装目标 | 设备和工作模式查询工具。 |

## 当前项目中的启动方式

推荐通过 `car_driver` 启动当前验证过的拆分链路：

```bash
ros2 launch car_driver orbbec_bringup.launch.py
```

该 launch 对 `orbbec_camera_node` 设置：

| 参数 | 当前默认值 | 说明 |
| --- | --- | --- |
| `product_id` | `0x0403` | Orbbec 设备 product id。 |
| `camera_name` | `camera` | 节点命名空间下的相机名。 |
| `enable_color` | `False` | 不启用 Orbbec SDK color stream。 |
| `enable_depth` | `True` | 启用 depth stream。 |
| `camera_depth_frame_id` | `camera_depth_frame` | depth frame。 |
| `depth_optical_frame_id` | `camera_depth_optical_frame` | depth optical frame。 |
| `depth_width` / `depth_height` | `320` / `240` | depth 分辨率。 |
| `depth_fps` | `30` | depth 帧率。 |
| `depth_format` | `Y12` | depth 格式。 |
| `enable_ir` | `False` | 不启用 IR。 |
| `enable_point_cloud` | `False` | 不发布点云。 |

如果需要验证 Orbbec 原生 RGB-D 对齐，可以使用：

```bash
ros2 launch car_driver orbbec_registered_bringup.launch.py
```

该入口会启用 `enable_color`、`enable_depth` 和 `depth_registration`。是否适合当前 Astra Pro，需要以现场驱动输出为准。

## 常见输出

在 `camera` namespace 下，驱动会按启用的 stream 发布图像和 CameraInfo。当前 `car_driver/orbbec_bringup.launch.py` depth-only 链路主要供下游使用：

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图。 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参。 |

RGB 话题在当前拆分链路中由 `v4l2_camera` 发布：

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 图像。 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB 内参。 |

## 服务

源码中 `ros_service.cpp` 按启用 stream 创建曝光、增益、自动曝光、设备信息、SDK 版本、保存图像/点云等服务。服务名称在节点 namespace 下生成，例如：

- `get_device_info`
- `get_sdk_version`
- `save_images`
- `save_point_cloud`
- `reboot_device`
- `get_<stream>_exposure`
- `set_<stream>_exposure`
- `set_<stream>_gain`

实际可用服务取决于相机型号、启用的 stream 和启动参数。

## 依赖

### ROS2 包依赖

`package.xml` 声明了 `rclcpp`、`sensor_msgs`、`std_msgs`、`std_srvs`、`image_transport`、`cv_bridge`、`camera_info_manager`、`tf2_ros`、`diagnostic_updater`、`orbbec_camera_msgs` 等依赖。

### 系统依赖

- Orbbec SDK 随包内 `SDK/` 使用。
- `libgflags-dev`
- `nlohmann-json-dev`
- `libgoogle-glog-dev`
- `libdw-dev`
- OpenGL 相关库
- 设备权限或 udev 规则

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_camera orbbec_camera_msgs
source install/setup.bash
```

## 常用检查

```bash
ros2 node list | grep camera
ros2 topic list | grep camera
ros2 topic echo /camera/depth/camera_info --once
ros2 service list | grep camera
```

检查 USB 设备：

```bash
lsusb
dmesg | tail -n 50
```

## 排查

如果 `orbbec_camera_node` 没有权限访问设备，先检查 udev 规则和当前用户权限。
如果 native color stream 启动失败，优先使用 `car_driver/orbbec_bringup.launch.py` 的拆分链路，让 RGB 由 `v4l2_camera` 发布。
如果下游目标落图没有深度，检查 `/camera/depth/image_raw` 和 `/camera/color/camera_info` 是否同时存在，并确认 TF 中有 `camera_color_optical_frame`。
