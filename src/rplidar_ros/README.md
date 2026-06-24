# rplidar_ros

`rplidar_ros` 是 Slamtec RPLidar 的 ROS2 驱动包，当前仓库把它作为第三方雷达驱动使用。

## 主要功能

- 连接串口、TCP 或 UDP 形式的 RPLidar/SLLidar 设备。
- 通过 Slamtec SDK 读取扫描点。
- 发布 `sensor_msgs/msg/LaserScan`，默认话题为 `scan`。
- 提供 `start_motor` 和 `stop_motor` 服务。
- 提供多个型号的 launch 文件，以及带 RViz 的 `view_*` launch。

在当前无人车系统中，它主要作为 `/scan` 来源，被 `car_driver` 的定位、导航和 SLAM launch 使用。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `src/rplidar_node.cpp` | 主驱动节点，连接设备并发布 LaserScan。 |
| `src/rplidar_client.cpp` | 简单订阅 `scan` 并打印数据的测试客户端。 |
| `sdk/` | Slamtec SDK 源码。 |
| `launch/` | 各型号 RPLidar/SLLidar 启动文件。 |
| `rviz/rplidar_ros.rviz` | 雷达显示 RViz 配置。 |

## 节点

| 节点名称 | 可执行文件 | 主要职责 |
| --- | --- | --- |
| `rplidar_node` | `rplidar_node` | 连接雷达并发布 LaserScan。 |
| `rplidar_client` | `rplidar_client` | 订阅 `scan` 并打印角度/距离，用于测试。 |
| `rplidar_composition` | `rplidar_composition` | 使用同一驱动源码编译的组合版本入口。 |

## 接口

### 发布话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `scan` | `sensor_msgs/msg/LaserScan` | 雷达扫描数据。可通过 `topic_name` 参数改名。 |

### 服务

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `start_motor` | `std_srvs/srv/Empty` | 启动雷达电机。 |
| `stop_motor` | `std_srvs/srv/Empty` | 停止雷达电机。 |

如果 `auto_standby` 为 true，源码会忽略手动启停电机请求。

## 主要参数

`rplidar_node.cpp` 中声明的参数：

| 参数 | 源码默认值 | 含义 |
| --- | --- | --- |
| `channel_type` | `serial` | 连接方式，可为 serial/tcp/udp。 |
| `serial_port` | `/dev/ttyUSB0` | 串口设备。 |
| `serial_baudrate` | `1000000` | 串口波特率。 |
| `tcp_ip` | `192.168.0.7` | TCP 雷达 IP。 |
| `tcp_port` | `20108` | TCP 端口。 |
| `udp_ip` | `192.168.11.2` | UDP 雷达 IP。 |
| `udp_port` | `8089` | UDP 端口。 |
| `frame_id` | `laser_frame` | LaserScan frame。 |
| `inverted` | `false` | 是否反转扫描数据。 |
| `angle_compensate` | `false` | 是否做角度补偿。 |
| `flip_x_axis` | `false` | 是否翻转 X 轴。 |
| `auto_standby` | `false` | 是否自动待机。 |
| `topic_name` | `scan` | 发布话题名。 |
| `scan_mode` | 空 | 指定扫描模式；空值时使用典型扫描模式。 |
| `scan_frequency` | serial 默认 `10.0`，udp 默认 `20.0` | 扫描频率。 |

当前 `car_driver/full_bringup.launch.py` 和 `lidar_bringup.launch.py` 覆盖的常用值：

| 参数 | 当前车端默认值 |
| --- | --- |
| `channel_type` | `serial` |
| `serial_port` | `/dev/ttyUSB0` |
| `serial_baudrate` | `460800` |
| `frame_id` | `laser_link` |
| `inverted` | `false` |
| `angle_compensate` | `true` |
| `scan_mode` | `Standard` |

## Launch 文件

`launch/` 下按型号提供启动文件，例如：

- `rplidar_a1_launch.py`
- `rplidar_a2m7_launch.py`
- `rplidar_a2m8_launch.py`
- `rplidar_a2m12_launch.py`
- `rplidar_a3_launch.py`
- `rplidar_c1_launch.py`
- `rplidar_s1_launch.py`
- `rplidar_s2_launch.py`
- `rplidar_s2e_launch.py`
- `rplidar_s3_launch.py`
- `rplidar_t1_launch.py`

对应的 `view_*` 文件会额外启动 RViz。

当前无人车更常用 `car_driver` 的包装入口：

```bash
ros2 launch car_driver lidar_bringup.launch.py serial_port:=/dev/ttyUSB0
```

或在整车基础链中启动：

```bash
ros2 launch car_driver full_bringup.launch.py lidar_port:=/dev/ttyUSB0
```

## 依赖

### ROS2 包依赖

- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `std_srvs`
- `rclcpp_components`

### 硬件依赖

- Slamtec RPLidar/SLLidar 设备。
- 串口或网络连接。
- Linux 下可访问的设备节点，例如 `/dev/ttyUSB0`。

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rplidar_ros
source install/setup.bash
```

## 常用指令

单独启动当前车端常用链路：

```bash
ros2 launch car_driver lidar_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=460800 \
  frame_id:=laser_link
```

直接启动某型号 launch：

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

查看扫描：

```bash
ros2 topic echo /scan --once
```

停止/启动电机：

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
ros2 service call /start_motor std_srvs/srv/Empty {}
```

## 排查

没有 `/scan`：

```bash
ros2 node list | grep rplidar
ros2 topic list | grep scan
ls /dev/ttyUSB*
dmesg | tail -n 50
```

`frame_id` 不对：

检查启动参数是否使用了当前车体模型中的 `laser_link`。Nav2、AMCL 和 SLAM 参数文件默认围绕 `/scan` 和车体 TF 使用，frame 不一致会影响定位和建图。

串口连接失败：

确认设备节点、权限和波特率。当前车端 bringup 使用 `460800`，而驱动源码默认 `1000000`，不要只按源码默认值排查。
