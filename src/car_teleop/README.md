# car_teleop

`car_teleop` 将 ROS2 手柄话题 `/joy` 转换为底盘速度指令 `/cmd_vel`，用于人工遥控麦轮底盘。

## 主要功能

该包当前只有一个 Python 节点：`joy_to_cmdvel_node`。节点读取 `sensor_msgs/msg/Joy` 中的摇杆轴值，经过死区、速度倍率、一阶平滑和加减速限幅后发布 `geometry_msgs/msg/Twist`。

`car_driver/full_bringup.launch.py` 和 `car_driver/mapping_ekf_tf.launch.py` 可以按参数自动启动该节点。也可以单独运行它，用于检查手柄到 `/cmd_vel` 的链路。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `car_teleop/joy_to_cmdvel_node.py` | 手柄输入整形和 `/cmd_vel` 发布逻辑。 |
| `setup.py` | 注册 `joy_to_cmdvel_node` 控制台入口。 |
| `package.xml` | 声明 `rclpy`、`sensor_msgs`、`geometry_msgs` 等依赖。 |

## 节点

| 节点名称 | 入口 | 主要职责 | 启动方式 |
| --- | --- | --- | --- |
| `joy_to_cmdvel_node` | `car_teleop/joy_to_cmdvel_node.py` | 订阅 `/joy`，发布平滑后的 `/cmd_vel`。 | `ros2 run car_teleop joy_to_cmdvel_node` |

## 接口

### 订阅话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄原始轴值和按键输入。 |

### 发布话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 下发给 `car_driver/base_driver_node` 或其他底盘控制链的速度指令。 |

## 摇杆映射

当前源码中的轴映射如下：

| 手柄轴 | 输出字段 | 说明 |
| --- | --- | --- |
| `axes[1]` | `Twist.linear.x` | 左摇杆上下，正值为前进。 |
| `axes[0]` | `Twist.linear.y` | 左摇杆左右，按 ROS `base_link` 约定，正值为车体左侧。 |
| `axes[2]` | `Twist.angular.z` | 右摇杆左右，正负取决于手柄驱动返回值。 |

节点没有在参数中暴露轴号和方向符号。如果手柄方向需要调整，当前需要修改源码中的 `forward_sign`、`lateral_sign`、`yaw_sign`。

## 参数

| 参数 | 默认值 | 含义 | 单位 |
| --- | --- | --- | --- |
| `deadzone` | `0.1` | 摇杆死区，小于该绝对值的输入视为 0。 | 无 |
| `linear_scale` | `0.4` | 前后速度倍率。 | m/s |
| `lateral_scale` | `0.4` | 横移速度倍率。 | m/s |
| `angular_scale` | `1.0` | 旋转速度倍率。 | rad/s |
| `smoothing_alpha` | `0.25` | 一阶滤波系数，范围在源码中限制到 `[0, 1]`。 | 无 |
| `max_linear_accel` | `0.6` | 前后方向最大加速度。 | m/s^2 |
| `max_lateral_accel` | `0.6` | 横移方向最大加速度。 | m/s^2 |
| `max_angular_accel` | `1.5` | 旋转最大加速度。 | rad/s^2 |
| `max_linear_decel` | `1.2` | 前后方向最大减速度。 | m/s^2 |
| `max_lateral_decel` | `1.2` | 横移方向最大减速度。 | m/s^2 |
| `max_angular_decel` | `3.0` | 旋转最大减速度。 | rad/s^2 |
| `target_linear_epsilon` | `0.015` | 前后目标速度变化小于该值时不更新目标。 | m/s |
| `target_lateral_epsilon` | `0.015` | 横移目标速度变化小于该值时不更新目标。 | m/s |
| `target_angular_epsilon` | `0.03` | 旋转目标速度变化小于该值时不更新目标。 | rad/s |
| `publish_rate` | `30.0` | `/cmd_vel` 发布频率。 | Hz |

这些参数在 `joy_callback` 或定时发布函数中读取，部分参数支持运行时调整。

## 依赖

### ROS2 包依赖

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`

### 硬件依赖

- 一个能被 ROS `joy_node` 读取的手柄设备，例如 `/dev/input/js0`。

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select car_teleop
source install/setup.bash
```

## 常用指令

单独启动手柄驱动：

```bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

单独启动转换节点：

```bash
ros2 run car_teleop joy_to_cmdvel_node
```

调低速度倍率：

```bash
ros2 run car_teleop joy_to_cmdvel_node --ros-args \
  -p linear_scale:=0.25 \
  -p lateral_scale:=0.20 \
  -p angular_scale:=0.65
```

通过 `car_driver` 一起启动：

```bash
ros2 launch car_driver full_bringup.launch.py use_joy:=true joy_dev:=/dev/input/js0
```

## 排查

没有 `/joy`：

```bash
ros2 topic echo /joy
ls /dev/input/js*
```

有 `/joy` 但没有 `/cmd_vel`：

```bash
ros2 node list | grep joy_to_cmdvel
ros2 topic echo /cmd_vel
```

车不动但 `/cmd_vel` 正常：

检查 `car_driver/base_driver_node` 是否启动、底盘串口是否正确、`/cmd_vel` 是否被其他节点覆盖。
