# orbbec_description

`orbbec_description` 是 Orbbec 相机的第三方模型描述包，包含多种 Orbbec 相机的 URDF/xacro、mesh、RViz 和模型查看 launch。

## 主要功能

该包用于单独显示 Orbbec 相机模型。当前无人车整车 TF 中使用的是 `car_description/urdf/car.urdf.xacro` 内定义的 Astra Pro 相机挂点；`car_driver` 的常用 bringup 没有直接 include 本包 launch。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `urdf/` | Astra2、Femto Bolt、Gemini2 等相机 xacro 和测试 xacro。 |
| `meshes/` | 相机模型 mesh。 |
| `launch/view_model.launch.py` | 查看 Orbbec 模型。 |
| `rviz/urdf.rviz` | 模型查看 RViz 配置。 |

## 启动入口

```bash
ros2 launch orbbec_description view_model.launch.py
```

具体加载哪个 xacro 由 `view_model.launch.py` 的参数和实现决定。若只是检查当前无人车上的相机 frame，应优先查看 `car_description`：

```bash
ros2 launch car_description display.launch.py
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 与本项目的关系

| 包 | 职责 |
| --- | --- |
| `orbbec_description` | Orbbec 官方/第三方相机模型资源。 |
| `car_description` | 当前无人车整车模型和实际使用的相机固定 frame。 |
| `orbbec_camera` | 相机驱动节点和图像/内参发布。 |
| `car_driver/orbbec_bringup.launch.py` | 当前 Astra Pro 拆分 RGB-D 输入启动入口。 |

## 依赖

`package.xml` 只声明了 `ament_cmake` 和测试依赖。实际查看模型通常还需要：

- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`
- `xacro`

这些运行依赖未在该包的 `package.xml` 中完整声明，使用时以当前 ROS2 环境为准。

## 编译

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_description
source install/setup.bash
```

## 排查

如果 RViz 中看不到相机模型，先检查：

```bash
ros2 pkg prefix orbbec_description
ros2 launch orbbec_description view_model.launch.py
```

如果问题发生在整车相机 TF 上，应检查 `car_description/urdf/car.urdf.xacro`，不是本包。
