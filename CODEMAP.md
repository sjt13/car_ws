# CODEMAP.md

`car_ws` 当前工程结构总览。

目标：让你先从“整套系统怎么拼起来”看懂，再回头读单文件，不至于一上来扎进源码泥潭。

---

## 1. 当前工程里有哪些包

### 1) `car_description`
**定位：** 机器人模型与 RViz 显示配置包。

主要内容：
- `urdf/car.urdf.xacro`
  - 车体结构、轮子、IMU、激光雷达等坐标系定义。
- `rviz/car.rviz`
  - RViz 默认显示配置。
- `rviz/nav2_params.yaml`
  - 当前导航/定位参数文件。
- `launch/display.launch.py`
  - 单独看模型、TF、RViz 的轻量入口。

**你读它的目的：**
- 弄清楚 `base_footprint / base_link / imu_link / laser_link` 怎么挂；
- 确认模型结构和 TF 命名是否一致；
- 确认 RViz 默认显示什么。

---

### 2) `car_driver`
**定位：** 底盘驱动与主启动入口包。

主要内容：
- `src/base_driver_node.cpp`
  - 核心底盘驱动节点。
  - 负责 `/cmd_vel -> 串口协议 -> STM32`。
  - 同时处理 STM32 上行 `$TEL` 遥测，发布：
    - `/imu/data_raw`
    - `/wheel_ticks`
    - `/odom`
    - `odom -> base_footprint` TF
- `include/car_driver/base_driver_node.hpp`
  - 上面这个节点的类声明。
- `launch/bringup.launch.py`
  - 最小底盘 bringup。
- `launch/lidar_bringup.launch.py`
  - 只起雷达与模型。
- `launch/full_bringup.launch.py`
  - 底盘 + 雷达 + 可选手柄 + 可选 RViz。
- `launch/localization_bringup.launch.py`
  - 底盘 + 雷达 + AMCL 定位。
- `launch/nav_bringup.launch.py`
  - 底盘 + 雷达 + AMCL + Nav2 完整导航。

**你读它的目的：**
- 看清车的“控制主链”到底怎么通；
- 搞清不同 launch 各自负责哪一层；
- 排查导航不准时，先知道 odom 是怎么来的。

---

### 3) `car_teleop`
**定位：** 手柄遥操作包。

主要内容：
- `car_teleop/joy_to_cmdvel_node.py`
  - 把 `/joy` 转成 `/cmd_vel`。
  - 做死区、方向修正、平滑滤波、加减速限幅。
- `setup.py`
  - ROS2 Python 包安装入口。

**你读它的目的：**
- 看手柄如何控制底盘；
- 调试“为什么摇杆打出去车的响应不对”；
- 调整手感参数。

---

### 4) `car_yolo`
**定位：** 车端视觉检测包。

主要内容：
- `car_yolo/yolo_detector_node.py`
  - 直接读本机相机；
  - 跑 RKNN YOLO；
  - 发布：
    - `/yolo/detections`
    - `/yolo/image_annotated`
    - `/yolo/debug_fps`
- `launch/yolo_detector.launch.py`
  - 视觉检测启动入口。

**你读它的目的：**
- 看车端视觉当前到底输出到哪一步；
- 明确它现在只有 2D 检测，没有地图落点；
- 后面做目标投影/定位时知道要接哪里。

---

### 5) `rplidar_ros`
**定位：** RPLidar 的 ROS2 包装层。

主要内容：
- `src/rplidar_node.cpp`
  - 连接雷达、取点、转 LaserScan、发布 `/scan`。
- `src/rplidar_client.cpp`
  - 简单订阅示例。
- `launch/*.py`
  - 各型号 RPLidar 默认启动参数。
- `sdk/`
  - 上游 vendor SDK。

**你读它的目的：**
- 弄清 `/scan` 是怎么来的；
- 调串口雷达时知道参数入口在哪；
- **不用优先读 `sdk/`**，除非你真要魔改底层协议。

---

## 2. 当前系统分层

可以把整个车端理解成 5 层：

### A. 硬件层
- STM32 底盘控制板
- RPLidar 雷达
- 车端摄像头
- RK3588/ELF2 主机

### B. 驱动桥接层
- `car_driver/base_driver_node`
- `rplidar_ros/rplidar_node`
- `car_yolo/yolo_detector_node`

这层干的事很朴素：
- 把硬件数据变成 ROS 话题；
- 把 ROS 指令变成硬件协议。

### C. 基础状态层
- TF：`odom -> base_footprint -> base_link -> laser_link / imu_link`
- `/odom`
- `/scan`
- `/imu/data_raw`
- `/wheel_ticks`

### D. 定位与导航层
- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `velocity_smoother`

### E. 人机交互层
- RViz
- 手柄 `/joy`
- 可视化路径 `/plan`、`/local_plan`

---

## 3. 当前最重要的话题流

### 3.1 手柄控制链
`/joy`
→ `joy_to_cmdvel_node`
→ `/cmd_vel`
→ `base_driver_node`
→ 串口 `$CAR:...!`
→ STM32
→ 电机执行

---

### 3.2 导航控制链
RViz `Nav2 Goal`
→ `/goal_pose`
→ `bt_navigator`
→ `planner_server`
→ `/plan`
→ `controller_server`
→ `/cmd_vel`
→ `base_driver_node`
→ STM32
→ 小车运动

---

### 3.3 雷达定位链
RPLidar 硬件
→ `rplidar_node`
→ `/scan`
→ `amcl`
→ `map -> odom`
→ Nav2 使用定位结果

---

### 3.4 底盘状态回传链
STM32 `$TEL`
→ `base_driver_node`
→ `/wheel_ticks`
→ `/imu/data_raw`
→ `/odom`
→ `odom -> base_footprint`

---

### 3.5 视觉检测链
相机 `/dev/videoX`
→ `yolo_detector_node`
→ RKNN 推理
→ `/yolo/detections`
→ `/yolo/image_annotated`
→ `/yolo/debug_fps`

**注意：**
当前没有直接产出目标在 `map` / `odom` 下的位置。

---

## 4. 几个常用 launch 各自负责什么

### `display.launch.py`
只看模型 / TF / RViz。

### `bringup.launch.py`
最小底盘启动。

### `lidar_bringup.launch.py`
只看雷达数据是否正常。

### `full_bringup.launch.py`
底盘 + 雷达 + 可选手柄 + RViz。

### `localization_bringup.launch.py`
底盘 + 雷达 + 地图 + AMCL。

### `nav_bringup.launch.py`
底盘 + 雷达 + 定位 + Nav2。

**当前推荐关系：**
- 纯底层联调：`full_bringup.launch.py`
- 做定位：`localization_bringup.launch.py`
- 做完整导航：`nav_bringup.launch.py`

---

## 5. 当前导航结构的关键判断

### 现在的导航入口为什么比旧版稳
因为当前 `nav_bringup.launch.py` 不是手搓一堆 Nav2 节点顺序，而是：
- 本地只负责硬件层：底盘、雷达、robot_state_publisher
- 定位直接 include 官方 `localization_launch.py`
- 导航直接 include 官方 `navigation_launch.py`

这样 lifecycle、autostart、依赖关系都交给官方 bringup 管，比自己拿 `TimerAction` 硬拼靠谱得多。

---

## 6. 你现在读代码的推荐顺序

### 第一轮：先看入口，不钻实现
1. `CODEMAP.md`
2. `car_driver/launch/full_bringup.launch.py`
3. `car_driver/launch/localization_bringup.launch.py`
4. `car_driver/launch/nav_bringup.launch.py`
5. `car_description/rviz/nav2_params.yaml`

### 第二轮：看核心桥接节点
6. `car_driver/include/car_driver/base_driver_node.hpp`
7. `car_driver/src/base_driver_node.cpp`
8. `rplidar_ros/src/rplidar_node.cpp`
9. `car_teleop/car_teleop/joy_to_cmdvel_node.py`
10. `car_yolo/car_yolo/yolo_detector_node.py`

### 第三轮：按问题导向回头补
- 底盘不动：回看 `base_driver_node`
- 雷达没图：回看 `rplidar_node`
- AMCL 飘：回看 `/odom + /scan + map`
- 导航不准：先看 odom，再看 map/AMCL，再看 Nav2 参数
- 视觉只会框不会定位：回看 `yolo_detector_node` 和相机外参问题

---

## 7. 当前最该盯的技术瓶颈

### 1) 导航精度主要受这几项限制
- `odom` 精度
  - 麦轮打滑
  - `odom_yaw_scale`
  - 横移/旋转误差
- 地图质量
  - 墙体是否拉直
  - 转角是否清晰
- AMCL 质量
  - 静止时是否抖
  - 走动时是否稳定贴图
- Nav2 参数
  - 这是后调项，不是第一嫌疑人

### 2) 视觉当前能力边界
- 已有：图像内目标类别、框、分数
- 没有：目标在地图中的 2D/3D 位置

要做地图落点，还缺：
- 相机外参
- 距离来源
- 时间同步
- TF 对齐

---

## 8. 一句话记忆版

这套 `car_ws` 当前可以粗暴记成：

- `car_driver`：车怎么动、里程计怎么回
- `rplidar_ros`：雷达怎么变成 `/scan`
- `car_description`：车长什么样、TF 怎么挂、RViz 看什么
- `car_teleop`：手柄怎么变 `/cmd_vel`
- `car_yolo`：相机怎么变检测框
- `localization_bringup`：让车知道自己在哪
- `nav_bringup`：让车自己走到目标点
