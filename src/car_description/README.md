# car_description

`car_description` 负责当前无人车的车体模型、TF 挂点、RViz 配置以及 Nav2 参数文件，是整车坐标系和可视化/导航参数的基础包。

这个包当前主要包含：
- `urdf/car.urdf.xacro`：整车结构模型
- `launch/display.launch.py`：单独显示车模与 RViz
- `rviz/car.rviz`：RViz 默认显示配置
- `rviz/nav2_params.yaml`：定位与导航参数

---

## 1. 车体模型当前包含什么

当前模型已经建立了无人车的第一版结构骨架，重点不是卷视觉细节，而是先把**结构关系、传感器挂点和导航所需几何信息**定准。

目前模型中已经包含：
- `base_footprint`
- `base_link`
- 四个轮子
- 四个轮侧下吊支架
- 上下车体板与立柱
- `imu_link`
- `laser_link`
- 雷达安装立柱
- `camera_link`
- `camera_depth_frame`
- `camera_depth_optical_frame`
- `camera_color_frame`
- `camera_color_optical_frame`

当前 `base_footprint -> base_link` 采用固定连接，`base_link` 作为车体主参考系，其余轮子、IMU、雷达等挂点都从这里继续展开。

推荐理解方式：

```text
odom -> base_footprint -> base_link -> {wheel links, imu_link, laser_link, camera_link}
```

其中：
- `odom -> base_footprint`：由 `car_driver` 发布
- `base_footprint -> base_link` 及其余固定结构：由 `robot_state_publisher + URDF` 发布

---

## 2. 关键几何参数

当前 `car.urdf.xacro` 中已经显式定义了车体与轮系关键尺寸，便于后续继续按实物修正。

### 车体主体
- `body_length = 0.280 m`
- `body_width = 0.150 m`
- `body_height = 0.049 m`

### 轮系参数
- `wheel_radius = 0.040 m`
- `wheel_width = 0.035 m`
- `wheel_track = 0.212 m`
- `wheel_base = 0.200 m`

### IMU 挂点
- `imu_link` 当前挂在 `base_link` 后部附近
- 参数：
  - `imu_x = -0.090`
  - `imu_y = 0.0`
  - `imu_z = 0.025`

### 雷达挂点
当前已在模型中定义 `laser_link`，并根据实装情况修正过安装方向与高度。

当前重要事实：
- 雷达安装在车体上方
- 实物前后反装时，已通过 yaw 修正方向
- 当前模型主要保证：
  - `laser_link` 能正确进入 TF 树
  - `/scan` 能与车体结构、地图、导航统一到同一套坐标关系里

### 相机挂点
当前已在模型中定义 Astra Pro 第一版挂点：

- `camera_link`
- `camera_depth_frame`
- `camera_depth_optical_frame`
- `camera_color_frame`
- `camera_color_optical_frame`

相机当前按车头安装建模，color/depth optical frame 遵循 REP-103 习惯：`z` 向前、`x` 向右、`y` 向下。这个挂点已经能支撑 `target_mapper_node` 从 `camera_color_optical_frame` 通过 TF 转到 `odom` / `map`，但外参仍应按实物继续校准。

---

## 3. 为什么这个包重要

这个包不只是“让 RViz 看起来好看一点”。它实际承担了三类基础角色：

### 3.1 TF 坐标骨架
后续所有模块都依赖它来统一坐标系，包括：
- 底盘 odom
- 激光雷达 `/scan`
- IMU 数据
- RGB-D 相机与目标落图
- Nav2 costmap

如果这个包里的结构关系乱了，后面定位、导航、感知全都会一起歪。

### 3.2 导航几何基础
`nav2_params.yaml` 中的很多配置都默认建立在当前车体几何基础上，例如：
- `robot_base_frame = base_footprint`
- footprint 尺寸
- costmap 的障碍膨胀范围
- AMCL 的 base frame 设定

### 3.3 后续传感器融合扩展入口
当前已经有 `imu_link`、`laser_link` 和相机 color/depth frame。后续如果继续调整相机安装，推荐仍在这个包里维护：
- `base_link -> camera_link`
- `camera_link -> camera_color_optical_frame`
- `camera_link -> camera_depth_optical_frame`
- 相机到车体的固定外参

这样视觉链、导航链、地图链才有机会真正接上，而不是各搞各的。

---

## 4. 当前 Nav2 参数文件说明

当前导航参数文件位于：

```text
src/car_description/rviz/nav2_params.yaml
```

虽然文件名放在 `rviz/` 目录下，但它实际上是**定位 + 导航通用参数文件**，不是单纯的 RViz 配置。

当前主要配置了这些模块：
- `amcl`
- `bt_navigator`
- `controller_server`
- `planner_server`
- `local_costmap`
- `global_costmap`
- 以及其他 Nav2 相关模块

### 关键配置点

#### AMCL 坐标与运动模型
- `base_frame_id = base_footprint`
- `global_frame_id = map`
- `odom_frame_id = odom`
- `scan_topic = scan`
- `robot_model_type = nav2_amcl::OmniMotionModel`

这里使用 `OmniMotionModel` 很关键，因为当前底盘是麦轮/全向底盘，不是普通差速车。

#### 局部控制器
当前局部控制器采用 `DWBLocalPlanner`，并已配置：
- `max_vel_x`
- `max_vel_y`
- `max_vel_theta`
- `acc_lim_x / y / theta`
- `decel_lim_x / y / theta`
- `vx_samples / vy_samples / vtheta_samples`
- 各类路径与目标评分 critic

这意味着当前导航链已经显式考虑了横移能力，而不是只按差速车在配。

#### 代价地图 footprint
当前 `local_costmap` 和 `global_costmap` 中都配置了 footprint：

```text
[[0.17, 0.11], [0.17, -0.11], [-0.17, -0.11], [-0.17, 0.11]]
```

这组 footprint 是导航避障和可通行性判断的重要基础，后续如果实车外廓变化明显，需要同步调整。

---

## 5. 常用使用方式

### 5.1 单独显示车模
只检查 URDF、TF 和 RViz 显示时，可直接启动：

```bash
cd /home/elf/car/car_ws
source /opt/ros/humble/setup.bash
source /home/elf/car/car_ws/install/setup.bash
ros2 launch car_description display.launch.py
```

### 5.2 作为整车 bringup 的模型来源
当前 `car_driver` 中多个 launch 都会自动引用本包的：
- `urdf/car.urdf.xacro`
- `rviz/car.rviz`
- `rviz/nav2_params.yaml`

例如：
- `full_bringup.launch.py`
- `localization_bringup.launch.py`
- `nav_bringup.launch.py`

也就是说，平时你多数时候不会单独启动 `car_description`，而是把它作为整车系统的一部分使用。

---

## 6. 后续建议

当前这个包已经够支撑底盘、雷达、定位、导航和第一版 RGB-D 目标落图联调，但相机外参仍需要继续按实物校准。

### 6.1 继续校准相机外参
不要只满足于“能连上 TF”。要把：
- 前后偏移
- 左右偏移
- 高度
- 俯仰/偏航

按实物测量后同步写进 URDF / TF 链。

### 6.2 继续校准 footprint 和传感器高度
如果后面车体外形或传感器安装位置继续变动，记得同步修：
- `footprint`
- `laser_link`
- `imu_link`
- `camera_link`
- color/depth optical frame

别让模型和实物越跑越分家，那会把后面的定位、导航、视觉全带沟里。
