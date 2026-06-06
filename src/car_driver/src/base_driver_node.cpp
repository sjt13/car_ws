// 这个文件实现车端底盘驱动节点 `base_driver_node`。
//
// 它的核心职责有四件事：
// 1. 把 ROS2 的 `/cmd_vel` 速度指令转换成 STM32 已有串口协议帧；
// 2. 通过 UART 下发到底盘控制板，形成“上位机 -> 底盘”的控制链；
// 3. 接收 STM32 上行 `$TEL` / `$DBG` 遥测，解析出底盘速度、轮速、IMU 和调试量；
// 4. 发布 `/imu/data_raw`、`/wheel_ticks`、`/stm32_debug`、`/odom` 以及 `odom -> base_footprint` TF。
//
// 你可以把它理解成：ROS2 世界和底盘 MCU 世界之间的“协议翻译 + 数据桥接节点”。
#include "car_driver/base_driver_node.hpp"

// 算法工具：用于限幅时的 std::min/std::max。
#include <algorithm>
// 固定长度数组：用于临时解析缓存。
#include <array>
// 错误码：用于判断串口读写失败原因（如 EAGAIN/EINTR）。
#include <cerrno>
// 时间工具：用于定时周期与超时计算。
#include <chrono>
// 数学函数：用于 std::lround 进行四舍五入。
#include <cmath>
// 固定宽度整数：用于 int16_t 等类型。
#include <cstdint>
#include <utility>
// C 数值转换：用于 std::strtol 解析整数字段。
#include <cstdlib>
// C 字符串工具：用于 strerror 将 errno 转可读文本。
#include <cstring>
// POSIX 文件控制：用于 open 打开串口设备节点。
#include <fcntl.h>
// 数值边界：用于 int16_t 上下限裁剪。
#include <limits>
// std::string 字符串处理。
#include <string>

// POSIX 类型定义。
#include <sys/types.h>
// termios 串口配置接口。
#include <termios.h>
// POSIX 读写与关闭接口（read/write/close）。
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace car_driver
{

namespace
{

// STM32 协议里速度使用“真实值 * 1000”后的整数（见 $CAR:x,y,z!）。
constexpr double kCmdScale = 1000.0;
constexpr double kStandardGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295;

// 对称限幅：把 value 裁剪到 [-limit, limit] 区间内。
double clampSymmetric(double value, double limit)
{
  return std::max(-limit, std::min(value, limit));
}

speed_t toTermiosBaud(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

}  // namespace

BaseDriverNode::BaseDriverNode()
: Node("base_driver_node")
{
  declareAndLoadParameters();
  RCLCPP_INFO(get_logger(), "base_driver_node 启动：订阅 /cmd_vel，串口下行协议复用 STM32 的 $CAR:x,y,z!。\n上行同时解析 $TEL 与 $DBG。");
  RCLCPP_INFO(
    get_logger(),
    "当前串口参数：port=%s baudrate=%d timeout=%.3fs rate=%.1fHz tf_rate=%.1fHz",
    port_.c_str(), baudrate_, cmd_timeout_sec_, publish_rate_hz_, tf_publish_rate_hz_);
  if (port_ == "/dev/ttyFIQ0") {
    RCLCPP_WARN(
      get_logger(),
      "当前端口是 /dev/ttyFIQ0（调试串口）。底盘控制建议使用 UART9 对应 /dev/ttyS9。");
  }
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10), std::bind(&BaseDriverNode::cmdVelCallback, this, std::placeholders::_1));
  imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
  wheel_ticks_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_ticks", rclcpp::QoS(10));
  stm32_debug_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/stm32_debug", rclcpp::QoS(10));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, std::bind(&BaseDriverNode::timerCallback, this));
  const auto tf_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / tf_publish_rate_hz_));
  tf_timer_ = create_wall_timer(tf_period, std::bind(&BaseDriverNode::tfTimerCallback, this));
  (void)ensureSerialConnected();
}

BaseDriverNode::~BaseDriverNode()
{
  if (serial_fd_ >= 0) {
    VelocityCommand zero_cmd {};
    (void)sendVelocityCommand(zero_cmd, "node shutdown");
  }
  closeSerialPort();
}

void BaseDriverNode::declareAndLoadParameters()
{
  declare_parameter<std::string>("port", "/dev/ttyS9");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<double>("cmd_timeout", 0.5);
  declare_parameter<double>("publish_rate", 30.0);
  declare_parameter<double>("tf_publish_rate", 20.0);
  declare_parameter<bool>("publish_odom_tf", true);
  declare_parameter<double>("odom_linear_deadband", 0.05);
  declare_parameter<double>("odom_angular_deadband", 0.10);
  declare_parameter<double>("odom_yaw_scale", 0.91);
  declare_parameter<double>("max_vx", 1.5);
  declare_parameter<double>("max_vy", 1.2);
  declare_parameter<double>("max_wz", 6.28);
  declare_parameter<double>("reconnect_interval", 1.0);
  declare_parameter<std::string>("imu_frame_id", "imu_link");
  declare_parameter<std::string>("odom_frame_id", "odom");
  declare_parameter<std::string>("base_frame_id", "base_footprint");
  declare_parameter<double>("imu_accel_lsb_per_g", 16384.0);
  declare_parameter<double>("imu_gyro_lsb_per_dps", 16.4);
  declare_parameter<double>("imu_accel_bias_x_raw", 0.0);
  declare_parameter<double>("imu_accel_bias_y_raw", 0.0);
  declare_parameter<double>("imu_accel_bias_z_raw", 0.0);
  declare_parameter<double>("imu_gyro_bias_x_raw", 0.0);
  declare_parameter<double>("imu_gyro_bias_y_raw", 0.0);
  declare_parameter<double>("imu_gyro_bias_z_raw", 0.0);
  declare_parameter<double>("imu_angular_velocity_covariance", 0.0025);
  declare_parameter<double>("imu_linear_acceleration_covariance", 0.25);
  declare_parameter<bool>("imu_auto_gyro_bias", true);
  declare_parameter<int>("imu_auto_gyro_bias_samples", 120);
  declare_parameter<double>("imu_auto_gyro_bias_max_motion_raw", 20.0);

  port_ = get_parameter("port").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  cmd_timeout_sec_ = get_parameter("cmd_timeout").as_double();
  publish_rate_hz_ = get_parameter("publish_rate").as_double();
  tf_publish_rate_hz_ = get_parameter("tf_publish_rate").as_double();
  publish_odom_tf_ = get_parameter("publish_odom_tf").as_bool();
  odom_linear_deadband_mps_ = get_parameter("odom_linear_deadband").as_double();
  odom_angular_deadband_radps_ = get_parameter("odom_angular_deadband").as_double();
  odom_yaw_scale_ = get_parameter("odom_yaw_scale").as_double();
  max_vx_mps_ = get_parameter("max_vx").as_double();
  max_vy_mps_ = get_parameter("max_vy").as_double();
  max_wz_radps_ = get_parameter("max_wz").as_double();
  reconnect_interval_sec_ = get_parameter("reconnect_interval").as_double();
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();
  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  imu_accel_lsb_per_g_ = get_parameter("imu_accel_lsb_per_g").as_double();
  imu_gyro_lsb_per_dps_ = get_parameter("imu_gyro_lsb_per_dps").as_double();
  imu_accel_bias_x_raw_ = get_parameter("imu_accel_bias_x_raw").as_double();
  imu_accel_bias_y_raw_ = get_parameter("imu_accel_bias_y_raw").as_double();
  imu_accel_bias_z_raw_ = get_parameter("imu_accel_bias_z_raw").as_double();
  imu_gyro_bias_x_raw_ = get_parameter("imu_gyro_bias_x_raw").as_double();
  imu_gyro_bias_y_raw_ = get_parameter("imu_gyro_bias_y_raw").as_double();
  imu_gyro_bias_z_raw_ = get_parameter("imu_gyro_bias_z_raw").as_double();
  imu_angular_velocity_covariance_ = get_parameter("imu_angular_velocity_covariance").as_double();
  imu_linear_acceleration_covariance_ = get_parameter("imu_linear_acceleration_covariance").as_double();
  imu_auto_gyro_bias_enabled_ = get_parameter("imu_auto_gyro_bias").as_bool();
  imu_auto_gyro_bias_samples_ = get_parameter("imu_auto_gyro_bias_samples").as_int();
  imu_auto_gyro_bias_max_motion_raw_ = get_parameter("imu_auto_gyro_bias_max_motion_raw").as_double();

  if (publish_rate_hz_ <= 0.0) { publish_rate_hz_ = 30.0; }
  if (tf_publish_rate_hz_ <= 0.0) { tf_publish_rate_hz_ = 20.0; }
  if (odom_linear_deadband_mps_ < 0.0) { odom_linear_deadband_mps_ = 0.05; }
  if (odom_angular_deadband_radps_ < 0.0) { odom_angular_deadband_radps_ = 0.10; }
  if (odom_yaw_scale_ <= 0.0) { odom_yaw_scale_ = 0.53; }
  if (cmd_timeout_sec_ <= 0.0) { cmd_timeout_sec_ = 0.5; }
  if (reconnect_interval_sec_ < 0.1) { reconnect_interval_sec_ = 0.1; }
  if (imu_frame_id_.empty()) { imu_frame_id_ = "imu_link"; }
  if (odom_frame_id_.empty()) { odom_frame_id_ = "odom"; }
  if (base_frame_id_.empty()) { base_frame_id_ = "base_footprint"; }
  if (imu_accel_lsb_per_g_ <= 0.0) { imu_accel_lsb_per_g_ = 16384.0; }
  if (imu_gyro_lsb_per_dps_ <= 0.0) { imu_gyro_lsb_per_dps_ = 16.4; }
  if (imu_angular_velocity_covariance_ < 0.0) { imu_angular_velocity_covariance_ = 0.0025; }
  if (imu_linear_acceleration_covariance_ < 0.0) { imu_linear_acceleration_covariance_ = 0.25; }
  if (imu_auto_gyro_bias_samples_ <= 0) { imu_auto_gyro_bias_samples_ = 120; }
  if (imu_auto_gyro_bias_max_motion_raw_ < 0.0) { imu_auto_gyro_bias_max_motion_raw_ = 20.0; }
}

void BaseDriverNode::tfTimerCallback()
{
  if (!publish_odom_tf_) {
    return;
  }
  if (!odom_initialized_) {
    return;
  }
  publishOdomTf(now());
}

void BaseDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto clamped = clampTwist(*msg);
  if (
    std::abs(clamped.vx_mps - msg->linear.x) > 1e-6 ||
    std::abs(clamped.vy_mps - msg->linear.y) > 1e-6 ||
    std::abs(clamped.wz_radps - msg->angular.z) > 1e-6)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "检测到 /cmd_vel 超限，已按 max_vx/max_vy/max_wz 裁剪。");
  }
  latest_cmd_ = clamped;
  has_received_cmd_ = true;
  timeout_mode_ = false;
  last_cmd_time_ = now();
}

void BaseDriverNode::timerCallback()
{
  if (!ensureSerialConnected()) {
    return;
  }
  pollSerialReceive();
  if (!has_received_cmd_) {
    return;
  }
  VelocityCommand cmd_to_send = latest_cmd_;
  const auto elapsed = (now() - last_cmd_time_).seconds();
  if (elapsed > cmd_timeout_sec_) {
    if (!timeout_mode_) {
      RCLCPP_WARN(get_logger(), "/cmd_vel 超时 %.3fs（阈值 %.3fs），自动下发零速度停车。", elapsed, cmd_timeout_sec_);
    }
    timeout_mode_ = true;
    cmd_to_send = VelocityCommand {};
  }
  (void)sendVelocityCommand(cmd_to_send, timeout_mode_ ? "timeout auto stop" : "cmd_vel");
}

BaseDriverNode::VelocityCommand BaseDriverNode::clampTwist(const geometry_msgs::msg::Twist & msg) const
{
  VelocityCommand cmd {};
  cmd.vx_mps = clampSymmetric(msg.linear.x, max_vx_mps_);
  cmd.vy_mps = clampSymmetric(msg.linear.y, max_vy_mps_);
  cmd.wz_radps = clampSymmetric(msg.angular.z, max_wz_radps_);
  return cmd;
}

double BaseDriverNode::applyOdomDeadband(double value, double deadband) const
{
  return std::abs(value) < deadband ? 0.0 : value;
}

std::string BaseDriverNode::encodeVelocityFrame(const VelocityCommand & cmd) const
{
  const auto to_i16 = [](double value) {
    long raw = std::lround(value * kCmdScale);
    raw = std::max(raw, static_cast<long>(std::numeric_limits<int16_t>::min()));
    raw = std::min(raw, static_cast<long>(std::numeric_limits<int16_t>::max()));
    return static_cast<int>(raw);
  };
  return "$CAR:" + std::to_string(to_i16(cmd.vx_mps)) + "," + std::to_string(to_i16(cmd.vy_mps)) + "," + std::to_string(to_i16(cmd.wz_radps)) + "!";
}

bool BaseDriverNode::ensureSerialConnected()
{
  if (serial_fd_ >= 0) {
    return true;
  }
  const auto now_time = now();
  if (
    last_reconnect_try_time_.nanoseconds() != 0 &&
    (now_time - last_reconnect_try_time_).seconds() < reconnect_interval_sec_)
  {
    return false;
  }
  last_reconnect_try_time_ = now_time;
  return openSerialPort();
}

bool BaseDriverNode::openSerialPort()
{
  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "串口打开失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }
  if (!configureSerialPort(fd)) {
    ::close(fd);
    return false;
  }
  serial_fd_ = fd;
  rx_buffer_.clear();
  RCLCPP_INFO(get_logger(), "串口已连接：%s @ %d", port_.c_str(), baudrate_);
  return true;
}

void BaseDriverNode::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool BaseDriverNode::configureSerialPort(int fd) const
{
  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "读取串口属性失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }
  const speed_t speed = toTermiosBaud(baudrate_);
  if (speed == 0) {
    RCLCPP_ERROR(get_logger(), "不支持的波特率：%d", baudrate_);
    return false;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "设置串口属性失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

bool BaseDriverNode::writeFrame(const std::string & frame)
{
  if (serial_fd_ < 0) {
    return false;
  }
  const char * data = frame.data();
  std::size_t remaining = frame.size();
  while (remaining > 0) {
    const ssize_t n = ::write(serial_fd_, data, remaining);
    if (n > 0) { data += n; remaining -= static_cast<std::size_t>(n); continue; }
    if (n < 0 && errno == EINTR) { continue; }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { continue; }
    RCLCPP_ERROR(get_logger(), "串口写入失败：%s (%s)", port_.c_str(), std::strerror(errno));
    closeSerialPort();
    return false;
  }
  return true;
}

bool BaseDriverNode::sendVelocityCommand(const VelocityCommand & cmd, const char * reason)
{
  const std::string frame = encodeVelocityFrame(cmd);
  if (!writeFrame(frame)) {
    return false;
  }
  if (timeout_mode_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "超时保护发送：%s", frame.c_str());
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500, "串口发送(%s)：%s", reason, frame.c_str());
  }
  return true;
}

void BaseDriverNode::pollSerialReceive()
{
  if (serial_fd_ < 0) {
    return;
  }
  uint8_t buffer[256];
  while (true) {
    const ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
    if (n > 0) {
      rx_buffer_.append(reinterpret_cast<const char *>(buffer), static_cast<std::size_t>(n));
      continue;
    }
    if (n == 0) {
      break;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    RCLCPP_ERROR(get_logger(), "串口读取失败：%s (%s)", port_.c_str(), std::strerror(errno));
    closeSerialPort();
    return;
  }
  processRxBuffer();
}

void BaseDriverNode::processRxBuffer()
{
  while (!rx_buffer_.empty()) {
    const std::size_t tel_start = rx_buffer_.find("$TEL:");
    const std::size_t dbg_start = rx_buffer_.find("$DBG:");
    std::size_t frame_start = std::string::npos;
    if (tel_start == std::string::npos) {
      frame_start = dbg_start;
    } else if (dbg_start == std::string::npos) {
      frame_start = tel_start;
    } else {
      frame_start = std::min(tel_start, dbg_start);
    }
    if (frame_start == std::string::npos) {
      if (rx_buffer_.size() > 16) {
        rx_buffer_.erase(0, rx_buffer_.size() - 16);
      }
      break;
    }
    if (frame_start > 0) {
      rx_buffer_.erase(0, frame_start);
    }
    const std::size_t end_pos = rx_buffer_.find('!');
    if (end_pos == std::string::npos) {
      break;
    }
    std::string frame = rx_buffer_.substr(0, end_pos + 1);
    rx_buffer_.erase(0, end_pos + 1);
    const std::size_t duplicated_tel = frame.rfind("$TEL:");
    const std::size_t duplicated_dbg = frame.rfind("$DBG:");
    const std::size_t duplicated = std::max(duplicated_tel, duplicated_dbg);
    if (duplicated != std::string::npos && duplicated > 0) {
      frame.erase(0, duplicated);
    }
    processReceivedFrame(frame);
  }
  if (rx_buffer_.size() > 1024) {
    rx_buffer_.erase(0, rx_buffer_.size() - 1024);
  }
}

void BaseDriverNode::processReceivedFrame(const std::string & frame)
{
  if (frame.rfind("$TEL:", 0) == 0) {
    if (frame.size() < 7 || frame.back() != '!') {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到 $TEL 帧但包尾不合法（缺少 !）：%s", frame.c_str());
      return;
    }
    TelTelemetryFrame telemetry {};
    if (!parseTelTelemetryFrame(frame, telemetry)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "解析 $TEL 帧失败：%s", frame.c_str());
      return;
    }
    const auto stamp = now();
    updateImuGyroAutoBias(telemetry.imu, telemetry.enc);
    publishImuRaw(telemetry.imu, stamp);
    publishWheelTicks(telemetry.enc);
    updateOdom(telemetry.enc, telemetry.imu, stamp);
    return;
  }
  if (frame.rfind("$DBG:", 0) == 0) {
    if (frame.size() < 7 || frame.back() != '!') {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到 $DBG 帧但包尾不合法（缺少 !）：%s", frame.c_str());
      return;
    }
    DebugTelemetry telemetry {};
    if (!parseDbgTelemetryFrame(frame, telemetry)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "解析 $DBG 帧失败：%s", frame.c_str());
      return;
    }
    publishDebugTelemetry(telemetry);
    return;
  }
  RCLCPP_DEBUG(get_logger(), "收到串口上行帧：%s", frame.c_str());
}

bool BaseDriverNode::parseTelTelemetryFrame(const std::string & frame, TelTelemetryFrame & out) const
{
  if (frame.rfind("$TEL:", 0) != 0 || frame.size() < 7 || frame.back() != '!') {
    return false;
  }
  const std::string payload = frame.substr(5, frame.size() - 6);
  std::array<int32_t, 13> values {};
  if (!parseTelIntegerFields(payload, values)) {
    return false;
  }
  out.enc.ix = values[0]; out.enc.iy = values[1]; out.enc.iw = values[2];
  out.enc.wa = values[3]; out.enc.wb = values[4]; out.enc.wc = values[5]; out.enc.wd = values[6];
  out.imu.ax = values[7]; out.imu.ay = values[8]; out.imu.az = values[9];
  out.imu.gx = values[10]; out.imu.gy = values[11]; out.imu.gz = values[12];
  return true;
}

bool BaseDriverNode::parseDbgTelemetryFrame(const std::string & frame, DebugTelemetry & out) const
{
  if (frame.rfind("$DBG:", 0) != 0 || frame.size() < 7 || frame.back() != '!') {
    return false;
  }
  const std::string payload = frame.substr(5, frame.size() - 6);
  std::array<int32_t, 12> values {};
  if (!parseDbgIntegerFields(payload, values)) {
    return false;
  }
  out.tgx = values[0]; out.tgy = values[1]; out.tgw = values[2];
  out.rta = values[3]; out.rtb = values[4]; out.rtc = values[5]; out.rtd = values[6];
  out.pwma = values[7]; out.pwmb = values[8]; out.pwmc = values[9]; out.pwmd = values[10];
  out.dt = values[11];
  return true;
}

bool BaseDriverNode::parseTelIntegerFields(const std::string & payload, std::array<int32_t, 13> & values) const
{
  std::size_t start = 0;
  std::size_t index = 0;
  while (start <= payload.size()) {
    const std::size_t comma_pos = payload.find(',', start);
    const std::string token = (comma_pos == std::string::npos) ? payload.substr(start) : payload.substr(start, comma_pos - start);
    if (token.empty()) {
      return false;
    }
    errno = 0;
    char * end_ptr = nullptr;
    const long parsed = std::strtol(token.c_str(), &end_ptr, 10);
    if (
      end_ptr == token.c_str() || *end_ptr != '\0' || errno == ERANGE ||
      parsed < static_cast<long>(std::numeric_limits<int32_t>::min()) ||
      parsed > static_cast<long>(std::numeric_limits<int32_t>::max()))
    {
      return false;
    }
    if (index >= values.size()) {
      return false;
    }
    values[index++] = static_cast<int32_t>(parsed);
    if (comma_pos == std::string::npos) {
      break;
    }
    start = comma_pos + 1;
  }
  return index == values.size();
}

bool BaseDriverNode::parseDbgIntegerFields(const std::string & payload, std::array<int32_t, 12> & values) const
{
  std::size_t start = 0;
  std::size_t index = 0;
  while (start <= payload.size()) {
    const std::size_t comma_pos = payload.find(',', start);
    const std::string token = (comma_pos == std::string::npos) ? payload.substr(start) : payload.substr(start, comma_pos - start);
    if (token.empty()) {
      return false;
    }
    errno = 0;
    char * end_ptr = nullptr;
    const long parsed = std::strtol(token.c_str(), &end_ptr, 10);
    if (
      end_ptr == token.c_str() || *end_ptr != '\0' || errno == ERANGE ||
      parsed < static_cast<long>(std::numeric_limits<int32_t>::min()) ||
      parsed > static_cast<long>(std::numeric_limits<int32_t>::max()))
    {
      return false;
    }
    if (index >= values.size()) {
      return false;
    }
    values[index++] = static_cast<int32_t>(parsed);
    if (comma_pos == std::string::npos) {
      break;
    }
    start = comma_pos + 1;
  }
  return index == values.size();
}

void BaseDriverNode::updateImuGyroAutoBias(const ImuTelemetry & imu, const EncoderTelemetry & enc)
{
  if (!imu_auto_gyro_bias_enabled_ || imu_auto_gyro_bias_ready_) {
    return;
  }
  const bool stationary =
    std::abs(static_cast<double>(enc.ix)) <= imu_auto_gyro_bias_max_motion_raw_ &&
    std::abs(static_cast<double>(enc.iy)) <= imu_auto_gyro_bias_max_motion_raw_ &&
    std::abs(static_cast<double>(enc.iw)) <= imu_auto_gyro_bias_max_motion_raw_;
  if (!stationary) {
    imu_auto_gyro_bias_count_ = 0;
    imu_auto_gyro_bias_sum_x_raw_ = 0.0;
    imu_auto_gyro_bias_sum_y_raw_ = 0.0;
    imu_auto_gyro_bias_sum_z_raw_ = 0.0;
    return;
  }

  imu_auto_gyro_bias_sum_x_raw_ += static_cast<double>(imu.gx);
  imu_auto_gyro_bias_sum_y_raw_ += static_cast<double>(imu.gy);
  imu_auto_gyro_bias_sum_z_raw_ += static_cast<double>(imu.gz);
  ++imu_auto_gyro_bias_count_;
  if (imu_auto_gyro_bias_count_ < imu_auto_gyro_bias_samples_) {
    return;
  }

  imu_gyro_bias_x_raw_ = imu_auto_gyro_bias_sum_x_raw_ / static_cast<double>(imu_auto_gyro_bias_count_);
  imu_gyro_bias_y_raw_ = imu_auto_gyro_bias_sum_y_raw_ / static_cast<double>(imu_auto_gyro_bias_count_);
  imu_gyro_bias_z_raw_ = imu_auto_gyro_bias_sum_z_raw_ / static_cast<double>(imu_auto_gyro_bias_count_);
  imu_auto_gyro_bias_ready_ = true;
  RCLCPP_INFO(
    get_logger(),
    "IMU gyro raw bias calibrated from %d stationary samples: x=%.3f y=%.3f z=%.3f",
    imu_auto_gyro_bias_count_, imu_gyro_bias_x_raw_, imu_gyro_bias_y_raw_, imu_gyro_bias_z_raw_);
}

void BaseDriverNode::publishImuRaw(const ImuTelemetry & imu, const rclcpp::Time & stamp)
{
  if (!imu_raw_pub_) { return; }
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = imu_frame_id_;
  msg.linear_acceleration.x =
    (static_cast<double>(imu.ax) - imu_accel_bias_x_raw_) / imu_accel_lsb_per_g_ * kStandardGravity;
  msg.linear_acceleration.y =
    (static_cast<double>(imu.ay) - imu_accel_bias_y_raw_) / imu_accel_lsb_per_g_ * kStandardGravity;
  msg.linear_acceleration.z =
    (static_cast<double>(imu.az) - imu_accel_bias_z_raw_) / imu_accel_lsb_per_g_ * kStandardGravity;
  msg.angular_velocity.x =
    (static_cast<double>(imu.gx) - imu_gyro_bias_x_raw_) / imu_gyro_lsb_per_dps_ * kDegToRad;
  msg.angular_velocity.y =
    (static_cast<double>(imu.gy) - imu_gyro_bias_y_raw_) / imu_gyro_lsb_per_dps_ * kDegToRad;
  msg.angular_velocity.z =
    (static_cast<double>(imu.gz) - imu_gyro_bias_z_raw_) / imu_gyro_lsb_per_dps_ * kDegToRad;
  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity_covariance[0] = imu_angular_velocity_covariance_;
  msg.angular_velocity_covariance[4] = imu_angular_velocity_covariance_;
  msg.angular_velocity_covariance[8] = imu_angular_velocity_covariance_;
  msg.linear_acceleration_covariance[0] = imu_linear_acceleration_covariance_;
  msg.linear_acceleration_covariance[4] = imu_linear_acceleration_covariance_;
  msg.linear_acceleration_covariance[8] = imu_linear_acceleration_covariance_;
  imu_raw_pub_->publish(msg);
}

void BaseDriverNode::publishWheelTicks(const EncoderTelemetry & enc)
{
  if (!wheel_ticks_pub_) { return; }
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {enc.ix, enc.iy, enc.iw, enc.wa, enc.wb, enc.wc, enc.wd};
  wheel_ticks_pub_->publish(msg);
}

void BaseDriverNode::publishDebugTelemetry(const DebugTelemetry & dbg)
{
  if (!stm32_debug_pub_) { return; }
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {dbg.tgx, dbg.tgy, dbg.tgw, dbg.rta, dbg.rtb, dbg.rtc, dbg.rtd, dbg.pwma, dbg.pwmb, dbg.pwmc, dbg.pwmd, dbg.dt};
  stm32_debug_pub_->publish(msg);
}

geometry_msgs::msg::Quaternion BaseDriverNode::yawToQuaternion(double yaw) const
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void BaseDriverNode::publishOdomTf(const rclcpp::Time & stamp) const
{
  if (!tf_broadcaster_) { return; }
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = odom_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;
  tf_msg.transform.translation.x = odom_x_m_;
  tf_msg.transform.translation.y = odom_y_m_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = yawToQuaternion(odom_yaw_rad_);
  tf_broadcaster_->sendTransform(tf_msg);
}

void BaseDriverNode::updateOdom(
  const EncoderTelemetry & enc, const ImuTelemetry & imu, const rclcpp::Time & stamp)
{
  (void)imu;
  if (!odom_pub_) { return; }
  const double vx = applyOdomDeadband(static_cast<double>(enc.ix) / 1000.0, odom_linear_deadband_mps_);
  const double vy = applyOdomDeadband(static_cast<double>(enc.iy) / 1000.0, odom_linear_deadband_mps_);
  const double wz = applyOdomDeadband((static_cast<double>(enc.iw) / 1000.0) * odom_yaw_scale_, odom_angular_deadband_radps_);
  if (!odom_initialized_) {
    odom_initialized_ = true;
    last_odom_stamp_ = stamp;
  } else {
    const double dt = (stamp - last_odom_stamp_).seconds();
    last_odom_stamp_ = stamp;
    if (dt > 0.0 && dt < 1.0) {
      const double cos_yaw = std::cos(odom_yaw_rad_);
      const double sin_yaw = std::sin(odom_yaw_rad_);
      const double vx_world = vx * cos_yaw - vy * sin_yaw;
      const double vy_world = vx * sin_yaw + vy * cos_yaw;
      odom_x_m_ += vx_world * dt;
      odom_y_m_ += vy_world * dt;
      odom_yaw_rad_ += wz * dt;
    }
  }
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;
  msg.pose.pose.position.x = odom_x_m_;
  msg.pose.pose.position.y = odom_y_m_;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yawToQuaternion(odom_yaw_rad_);
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.angular.z = wz;
  msg.pose.covariance[0] = 0.05;
  msg.pose.covariance[7] = 0.05;
  msg.pose.covariance[35] = 0.1;
  msg.twist.covariance[0] = 0.05;
  msg.twist.covariance[7] = 0.05;
  msg.twist.covariance[35] = 0.1;
  odom_pub_->publish(msg);
}

}  // namespace car_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_driver::BaseDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
