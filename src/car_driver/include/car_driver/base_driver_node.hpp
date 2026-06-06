#ifndef CAR_DRIVER__BASE_DRIVER_NODE_HPP_
#define CAR_DRIVER__BASE_DRIVER_NODE_HPP_

// 这个头文件声明车端底盘驱动节点 `BaseDriverNode` 的全部接口和成员。
//
// 作用概括：
// 1. 订阅 ROS2 `/cmd_vel`；
// 2. 通过串口把速度指令编码为 STM32 已有串口协议帧并下发；
// 3. 接收 STM32 上行 `$TEL` / `$DBG` 遥测；
// 4. 发布 IMU、轮速、里程计、调试帧和 `odom -> base_footprint` TF。
//
// 如果把 `base_driver_node.cpp` 看作“怎么做”，这个头文件就是“有哪些能力和状态”。

#include <array>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace car_driver
{

class BaseDriverNode : public rclcpp::Node
{
public:
  BaseDriverNode();
  ~BaseDriverNode() override;

private:
  struct VelocityCommand
  {
    double vx_mps {0.0};
    double vy_mps {0.0};
    double wz_radps {0.0};
  };

  // 编码器相关遥测：底盘层(ix/iy/iw) + 车轮层(wa/wb/wc/wd)。
  struct EncoderTelemetry
  {
    int32_t ix {0};
    int32_t iy {0};
    int32_t iw {0};
    int32_t wa {0};
    int32_t wb {0};
    int32_t wc {0};
    int32_t wd {0};
  };

  // 低频调试遥测：目标速度、实际轮速、PWM 和控制周期。
  struct DebugTelemetry
  {
    int32_t tgx {0};
    int32_t tgy {0};
    int32_t tgw {0};
    int32_t rta {0};
    int32_t rtb {0};
    int32_t rtc {0};
    int32_t rtd {0};
    int32_t pwma {0};
    int32_t pwmb {0};
    int32_t pwmc {0};
    int32_t pwmd {0};
    int32_t dt {0};
  };

  // IMU 相关遥测：加速度/角速度（均为 STM32 上行整型原始值）。
  struct ImuTelemetry
  {
    int32_t ax {0};
    int32_t ay {0};
    int32_t az {0};
    int32_t gx {0};
    int32_t gy {0};
    int32_t gz {0};
  };

  // $TEL 一帧上行遥测聚合结构。
  struct TelTelemetryFrame
  {
    EncoderTelemetry enc {};
    ImuTelemetry imu {};
  };

  void declareAndLoadParameters();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  void tfTimerCallback();

  VelocityCommand clampTwist(const geometry_msgs::msg::Twist & msg) const;
  double applyOdomDeadband(double value, double deadband) const;
  std::string encodeVelocityFrame(const VelocityCommand & cmd) const;

  bool ensureSerialConnected();
  bool openSerialPort();
  void closeSerialPort();
  bool configureSerialPort(int fd) const;
  bool writeFrame(const std::string & frame);
  bool sendVelocityCommand(const VelocityCommand & cmd, const char * reason);

  void pollSerialReceive();
  void processRxBuffer();
  void processReceivedFrame(const std::string & frame);
  bool parseTelTelemetryFrame(const std::string & frame, TelTelemetryFrame & out) const;
  bool parseDbgTelemetryFrame(const std::string & frame, DebugTelemetry & out) const;
  bool parseTelIntegerFields(const std::string & payload, std::array<int32_t, 13> & values) const;
  bool parseDbgIntegerFields(const std::string & payload, std::array<int32_t, 12> & values) const;
  void updateImuGyroAutoBias(const ImuTelemetry & imu, const EncoderTelemetry & enc);
  void publishImuRaw(const ImuTelemetry & imu, const rclcpp::Time & stamp);
  void publishWheelTicks(const EncoderTelemetry & enc);
  void publishDebugTelemetry(const DebugTelemetry & dbg);
  void updateOdom(
    const EncoderTelemetry & enc, const ImuTelemetry & imu, const rclcpp::Time & stamp);
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;
  void publishOdomTf(const rclcpp::Time & stamp) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr stm32_debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::string port_;
  std::string imu_frame_id_ {"imu_link"};
  std::string odom_frame_id_ {"odom"};
  std::string base_frame_id_ {"base_footprint"};
  int baudrate_ {115200};
  double cmd_timeout_sec_ {0.5};
  double publish_rate_hz_ {30.0};
  double tf_publish_rate_hz_ {20.0};
  double odom_linear_deadband_mps_ {0.05};
  double odom_angular_deadband_radps_ {0.10};
  double odom_yaw_scale_ {0.53};
  double max_vx_mps_ {1.5};
  double max_vy_mps_ {1.2};
  double max_wz_radps_ {6.28};
  double reconnect_interval_sec_ {1.0};
  double imu_accel_lsb_per_g_ {16384.0};
  double imu_gyro_lsb_per_dps_ {16.4};
  double imu_accel_bias_x_raw_ {0.0};
  double imu_accel_bias_y_raw_ {0.0};
  double imu_accel_bias_z_raw_ {0.0};
  double imu_gyro_bias_x_raw_ {0.0};
  double imu_gyro_bias_y_raw_ {0.0};
  double imu_gyro_bias_z_raw_ {0.0};
  double imu_angular_velocity_covariance_ {0.0025};
  double imu_linear_acceleration_covariance_ {0.25};
  bool imu_auto_gyro_bias_enabled_ {true};
  int imu_auto_gyro_bias_samples_ {120};
  double imu_auto_gyro_bias_max_motion_raw_ {20.0};
  bool imu_auto_gyro_bias_ready_ {false};
  bool publish_odom_tf_ {true};
  int imu_auto_gyro_bias_count_ {0};
  double imu_auto_gyro_bias_sum_x_raw_ {0.0};
  double imu_auto_gyro_bias_sum_y_raw_ {0.0};
  double imu_auto_gyro_bias_sum_z_raw_ {0.0};

  int serial_fd_ {-1};
  bool has_received_cmd_ {false};
  bool timeout_mode_ {false};
  VelocityCommand latest_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_reconnect_try_time_;

  std::string rx_buffer_;
  bool odom_initialized_ {false};
  rclcpp::Time last_odom_stamp_;
  double odom_x_m_ {0.0};
  double odom_y_m_ {0.0};
  double odom_yaw_rad_ {0.0};
};

}  // namespace car_driver

#endif  // CAR_DRIVER__BASE_DRIVER_NODE_HPP_
