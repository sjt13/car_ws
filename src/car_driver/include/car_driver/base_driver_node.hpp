#ifndef CAR_DRIVER__BASE_DRIVER_NODE_HPP_
#define CAR_DRIVER__BASE_DRIVER_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

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

  VelocityCommand clampTwist(const geometry_msgs::msg::Twist & msg) const;
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
  bool parseTelIntegerFields(const std::string & payload, std::array<int32_t, 13> & values) const;
  void publishImuRaw(const ImuTelemetry & imu, const rclcpp::Time & stamp);
  void publishWheelTicks(const EncoderTelemetry & enc);
  void updateOdom(
    const EncoderTelemetry & enc, const ImuTelemetry & imu, const rclcpp::Time & stamp);
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_ticks_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string port_;
  std::string imu_frame_id_ {"imu_link"};
  std::string odom_frame_id_ {"odom"};
  std::string base_frame_id_ {"base_link"};
  int baudrate_ {115200};
  double cmd_timeout_sec_ {0.5};
  double publish_rate_hz_ {30.0};
  double max_vx_mps_ {1.5};
  double max_vy_mps_ {1.2};
  double max_wz_radps_ {6.28};
  double reconnect_interval_sec_ {1.0};

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
