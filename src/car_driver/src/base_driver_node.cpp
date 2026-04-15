// 这个文件负责把 ROS2 的 /cmd_vel 速度指令转换成 STM32 已有串口协议帧，
// 并通过 UART 下发到底盘控制板，形成“上位机 -> 底盘”的最小可用主链路。
#include "car_driver/base_driver_node.hpp"

// 算法工具：用于限幅时的 std::min/std::max。
#include <algorithm>
// 固定长度数组：用于 16 字段临时解析缓存。
#include <array>
// 错误码：用于判断串口读写失败原因（如 EAGAIN/EINTR）。
#include <cerrno>
// 时间工具：用于定时周期与超时计算。
#include <chrono>
// 数学函数：用于 std::lround 进行四舍五入。
#include <cmath>
// 固定宽度整数：用于 int16_t 等类型。
#include <cstdint>
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

namespace car_driver
{

namespace
{

// STM32 协议里速度使用“真实值 * 1000”后的整数（见 $CAR:x,y,z!）。
constexpr double kCmdScale = 1000.0;

// 对称限幅：把 value 裁剪到 [-limit, limit] 区间内。
double clampSymmetric(double value, double limit)
{
  // 先把 value 与上限比较得到不大于 limit 的结果，再和下限比较得到最终结果。
  return std::max(-limit, std::min(value, limit));
}

// 把整型波特率转换为 termios 所需的 speed_t 常量。
speed_t toTermiosBaud(int baudrate)
{
  // 根据用户参数选择对应波特率宏。
  switch (baudrate) {
    case 9600:
      return B9600;    // 9.6 kbps
    case 19200:
      return B19200;   // 19.2 kbps
    case 38400:
      return B38400;   // 38.4 kbps
    case 57600:
      return B57600;   // 57.6 kbps
    case 115200:
      return B115200;  // 115.2 kbps（本项目常用）
    case 230400:
      return B230400;  // 230.4 kbps
    case 460800:
      return B460800;  // 460.8 kbps
#ifdef B921600
    case 921600:
      return B921600;  // 921.6 kbps（部分平台支持）
#endif
    default:
      // 返回 0 表示当前平台不支持该波特率，后续会报错。
      return 0;
  }
}

}  // namespace

// 构造函数：完成参数读取、话题订阅、定时器创建和串口初次连接尝试。
BaseDriverNode::BaseDriverNode()
: Node("base_driver_node")  // ROS2 节点名固定为 base_driver_node。
{
  // 第一步：先声明并读取全部参数，确保后续逻辑有默认值或用户配置值。
  declareAndLoadParameters();

  // 硬件提醒：
  // UART9 是 3.3V TTL 电平（0~3.3V），请与 STM32 交叉连接并共地，避免接错到 5V/RS232 电平。
  RCLCPP_INFO(
    get_logger(),
    "base_driver_node 启动：订阅 /cmd_vel，串口下行协议复用 STM32 的 $CAR:x,y,z!。");
  // 打印本次运行的关键参数，便于现场调试快速确认配置。
  RCLCPP_INFO(
    get_logger(),
    "当前串口参数：port=%s baudrate=%d timeout=%.3fs rate=%.1fHz",
    port_.c_str(), baudrate_, cmd_timeout_sec_, publish_rate_hz_);

  // 按需求避免误用调试串口：如果检测到 /dev/ttyFIQ0，给出明确警告。
  if (port_ == "/dev/ttyFIQ0") {
    RCLCPP_WARN(
      get_logger(),
      "当前端口是 /dev/ttyFIQ0（调试串口）。底盘控制建议使用 UART9 对应 /dev/ttyS9。");
  }

  // 创建 /cmd_vel 订阅：收到速度指令时进入 cmdVelCallback。
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",               // 订阅话题名
    rclcpp::QoS(10),          // 轻量队列，够用且实时性较好
    std::bind(&BaseDriverNode::cmdVelCallback, this, std::placeholders::_1));  // 绑定回调

  // 上行发布：IMU 原始数据（按传感器数据语义使用 SensorDataQoS）。
  imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    rclcpp::SensorDataQoS());
  // 上行发布：编码器数据（发布 ix/iy/iw/wa/wb/wc/wd 共 7 个整型字段）。
  wheel_ticks_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "/wheel_ticks",
    rclcpp::QoS(10));

  // 把发布频率（Hz）转换为 wall timer 的时间周期。
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  // 创建周期定时器：负责发送、超时保护、串口重连和上行轮询。
  timer_ = create_wall_timer(period, std::bind(&BaseDriverNode::timerCallback, this));

  // 启动时先尝试打开一次串口；失败时不会崩溃，后续定时器会按间隔重试。
  (void)ensureSerialConnected();
}

// 析构函数：节点退出前尽量下发一次零速，然后关闭串口。
BaseDriverNode::~BaseDriverNode()
{
  // 仅当串口已连接时，才尝试发送最后一次零速停车命令。
  if (serial_fd_ >= 0) {
    VelocityCommand zero_cmd {};                         // 全零速度
    (void)sendVelocityCommand(zero_cmd, "node shutdown");  // 忽略返回值，退出流程继续
  }
  // 最后关闭串口文件描述符，释放系统资源。
  closeSerialPort();
}

// 读取并校验参数：集中管理所有可调配置。
void BaseDriverNode::declareAndLoadParameters()
{
  // 串口设备名：默认按当前硬件场景设置为 UART9 对应的 /dev/ttyS9。
  declare_parameter<std::string>("port", "/dev/ttyS9");
  // 串口波特率：与 STM32 侧默认一致为 115200。
  declare_parameter<int>("baudrate", 115200);
  // 指令超时时间（秒）：超过该时间未收到新 /cmd_vel，则触发自动停车。
  declare_parameter<double>("cmd_timeout", 0.5);
  // 发送循环频率（Hz）：决定下行发送与超时检测频率。
  declare_parameter<double>("publish_rate", 30.0);
  // x 方向最大线速度限幅（m/s）。
  declare_parameter<double>("max_vx", 1.5);
  // y 方向最大线速度限幅（m/s）。
  declare_parameter<double>("max_vy", 1.2);
  // z 轴最大角速度限幅（rad/s）。
  declare_parameter<double>("max_wz", 6.28);
  // 串口断开后的重连尝试间隔（秒）。
  declare_parameter<double>("reconnect_interval", 1.0);
  // IMU 消息 frame_id（用于 /imu/data_raw）。
  declare_parameter<std::string>("imu_frame_id", "imu_link");

  // 从参数服务器读取值到成员变量，后续逻辑统一使用成员变量。
  port_ = get_parameter("port").as_string();                          // 设备节点
  baudrate_ = get_parameter("baudrate").as_int();                     // 波特率
  cmd_timeout_sec_ = get_parameter("cmd_timeout").as_double();        // 超时阈值
  publish_rate_hz_ = get_parameter("publish_rate").as_double();       // 发送频率
  max_vx_mps_ = get_parameter("max_vx").as_double();                  // vx 限幅
  max_vy_mps_ = get_parameter("max_vy").as_double();                  // vy 限幅
  max_wz_radps_ = get_parameter("max_wz").as_double();                // wz 限幅
  reconnect_interval_sec_ = get_parameter("reconnect_interval").as_double();  // 重连间隔
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();          // IMU 坐标系

  // 防御性校验：频率不能 <= 0，否则会导致 timer 周期非法。
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate <= 0 不合法，已回退到 30.0Hz");
    publish_rate_hz_ = 30.0;  // 回退到安全默认值
  }
  // 防御性校验：超时不能 <= 0，否则会一直判定超时。
  if (cmd_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "cmd_timeout <= 0 不合法，已回退到 0.5s");
    cmd_timeout_sec_ = 0.5;   // 回退到安全默认值
  }
  // 防御性校验：重连间隔过小会导致日志刷屏与无意义 busy retry。
  if (reconnect_interval_sec_ < 0.1) {
    reconnect_interval_sec_ = 0.1;  // 设定一个最小重试间隔
  }
  // 防御性校验：frame_id 为空时回退默认值，避免发布空 frame_id。
  if (imu_frame_id_.empty()) {
    RCLCPP_WARN(get_logger(), "imu_frame_id 为空，已回退到 imu_link");
    imu_frame_id_ = "imu_link";
  }
}

// /cmd_vel 回调：只做“接收+限幅+缓存+更新时间戳”，不直接写串口。
void BaseDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 先执行限幅，确保进入下游的速度总是合法范围。
  const auto clamped = clampTwist(*msg);

  // 如果限幅前后有差异，说明输入超限，打印节流警告帮助调参。
  if (
    std::abs(clamped.vx_mps - msg->linear.x) > 1e-6 ||
    std::abs(clamped.vy_mps - msg->linear.y) > 1e-6 ||
    std::abs(clamped.wz_radps - msg->angular.z) > 1e-6)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "检测到 /cmd_vel 超限，已按 max_vx/max_vy/max_wz 裁剪。");
  }

  // 用最新有效指令覆盖缓存，定时器发送时读取该缓存值。
  latest_cmd_ = clamped;
  // 标记已经收到过有效控制，后续才允许真正下发运动命令。
  has_received_cmd_ = true;
  // 一旦收到新指令，退出“超时停车模式”。
  timeout_mode_ = false;
  // 记录最近一次收到命令的时刻，供超时判断使用。
  last_cmd_time_ = now();
}

// 定时器回调：统一驱动串口重连、上行轮询、下行发送和超时自动停车。
void BaseDriverNode::timerCallback()
{
  // 先确认串口可用；若不可用则本周期直接返回，避免后续无效操作。
  if (!ensureSerialConnected()) {
    return;
  }

  // 上行轮询：接收串口数据、分帧，并在 processReceivedFrame 中解析发布。
  pollSerialReceive();

  // 启动后在未收到有效 /cmd_vel 前，不主动发送运动指令，避免任何随机控制量。
  if (!has_received_cmd_) {
    return;
  }

  // 默认先取最新指令；若检测超时会在下方替换为零速。
  VelocityCommand cmd_to_send = latest_cmd_;
  // 计算“当前时刻 - 最后一次收到 /cmd_vel 的时刻”。
  const auto elapsed = (now() - last_cmd_time_).seconds();
  // 如果超过超时阈值，执行自动停车逻辑。
  if (elapsed > cmd_timeout_sec_) {
    // 仅在刚进入超时状态时打印一次明显日志，避免每周期重复刷屏。
    if (!timeout_mode_) {
      RCLCPP_WARN(
        get_logger(),
        "/cmd_vel 超时 %.3fs（阈值 %.3fs），自动下发零速度停车。",
        elapsed, cmd_timeout_sec_);
    }
    // 标记当前处于超时保护状态。
    timeout_mode_ = true;
    // 超时时把本周期待发指令强制置零，实现安全停车。
    cmd_to_send = VelocityCommand {};
  }

  // 真正发送：正常模式 reason=cmd_vel，超时模式 reason=timeout auto stop。
  (void)sendVelocityCommand(cmd_to_send, timeout_mode_ ? "timeout auto stop" : "cmd_vel");
}

// Twist 限幅：把 ROS 速度语义映射到底盘语义并做参数化裁剪。
BaseDriverNode::VelocityCommand BaseDriverNode::clampTwist(
  const geometry_msgs::msg::Twist & msg) const
{
  VelocityCommand cmd {};  // 先初始化为全零，防止未赋值字段。

  // 语义映射说明：
  // linear.x -> 底盘前后速度 vx（m/s）
  // linear.y -> 麦克纳姆横移速度 vy（m/s，ROS 中 +Y 通常是车体左侧）
  // angular.z -> 绕 Z 轴角速度 wz（rad/s，+ 表示逆时针）
  // clampSymmetric 会把每个分量都限制在对应最大绝对值范围内。
  cmd.vx_mps = clampSymmetric(msg.linear.x, max_vx_mps_);
  cmd.vy_mps = clampSymmetric(msg.linear.y, max_vy_mps_);
  cmd.wz_radps = clampSymmetric(msg.angular.z, max_wz_radps_);
  // 返回裁剪后的安全速度。
  return cmd;
}

// 编码函数：把速度命令转成 STM32 现有协议帧 "$CAR:x,y,z!"。
std::string BaseDriverNode::encodeVelocityFrame(const VelocityCommand & cmd) const
{
  // 复用 STM32 既有协议：$CAR:x,y,z!
  // 其中 x/y 单位为 m/s*1000，z 单位为 rad/s*1000（见 STM32 的 Vel.TG_IX/TG_IY/TG_IW）。
  const auto to_i16 = [](double value) {
      // 先按协议比例放大并四舍五入到最接近的整数。
      long raw = std::lround(value * kCmdScale);
      // 再做 int16 边界保护，避免极端值溢出。
      raw = std::max(raw, static_cast<long>(std::numeric_limits<int16_t>::min()));
      // 上边界同理。
      raw = std::min(raw, static_cast<long>(std::numeric_limits<int16_t>::max()));
      // 返回 int（用于 to_string 拼接）。
      return static_cast<int>(raw);
    };

  // 计算 x 分量（前后速度）。
  const int x = to_i16(cmd.vx_mps);
  // 计算 y 分量（横移速度）。
  const int y = to_i16(cmd.vy_mps);
  // 计算 z 分量（角速度）。
  const int z = to_i16(cmd.wz_radps);

  // 按 STM32 既有解析格式拼接字符串，不额外加入换行符。
  return "$CAR:" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "!";
}

// 串口连接守护：已连接直接返回；未连接按间隔触发重连尝试。
bool BaseDriverNode::ensureSerialConnected()
{
  // 如果 fd 有效，说明串口已连接，直接可用。
  if (serial_fd_ >= 0) {
    return true;
  }

  // 记录当前时刻，用于做“重连节流”。
  const auto now_time = now();
  // 如果不是第一次重连，且距离上次重连尝试还不到设定间隔，则暂不重试。
  if (
    last_reconnect_try_time_.nanoseconds() != 0 &&
    (now_time - last_reconnect_try_time_).seconds() < reconnect_interval_sec_)
  {
    return false;
  }

  // 更新“最近一次重连尝试时间”。
  last_reconnect_try_time_ = now_time;
  // 真正执行打开串口流程。
  return openSerialPort();
}

// 打开串口设备并完成参数配置。
bool BaseDriverNode::openSerialPort()
{
  // 以读写方式打开设备；O_NOCTTY 防止进程成为该串口控制终端；O_NONBLOCK 采用非阻塞 I/O。
  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  // 打开失败：常见原因是设备不存在、权限不足、被占用等。
  if (fd < 0) {
    // 使用节流日志降低刷屏频率（每 3 秒最多一条）。
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "串口打开失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }

  // 打开成功后配置串口参数（波特率、数据位、停止位、校验位等）。
  if (!configureSerialPort(fd)) {
    // 配置失败要关闭临时 fd，避免资源泄漏。
    ::close(fd);
    return false;
  }

  // 配置成功：把 fd 保存到成员变量，供后续读写使用。
  serial_fd_ = fd;
  // 清空接收缓存，避免复用旧数据。
  rx_buffer_.clear();
  // 打印连接成功信息。
  RCLCPP_INFO(get_logger(), "串口已连接：%s @ %d", port_.c_str(), baudrate_);
  return true;
}

// 关闭串口：仅当 fd 有效时执行 close。
void BaseDriverNode::closeSerialPort()
{
  // 判断 fd 是否有效。
  if (serial_fd_ >= 0) {
    // 关闭文件描述符。
    ::close(serial_fd_);
    // 复位 fd，表示当前未连接。
    serial_fd_ = -1;
  }
}

// 配置 termios 参数：8N1、无流控、原始模式、非阻塞读。
bool BaseDriverNode::configureSerialPort(int fd) const
{
  // 定义 termios 结构体并清零初始化。
  termios tty {};
  // 读取当前串口属性作为配置基础。
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(
      get_logger(), "读取串口属性失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }

  // 把用户配置波特率转换为 termios 枚举值。
  const speed_t speed = toTermiosBaud(baudrate_);
  // 转换失败说明平台不支持该波特率。
  if (speed == 0) {
    RCLCPP_ERROR(get_logger(), "不支持的波特率：%d", baudrate_);
    return false;
  }

  // 开启原始模式：关闭行编辑、回显等终端行为，按字节透明收发。
  cfmakeraw(&tty);
  // 设置输入波特率。
  cfsetispeed(&tty, speed);
  // 设置输出波特率。
  cfsetospeed(&tty, speed);

  // CLOCAL：忽略调制解调器控制线；CREAD：允许接收数据。
  tty.c_cflag |= (CLOCAL | CREAD);
  // 关闭奇偶校验（N）。
  tty.c_cflag &= ~PARENB;
  // 设置 1 位停止位（1）。
  tty.c_cflag &= ~CSTOPB;
  // 先清除数据位掩码。
  tty.c_cflag &= ~CSIZE;
  // 设置 8 位数据位（8）。
  tty.c_cflag |= CS8;
  // 关闭硬件流控（RTS/CTS）。
  tty.c_cflag &= ~CRTSCTS;

  // 非阻塞读取：VMIN=0 表示不要求最少字节数。
  tty.c_cc[VMIN] = 0;
  // 非阻塞读取：VTIME=0 表示不等待超时，立即返回。
  tty.c_cc[VTIME] = 0;

  // 立即应用新配置。
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      get_logger(), "设置串口属性失败：%s (%s)", port_.c_str(), std::strerror(errno));
    return false;
  }

  // 清空输入输出缓冲，避免历史脏数据影响协议解析。
  tcflush(fd, TCIOFLUSH);
  return true;
}

// 实际发送函数：把 frame 完整写入串口，支持部分写与中断重试。
bool BaseDriverNode::writeFrame(const std::string & frame)
{
  // 若串口当前未连接，直接返回失败。
  if (serial_fd_ < 0) {
    return false;
  }

  // 指向待发送数据首地址。
  const char * data = frame.data();
  // 记录尚未发送的字节数。
  std::size_t remaining = frame.size();

  // 循环直到所有字节都写完，保证“整帧发送”语义。
  while (remaining > 0) {
    // 尝试写入剩余数据。
    const ssize_t n = ::write(serial_fd_, data, remaining);
    // n>0 表示成功写入了 n 字节。
    if (n > 0) {
      data += n;                                  // 指针前移
      remaining -= static_cast<std::size_t>(n);   // 剩余字节减少
      continue;                                   // 继续写剩余部分
    }

    // EINTR 表示被信号中断，通常可直接重试。
    if (n < 0 && errno == EINTR) {
      continue;
    }

    // 非阻塞 fd 上遇到“暂时不可写”也继续重试。
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }

    // 其余错误视为写失败：打印日志并关闭串口，触发后续自动重连。
    RCLCPP_ERROR(
      get_logger(), "串口写入失败：%s (%s)", port_.c_str(), std::strerror(errno));
    closeSerialPort();
    return false;
  }

  // 所有字节发送完成。
  return true;
}

// 发送速度命令：编码 + 发送 + 日志。
bool BaseDriverNode::sendVelocityCommand(const VelocityCommand & cmd, const char * reason)
{
  // 先把速度命令编码成 STM32 可识别的协议字符串。
  const std::string frame = encodeVelocityFrame(cmd);
  // 再调用底层写函数发送。
  if (!writeFrame(frame)) {
    return false;
  }

  // 超时停车模式下，用 warn 节流日志更醒目。
  if (timeout_mode_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "超时保护发送：%s", frame.c_str());
  } else {
    // 正常模式下，打印 info 节流日志便于观察当前下行内容。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1500,
      "串口发送(%s)：%s", reason, frame.c_str());
  }
  return true;
}

// 串口接收轮询：读取所有可读字节并交给分帧逻辑处理。
void BaseDriverNode::pollSerialReceive()
{
  // 未连接时无需读取。
  if (serial_fd_ < 0) {
    return;
  }

  // 临时读缓存：每次最多读取 256 字节。
  uint8_t buffer[256];
  // 持续读取直到当前无更多数据。
  while (true) {
    // 非阻塞读取串口数据。
    const ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
    // n>0 表示实际读到了数据。
    if (n > 0) {
      // 把本次读取的数据追加到成员接收缓存，等待分帧。
      rx_buffer_.append(reinterpret_cast<const char *>(buffer), static_cast<std::size_t>(n));
      continue;
    }

    // n==0 在非阻塞下通常表示当前没有可读数据，退出读取循环。
    if (n == 0) {
      break;
    }

    // 暂无数据可读时（EAGAIN/EWOULDBLOCK）结束本轮轮询。
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }

    // 被信号中断时继续重试。
    if (errno == EINTR) {
      continue;
    }

    // 其他读取错误：打印并关闭串口，等待自动重连。
    RCLCPP_ERROR(
      get_logger(), "串口读取失败：%s (%s)", port_.c_str(), std::strerror(errno));
    closeSerialPort();
    return;
  }

  // 读取结束后尝试从缓存中提取完整帧。
  processRxBuffer();
}

// 分帧：按结束符从 rx_buffer_ 提取完整报文。
void BaseDriverNode::processRxBuffer()
{
  // 只要缓存不为空，就持续尝试提取一帧。
  while (!rx_buffer_.empty()) {
    // 查找第一个“结束符”位置：
    // 兼容 ! / \n / \r / > / } 这些在现有固件中常见的包尾字符。
    const std::size_t pos = rx_buffer_.find_first_of("!\n\r>}");
    // 没找到结束符说明是半包，等待下轮读到更多字节。
    if (pos == std::string::npos) {
      break;
    }

    // 截取完整帧（包含结束符本身）。
    const std::string frame = rx_buffer_.substr(0, pos + 1);
    // 从缓存中删除已处理帧。
    rx_buffer_.erase(0, pos + 1);
    // 交给上层处理函数（当前仅 debug，后续可扩展业务解析）。
    processReceivedFrame(frame);
  }

  // 防御性保护：若持续收不到结束符，限制缓存长度避免无限增长。
  if (rx_buffer_.size() > 1024) {
    // 仅保留最后 1024 字节，丢弃更早数据。
    rx_buffer_.erase(0, rx_buffer_.size() - 1024);
  }
}

// 上行帧处理入口：解析 STM32 的 $TEL 帧并发布 ROS2 话题。
void BaseDriverNode::processReceivedFrame(const std::string & frame)
{
  // 只处理 $TEL 开头的遥测帧；其他帧保持原调试行为，避免破坏兼容性。
  if (frame.rfind("$TEL:", 0) != 0) {
    RCLCPP_DEBUG(get_logger(), "收到串口上行帧：%s", frame.c_str());
    return;
  }

  // $TEL 协议规定包尾必须是 '!'，用于明确帧结束。
  if (frame.size() < 7 || frame.back() != '!') {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "收到 $TEL 帧但包尾不合法（缺少 !）：%s", frame.c_str());
    return;
  }

  TelTelemetryFrame telemetry {};
  if (!parseTelTelemetryFrame(frame, telemetry)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "解析 $TEL 帧失败：%s", frame.c_str());
    return;
  }

  // 成功解析后，统一使用当前 ROS 时间戳发布两个上行话题。
  const auto stamp = now();
  publishImuRaw(telemetry.imu, stamp);
  publishWheelTicks(telemetry.enc);
  // odom 接口预留：本版本不做 odom 计算与发布。
  updateOdomPlaceholder(telemetry.enc, telemetry.imu, stamp);
}

// 解析完整 $TEL 帧，输出结构化的遥测数据。
bool BaseDriverNode::parseTelTelemetryFrame(const std::string & frame, TelTelemetryFrame & out) const
{
  if (frame.rfind("$TEL:", 0) != 0 || frame.size() < 7 || frame.back() != '!') {
    return false;
  }

  // 去掉 "$TEL:" 前缀和末尾 "!"，得到纯字段负载区。
  const std::string payload = frame.substr(5, frame.size() - 6);
  std::array<int32_t, 16> values {};
  if (!parseTelIntegerFields(payload, values)) {
    return false;
  }

  // 按协议固定顺序映射字段：
  // ix,iy,iw,wa,wb,wc,wd,ax,ay,az,gx,gy,gz,pit,rol,yaw
  out.enc.ix = values[0];
  out.enc.iy = values[1];
  out.enc.iw = values[2];
  out.enc.wa = values[3];
  out.enc.wb = values[4];
  out.enc.wc = values[5];
  out.enc.wd = values[6];

  out.imu.ax = values[7];
  out.imu.ay = values[8];
  out.imu.az = values[9];
  out.imu.gx = values[10];
  out.imu.gy = values[11];
  out.imu.gz = values[12];
  out.imu.pit = values[13];
  out.imu.rol = values[14];
  out.imu.yaw = values[15];
  return true;
}

// 解析 $TEL 负载中的 16 个有符号整数字段，失败即返回 false。
bool BaseDriverNode::parseTelIntegerFields(
  const std::string & payload, std::array<int32_t, 16> & values) const
{
  std::size_t start = 0;
  std::size_t index = 0;

  while (start <= payload.size()) {
    const std::size_t comma_pos = payload.find(',', start);
    const std::string token =
      (comma_pos == std::string::npos) ? payload.substr(start) : payload.substr(start, comma_pos - start);

    // 空字段视为非法，例如连续 ",," 或前后多余逗号。
    if (token.empty()) {
      return false;
    }

    errno = 0;
    char * end_ptr = nullptr;
    const long parsed = std::strtol(token.c_str(), &end_ptr, 10);
    // 要求 token 完整是整数文本，且数值落在 int32 范围内。
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

// 发布 /imu/data_raw：当前版本按 STM32 上行原始计数值直发。
void BaseDriverNode::publishImuRaw(const ImuTelemetry & imu, const rclcpp::Time & stamp)
{
  if (!imu_raw_pub_) {
    return;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = imu_frame_id_;
  msg.linear_acceleration.x = static_cast<double>(imu.ax);
  msg.linear_acceleration.y = static_cast<double>(imu.ay);
  msg.linear_acceleration.z = static_cast<double>(imu.az);
  msg.angular_velocity.x = static_cast<double>(imu.gx);
  msg.angular_velocity.y = static_cast<double>(imu.gy);
  msg.angular_velocity.z = static_cast<double>(imu.gz);
  // 本版不发布姿态融合结果，按惯例将 orientation_covariance[0] 置为 -1。
  msg.orientation_covariance[0] = -1.0;
  imu_raw_pub_->publish(msg);
}

// 发布 /wheel_ticks：按固定顺序发布 7 个整型字段。
void BaseDriverNode::publishWheelTicks(const EncoderTelemetry & enc)
{
  if (!wheel_ticks_pub_) {
    return;
  }

  std_msgs::msg::Int32MultiArray msg;
  msg.data = {enc.ix, enc.iy, enc.iw, enc.wa, enc.wb, enc.wc, enc.wd};
  wheel_ticks_pub_->publish(msg);
}

// odom 预留接口：本版本只保留入口，不实现 odom 计算/发布。
void BaseDriverNode::updateOdomPlaceholder(
  const EncoderTelemetry & enc, const ImuTelemetry & imu, const rclcpp::Time & stamp)
{
  (void)enc;
  (void)imu;
  (void)stamp;
  // TODO(odom): 后续在此接入里程计融合与 /odom 发布逻辑。
}

}  // namespace car_driver

// 程序入口：初始化 ROS2 -> 创建节点 -> 阻塞 spin -> 收尾 shutdown。
int main(int argc, char ** argv)
{
  // 初始化 ROS2 运行时。
  rclcpp::init(argc, argv);
  // 创建 base_driver_node 实例。
  auto node = std::make_shared<car_driver::BaseDriverNode>();
  // 进入事件循环，等待订阅回调与定时器回调。
  rclcpp::spin(node);
  // 退出前释放 ROS2 全局资源。
  rclcpp::shutdown();
  // 返回 0 表示正常退出。
  return 0;
}
