#ifndef RM_SERIAL_DRIVER__PROTOCOL__QYG_SENTRY_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL__QYG_SENTRY_PROTOCOL_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol/qyg_protocol.hpp"
#include "rm_serial_driver/transporter_interface.hpp"

namespace qd::serial_driver::protocol
{

/** @brief 通过 QYG 串口协议连接哨兵电控的串口协议实现。 */
class ProtocolQygSentry : public Protocol
{
public:
  /**
   * @brief 创建 QYG 哨兵协议并打开串口。
   * @param port_name 串口设备路径。
   * @param speed 串口波特率。
   * @param enable_data_print 是否打印收发字节调试信息。
   */
  explicit ProtocolQygSentry(std::string_view port_name, int speed, bool enable_data_print);
  ~ProtocolQygSentry() override;

  /** @brief 发送最新云台指令及底盘速度。 */
  void send(const rm_interfaces::msg::GimbalCmd & data) override;
  /** @brief 从串口读取并解析一帧 QYG 回传数据。 */
  bool receive(rm_interfaces::msg::SerialReceiveData & data) override;

  /** @brief 创建云台、底盘指令订阅。 */
  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) override;

  /** @brief 创建视觉模式切换服务客户端。 */
  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  /** @brief 返回底层串口最近一次错误信息。 */
  std::string getErrorMessage() override;

private:
  void updateChassis(const geometry_msgs::msg::Twist & data);
  void updateChassis(const rm_interfaces::msg::ChassisCmd & data);
  void sendLatestLocked();

  TransporterInterface::SharedPtr transporter_;
  bool enable_data_print_{false};
  qyg::QygStreamParser stream_parser_;
  std::mutex send_mutex_;
  rm_interfaces::msg::GimbalCmd latest_gimbal_;
  geometry_msgs::msg::Twist latest_chassis_;
  std::atomic<qyg::QygVisionMode> qyg_mode_{qyg::QygVisionMode::IDLE};
  std::atomic<bool> enemy_is_red_{false};
  std::atomic<float> bullet_speed_{15.0F};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr actual_velocity_pub_;
};

}  // namespace qd::serial_driver::protocol

#endif  // RM_SERIAL_DRIVER__PROTOCOL__QYG_SENTRY_PROTOCOL_HPP_
