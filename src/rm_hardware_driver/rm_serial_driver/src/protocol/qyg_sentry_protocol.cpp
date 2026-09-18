#include "rm_serial_driver/protocol/qyg_sentry_protocol.hpp"

#include <array>
#include <iomanip>
#include <sstream>

#include "rm_serial_driver/uart_transporter.hpp"

namespace qd::serial_driver::protocol
{

ProtocolQygSentry::ProtocolQygSentry(std::string_view port_name, int speed, bool enable_data_print)
: transporter_(std::make_shared<UartTransporter>(std::string(port_name), speed)),
  enable_data_print_(enable_data_print)
{
  latest_gimbal_.distance = -1.0;
  if (!transporter_->open()) {
    FYT_ERROR("serial_driver", "Failed to open QYG uart: {}", transporter_->errorMessage());
  }
}

ProtocolQygSentry::~ProtocolQygSentry() { transporter_->close(); }

void ProtocolQygSentry::send(const rm_interfaces::msg::GimbalCmd & data)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  latest_gimbal_ = data;
  sendLatestLocked();

  if (data.distance >= 0.0) {
    FYT_DEBUG(
      "serial_driver", "Latency(predict to send): {:.2f} ms",
      (rclcpp::Clock(RCL_ROS_TIME).now() - data.header.stamp).seconds() * 1e3);
  }
}

void ProtocolQygSentry::updateChassis(const geometry_msgs::msg::Twist & data)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  latest_chassis_ = data;
  // 导航速度变化时立即发送；即使没有识别到目标，底盘也能继续接收导航控制。
  sendLatestLocked();
}

void ProtocolQygSentry::updateChassis(const rm_interfaces::msg::ChassisCmd & data)
{
  updateChassis(data.twist);
}

void ProtocolQygSentry::sendLatestLocked()
{
  const auto task_mode = qyg_mode_.load();
  const bool control = latest_gimbal_.distance >= 0.0 && task_mode != qyg::QygVisionMode::IDLE;
  const float distance = control ? static_cast<float>(latest_gimbal_.distance) : -1.0F;
  const uint8_t target_id = control ? armor_id_to_uint8(latest_gimbal_.id) : 0U;
  const float target_v_yaw = control ? static_cast<float>(latest_gimbal_.target_v_yaw) : 0.0F;
  const auto frame = qyg::makeSendFrame(
    control, control && latest_gimbal_.fire_advice,
    qyg::degreesToRadians(static_cast<float>(latest_gimbal_.yaw)),
    qyg::degreesToRadians(static_cast<float>(latest_gimbal_.pitch)),
    static_cast<float>(latest_chassis_.linear.x), static_cast<float>(latest_chassis_.linear.y),
    static_cast<float>(latest_chassis_.angular.z), distance, target_id, target_v_yaw);

  if (enable_data_print_) {
    const auto * bytes = reinterpret_cast<const uint8_t *>(&frame);
    std::ostringstream stream;
    for (size_t i = 0; i < sizeof(frame); ++i) {
      stream << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
             << static_cast<int>(bytes[i]) << ' ';
    }
    FYT_INFO("serial_driver", "QYG TX ({}B): {}", sizeof(frame), stream.str());
  }

  if (!transporter_->isOpen() && !transporter_->open()) {
    return;
  }
  if (transporter_->write(&frame, sizeof(frame)) != static_cast<int>(sizeof(frame))) {
    FYT_ERROR("serial_driver", "Failed to send QYG frame, reconnecting uart");
    transporter_->close();
    transporter_->open();
  }
}

bool ProtocolQygSentry::receive(rm_interfaces::msg::SerialReceiveData & data)
{
  while (rclcpp::ok()) {
    const auto parsed = stream_parser_.popFrame();
    if (parsed.has_value()) {
      const auto & frame = parsed.value();
      const auto task_mode = qyg::getVisionMode(frame.sentry_state);
      qyg_mode_.store(task_mode);
      data.mode = qyg::mapToQdVisionMode(task_mode, enemy_is_red_.load());
      const auto gimbal_angles = qyg::decodeGimbalFeedback(frame);
      data.roll = gimbal_angles.roll_degrees;
      data.pitch = gimbal_angles.pitch_degrees;
      data.yaw = gimbal_angles.yaw_degrees;
      data.bullet_speed = frame.bullet_speed;
      data.mcu_timestamp = frame.mcu_timestamp;
      data.current_mode = frame.current_mode;
      data.actual_vx = frame.actual_vx;
      data.actual_vy = frame.actual_vy;
      data.actual_wz = frame.actual_wz;
      data.sentry_state = frame.sentry_state;
      if (actual_velocity_pub_ != nullptr) {
        geometry_msgs::msg::Twist actual_velocity;
        actual_velocity.linear.x = frame.actual_vx;
        actual_velocity.linear.y = frame.actual_vy;
        actual_velocity.angular.z = frame.actual_wz;
        actual_velocity_pub_->publish(actual_velocity);
      }
      return true;
    }

    std::array<uint8_t, 256> bytes{};
    if (!transporter_->isOpen() && !transporter_->open()) {
      return false;
    }
    const int length = transporter_->read(bytes.data(), bytes.size());
    if (length <= 0) {
      transporter_->close();
      transporter_->open();
      return false;
    }
    if (enable_data_print_) {
      FYT_INFO("serial_driver", "QYG RX chunk: {} bytes", length);
    }
    stream_parser_.append(bytes.data(), static_cast<size_t>(length));
  }
  return false;
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolQygSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node)
{
  const auto enemy_color = node->declare_parameter<std::string>("qyg_enemy_color", "blue");
  enemy_is_red_.store(enemy_color == "red");
  if (enemy_color != "red" && enemy_color != "blue") {
    FYT_WARN("serial_driver", "Invalid qyg_enemy_color '{}', using blue", enemy_color);
  }
  actual_velocity_pub_ =
    node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_real", rclcpp::SensorDataQoS());

  auto armor_sub = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { send(*msg); });
  auto rune_sub = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { send(*msg); });
  auto chassis_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) { updateChassis(*msg); });
  auto chassis_cmd_sub = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
    "/cmd_chassis", rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::ChassisCmd::ConstSharedPtr msg) { updateChassis(*msg); });
  return {armor_sub, rune_sub, chassis_sub, chassis_cmd_sub};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolQygSentry::getClients(
  rclcpp::Node::SharedPtr node) const
{
  return {
    node->create_client<rm_interfaces::srv::SetMode>(
      "armor_detector/set_mode", rmw_qos_profile_services_default),
    node->create_client<rm_interfaces::srv::SetMode>(
      "armor_solver/set_mode", rmw_qos_profile_services_default),
    node->create_client<rm_interfaces::srv::SetMode>(
      "rune_detector/set_mode", rmw_qos_profile_services_default),
    node->create_client<rm_interfaces::srv::SetMode>(
      "rune_solver/set_mode", rmw_qos_profile_services_default),
  };
}

std::string ProtocolQygSentry::getErrorMessage() { return transporter_->errorMessage(); }

}  // namespace qd::serial_driver::protocol
