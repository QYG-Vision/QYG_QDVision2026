// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using qd::serial_driver::protocol::ProtocolSentry;

void put_uint32_be(std::array<uint8_t, 4> &out, uint32_t value) {
  for (unsigned i = 0; i < out.size(); ++i) {
    out[out.size() - 1 - i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

uint32_t get_uint32_be(const std::array<uint8_t, 4> &in) {
  uint32_t value = 0;
  for (const uint8_t byte : in) {
    value = (value << 8) | byte;
  }
  return value;
}

void put_float32_be(ProtocolSentry::Float32Be &out, float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_uint32_be(out.bytes, bits);
}

float get_float32_be(const ProtocolSentry::Float32Be &in) {
  const uint32_t bits = get_uint32_be(in.bytes);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

namespace qd::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, int speed, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name), speed);
  packet_tool_ = std::make_shared<FixedPacketTool<sizeof(ReceivePacket)>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
  auto reference = data;
  if (feedback_mode_.load() > 1) {
    reference.reference_valid = false;
  }
  const SendPacket encoded = encode(reference, armor_id_to_uint8(data.id));
  FixedPacket<sizeof(SendPacket)> packet;
  packet.copyFrom(reinterpret_cast<const uint8_t *>(&encoded));
  packet_tool_->sendPacket(packet);

  // 仅打印实际下发的完整规划参考前置时间，header.stamp 是 p/v/a 的共同参考时刻。
  if (encoded.target_state == 0) {
    return;
  }
  FYT_DEBUG(
    "serial_driver",
    "Reference lead(send to sample): {:.2f} ms",
    (rclcpp::Time(data.header.stamp) - rclcpp::Clock(RCL_ROS_TIME).now()).seconds() * 1e3);
}

ProtocolSentry::SendPacket ProtocolSentry::encode(
  const rm_interfaces::msg::GimbalCmd &data, uint8_t id) {
  SendPacket packet;
  put_float32_be(packet.distance, -1.0f);

  const std::array<double, 8> values{
    data.yaw,           data.pitch,           data.distance,       data.target_v_yaw,
    data.yaw_vel_rad_s, data.pitch_vel_rad_s, data.yaw_acc_rad_s2, data.pitch_acc_rad_s2};
  const bool valid = data.reference_valid && id <= 9 && data.distance > 0.0 &&
    std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    });
  if (!valid || static_cast<float>(data.distance) <= 0.0f) {
    return packet;
  }

  packet.fire_advice = data.fire_advice ? 1 : 0;
  put_float32_be(packet.yaw, static_cast<float>(data.yaw));
  put_float32_be(packet.pitch, static_cast<float>(data.pitch));
  put_float32_be(packet.distance, static_cast<float>(data.distance));
  packet.id = id;
  put_float32_be(packet.target_v_yaw, static_cast<float>(data.target_v_yaw));
  packet.target_state = 1;
  put_float32_be(packet.yaw_vel_rad_s, data.yaw_vel_rad_s);
  put_float32_be(packet.pitch_vel_rad_s, data.pitch_vel_rad_s);
  put_float32_be(packet.yaw_acc_rad_s2, data.yaw_acc_rad_s2);
  put_float32_be(packet.pitch_acc_rad_s2, data.pitch_acc_rad_s2);
  return packet;
}

bool ProtocolSentry::decode(
  const ReceivePacket &packet, rm_interfaces::msg::SerialReceiveData &data) {
  if (packet.header != 0xfe || packet.tail_byte != 0x0d || packet.mode > 5) {
    return false;
  }

  const std::array<float, 7> values{
    get_float32_be(packet.roll),
    get_float32_be(packet.pitch),
    get_float32_be(packet.yaw_large),
    get_float32_be(packet.yaw_small),
    get_float32_be(packet.bullet_speed),
    get_float32_be(packet.yaw_actual_vel_rad_s),
    get_float32_be(packet.pitch_actual_vel_rad_s)};
  if (!std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
      })) {
    return false;
  }

  auto decoded = data;
  decoded.mode = packet.mode;
  decoded.roll = values[0];
  decoded.pitch = values[1];
  decoded.yaw_large = values[2];
  decoded.yaw_small = values[3];
  decoded.yaw = decoded.yaw_small;
  decoded.bullet_speed = values[4];
  decoded.mcu_timestamp = get_uint32_be(packet.mcu_timestamp.bytes);
  decoded.yaw_actual_vel_rad_s = values[5];
  decoded.pitch_actual_vel_rad_s = values[6];
  decoded.dual_yaw_feedback = true;
  data = decoded;
  return true;
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<sizeof(ReceivePacket)> wire_packet;
  if (!packet_tool_->recvPacket(wire_packet)) {
    return false;
  }
  ReceivePacket packet;
  std::memcpy(&packet, wire_packet.buffer(), sizeof(packet));
  if (!decode(packet, data)) {
    return false;
  }
  feedback_mode_.store(data.mode);
  return true;
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { this->send(*msg); });
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) {
        auto invalid = *msg;
        invalid.reference_valid = false;
        this->send(invalid);
    });
  return {sub1, sub2};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
}
}  // namespace qd::serial_driver::protocol
