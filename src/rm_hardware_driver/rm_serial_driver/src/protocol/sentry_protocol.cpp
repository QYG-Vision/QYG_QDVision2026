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
// ros2
#include <geometry_msgs/msg/twist.hpp>

namespace qd::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, int speed, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name), speed);
  packet_tool_ = std::make_shared<FixedPacketTool<24>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
    FixedPacket<32> packet;
    const SendPacket pkt = encodeGimbalCommand(data);
    packet.copyFrom(pkt.data());

    packet_tool_->sendPacket(packet);

    // 不在 tracking 就不打印延迟了
    if (data.distance < 0) {
        return;
    }
    FYT_DEBUG(
        "serial_driver",
        "Latency(predict to send): {:.2f} ms",
        (rclcpp::Clock(RCL_ROS_TIME).now() - data.header.stamp).seconds() * 1e3
    );
}

void ProtocolSentry::send(const rm_interfaces::msg::ChassisCmd &data) {
/*   // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
  packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  packet_.loadData<float>(data.twist.linear.x, 16);
  // linear y
  packet_.loadData<float>(data.twist.linear.y, 20);
  // angular z
  packet_.loadData<float>(data.twist.angular.z, 24);
  // useless data
  // packet_.loadData<float>(0, 28);
  packet_tool_->sendPacket(packet_); */
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
    FixedPacket<24> packet;
    if (!packet_tool_->recvPacket(packet)) {
        return false;
    }
    ReceivePacket frame{};
    std::copy_n(packet.buffer(), frame.size(), frame.begin());
    return decodeFeedback(frame, data);
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
/*   auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
    "/cmd_chassis",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::ChassisCmd::SharedPtr msg) { this->send(*msg); }); */
  // return {sub1, sub2, sub3};
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
