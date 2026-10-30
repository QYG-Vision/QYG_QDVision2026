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

#include "rm_serial_driver/protocol/default_protocol.hpp"

namespace qd::serial_driver::protocol {

DefaultProtocol::DefaultProtocol(std::string_view port_name, int speed, bool enable_data_print) {
    auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name), speed);
    packet_tool_ = std::make_shared<FixedPacketTool<24>>(uart_transporter);
    packet_tool_->enbaleDataPrint(enable_data_print);
}

std::vector<rclcpp::SubscriptionBase::SharedPtr>
DefaultProtocol::getSubscriptions(rclcpp::Node::SharedPtr node) {
    return { node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal",
        rclcpp::SensorDataQoS(),
        [this](const rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { this->send(*msg); }
    )

    };
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
DefaultProtocol::getClients(rclcpp::Node::SharedPtr node) const {
    return { node->create_client<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode",
        rmw_qos_profile_services_default
    ) };
}

void DefaultProtocol::send(const rm_interfaces::msg::GimbalCmd& data) {
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

bool DefaultProtocol::receive(rm_interfaces::msg::SerialReceiveData& data) {
    FixedPacket<24> packet;
    if (!packet_tool_->recvPacket(packet)) {
        return false;
    }
    ReceivePacket frame{};
    std::copy_n(packet.buffer(), frame.size(), frame.begin());
    return decodeFeedback(frame, data);
}

} // namespace qd::serial_driver::protocol
