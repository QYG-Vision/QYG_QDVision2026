// Created by Chengfu Zou on 2023.7.6
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

#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_

// std
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/v2_codec.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

namespace qd::serial_driver::protocol {
typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol {
public:
    virtual ~Protocol() = default;

    // Send gimbal command
    virtual void send(const rm_interfaces::msg::GimbalCmd& data) = 0;

    // Receive data from serial port
    virtual bool receive(rm_interfaces::msg::SerialReceiveData& data) = 0;

    // Create subscriptions for SerialDriverNode
    virtual std::vector<rclcpp::SubscriptionBase::SharedPtr>
    getSubscriptions(rclcpp::Node::SharedPtr node) = 0;

    // Cretate setMode client for SerialDriverNode
    virtual std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr node) const = 0;

    virtual std::string getErrorMessage() = 0;

    // 字节交换辅助函数
    inline int16_t swap_bytes(int16_t val) {
        const auto bits = static_cast<uint16_t>(val);
        const auto swapped = static_cast<uint16_t>((bits << 8) | (bits >> 8));
        return static_cast<int16_t>(swapped);
    }
    inline uint32_t swap_bytes(uint32_t val) {
        return ((val << 24) & 0xFF000000) | ((val << 8) & 0x00FF0000) | ((val >> 8) & 0x0000FF00)
            | ((val >> 24) & 0x000000FF);
    }

    using SendPacket = v2::CommandFrame;
    using ReceivePacket = v2::FeedbackFrame;

    uint8_t armor_id_to_uint8(const std::string& id) {
        const auto it = id_unit8_map.find(id);
        if (it != id_unit8_map.end()) {
            return it->second;
        }
        if (warned_unknown_ids_.insert(id).second) {
            FYT_WARN("serial_driver", "Unknown armor id '{}', fallback to 0", id);
        }
        return 0;
    }

    const std::map<std::string, uint8_t> id_unit8_map { { "1", 1 },      { "2", 2 },
                                                         { "3", 3 },      { "4", 4 },
                                                         { "5", 5 },      { "outpost", 6 },
                                                         { "sentry", 7 }, { "base", 8 },
                                                         { "negative", 9 },
                                                         { "0", 0 } };

protected:
    /**
     * @brief 显式编码 V2 规划参考，完整规划未就绪时发送无目标帧。
     * @param data 绝对角度和规划导数，同一时刻且符号一致。
     * @return 32 字节 V2 帧。
     */
    SendPacket encodeGimbalCommand(const rm_interfaces::msg::GimbalCmd& data) {
        auto reference = data;
        if (feedback_mode_.load() > 1) {
            reference.reference_valid = false;
        }
        return v2::encode(reference, armor_id_to_uint8(data.id));
    }

    /** @brief 通过完整反馈校验后更新模式，能量机关模式禁止有效规划发送。 */
    bool decodeFeedback(const ReceivePacket& frame, rm_interfaces::msg::SerialReceiveData& data) {
        if (!v2::decode(frame, data)) {
            return false;
        }
        feedback_mode_.store(data.mode);
        return true;
    }

private:
    std::atomic<uint8_t> feedback_mode_{0};
    std::set<std::string> warned_unknown_ids_;
};

} // namespace qd::serial_driver::protocol

#endif // SERIAL_DRIVER_PROTOCOLS_HPP_
