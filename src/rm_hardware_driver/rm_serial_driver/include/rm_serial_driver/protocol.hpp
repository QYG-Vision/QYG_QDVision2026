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
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <vector>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/float_protocol.hpp"
#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"

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

    using SendPacket = wire::Frame;
    using ReceivePacket = wire::Frame;

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
     * @brief 将弧度云台指令编码为一个 24 字节帧。
     * @param data 绝对角（rad）、目标距离（m）、目标自转速度（rad/s）和开火建议。
     * @return 包含全部云台指令字段的单帧数据。
     * @note 调用方必须持有 send_mutex_，避免多个发送回调并发访问串口及编号日志缓存。
     */
    SendPacket encodeGimbalCommand(const rm_interfaces::msg::GimbalCmd& data) {
        return wire::encode_command(data, armor_id_to_uint8(data.id));
    }

    /**
     * @brief 校验并解码电控完整反馈。
     * @param frame 一个完整的 24 字节帧。
     * @param data 通过校验后写入的 ROS 数据；非法帧不修改原值。
     * @return 取得完整反馈快照时为 true。
     * @note 仅由接收线程调用；角度单位为 rad，弹速为 m/s。
     */
    bool decodeFeedback(const ReceivePacket& frame, rm_interfaces::msg::SerialReceiveData& data) {
        wire::Feedback decoded;
        if (!wire::decode_feedback(frame, decoded)) {
            return false;
        }
        data.mode = decoded.mode;
        data.roll = decoded.roll;
        data.pitch = decoded.pitch;
        data.yaw = decoded.yaw;
        data.bullet_speed = decoded.bullet_speed;
        data.mcu_timestamp = decoded.mcu_timestamp;
        return true;
    }

    std::mutex send_mutex_;

private:
    std::set<std::string> warned_unknown_ids_;
};

} // namespace qd::serial_driver::protocol

#endif // SERIAL_DRIVER_PROTOCOLS_HPP_
