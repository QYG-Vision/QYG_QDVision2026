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

// 确保结构体按1字节对齐，禁止编译器填充
#pragma pack(push, 1)
    struct SendPacket {
        int8_t header_pad = 0xff;       // 偏移 0：帧头
        int8_t fire_advice = 0;         // 偏移 1：开火建议
        int16_t yaw = 0;                // 偏移 2-3：目标 yaw，degree * 100
        int16_t pitch = 0;              // 偏移 4-5：目标 pitch，degree * 100
        int16_t distance = 0;           // 偏移 6-7：目标距离，m * 100
        uint8_t id = 0;                 // 偏移 8：目标类型
        int16_t target_v_yaw = 0;       // 偏移 9-10：目标自转角速度，rad/s * 100
        uint8_t target_state = 0;       // 偏移 11：0 无目标，1 跟踪中
        uint8_t reserved4 = 0;          // 偏移 12：保留
        uint8_t reserved5 = 0;          // 偏移 13：保留
        uint8_t check_byte = 0;         // 偏移 14：预留校验字节
        uint8_t tail_byte = 0x0d;       // 偏移 15：帧尾
    };

    static_assert(sizeof(SendPacket) == 16);
    static_assert(offsetof(SendPacket, target_state) == 11);
    static_assert(offsetof(SendPacket, tail_byte) == 15);

    struct ReceivePacket {
        uint8_t header = 0xff;          // 帧头 1
        uint8_t mode;                   // 模式 2
        int16_t roll;                   // 3 4
        int16_t pitch;                  // 5 6
        int16_t yaw;                    // 7 8
        int16_t bullet_speed;           // 9 10
        uint32_t mcu_timestamp;         // 下位机时间戳，单位毫秒 11 12 13 14
        uint8_t check_byte;             // 校验位 15
        uint8_t tail_byte = 0x0d;       // 帧尾 16
    };
#pragma pack(pop)

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
     * @brief 将云台指令编码为定长串口帧。
     * @param data 云台指令，正距离表示正在跟踪目标。
     * @return 含目标状态标志和限幅定点字段的 16 字节数据包。
     */
    SendPacket encodeGimbalCommand(const rm_interfaces::msg::GimbalCmd& data) {
        SendPacket packet;
        packet.fire_advice = data.fire_advice ? FireState::Fire : FireState::NotFire;
        packet.yaw = swap_bytes(scaleToInt16(data.yaw, 100.0));
        packet.pitch = swap_bytes(scaleToInt16(data.pitch, 100.0));
        packet.distance = swap_bytes(scaleToInt16(data.distance, 100.0));
        packet.id = armor_id_to_uint8(data.id);
        packet.target_v_yaw = swap_bytes(scaleToInt16(data.target_v_yaw, 100.0));
        packet.target_state = std::isfinite(data.distance) && data.distance > 0.0 ? 1 : 0;
        return packet;
    }

private:
    /**
     * @brief 将定标后的数值转换为有符号 16 位字段。
     * @param value 缩放前的物理量。
     * @param scale 协议缩放系数。
     * @return 截断并限幅后的 16 位值；非有限输入转换为 0。
     */
    static int16_t scaleToInt16(const double value, const double scale) noexcept {
        if (!std::isfinite(value)) {
            return 0;
        }
        const double scaled = std::clamp(
            value * scale,
            static_cast<double>(std::numeric_limits<int16_t>::min()),
            static_cast<double>(std::numeric_limits<int16_t>::max())
        );
        return static_cast<int16_t>(scaled);
    }

    std::set<std::string> warned_unknown_ids_;
};

} // namespace qd::serial_driver::protocol

#endif // SERIAL_DRIVER_PROTOCOLS_HPP_
