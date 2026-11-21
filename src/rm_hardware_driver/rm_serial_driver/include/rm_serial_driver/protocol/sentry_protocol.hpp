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

#ifndef SERIAL_DRIVER__SENTRY_PROTOCOL_HPP_
#define SERIAL_DRIVER__SENTRY_PROTOCOL_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// project
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/protocol.hpp"

namespace qd::serial_driver::protocol {
// 哨兵通信协议
class ProtocolSentry : public Protocol {
public:
  /** @brief 线路上的大端 IEEE 754 float32，占 4 字节。 */
  struct Float32Be {
    std::array<uint8_t, 4> bytes{};
  };

  /** @brief 线路上的大端 uint32，占 4 字节。 */
  struct Uint32Be {
    std::array<uint8_t, 4> bytes{};
  };

  /** @brief 视觉发给电控的 40 字节哨兵控制帧。 */
  struct SendPacket {
    uint8_t header = 0xfe;                 // 偏移 0：帧头
    uint8_t fire_advice = 0;               // 偏移 1：开火建议
    Float32Be yaw;                         // 偏移 2～5：小 yaw，rad
    Float32Be pitch;                       // 偏移 6～9：pitch，rad，向下为正
    Float32Be distance;                    // 偏移 10～13：目标距离，m
    uint8_t id = 0;                        // 偏移 14：目标编号
    Float32Be target_v_yaw;                // 偏移 15～18：目标自转速度，rad/s
    uint8_t target_state = 0;              // 偏移 19：完整参考有效标志
    std::array<uint8_t, 2> reserved{};      // 偏移 20～21：保留
    Float32Be yaw_vel_rad_s;                // 偏移 22～25：小 yaw 规划速度
    Float32Be pitch_vel_rad_s;              // 偏移 26～29：pitch 规划速度
    Float32Be yaw_acc_rad_s2;               // 偏移 30～33：小 yaw 规划加速度
    Float32Be pitch_acc_rad_s2;             // 偏移 34～37：pitch 规划加速度
    uint8_t check_byte = 0;                 // 偏移 38：预留校验位
    uint8_t tail_byte = 0x0d;               // 偏移 39：帧尾
  };

  static_assert(sizeof(SendPacket) == 40);
  static_assert(offsetof(SendPacket, yaw) == 2);
  static_assert(offsetof(SendPacket, target_state) == 19);
  static_assert(offsetof(SendPacket, yaw_vel_rad_s) == 22);
  static_assert(offsetof(SendPacket, tail_byte) == 39);
  static_assert(std::is_standard_layout_v<SendPacket>);
  static_assert(std::is_trivially_copyable_v<SendPacket>);

  /** @brief 电控发给视觉的 36 字节哨兵反馈帧。 */
  struct ReceivePacket {
    uint8_t header = 0xfe;                  // 偏移 0：帧头
    uint8_t mode = 0;                       // 偏移 1：视觉模式
    Float32Be roll;                         // 偏移 2～5：roll，rad
    Float32Be pitch;                        // 偏移 6～9：pitch，rad
    Float32Be yaw_large;                    // 偏移 10～13：大 yaw，rad
    Float32Be yaw_small;                    // 偏移 14～17：小 yaw，rad
    Float32Be bullet_speed;                 // 偏移 18～21：弹速，m/s
    Uint32Be mcu_timestamp;                 // 偏移 22～25：MCU 时间戳，ms
    Float32Be yaw_actual_vel_rad_s;         // 偏移 26～29：小 yaw 实际速度
    Float32Be pitch_actual_vel_rad_s;       // 偏移 30～33：pitch 实际速度
    uint8_t check_byte = 0;                 // 偏移 34：预留校验位
    uint8_t tail_byte = 0x0d;               // 偏移 35：帧尾
  };

  static_assert(sizeof(ReceivePacket) == 36);
  static_assert(offsetof(ReceivePacket, roll) == 2);
  static_assert(offsetof(ReceivePacket, yaw_large) == 10);
  static_assert(offsetof(ReceivePacket, mcu_timestamp) == 22);
  static_assert(offsetof(ReceivePacket, tail_byte) == 35);
  static_assert(std::is_standard_layout_v<ReceivePacket>);
  static_assert(std::is_trivially_copyable_v<ReceivePacket>);

  explicit ProtocolSentry(std::string_view port_name, int speed, bool enable_data_print);

  ~ProtocolSentry() = default;

  void send(const rm_interfaces::msg::GimbalCmd &data) override;

  bool receive(rm_interfaces::msg::SerialReceiveData &data) override;

  /**
   * @brief 将 ROS 控制消息编码为字段明确的 40 字节线路帧。
   * @param data 同一规划时刻的小 yaw、pitch 及其导数。
   * @param id 目标编号，合法范围 0～9。
   * @return 大端哨兵控制帧；输入无效时返回安全无目标帧。
   */
  static SendPacket encode(const rm_interfaces::msg::GimbalCmd &data, uint8_t id);

  /**
   * @brief 校验并解码字段明确的 36 字节哨兵反馈帧。
   * @param packet 电控反馈的完整线路帧。
   * @param data 解码成功时更新的 ROS 反馈消息。
   * @return 帧头尾、模式和全部浮点字段有效时为 true。
   */
  static bool decode(const ReceivePacket &packet, rm_interfaces::msg::SerialReceiveData &data);

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(rclcpp::Node::SharedPtr node) override;

  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  std::string getErrorMessage() override { return packet_tool_->getErrorMessage(); }

private:
  FixedPacketTool<sizeof(ReceivePacket)>::SharedPtr packet_tool_;
  std::atomic<uint8_t> feedback_mode_{0};
};
}  // namespace qd::serial_driver::protocol

#endif  // SERIAL_DRIVER_SENTRY_PROTOCOL_HPP_
