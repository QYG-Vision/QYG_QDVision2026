#include "rm_serial_driver/protocol/qyg_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iterator>

namespace qd::serial_driver::protocol::qyg
{

uint16_t crc16(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x0001U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                                  : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

QygVisionMode getVisionMode(uint16_t sentry_state)
{
  return static_cast<QygVisionMode>(sentry_state & 0x0003U);
}

uint8_t mapToQdVisionMode(QygVisionMode mode, bool enemy_is_red)
{
  const uint8_t color_offset = enemy_is_red ? 0U : 1U;
  switch (mode) {
    case QygVisionMode::SMALL_BUFF:
      return static_cast<uint8_t>(2U + color_offset);
    case QygVisionMode::BIG_BUFF:
      return static_cast<uint8_t>(4U + color_offset);
    case QygVisionMode::IDLE:
    case QygVisionMode::AUTO_AIM:
    default:
      // QD 没有 IDLE 枚举，空闲时保持合法模式；控制使能在发送侧单独关闭。
      return color_offset;
  }
}

float degreesToRadians(float degrees)
{
  constexpr float DEG_TO_RAD = 3.14159265358979323846F / 180.0F;
  return degrees * DEG_TO_RAD;
}

GimbalFeedbackAngles decodeGimbalFeedback(const QygReceiveFrame& frame) {
    return { frame.vroll, frame.vpitch, frame.vyaw };
}

QygSendFrame makeSendFrame(
  bool control, bool fire, float yaw, float pitch, float linear_x, float linear_y, float angular_z)
{
  QygSendFrame frame;
  if (
    !std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(linear_x) ||
    !std::isfinite(linear_y) || !std::isfinite(angular_z)) {
    frame.crc16 = crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - 2);
    return frame;
  }

  constexpr float PI = 3.14159265358979323846F;
  frame.mode = control ? (fire ? 2U : 1U) : 0U;
  frame.yaw = std::clamp(yaw, -PI, PI);
  frame.pitch = std::clamp(pitch, -PI, PI);
  // 与当前 QYG 视觉仓库保持一致：底盘三个控制量限幅后取反。
  frame.linear_x = -std::clamp(linear_x, -1.0F, 1.0F);
  frame.linear_y = -std::clamp(linear_y, -1.0F, 1.0F);
  frame.angular_z = -std::clamp(angular_z, -1.0F, 1.0F);
  frame.crc16 = crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - 2);
  return frame;
}

std::optional<QygReceiveFrame> parseReceiveFrame(const uint8_t * data, size_t length)
{
  if (data == nullptr || length < sizeof(QygReceiveFrame) || data[0] != 'G' || data[1] != 'D') {
    return std::nullopt;
  }

  QygReceiveFrame frame;
  std::memcpy(&frame, data, sizeof(frame));
  const auto expected_crc = crc16(data, sizeof(frame) - 2);
  if (frame.crc16 != expected_crc) {
    return std::nullopt;
  }
  return frame;
}

void QygStreamParser::append(const uint8_t * data, size_t length)
{
  if (data == nullptr || length == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + length);
  if (buffer_.size() > 4096) {
    buffer_.erase(buffer_.begin(), buffer_.end() - 1024);
  }
}

std::optional<QygReceiveFrame> QygStreamParser::popFrame()
{
  constexpr std::array<uint8_t, 2> HEADER{'G', 'D'};
  while (buffer_.size() >= HEADER.size()) {
    const auto start = std::search(buffer_.begin(), buffer_.end(), HEADER.begin(), HEADER.end());
    if (start == buffer_.end()) {
      const bool keep_g = buffer_.back() == 'G';
      buffer_.clear();
      if (keep_g) {
        buffer_.push_back('G');
      }
      return std::nullopt;
    }

    buffer_.erase(buffer_.begin(), start);
    if (buffer_.size() < sizeof(QygReceiveFrame)) {
      return std::nullopt;
    }

    const auto frame = parseReceiveFrame(buffer_.data(), buffer_.size());
    if (frame.has_value()) {
      buffer_.erase(buffer_.begin(), std::next(buffer_.begin(), sizeof(QygReceiveFrame)));
      return frame;
    }
    // 假帧头或 CRC 错误时仅丢一个字节，然后重新搜索 GD。
    buffer_.erase(buffer_.begin());
  }
  return std::nullopt;
}

size_t QygStreamParser::bufferedSize() const { return buffer_.size(); }

}  // namespace qd::serial_driver::protocol::qyg
