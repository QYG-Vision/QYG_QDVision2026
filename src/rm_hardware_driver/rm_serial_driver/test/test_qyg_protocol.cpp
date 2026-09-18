#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>

#include "gtest/gtest.h"
#include "rm_serial_driver/protocol/qyg_protocol.hpp"

namespace qyg = qd::serial_driver::protocol::qyg;

TEST(QygProtocol, frameSizeAndOffsets)
{
  EXPECT_EQ(sizeof(qyg::QygSendFrame), 25U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, mode), 2U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, yaw), 3U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, pitch), 7U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, linear_x), 11U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, linear_y), 15U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, angular_z), 19U);
  EXPECT_EQ(offsetof(qyg::QygSendFrame, crc16), 23U);

  EXPECT_EQ(sizeof(qyg::QygReceiveFrame), 31U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, current_mode), 2U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, actual_vx), 3U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, actual_vy), 7U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, actual_wz), 11U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, sentry_state), 15U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, vyaw), 17U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, vpitch), 21U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, vroll), 25U);
  EXPECT_EQ(offsetof(qyg::QygReceiveFrame, crc16), 29U);
}

TEST(QygProtocol, officialCrcCheckValue)
{
  constexpr std::array<uint8_t, 9> DATA{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(qyg::crc16(DATA.data(), DATA.size()), 0x6F91);
}

TEST(QygProtocol, makeSendFrameUsesQygUnitsAndSigns)
{
  constexpr float PI = 3.14159265358979323846F;
  const auto frame = qyg::makeSendFrame(true, true, 4.0F, -4.0F, 0.25F, -0.5F, 2.0F);

  EXPECT_EQ(sizeof(frame), 25U);
  EXPECT_EQ(frame.header[0], 'Q');
  EXPECT_EQ(frame.header[1], 'Y');
  EXPECT_EQ(frame.mode, 2U);
  EXPECT_FLOAT_EQ(frame.yaw, PI);
  EXPECT_FLOAT_EQ(frame.pitch, -PI);
  EXPECT_FLOAT_EQ(frame.linear_x, -0.25F);
  EXPECT_FLOAT_EQ(frame.linear_y, 0.5F);
  EXPECT_FLOAT_EQ(frame.angular_z, -1.0F);
  EXPECT_EQ(frame.crc16, qyg::crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - 2));
}

TEST(QygProtocol, qdDegreesConvertToQygRadians)
{
  constexpr float PI = 3.14159265358979323846F;
  EXPECT_NEAR(qyg::degreesToRadians(180.0F), PI, 1e-6F);
  EXPECT_NEAR(qyg::degreesToRadians(30.0F), PI / 6.0F, 1e-6F);
  EXPECT_NEAR(qyg::degreesToRadians(-90.0F), -PI / 2.0F, 1e-6F);
}

TEST(QygProtocol, decodeGimbalFeedbackKeepsHeadUpPitchPositive) {
    qyg::QygReceiveFrame frame;
    frame.vroll = 1.5F;
    frame.vpitch = 10.0F;
    frame.vyaw = -20.0F;

    const auto angles = qyg::decodeGimbalFeedback(frame);

    EXPECT_FLOAT_EQ(angles.roll_degrees, 1.5F);
    EXPECT_FLOAT_EQ(angles.pitch_degrees, 10.0F);
    EXPECT_FLOAT_EQ(angles.yaw_degrees, -20.0F);
}

TEST(QygProtocol, qygModesMapToQdModesWithEnemyColor)
{
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::AUTO_AIM, true), 0U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::AUTO_AIM, false), 1U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::SMALL_BUFF, true), 2U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::SMALL_BUFF, false), 3U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::BIG_BUFF, true), 4U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::BIG_BUFF, false), 5U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::IDLE, true), 0U);
  EXPECT_EQ(qyg::mapToQdVisionMode(qyg::QygVisionMode::IDLE, false), 1U);
}

TEST(QygProtocol, makeSendFrameMatchesKnownBytes)
{
  constexpr std::array<uint8_t, 25> EXPECTED{
    0x51, 0x59, 0x01, 0x92, 0x0A, 0x06, 0x3F, 0xC2, 0xB8, 0xB2, 0xBD, 0xCD, 0xCC,
    0x4C, 0xBE, 0x9A, 0x99, 0x99, 0x3E, 0xCD, 0xCC, 0xCC, 0xBE, 0xE7, 0xA0,
  };
  const auto frame = qyg::makeSendFrame(
    true, false, qyg::degreesToRadians(30.0F), qyg::degreesToRadians(-5.0F), 0.2F, -0.3F, 0.4F);
  EXPECT_EQ(std::memcmp(&frame, EXPECTED.data(), EXPECTED.size()), 0);
}

TEST(QygProtocol, parseReceiveFrameChecksHeaderAndCrc)
{
  qyg::QygReceiveFrame frame;
  frame.current_mode = 7U;
  frame.actual_vx = 1.25F;
  frame.actual_vy = -2.5F;
  frame.actual_wz = 0.75F;
  frame.sentry_state = static_cast<uint16_t>((123U << 2U) | 3U);
  frame.vyaw = 12.5F;
  frame.vpitch = -3.25F;
  frame.vroll = 1.5F;
  frame.crc16 =
    qyg::crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - sizeof(frame.crc16));

  std::array<uint8_t, sizeof(frame)> bytes{};
  std::memcpy(bytes.data(), &frame, sizeof(frame));
  const auto parsed = qyg::parseReceiveFrame(bytes.data(), bytes.size());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(qyg::getVisionMode(parsed->sentry_state), qyg::QygVisionMode::BIG_BUFF);
  EXPECT_EQ(parsed->current_mode, 7U);
  EXPECT_FLOAT_EQ(parsed->actual_vx, 1.25F);
  EXPECT_FLOAT_EQ(parsed->actual_vy, -2.5F);
  EXPECT_FLOAT_EQ(parsed->actual_wz, 0.75F);
  EXPECT_EQ(parsed->sentry_state >> 2U, 123U);
  EXPECT_FLOAT_EQ(parsed->vyaw, 12.5F);
  EXPECT_FLOAT_EQ(parsed->vpitch, -3.25F);
  EXPECT_FLOAT_EQ(parsed->vroll, 1.5F);

  bytes[10] ^= 0x01U;
  EXPECT_FALSE(qyg::parseReceiveFrame(bytes.data(), bytes.size()).has_value());
}

TEST(QygProtocol, streamParserHandlesNoiseFragmentsAndBadCrc)
{
  qyg::QygReceiveFrame valid_frame;
  valid_frame.current_mode = 9U;
  valid_frame.actual_vx = 0.8F;
  valid_frame.sentry_state = 1U;
  valid_frame.vyaw = 45.0F;
  valid_frame.crc16 =
    qyg::crc16(reinterpret_cast<const uint8_t *>(&valid_frame), sizeof(valid_frame) - 2);

  auto bad_frame = valid_frame;
  bad_frame.actual_vx = -99.0F;
  // 故意不重新计算 CRC，使其成为错误帧。

  constexpr std::array<uint8_t, 5> NOISE{0x12, 0x47, 0x00, 0x44, 0xFF};
  qyg::QygStreamParser parser;
  parser.append(NOISE.data(), NOISE.size());
  parser.append(reinterpret_cast<const uint8_t *>(&bad_frame), sizeof(bad_frame));

  const auto * valid_bytes = reinterpret_cast<const uint8_t *>(&valid_frame);
  parser.append(valid_bytes, 17);
  EXPECT_FALSE(parser.popFrame().has_value());
  parser.append(valid_bytes + 17, sizeof(valid_frame) - 17);

  const auto parsed = parser.popFrame();
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->current_mode, 9U);
  EXPECT_FLOAT_EQ(parsed->actual_vx, 0.8F);
  EXPECT_FLOAT_EQ(parsed->vyaw, 45.0F);
  EXPECT_FALSE(parser.popFrame().has_value());
}

TEST(QygProtocol, streamParserHandlesStickyFrames)
{
  qyg::QygReceiveFrame first;
  first.current_mode = 1U;
  first.crc16 = qyg::crc16(reinterpret_cast<const uint8_t *>(&first), sizeof(first) - 2);
  auto second = first;
  second.current_mode = 2U;
  second.crc16 = qyg::crc16(reinterpret_cast<const uint8_t *>(&second), sizeof(second) - 2);

  qyg::QygStreamParser parser;
  parser.append(reinterpret_cast<const uint8_t *>(&first), sizeof(first));
  parser.append(reinterpret_cast<const uint8_t *>(&second), sizeof(second));
  const auto parsed_first = parser.popFrame();
  const auto parsed_second = parser.popFrame();
  ASSERT_TRUE(parsed_first.has_value());
  ASSERT_TRUE(parsed_second.has_value());
  EXPECT_EQ(parsed_first->current_mode, 1U);
  EXPECT_EQ(parsed_second->current_mode, 2U);
}
