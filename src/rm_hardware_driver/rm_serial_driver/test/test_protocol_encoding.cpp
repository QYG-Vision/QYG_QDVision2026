#include <array>
#include <cstdint>
#include <cstring>
#include <limits>

#include "gtest/gtest.h"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include "rm_utils/time_synchronizer.hpp"

namespace {
using ProtocolSentry = qd::serial_driver::protocol::ProtocolSentry;

template <typename Packet>
std::array<uint8_t, sizeof(Packet)> raw_bytes(const Packet &packet) {
    std::array<uint8_t, sizeof(Packet)> bytes{};
    std::memcpy(bytes.data(), &packet, bytes.size());
    return bytes;
}

float read_float32_be(const ProtocolSentry::Float32Be &field) {
    uint32_t bits = 0;
    for (const uint8_t byte : field.bytes) {
        bits = (bits << 8) | byte;
    }
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
 * @brief 构造各轴同一时刻、以弧度及其导数表示的完整规划参考。
 * @return 用于独立字节样例核对的有效规划消息。
 */
rm_interfaces::msg::GimbalCmd reference() {
    rm_interfaces::msg::GimbalCmd command;
    command.reference_valid = true;
    command.fire_advice = true;
    command.yaw = 0.5;
    command.pitch = -0.25;
    command.distance = 1.25;
    command.id = "3";
    command.target_v_yaw = 0.125;
    command.yaw_vel_rad_s = 1.0f;
    command.pitch_vel_rad_s = -1.0f;
    command.yaw_acc_rad_s2 = 2.0f;
    command.pitch_acc_rad_s2 = -2.0f;
    return command;
}

const std::array<uint8_t, 40> INVALID_COMMAND{
    0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xbf, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d
};
TEST(SentryCommandEncoding, matchesIndependent40ByteGoldenCommand) {
    const std::array<uint8_t, 40> expected{
        0xfe, 0x01, 0x3f, 0x00, 0x00, 0x00, 0xbe, 0x80,
        0x00, 0x00, 0x3f, 0xa0, 0x00, 0x00, 0x03, 0x3e,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x3f, 0x80,
        0x00, 0x00, 0xbf, 0x80, 0x00, 0x00, 0x40, 0x00,
        0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x0d
    };
    EXPECT_EQ(sizeof(ProtocolSentry::SendPacket), 40U);
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(reference(), 3)), expected);
}

TEST(SentryCommandEncoding, rejectsMissingPlanningAndNonpositiveOrUnderflowingDistance) {
    auto command = reference();
    command.reference_valid = false;
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 3)), INVALID_COMMAND);
    for (const double distance : {0.0, -1.0, 1e-300}) {
        command = reference();
        command.distance = distance;
        EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 3)), INVALID_COMMAND);
    }
}

TEST(SentryCommandEncoding, preservesDirectFloatValuesAndSmallPositiveDistance) {
    auto command = reference();
    command.yaw = 1000.125;
    command.pitch = -1000.125;
    command.target_v_yaw = 10000.125;
    command.distance = 0.001;
    command.yaw_acc_rad_s2 = 1000.0f;
    const auto packet = ProtocolSentry::encode(command, 3);
    EXPECT_FLOAT_EQ(read_float32_be(packet.yaw), 1000.125f);
    EXPECT_FLOAT_EQ(read_float32_be(packet.pitch), -1000.125f);
    EXPECT_FLOAT_EQ(read_float32_be(packet.distance), 0.001f);
    EXPECT_FLOAT_EQ(read_float32_be(packet.target_v_yaw), 10000.125f);
    EXPECT_FLOAT_EQ(read_float32_be(packet.yaw_acc_rad_s2), 1000.0f);
    EXPECT_EQ(packet.target_state, 1);
}

TEST(SentryCommandEncoding, rejectsNonfiniteAndOverflowInEveryPhysicalField) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity(), 1e300, -1e300}) {
        auto command = reference();
        command.yaw = invalid;
        EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
        command = reference();
        command.pitch = invalid;
        EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
        command = reference();
        command.distance = invalid;
        EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
        command = reference();
        command.target_v_yaw = invalid;
        EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
    }
    auto command = reference();
    command.yaw_vel_rad_s = std::numeric_limits<float>::infinity();
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
    command = reference();
    command.pitch_vel_rad_s = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
    command = reference();
    command.yaw_acc_rad_s2 = -std::numeric_limits<float>::infinity();
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
    command = reference();
    command.pitch_acc_rad_s2 = std::numeric_limits<float>::infinity();
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(command, 1)), INVALID_COMMAND);
    EXPECT_EQ(raw_bytes(ProtocolSentry::encode(reference(), 10)), INVALID_COMMAND);
}

TEST(SentryProtocolEncoding, decodes36ByteFeedbackAndPreservesLargeYaw) {
    const std::array<uint8_t, 36> bytes{
        0xfe, 0x01, 0xbe, 0x00, 0x00, 0x00, 0x3e, 0x80,
        0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x3f, 0x00,
        0x00, 0x00, 0x41, 0x80, 0x00, 0x00, 0x12, 0x34,
        0x56, 0x78, 0x3f, 0x00, 0x00, 0x00, 0xbf, 0x00,
        0x00, 0x00, 0x00, 0x0d
    };
    ProtocolSentry::ReceivePacket frame;
    std::memcpy(&frame, bytes.data(), bytes.size());
    rm_interfaces::msg::SerialReceiveData data;
    ASSERT_TRUE(ProtocolSentry::decode(frame, data));
    EXPECT_EQ(data.mode, 1);
    EXPECT_FLOAT_EQ(data.roll, -0.125f);
    EXPECT_FLOAT_EQ(data.pitch, 0.25f);
    EXPECT_FLOAT_EQ(data.yaw_large, 1.0f);
    EXPECT_FLOAT_EQ(data.yaw_small, 0.5f);
    EXPECT_FLOAT_EQ(data.yaw, data.yaw_small);
    EXPECT_FLOAT_EQ(data.bullet_speed, 16.0f);
    EXPECT_EQ(data.mcu_timestamp, 0x12345678U);
    EXPECT_FLOAT_EQ(data.yaw_actual_vel_rad_s, 0.5f);
    EXPECT_FLOAT_EQ(data.pitch_actual_vel_rad_s, -0.5f);
    EXPECT_TRUE(data.dual_yaw_feedback);

    const auto original = data;
    auto invalid = frame;
    invalid.yaw_large.bytes = {0x7f, 0xc0, 0x00, 0x00};
    EXPECT_FALSE(ProtocolSentry::decode(invalid, data));
    EXPECT_EQ(data, original);
}

TEST(SentryTimeSynchronizer, retainsDuplicateTimeAndUnwrapsThroughZero) {
    qd::utils::TimeSynchronizer sync;
    const auto first = sync.sync(0xfffffffeU, 1000.0);
    EXPECT_DOUBLE_EQ(sync.sync(0xfffffffeU, 1001.0), first);
    const auto wrapped = sync.sync(0, 1000.002);
    EXPECT_NEAR(wrapped, 1000.002, 1e-6);
    EXPECT_DOUBLE_EQ(sync.sync(0, 1002.0), wrapped);
    EXPECT_DOUBLE_EQ(sync.sync(0xffffffffU, 1003.0), wrapped);
    EXPECT_NEAR(sync.sync(1, 1000.003), 1000.003, 1e-6);
    sync.reset();
    EXPECT_DOUBLE_EQ(sync.sync(0, 2000.0), 2000.0);
    EXPECT_DOUBLE_EQ(sync.sync(0, 2001.0), 2000.0);
}

TEST(SentryTimeSynchronizer, recoversAfterMcuTimestampRestart) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(1000000U, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(1000100U, 1000.1), 1000.1, 1e-6);

    // MCU 重启后时间戳从较大值回到零，连续几个新包后应重新锚定到 PC 时钟。
    EXPECT_DOUBLE_EQ(sync.sync(0U, 1000.2), 1000.1);
    EXPECT_DOUBLE_EQ(sync.sync(1U, 1000.3), 1000.1);
    EXPECT_NEAR(sync.sync(2U, 1000.4), 1000.4, 1e-6);
    EXPECT_NEAR(sync.sync(102U, 1000.5), 1000.5, 1e-6);
}

TEST(SentryTimeSynchronizer, rejectsOneOutOfOrderPacketWithoutReset) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(0xfffffffeU, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(0U, 1000.002), 1000.002, 1e-6);

    // 回绕后的旧包只应被丢弃，后续正常包仍按回绕后的序列继续同步。
    EXPECT_DOUBLE_EQ(sync.sync(0xffffffffU, 1000.003), 1000.002);
    EXPECT_NEAR(sync.sync(1U, 1000.003), 1000.003, 1e-6);
}

TEST(SentryTimeSynchronizer, recoversFromLargeInRangeTimestampJump) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(3000000000U, 1000.0), 1000.0);

    // 重启后的新值仍可能落在半回绕范围内，需按 PC 时间重新锚定。
    EXPECT_NEAR(sync.sync(1000U, 1000.1), 1000.1, 1e-6);
    EXPECT_NEAR(sync.sync(1100U, 1000.2), 1000.2, 1e-6);
}
} // namespace
