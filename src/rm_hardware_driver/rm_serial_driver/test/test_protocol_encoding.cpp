#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/sentry_codec.hpp"
#include "rm_utils/time_synchronizer.hpp"

namespace {
namespace v2 = qd::serial_driver::v2;

class TestProtocol : public qd::serial_driver::protocol::Protocol {
public:
    using Protocol::encodeGimbalCommand;
    using Protocol::decodeFeedback;
    void send(const rm_interfaces::msg::GimbalCmd&) override {}
    bool receive(rm_interfaces::msg::SerialReceiveData&) override { return false; }
    std::vector<rclcpp::SubscriptionBase::SharedPtr>
    getSubscriptions(rclcpp::Node::SharedPtr) override { return {}; }
    std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr) const override { return {}; }
    std::string getErrorMessage() override { return {}; }
};

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

const v2::CommandFrame INVALID_COMMAND{
    0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xbf, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d
};
const v2::FeedbackFrame FEEDBACK{
    0xfe, 0x01, 0xbe, 0x00, 0x00, 0x00, 0x3e, 0x80,
    0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x41, 0x80,
    0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 0x3f, 0x00,
    0x00, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x0d
};

TEST(ProtocolEncoding, matchesIndependent40ByteGoldenCommand) {
    TestProtocol protocol;
    const v2::CommandFrame expected{
        0xfe, 0x01, 0x3f, 0x00, 0x00, 0x00, 0xbe, 0x80,
        0x00, 0x00, 0x3f, 0xa0, 0x00, 0x00, 0x03, 0x3e,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x3f, 0x80,
        0x00, 0x00, 0xbf, 0x80, 0x00, 0x00, 0x40, 0x00,
        0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x0d
    };
    EXPECT_EQ(sizeof(v2::CommandFrame), 40U);
    EXPECT_EQ(sizeof(v2::FeedbackFrame), 32U);
    EXPECT_EQ(protocol.encodeGimbalCommand(reference()), expected);
}

TEST(ProtocolEncoding, rejectsMissingPlanningAndNonpositiveOrUnderflowingDistance) {
    TestProtocol protocol;
    auto command = reference();
    command.reference_valid = false;
    EXPECT_EQ(protocol.encodeGimbalCommand(command), INVALID_COMMAND);
    for (const double distance : {0.0, -1.0, 1e-300}) {
        command = reference();
        command.distance = distance;
        EXPECT_EQ(protocol.encodeGimbalCommand(command), INVALID_COMMAND);
    }
}

TEST(ProtocolEncoding, preservesDirectFloatValuesAndSmallPositiveDistance) {
    TestProtocol protocol;
    auto command = reference();
    command.yaw = 1000.125;
    command.pitch = -1000.125;
    command.target_v_yaw = 10000.125;
    command.distance = 0.001;
    command.yaw_acc_rad_s2 = 1000.0f;
    const auto packet = protocol.encodeGimbalCommand(command);
    EXPECT_FLOAT_EQ(v2::getFloat(packet.data() + 2), 1000.125f);
    EXPECT_FLOAT_EQ(v2::getFloat(packet.data() + 6), -1000.125f);
    EXPECT_FLOAT_EQ(v2::getFloat(packet.data() + 10), 0.001f);
    EXPECT_FLOAT_EQ(v2::getFloat(packet.data() + 15), 10000.125f);
    EXPECT_FLOAT_EQ(v2::getFloat(packet.data() + 30), 1000.0f);
    EXPECT_EQ(packet[19], 1);
}

TEST(ProtocolEncoding, rejectsNonfiniteAndOverflowInEveryPhysicalField) {
    struct WideCommand {
        bool reference_valid = true;
        bool fire_advice = true;
        double yaw = 0, pitch = 0, distance = 1, target_v_yaw = 0;
        double yaw_vel_rad_s = 0, pitch_vel_rad_s = 0;
        double yaw_acc_rad_s2 = 0, pitch_acc_rad_s2 = 0;
    };
    const std::array<double WideCommand::*, 8> fields{
        &WideCommand::yaw, &WideCommand::pitch, &WideCommand::distance,
        &WideCommand::target_v_yaw, &WideCommand::yaw_vel_rad_s,
        &WideCommand::pitch_vel_rad_s, &WideCommand::yaw_acc_rad_s2,
        &WideCommand::pitch_acc_rad_s2
    };
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity(), 1e300, -1e300}) {
        for (const auto field : fields) {
            WideCommand command;
            command.*field = invalid;
            EXPECT_EQ(v2::encode(command, 1), INVALID_COMMAND);
        }
    }
    EXPECT_EQ(v2::encode(WideCommand{}, 10), INVALID_COMMAND);
}

TEST(ProtocolEncoding, decodes32ByteGoldenFeedbackAndRejectsWithoutMutation) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    ASSERT_TRUE(protocol.decodeFeedback(FEEDBACK, data));
    EXPECT_EQ(data.mode, 1);
    EXPECT_FLOAT_EQ(data.roll, -0.125f);
    EXPECT_FLOAT_EQ(data.pitch, 0.25f);
    EXPECT_FLOAT_EQ(data.yaw, 1.0f);
    EXPECT_FLOAT_EQ(data.bullet_speed, 16.0f);
    EXPECT_EQ(data.mcu_timestamp, 0x12345678U);
    EXPECT_FLOAT_EQ(data.yaw_actual_vel_rad_s, 0.5f);
    EXPECT_FLOAT_EQ(data.pitch_actual_vel_rad_s, -0.5f);
    const auto original = data;
    for (const auto index : {0, 1, 31}) {
        auto bad = FEEDBACK;
        bad[index] = 0xff;
        EXPECT_FALSE(protocol.decodeFeedback(bad, data));
        EXPECT_EQ(data, original);
    }
    for (const auto offset : {2, 6, 10, 14, 22, 26}) {
        for (const float invalid : {std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity(),
                                    std::numeric_limits<float>::quiet_NaN()}) {
            auto bad = FEEDBACK;
            v2::putFloat(bad.data() + offset, invalid);
            EXPECT_FALSE(protocol.decodeFeedback(bad, data));
            EXPECT_EQ(data, original);
        }
    }
    ASSERT_TRUE(protocol.decodeFeedback(FEEDBACK, data));
    EXPECT_EQ(data, original);
}

TEST(ProtocolEncoding, energyModesCannotSendValidReferences) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    for (uint8_t mode = 0; mode <= 5; ++mode) {
        auto frame = FEEDBACK;
        frame[1] = mode;
        ASSERT_TRUE(protocol.decodeFeedback(frame, data));
        const auto command = protocol.encodeGimbalCommand(reference());
        EXPECT_EQ(command[19], mode <= 1 ? 1 : 0);
        if (mode > 1) EXPECT_EQ(command, INVALID_COMMAND);
    }
}

TEST(ProtocolEncoding, acceptsTimestampZeroAndMaximum) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    for (const uint32_t timestamp : {0U, std::numeric_limits<uint32_t>::max()}) {
        auto frame = FEEDBACK;
        v2::put(frame.data() + 18, timestamp, 4);
        ASSERT_TRUE(protocol.decodeFeedback(frame, data));
        EXPECT_EQ(data.mcu_timestamp, timestamp);
    }
}

TEST(SentryProtocolEncoding, decodes36ByteFeedbackAndPreservesLargeYaw) {
    const qd::serial_driver::sentry::FeedbackFrame frame{
        0xfe, 0x01, 0xbe, 0x00, 0x00, 0x00, 0x3e, 0x80,
        0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x3f, 0x00,
        0x00, 0x00, 0x41, 0x80, 0x00, 0x00, 0x12, 0x34,
        0x56, 0x78, 0x3f, 0x00, 0x00, 0x00, 0xbf, 0x00,
        0x00, 0x00, 0x00, 0x0d
    };
    rm_interfaces::msg::SerialReceiveData data;
    ASSERT_TRUE(qd::serial_driver::sentry::decode(frame, data));
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
    invalid[10] = 0x7f;
    invalid[11] = 0xc0;
    EXPECT_FALSE(qd::serial_driver::sentry::decode(invalid, data));
    EXPECT_EQ(data, original);
}

TEST(V2TimeSynchronizer, retainsDuplicateTimeAndUnwrapsThroughZero) {
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

TEST(V2TimeSynchronizer, recoversAfterMcuTimestampRestart) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(1000000U, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(1000100U, 1000.1), 1000.1, 1e-6);

    // MCU 重启后时间戳从较大值回到零，连续几个新包后应重新锚定到 PC 时钟。
    EXPECT_DOUBLE_EQ(sync.sync(0U, 1000.2), 1000.1);
    EXPECT_DOUBLE_EQ(sync.sync(1U, 1000.3), 1000.1);
    EXPECT_NEAR(sync.sync(2U, 1000.4), 1000.4, 1e-6);
    EXPECT_NEAR(sync.sync(102U, 1000.5), 1000.5, 1e-6);
}

TEST(V2TimeSynchronizer, rejectsOneOutOfOrderPacketWithoutReset) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(0xfffffffeU, 1000.0), 1000.0);
    EXPECT_NEAR(sync.sync(0U, 1000.002), 1000.002, 1e-6);

    // 回绕后的旧包只应被丢弃，后续正常包仍按回绕后的序列继续同步。
    EXPECT_DOUBLE_EQ(sync.sync(0xffffffffU, 1000.003), 1000.002);
    EXPECT_NEAR(sync.sync(1U, 1000.003), 1000.003, 1e-6);
}

TEST(V2TimeSynchronizer, recoversFromLargeInRangeTimestampJump) {
    qd::utils::TimeSynchronizer sync;
    EXPECT_DOUBLE_EQ(sync.sync(3000000000U, 1000.0), 1000.0);

    // 重启后的新值仍可能落在半回绕范围内，需按 PC 时间重新锚定。
    EXPECT_NEAR(sync.sync(1000U, 1000.1), 1000.1, 1e-6);
    EXPECT_NEAR(sync.sync(1100U, 1000.2), 1000.2, 1e-6);
}
} // namespace
