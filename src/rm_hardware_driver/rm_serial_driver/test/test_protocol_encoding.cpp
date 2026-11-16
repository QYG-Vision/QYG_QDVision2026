#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rm_serial_driver/protocol.hpp"

namespace {
namespace wire = qd::serial_driver::wire;

class TestProtocol: public qd::serial_driver::protocol::Protocol {
public:
    using Protocol::decodeFeedback;
    using Protocol::encodeGimbalCommand;

    void send(const rm_interfaces::msg::GimbalCmd&) override {}
    bool receive(rm_interfaces::msg::SerialReceiveData&) override { return false; }
    std::vector<rclcpp::SubscriptionBase::SharedPtr>
    getSubscriptions(rclcpp::Node::SharedPtr) override { return {}; }
    std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr) const override { return {}; }
    std::string getErrorMessage() override { return {}; }
};

const wire::Frame FEEDBACK_BYTES{
    0xff, 0x01, 0xbe, 0x00, 0x00, 0x00, 0x3e, 0x80,
    0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x41, 0x80,
    0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 0x00, 0x0d
};

TEST(ProtocolEncoding, commandMatches24ByteBigEndianFloat32GoldenBytes) {
    TestProtocol protocol;
    rm_interfaces::msg::GimbalCmd command;
    command.yaw = 0.5;
    command.pitch = -0.25;
    command.distance = 1.25;
    command.target_v_yaw = 0.125;
    command.fire_advice = true;
    command.id = "3";
    const wire::Frame expected{
        0xff, 0x01, 0x3f, 0x00, 0x00, 0x00, 0xbe, 0x80,
        0x00, 0x00, 0x3f, 0xa0, 0x00, 0x00, 0x03, 0x3e,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0d
    };
    EXPECT_EQ(sizeof(wire::Frame), 24U);
    EXPECT_EQ(protocol.encodeGimbalCommand(command), expected);
}

TEST(ProtocolEncoding, preservesDirectValuesBeyondOldFixedPointRange) {
    TestProtocol protocol;
    rm_interfaces::msg::GimbalCmd command;
    command.yaw = 1000.125;
    command.pitch = -1000.125;
    command.distance = 0.001;
    command.id = "outpost";
    const auto frame = protocol.encodeGimbalCommand(command);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 2), 1000.125f);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 6), -1000.125f);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 10), 0.001f);
    EXPECT_EQ(frame[19], 1);
    EXPECT_EQ(frame[14], 6);
}

TEST(ProtocolEncoding, sanitizesIndividualFieldsAndPreservesIndependentFlags) {
    TestProtocol protocol;
    rm_interfaces::msg::GimbalCmd command;
    command.yaw = std::numeric_limits<double>::quiet_NaN();
    command.pitch = std::numeric_limits<double>::infinity();
    command.target_v_yaw = 1e300;
    command.distance = -1.0;
    command.fire_advice = true;
    command.id = "0";
    const auto frame = protocol.encodeGimbalCommand(command);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 2), 0.0f);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 6), 0.0f);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 15), 0.0f);
    EXPECT_FLOAT_EQ(wire::get_float(frame.data() + 10), -1.0f);
    EXPECT_EQ(frame[1], 1);
    EXPECT_EQ(frame[19], 0);
    command.distance = std::numeric_limits<double>::quiet_NaN();
    const auto invalid_distance = protocol.encodeGimbalCommand(command);
    EXPECT_FLOAT_EQ(wire::get_float(invalid_distance.data() + 10), 0.0f);
    EXPECT_EQ(invalid_distance[19], 0);
}

TEST(ProtocolDecoding, decodesComplete24ByteGoldenFeedback) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    ASSERT_TRUE(protocol.decodeFeedback(FEEDBACK_BYTES, data));
    EXPECT_FLOAT_EQ(data.roll, -0.125f);
    EXPECT_FLOAT_EQ(data.pitch, 0.25f);
    EXPECT_FLOAT_EQ(data.yaw, 1.0f);
    EXPECT_FLOAT_EQ(data.bullet_speed, 16.0f);
    EXPECT_EQ(data.mode, 1);
    EXPECT_EQ(data.mcu_timestamp, 0x12345678U);
    wire::Feedback feedback;
    feedback.roll = data.roll;
    feedback.pitch = data.pitch;
    feedback.yaw = data.yaw;
    feedback.bullet_speed = data.bullet_speed;
    feedback.mode = data.mode;
    feedback.mcu_timestamp = data.mcu_timestamp;
    EXPECT_EQ(wire::encode_feedback(feedback), FEEDBACK_BYTES);
}

TEST(ProtocolDecoding, rejectsInvalidFeedbackWithoutChangingPublishedDataAndRecovers) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    data.yaw = 9.0f;
    data.mode = 5;
    data.mcu_timestamp = 42;
    const auto expect_unchanged = [&]() {
        EXPECT_FLOAT_EQ(data.yaw, 9.0f);
        EXPECT_EQ(data.mode, 5);
        EXPECT_EQ(data.mcu_timestamp, 42U);
    };
    for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity()}) {
        for (const int offset : {2, 6, 10, 14}) {
            auto frame = FEEDBACK_BYTES;
            wire::put_float(frame.data() + offset, invalid);
            EXPECT_FALSE(protocol.decodeFeedback(frame, data));
            expect_unchanged();
        }
    }
    for (const int offset : {0, 1, 23}) {
        auto frame = FEEDBACK_BYTES;
        frame[offset] = offset == 1 ? 6 : 0;
        EXPECT_FALSE(protocol.decodeFeedback(frame, data));
        expect_unchanged();
    }
    ASSERT_TRUE(protocol.decodeFeedback(FEEDBACK_BYTES, data));
    EXPECT_FLOAT_EQ(data.yaw, 1.0f);
}

TEST(ProtocolDecoding, acceptsTimestampBoundariesAndAllDefinedModes) {
    for (const uint32_t timestamp : {0U, std::numeric_limits<uint32_t>::max()}) {
        for (uint8_t mode = 0; mode <= 5; ++mode) {
            wire::Feedback expected;
            expected.mcu_timestamp = timestamp;
            expected.mode = mode;
            wire::Feedback actual;
            ASSERT_TRUE(wire::decode_feedback(wire::encode_feedback(expected), actual));
            EXPECT_EQ(actual.mcu_timestamp, timestamp);
            EXPECT_EQ(actual.mode, mode);
        }
    }
}

} // namespace
