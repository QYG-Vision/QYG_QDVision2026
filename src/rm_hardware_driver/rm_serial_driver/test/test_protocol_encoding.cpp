#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rm_serial_driver/protocol.hpp"
#include "rm_utils/time_synchronizer.hpp"

namespace {
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

rm_interfaces::msg::GimbalCmd reference() {
    rm_interfaces::msg::GimbalCmd command;
    command.reference_valid = true;
    command.fire_advice = true;
    command.yaw = 21.9;
    command.pitch = -11.5;
    command.distance = 1.25;
    command.id = "3";
    command.target_v_yaw = 1.23;
    command.yaw_vel_deg_s = 1.0f;
    command.pitch_vel_deg_s = -1.0f;
    command.yaw_acc_deg_s2 = 1000.0f;
    command.pitch_acc_deg_s2 = -1000.0f;
    return command;
}

TEST(ProtocolEncoding, matchesIndependentGoldenCommand) {
    TestProtocol protocol;
    const auto packet = protocol.encodeGimbalCommand(reference());
    const qd::serial_driver::v2::CommandFrame expected{
        0xfe, 1, 0x08, 0x8e, 0xfb, 0x82, 0, 0x7d, 3, 0, 0x7b, 1, 0, 0,
        0x3f, 0x80, 0, 0, 0xbf, 0x80, 0, 0,
        0x44, 0x7a, 0, 0, 0xc4, 0x7a, 0, 0, 0, 0x0d};
    EXPECT_EQ(packet, expected);
}

TEST(ProtocolEncoding, rejectsMissingPlanningAndInvalidFields) {
    TestProtocol protocol;
    auto command = reference();
    command.reference_valid = false;
    const qd::serial_driver::v2::CommandFrame invalid{
        0xfe, 0, 0, 0, 0, 0, 0xff, 0x9c, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0d};
    EXPECT_EQ(protocol.encodeGimbalCommand(command), invalid);
    command = reference();
    command.yaw_acc_deg_s2 = std::numeric_limits<float>::infinity();
    EXPECT_EQ(protocol.encodeGimbalCommand(command), invalid);
    command = reference();
    command.pitch = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(protocol.encodeGimbalCommand(command), invalid);
    command = reference();
    command.distance = 0;
    EXPECT_EQ(protocol.encodeGimbalCommand(command), invalid);
    command.distance = 0.001;
    EXPECT_EQ(protocol.encodeGimbalCommand(command), invalid);
}

TEST(ProtocolEncoding, truncatesAndSaturatesFixedPointWithoutSaturatingDerivatives) {
    TestProtocol protocol;
    auto command = reference();
    command.yaw = 1000;
    command.pitch = -1000;
    command.distance = 1.239;
    const auto packet = protocol.encodeGimbalCommand(command);
    EXPECT_EQ(packet[2], 0x7f);
    EXPECT_EQ(packet[3], 0xff);
    EXPECT_EQ(packet[4], 0x80);
    EXPECT_EQ(packet[5], 0);
    EXPECT_EQ(packet[7], 123);
    EXPECT_FLOAT_EQ(qd::serial_driver::v2::getFloat(packet.data() + 22), 1000.0f);
}

TEST(ProtocolEncoding, rejectsDerivativeFloatOverflowBeforeConversion) {
    struct WideCommand {
        bool reference_valid = true;
        bool fire_advice = true;
        double yaw = 0, pitch = 0, distance = 1, target_v_yaw = 0;
        double yaw_vel_deg_s = 1e300, pitch_vel_deg_s = 0;
        double yaw_acc_deg_s2 = 0, pitch_acc_deg_s2 = 0;
    } command;
    const auto packet = qd::serial_driver::v2::encode(command, 1);
    EXPECT_EQ(packet[11], 0);
    EXPECT_EQ(packet[1], 0);
}

qd::serial_driver::v2::FeedbackFrame feedback() {
    return {0xfe, 1, 0xff, 0x9c, 0, 0xc8, 0x46, 0x50, 0x06, 0x40,
            0x12, 0x34, 0x56, 0x78, 0x3f, 0x80, 0, 0, 0xbf, 0x80, 0, 0, 0, 0x0d};
}

TEST(ProtocolEncoding, decodesGoldenFeedbackAndRejectsWithoutMutation) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    ASSERT_TRUE(protocol.decodeFeedback(feedback(), data));
    EXPECT_EQ(data.mode, 1);
    EXPECT_FLOAT_EQ(data.roll, -1);
    EXPECT_FLOAT_EQ(data.pitch, 2);
    EXPECT_FLOAT_EQ(data.yaw, 180);
    EXPECT_FLOAT_EQ(data.bullet_speed, 16);
    EXPECT_EQ(data.mcu_timestamp, 0x12345678U);
    EXPECT_FLOAT_EQ(data.yaw_actual_vel_deg_s, 1);
    EXPECT_FLOAT_EQ(data.pitch_actual_vel_deg_s, -1);
    const auto original = data;
    for (const auto index : {0, 1, 23}) {
        auto bad = feedback();
        bad[index] = 0xff;
        EXPECT_FALSE(protocol.decodeFeedback(bad, data));
        EXPECT_EQ(data, original);
    }
    for (const auto offset : {14, 18}) {
        for (const float bad_value : {std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::quiet_NaN()}) {
            auto bad = feedback();
            qd::serial_driver::v2::putFloat(bad.data() + offset, bad_value);
            EXPECT_FALSE(protocol.decodeFeedback(bad, data));
            EXPECT_EQ(data, original);
        }
    }
}

TEST(ProtocolEncoding, energyModesCannotSendValidReferences) {
    TestProtocol protocol;
    rm_interfaces::msg::SerialReceiveData data;
    for (uint8_t mode = 0; mode <= 5; ++mode) {
        auto frame = feedback();
        frame[1] = mode;
        ASSERT_TRUE(protocol.decodeFeedback(frame, data));
        EXPECT_EQ(protocol.encodeGimbalCommand(reference())[11], mode <= 1 ? 1 : 0);
    }
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
} // namespace
