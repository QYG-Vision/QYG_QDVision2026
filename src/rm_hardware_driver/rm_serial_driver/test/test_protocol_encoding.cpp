#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rm_serial_driver/protocol.hpp"
#include "gtest/gtest.h"

namespace {

class TestProtocol: public qd::serial_driver::protocol::Protocol {
public:
    using qd::serial_driver::protocol::Protocol::encodeGimbalCommand;

    void send(const rm_interfaces::msg::GimbalCmd&) override {}

    bool receive(rm_interfaces::msg::SerialReceiveData&) override {
        return false;
    }

    std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(rclcpp::Node::SharedPtr
    ) override {
        return {};
    }

    std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr) const override {
        return {};
    }

    std::string getErrorMessage() override {
        return {};
    }
};

TEST(ProtocolEncoding, setsTargetStateAndEncodesFixedPointFields) {
    TestProtocol protocol;
    rm_interfaces::msg::GimbalCmd command;
    command.fire_advice = true;
    command.yaw = 21.9;
    command.pitch = -11.5;
    command.distance = 1.25;
    command.id = "3";
    command.target_v_yaw = 1.23;

    const auto packet = protocol.encodeGimbalCommand(command);

    EXPECT_EQ(protocol.swap_bytes(packet.yaw), 2190);
    EXPECT_EQ(protocol.swap_bytes(packet.pitch), -1150);
    EXPECT_EQ(protocol.swap_bytes(packet.distance), 125);
    EXPECT_EQ(protocol.swap_bytes(packet.target_v_yaw), 123);
    EXPECT_EQ(packet.target_state, 1);
    EXPECT_EQ(packet.fire_advice, 1);
    EXPECT_EQ(packet.id, 3);
    EXPECT_EQ(packet.reserved4, 0);
    EXPECT_EQ(packet.reserved5, 0);
    EXPECT_EQ(packet.check_byte, 0);
}

TEST(ProtocolEncoding, marksMissingTargetAndSaturatesFields) {
    TestProtocol protocol;
    rm_interfaces::msg::GimbalCmd command;
    command.yaw = std::numeric_limits<double>::infinity();
    command.pitch = 1000.0;
    command.distance = -1.0;
    command.id = "0";

    const auto packet = protocol.encodeGimbalCommand(command);

    EXPECT_EQ(protocol.swap_bytes(packet.yaw), 0);
    EXPECT_EQ(protocol.swap_bytes(packet.pitch), std::numeric_limits<int16_t>::max());
    EXPECT_EQ(protocol.swap_bytes(packet.distance), -100);
    EXPECT_EQ(packet.target_state, 0);
}

} // namespace
