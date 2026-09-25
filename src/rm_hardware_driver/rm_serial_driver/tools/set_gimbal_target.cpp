#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_serial_driver/gimbal_target_cli.hpp"

int main(int argc, char* argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    const auto target = qd::serial_driver::parseGimbalTargetArguments(arguments);
    if (!target.has_value()) {
        std::cerr << "Usage: ros2 run rm_serial_driver set_gimbal_target <yaw_deg> <pitch_deg>\n";
        return 2;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("set_gimbal_target");
    const std::string target_topic =
        node->declare_parameter("target_topic", "zero_order_gimbal/target");
    const auto publisher = node->create_publisher<rm_interfaces::msg::GimbalCmd>(
        target_topic,
        rclcpp::SensorDataQoS()
    );

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (rclcpp::ok() && publisher->get_subscription_count() == 0U
           && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (publisher->get_subscription_count() == 0U) {
        RCLCPP_ERROR(node->get_logger(), "No subscriber on %s", target_topic.c_str());
        rclcpp::shutdown();
        return 1;
    }

    rm_interfaces::msg::GimbalCmd message;
    message.yaw = target->yaw;
    message.pitch = target->pitch;
    message.fire_advice = false;
    publisher->publish(message);
    RCLCPP_INFO(
        node->get_logger(),
        "Published absolute target yaw=%.3f deg, pitch=%.3f deg",
        message.yaw,
        message.pitch
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
    return 0;
}
