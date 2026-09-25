#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_serial_driver/zero_order_gimbal_controller.hpp"

namespace qd::serial_driver {

/**
 * @brief 发布固定绝对角度命令以测试云台零阶保持响应。
 */
class ZeroOrderGimbalTestNode final: public rclcpp::Node {
public:
    /**
     * @brief 创建零阶云台测试节点。
     * @param options ROS 2 节点选项。
     * @note 节点仅支持 `zero_order` 模型，且始终禁止开火。
     */
    explicit ZeroOrderGimbalTestNode(const rclcpp::NodeOptions& options):
        Node("zero_order_gimbal_test", options) {
        const std::string response_model = this->declare_parameter("response_model", "zero_order");
        const double publish_rate_hz = this->declare_parameter("publish_rate_hz", 250.0);
        const double command_distance = this->declare_parameter("command_distance", 0.0);
        const std::string target_id = this->declare_parameter("target_id", "0");
        target_frame_ = this->declare_parameter("target_frame", "odom");
        const std::string feedback_topic =
            this->declare_parameter("topics.feedback", "serial/receive");
        const std::string target_topic =
            this->declare_parameter("topics.target", "zero_order_gimbal/target");
        const std::string command_topic =
            this->declare_parameter("topics.command", "armor_solver/cmd_gimbal");

        this->declare_parameter("first_order.time_constant_s", 0.05);
        this->declare_parameter("first_order.max_rate_deg_s", 360.0);

        if (response_model != "zero_order") {
            throw std::invalid_argument("Only response_model=zero_order is implemented");
        }
        if (publish_rate_hz <= 0.0) {
            throw std::invalid_argument("publish_rate_hz must be positive");
        }
        if (command_distance < 0.0) {
            throw std::invalid_argument("command_distance must be non-negative for QYG control mode"
            );
        }

        controller_ = std::make_unique<ZeroOrderGimbalController>(command_distance, target_id);
        command_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
            command_topic,
            rclcpp::SensorDataQoS()
        );
        feedback_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
            feedback_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ZeroOrderGimbalTestNode::feedbackCallback, this, std::placeholders::_1)
        );
        target_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
            target_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ZeroOrderGimbalTestNode::targetCallback, this, std::placeholders::_1)
        );

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz)
        );
        publish_timer_ = this->create_wall_timer(
            period,
            std::bind(&ZeroOrderGimbalTestNode::publishCommand, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Zero-order gimbal test ready: waiting for %s before publishing to %s",
            feedback_topic.c_str(),
            command_topic.c_str()
        );
    }

private:
    /**
     * @brief 使用首帧有效反馈初始化保持目标。
     * @param message 电控回传的云台角度，单位为度。
     */
    void feedbackCallback(const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr message) {
        if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch)) {
            RCLCPP_WARN(this->get_logger(), "Ignore non-finite gimbal feedback");
            return;
        }

        controller_->updateFeedback(message->yaw, message->pitch);
        if (!received_first_feedback_) {
            received_first_feedback_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Holding first feedback yaw=%.3f deg, pitch=%.3f deg",
                message->yaw,
                message->pitch
            );
        }
    }

    /**
     * @brief 接收终端下发的世界坐标系绝对 yaw/pitch 目标。
     * @param message 测试目标，仅读取 yaw 和 pitch，单位为度。
     */
    void targetCallback(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr message) {
        if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch)) {
            RCLCPP_WARN(this->get_logger(), "Ignore non-finite gimbal target");
            return;
        }

        controller_->updateTarget(message->yaw, message->pitch);
        RCLCPP_INFO(
            this->get_logger(),
            "Updated absolute target yaw=%.3f deg, pitch=%.3f deg",
            message->yaw,
            message->pitch
        );
    }

    /**
     * @brief 以配置频率发布保持命令，并强制禁止开火。
     */
    void publishCommand() {
        const auto command = controller_->makeCommand();
        if (!command.has_value()) {
            return;
        }

        auto message = command.value();
        message.header.stamp = this->now();
        message.header.frame_id = target_frame_;
        command_pub_->publish(message);
    }

    std::unique_ptr<ZeroOrderGimbalController> controller_;
    std::string target_frame_;
    bool received_first_feedback_ { false };
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr command_pub_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr feedback_sub_;
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace qd::serial_driver

RCLCPP_COMPONENTS_REGISTER_NODE(qd::serial_driver::ZeroOrderGimbalTestNode)
