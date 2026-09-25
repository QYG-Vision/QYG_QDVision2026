#include "rm_serial_driver/zero_order_gimbal_controller.hpp"

#include <utility>

namespace qd::serial_driver {

ZeroOrderGimbalController::ZeroOrderGimbalController(double distance, std::string id):
    distance_(distance),
    id_(std::move(id)) {}

void ZeroOrderGimbalController::updateFeedback(double yaw, double pitch) noexcept {
    has_feedback_ = true;
    if (has_explicit_target_) {
        return;
    }
    target_yaw_ = yaw;
    target_pitch_ = pitch;
}

void ZeroOrderGimbalController::updateTarget(double yaw, double pitch) noexcept {
    target_yaw_ = yaw;
    target_pitch_ = pitch;
    has_explicit_target_ = true;
}

std::optional<rm_interfaces::msg::GimbalCmd> ZeroOrderGimbalController::makeCommand() const {
    if (!has_feedback_) {
        return std::nullopt;
    }

    rm_interfaces::msg::GimbalCmd command;
    command.yaw = target_yaw_;
    command.pitch = target_pitch_;
    command.distance = distance_;
    command.id = id_;
    command.fire_advice = false;
    return command;
}

} // namespace qd::serial_driver
