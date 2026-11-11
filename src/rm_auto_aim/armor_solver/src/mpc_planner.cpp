#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>

namespace qd::auto_aim {
void Solver::update_feedback(const rm_interfaces::msg::SerialReceiveData& feedback) {
    if (!std::isfinite(feedback.yaw) || !std::isfinite(feedback.pitch)
        || !std::isfinite(feedback.yaw_actual_vel_deg_s)
        || !std::isfinite(feedback.pitch_actual_vel_deg_s) || feedback.mode > 5) {
        feedback_.reset();
        return;
    }
    if (feedback_ && rclcpp::Time(feedback.header.stamp)
                         < rclcpp::Time(feedback_->header.stamp)) {
        return;
    }
    feedback_ = feedback;
}

bool Solver::plan_mpc(const AimReference& reference, const rclcpp::Time& now,
                      rm_interfaces::msg::GimbalCmd& command) {
    command.reference_valid = false;
    command.yaw_vel_deg_s = 0.0f;
    command.pitch_vel_deg_s = 0.0f;
    command.yaw_acc_deg_s2 = 0.0f;
    command.pitch_acc_deg_s2 = 0.0f;
    const auto reject = [&](const char* reason) {
        command.reference_status = reason;
        return false;
    };
    if (!mpc_enabled_) {
        return reject("mpc_disabled");
    }
    if (!feedback_) {
        return reject("no_feedback");
    }
    if (feedback_->mode > 1) {
        return reject("not_armor_mode");
    }
    if (feedback_->header.frame_id != reference.header.frame_id) {
        return reject("frame_mismatch");
    }
    if (mpc_config_.horizon < 3 || mpc_config_.horizon > 100
        || !std::isfinite(mpc_config_.dt) || mpc_config_.dt <= 0.0
        || !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0.0
        || !std::isfinite(target_timeout_)
        || target_timeout_ <= 0.0 || !std::isfinite(predict2send_delay_)
        || predict2send_delay_ < 0.0 || !std::isfinite(additional_prediction_time)
        || additional_prediction_time < 0.0) {
        return reject("invalid_mpc_config");
    }
    const double feedback_age = (now - rclcpp::Time(feedback_->header.stamp,
                                                    now.get_clock_type())).seconds();
    const double target_age = (now - rclcpp::Time(reference.header.stamp,
                                                 now.get_clock_type())).seconds();
    if (feedback_age < 0.0 || feedback_age > feedback_timeout_) {
        return reject("stale_feedback");
    }
    if (target_age < 0.0 || target_age > target_timeout_) {
        return reject("stale_target");
    }
    if (!reference.valid) return reject(reference.status.c_str());
    if (reference.yaw.size() != static_cast<size_t>(mpc_config_.horizon)
        || reference.pitch.size() != reference.yaw.size()
        || reference.output_index != mpc_config_.horizon / 2
        || reference.dt != mpc_config_.dt
        || !std::isfinite(reference.distance) || reference.distance < 0.01) {
        return reject("invalid_reference");
    }
    const auto yaw_plan = yaw_mpc_->plan(reference.yaw, &yaw_mpc_debug_.emplace());
    if (!yaw_plan) {
        return reject("yaw_mpc_failed");
    }
    const auto pitch_plan = pitch_mpc_->plan(reference.pitch, &pitch_mpc_debug_.emplace());
    if (!pitch_plan) {
        return reject("pitch_mpc_failed");
    }
    if (std::abs(pitch_plan->position) > 327.67) {
        return reject("pitch_out_of_wire_range");
    }

    command.yaw = std::remainder(yaw_plan->position, 360.0);
    command.pitch = pitch_plan->position;
    command.yaw_vel_deg_s = static_cast<float>(yaw_plan->velocity);
    command.pitch_vel_deg_s = static_cast<float>(pitch_plan->velocity);
    command.yaw_acc_deg_s2 = static_cast<float>(yaw_plan->acceleration);
    command.pitch_acc_deg_s2 = static_cast<float>(pitch_plan->acceleration);
    if (!std::isfinite(command.yaw_vel_deg_s) || !std::isfinite(command.pitch_vel_deg_s)
        || !std::isfinite(command.yaw_acc_deg_s2) || !std::isfinite(command.pitch_acc_deg_s2)) {
        return reject("wire_float_overflow");
    }
    command.distance = reference.distance;
    command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 360.0);
    command.pitch_diff = command.pitch - feedback_->pitch;
    command.header.stamp = now + rclcpp::Duration::from_seconds(predict2send_delay_);
    command.reference_valid = true;
    command.reference_status = "valid_mpc";
    return true;
}
} // namespace qd::auto_aim
