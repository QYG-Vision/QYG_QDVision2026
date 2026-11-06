#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <array>
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

bool Solver::plan_mpc(const ShootState& aim, const rm_interfaces::msg::Target& target,
                      const rclcpp::Time& now, rm_interfaces::msg::GimbalCmd& command) {
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
    if (feedback_->header.frame_id != target.header.frame_id) {
        return reject("frame_mismatch");
    }
    if (!std::isfinite(feedback_timeout_)
        || feedback_timeout_ <= 0.0 || !std::isfinite(target_timeout_)
        || target_timeout_ <= 0.0 || !std::isfinite(predict2send_delay_)
        || predict2send_delay_ < 0.0 || !std::isfinite(additional_prediction_time)
        || additional_prediction_time < 0.0) {
        return reject("invalid_mpc_config");
    }
    const double feedback_age = (now - rclcpp::Time(feedback_->header.stamp,
                                                    now.get_clock_type())).seconds();
    const double target_age = (now - rclcpp::Time(target.header.stamp,
                                                 now.get_clock_type())).seconds();
    if (feedback_age < 0.0 || feedback_age > feedback_timeout_) {
        return reject("stale_feedback");
    }
    if (target_age < 0.0 || target_age > target_timeout_) {
        return reject("stale_target");
    }
    if (!target.tracking || !std::isfinite(target.v_yaw) || !aim.planning_state_valid
        || !aim.planning_state.allFinite()
        || !std::isfinite(aim.aim_ypd.distance) || aim.aim_ypd.distance <= 0.0
        || !std::isfinite(aim.wait_duration) || aim.wait_duration < 0.0) {
        return reject("invalid_target");
    }

    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    const double lead = additional_prediction_time;
    auto hit_state = solve_impact_state(aim.planning_state,
                                       target_age + predict2send_delay_ + lead);
    hit_state = state_predict(hit_state, std::max(0.0, aim.wait_duration - lead));
    const auto point = state_to_armor_pos(hit_state);
    if (!point.allFinite() || point.norm() <= 0.0) {
        return reject("invalid_prediction");
    }
    const auto result = target_pos_to_aim_ypd(point);
    if (use_ballistic_compensation_
        && (!result.ballistic_debug || !result.ballistic_debug->ballistic_success)) {
        return reject("ballistic_failed");
    }
    const double yaw = result.aim_ypd.yaw * RAD_TO_DEG;
    const double pitch = -result.aim_ypd.pitch * RAD_TO_DEG;
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(pitch) > 327.67) {
        return reject("invalid_reference");
    }
    const double command_distance = point.norm();
    if (!std::isfinite(command_distance) || command_distance < 0.01) {
        return reject("invalid_target");
    }
    command.yaw = std::remainder(yaw, 360.0);
    command.pitch = pitch;
    command.distance = command_distance;
    command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 360.0);
    command.pitch_diff = command.pitch - feedback_->pitch;
    command.header.stamp = now + rclcpp::Duration::from_seconds(predict2send_delay_);
    command.reference_valid = true;
    command.reference_status = "valid_direct";
    if (use_acceleration_mpc_) {
        constexpr double STEP = 0.01;
        constexpr std::size_t HORIZON = 10;
        std::array<double, HORIZON> yaw_reference{};
        std::array<double, HORIZON> pitch_reference{};
        for (std::size_t i = 0; i < HORIZON; ++i) {
            const auto future_point = state_to_armor_pos(state_predict(hit_state, (i + 1) * STEP));
            if (!future_point.allFinite() || future_point.norm() <= 0.0) {
                return reject("invalid_prediction");
            }
            const auto future = target_pos_to_aim_ypd(future_point);
            if (use_ballistic_compensation_
                && (!future.ballistic_debug || !future.ballistic_debug->ballistic_success)) {
                return reject("ballistic_failed");
            }
            yaw_reference[i] = command.yaw_diff + std::remainder(
                future.aim_ypd.yaw * RAD_TO_DEG - command.yaw, 360.0);
            pitch_reference[i] = -future.aim_ypd.pitch * RAD_TO_DEG - feedback_->pitch;
            if (!std::isfinite(yaw_reference[i]) || !std::isfinite(pitch_reference[i])) {
                return reject("invalid_reference");
            }
        }

        // 每轴在预测窗口内使用一个常值加速度，最小化角度跟踪误差平方和。
        // 单变量二次问题的有界最优解可直接由无约束最优解限幅得到。
        const auto acceleration = [&](const auto& reference, double velocity) {
            double numerator = 0.0;
            double denominator = 0.0;
            for (std::size_t i = 0; i < reference.size(); ++i) {
                const double t = (i + 1) * STEP;
                const double gain = 0.5 * t * t;
                numerator += gain * (reference[i] - velocity * t);
                denominator += gain * gain;
            }
            return std::clamp(numerator / denominator,
                              -mpc_max_acceleration_, mpc_max_acceleration_);
        };
        const double yaw_velocity = feedback_->yaw_actual_vel_deg_s;
        const double pitch_velocity = feedback_->pitch_actual_vel_deg_s;
        const double yaw_acc = acceleration(yaw_reference, yaw_velocity);
        const double pitch_acc = acceleration(pitch_reference, pitch_velocity);
        command.yaw = std::remainder(
            feedback_->yaw + (yaw_velocity + 0.5 * yaw_acc * STEP) * STEP, 360.0);
        command.pitch = feedback_->pitch + (pitch_velocity + 0.5 * pitch_acc * STEP) * STEP;
        command.yaw_vel_deg_s = yaw_velocity + yaw_acc * STEP;
        command.pitch_vel_deg_s = pitch_velocity + pitch_acc * STEP;
        command.yaw_acc_deg_s2 = yaw_acc;
        command.pitch_acc_deg_s2 = pitch_acc;
        command.header.stamp = rclcpp::Time(command.header.stamp)
            + rclcpp::Duration::from_seconds(STEP);
        command.reference_status = "valid_mpc";
    }
    const double yaw_tolerance = std::atan2(shooting_range_w_ * 0.5
        * max_out_error[top_level], command_distance) * RAD_TO_DEG;
    const double pitch_tolerance = std::atan2(shooting_range_h_ * 0.5
        * max_out_error[top_level], command_distance) * RAD_TO_DEG;
    command.fire_advice = command.fire_advice
        && std::abs(command.yaw_diff) <= yaw_tolerance
        && std::abs(command.pitch_diff) <= pitch_tolerance;
    return true;
}
} // namespace qd::auto_aim
