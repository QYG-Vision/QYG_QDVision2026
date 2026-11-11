#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

bool Solver::plan_mpc(const rm_interfaces::msg::Target& target,
                      const rclcpp::Time& now, rm_interfaces::msg::GimbalCmd& command) {
    command.reference_valid = false;
    command.fire_advice = false;
    const auto reject = [&](const char* reason) {
        command.reference_status = reason;
        return false;
    };
    if (!mpc_enabled_) return reject("mpc_disabled");
    if (mpc_config_.horizon != 100 || mpc_config_.dt != 0.01
        || !std::isfinite(fire_thresh_) || fire_thresh_ <= 0.0) {
        return reject("invalid_mpc_config");
    }
    const double age = (now - rclcpp::Time(target.header.stamp,
                                         now.get_clock_type())).seconds();
    if (!target.tracking || !std::isfinite(age) || age < 0.0) {
        return reject("invalid_target");
    }
    auto candidates = get_armors_state(RosTarget(target));
    if (candidates.empty()) return reject("invalid_target");
    for (const auto& state: candidates) {
        if (!state.allFinite() || state[8] <= 0.0) return reject("invalid_target");
    }
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    const int half = mpc_config_.horizon / 2;
    // 保留奇点图像年龄、发送延迟、额外前移及飞行时间求解；不叠加旧等板时间。
    for (auto& state: candidates) {
        state = solve_impact_state(state, age + predict2send_delay_
                                          + additional_prediction_time);
    }
    // 与济瞄 aim() 一致：每个时刻按水平距离选择最近板，再使用奇点弹道。
    const auto aim = [&](const std::vector<State>& states) {
        size_t selected = 0;
        double min_distance = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < states.size(); ++i) {
            const double distance = state_to_armor_pos(states[i]).head<2>().norm();
            if (distance < min_distance) {
                selected = i;
                min_distance = distance;
            }
        }
        const auto point = state_to_armor_pos(states[selected]);
        if (!point.allFinite() || point.norm() < 0.01) {
            throw std::runtime_error("invalid_prediction");
        }
        const auto result = target_pos_to_aim_ypd(point);
        if (use_ballistic_compensation_
            && (!result.ballistic_debug || !result.ballistic_debug->ballistic_success)) {
            throw std::runtime_error("ballistic_failed");
        }
        return std::make_pair(result, point.norm());
    };
    const auto advance = [&](double dt) {
        for (auto& state: candidates) state = state_predict(state, dt);
    };
    std::vector<double> yaw, pitch, yaw_velocity, pitch_velocity;
    yaw.reserve(100);
    pitch.reserve(100);
    yaw_velocity.reserve(100);
    pitch_velocity.reserve(100);
    double yaw0;
    try {
        yaw0 = aim(candidates).first.aim_ypd.yaw;
        // 原样采用济瞄 get_trajectory() 的回退和中心差分采样顺序。
        advance(-mpc_config_.dt * (half + 1));
        auto last = aim(candidates);
        advance(mpc_config_.dt);
        auto current = aim(candidates);
        for (int k = 0; k < mpc_config_.horizon; ++k) {
            advance(mpc_config_.dt);
            const auto next = aim(candidates);
            yaw.push_back(reduced_angle(current.first.aim_ypd.yaw - yaw0) * RAD_TO_DEG);
            pitch.push_back(-current.first.aim_ypd.pitch * RAD_TO_DEG);
            yaw_velocity.push_back(reduced_angle(next.first.aim_ypd.yaw
                - last.first.aim_ypd.yaw) / (2 * mpc_config_.dt) * RAD_TO_DEG);
            pitch_velocity.push_back(-(next.first.aim_ypd.pitch
                - last.first.aim_ypd.pitch) / (2 * mpc_config_.dt) * RAD_TO_DEG);
            if (k == half) {
                command.distance = current.second;
                command.exp_yaw = reduced_angle(current.first.aim_ypd.yaw) * RAD_TO_DEG;
                command.exp_pitch = pitch.back();
                command.yaw_vel = yaw_velocity.back();
                command.pitch_vel = pitch_velocity.back();
                ballistic_debug_ = current.first.ballistic_debug;
            }
            last = current;
            current = next;
        }
    } catch (const std::runtime_error& error) {
        return reject(error.what());
    }
    const auto yaw_plan = yaw_mpc_->plan(yaw, &yaw_velocity);
    const auto pitch_plan = pitch_mpc_->plan(pitch, &pitch_velocity);
    if (!yaw_plan || !pitch_plan) return reject("invalid_mpc_output");
    command.yaw = std::remainder(yaw_plan->position + yaw0 * RAD_TO_DEG, 360.0);
    command.pitch = pitch_plan->position;
    command.yaw_vel_deg_s = yaw_plan->velocity;
    command.pitch_vel_deg_s = pitch_plan->velocity;
    command.yaw_acc_deg_s2 = yaw_plan->acceleration;
    command.pitch_acc_deg_s2 = pitch_plan->acceleration;
    // 与济瞄 plan.fire 一致：中点后2拍的参考与规划误差，阈值单位 rad。
    const int fire_index = half + 2;
    command.fire_advice = std::hypot(
        yaw[fire_index] - yaw_mpc_->sample_at(fire_index).position,
        pitch[fire_index] - pitch_mpc_->sample_at(fire_index).position)
        / RAD_TO_DEG < fire_thresh_;
    // 实际反馈仅用于误差诊断，不作为规划或开火门槛。
    if (feedback_) {
        command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 360.0);
        command.pitch_diff = command.pitch - feedback_->pitch;
    }
    command.header.stamp = now + rclcpp::Duration::from_seconds(predict2send_delay_);
    command.reference_valid = true;
    command.reference_status = "valid_mpc";
    return true;
}
} // namespace qd::auto_aim
