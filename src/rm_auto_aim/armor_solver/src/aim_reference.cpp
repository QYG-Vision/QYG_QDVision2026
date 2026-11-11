#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_decision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace qd::auto_aim {
AimReference Solver::build_aim_reference(const rm_interfaces::msg::Target& target,
                                         const rclcpp::Time& now) {
    AimReference reference;
    reference.header = target.header;
    reference.dt = mpc_config_.dt;
    reference.output_index = mpc_config_.horizon / 2;
    const auto reject = [&](const char* reason) {
        reference.status = reason;
        return reference;
    };
    const double age = (now - rclcpp::Time(target.header.stamp, now.get_clock_type())).seconds();
    if (mpc_config_.horizon < 3 || mpc_config_.horizon > 100
        || !std::isfinite(reference.dt) || reference.dt <= 0.0
        || !std::isfinite(predict2send_delay_) || predict2send_delay_ < 0.0
        || !std::isfinite(controller_delay_) || controller_delay_ < 0.0
        || !std::isfinite(additional_prediction_time) || additional_prediction_time < 0.0) {
        return reject("invalid_mpc_config");
    }
    if (!std::isfinite(age) || age < 0.0 || age > target_timeout_) {
        return reject("stale_target");
    }
    if (!target.tracking) return reject("invalid_target");
    const bool translation = top_level == 0 && use_armor_top_;
    std::vector<State> candidates;
    if (translation) {
        candidates.push_back((State() << target.position_armor.x, target.velocity_armor.x,
            target.position_armor.y, target.velocity_armor.y, target.position_armor.z,
            target.velocity_armor.z, 0.0, 0.0, 0.0).finished());
    } else {
        if (target.armors_num <= 0
            || static_cast<size_t>(target.armors_num) > target.radius_list.size()
            || static_cast<size_t>(target.armors_num) > target.dz_list.size()) {
            return reject("invalid_target");
        }
        candidates = get_armors_state(RosTarget(target));
    }
    for (const auto& state: candidates) {
        if (!state.allFinite() || (!translation && state[8] <= 0.0)) {
            return reject("invalid_target");
        }
    }

    int preferred = -1;
    // 上轮成功规划的板按时间外推后再关联，允许跟踪器重新编号候选数组。
    if (!translation && last_decision_state_ && last_decision_id_ == target.id
        && last_decision_frame_ == target.header.frame_id) {
        const double elapsed = (rclcpp::Time(target.header.stamp, now.get_clock_type())
            - last_decision_stamp_).seconds();
        if (elapsed >= 0.0 && elapsed <= target_timeout_) {
            const double phase = (*last_decision_state_)[6]
                + (*last_decision_state_)[7] * elapsed;
            double error = M_PI / candidates.size();
            for (size_t i = 0; i < candidates.size(); ++i) {
                const double difference = std::abs(reduced_angle(candidates[i][6] - phase));
                if (difference < error) {
                    error = difference;
                    preferred = static_cast<int>(i);
                }
            }
        }
    }
    int timeline_preferred = -1;
    bool timeline_waiting = false;
    double yaw_anchor = gimbal_rpy_[2] * 180.0 / M_PI;
    for (int k = 0; k < mpc_config_.horizon; ++k) {
        const double lead = (k - reference.output_index) * reference.dt
            + additional_prediction_time;
        std::vector<State> hit_states;
        hit_states.reserve(candidates.size());
        for (const auto& candidate: candidates) {
            // 负时间采样由运动模型处理，正的图像年龄、发送延迟和飞行时间只叠加一次。
            hit_states.push_back(solve_impact_state(state_predict(candidate, lead),
                                                   age + predict2send_delay_));
        }
        if (k == reference.output_index) {
            timeline_preferred = preferred;
            timeline_waiting = preferred >= 0 && last_decision_waiting_;
        }
        ArmorChoice choice;
        if (translation) {
            choice = ArmorChoice{0, hit_states.front(), 0.0, true};
        } else {
            const auto decision = choose_armor_at_time(
                hit_states, gimbal_rpy_, max_orientation_angle[top_level], top_level > 0,
                timeline_preferred, timeline_waiting);
            if (!decision) return reject("no_aim_window");
            choice = *decision;
        }
        timeline_preferred = choice.index;
        timeline_waiting = !choice.direct;
        const auto point = state_to_armor_pos(choice.state);
        if (!point.allFinite() || point.norm() < 0.01) return reject("invalid_prediction");
        const auto result = target_pos_to_aim_ypd(point);
        if (use_ballistic_compensation_
            && (!result.ballistic_debug || !result.ballistic_debug->ballistic_success)) {
            return reject("ballistic_failed");
        }
        const double yaw = result.aim_ypd.yaw * 180.0 / M_PI;
        const double pitch = -result.aim_ypd.pitch * 180.0 / M_PI;
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(pitch) > 327.67) {
            return reject("invalid_reference");
        }
        yaw_anchor = unwrap_degrees(yaw, yaw_anchor);
        reference.yaw.push_back(yaw_anchor);
        reference.pitch.push_back(pitch);
        reference.armor_indices.push_back(translation ? -1 : choice.index);
        reference.fire_allowed.push_back(choice.direct);
        reference.wait_durations.push_back(choice.wait_duration);
        if (k == reference.output_index) {
            reference.current_state = candidates[choice.index];
            reference.distance = point.norm();
            reference.current_aim = result.aim_ypd;
            reference.current_ballistic = result.ballistic_debug;
            reference.current_velocity = get_gimbal_ypd_v(
                point, state_to_armor_v(choice.state), gimbal_rpy_);
            reference.current_armor_index = translation ? -1 : choice.index;
            // 火控仍使用同一目标；等待位置只是转移参考，不允许据此开火。
            const auto fire_state = solve_impact_state(
                state_predict(candidates[choice.index], lead),
                age + predict2send_delay_ + controller_delay_);
            const auto fire_point = state_to_armor_pos(fire_state);
            if (!fire_point.allFinite() || fire_point.norm() < 0.01) {
                return reject("invalid_prediction");
            }
            const auto fire_result = target_pos_to_aim_ypd(fire_point);
            if (use_ballistic_compensation_
                && (!fire_result.ballistic_debug
                    || !fire_result.ballistic_debug->ballistic_success)) {
                return reject("ballistic_failed");
            }
            reference.suggest_fire = choice.direct && get_bullet_speed() > 0.0
                && isSuggestFire(result.aim_ypd, fire_result.aim_ypd, gimbal_rpy_,
                    max_out_error[top_level],
                    translation ? 0.0 : reduced_angle(choice.state[6] - gimbal_rpy_[2]),
                    pitch_inclined_) == ShootMode::SHOOT_NOW;
        }
    }
    for (int k = reference.output_index + 1; k < mpc_config_.horizon; ++k) {
        if (reference.armor_indices[k] != reference.current_armor_index) {
            reference.next_armor_index = reference.armor_indices[k];
            reference.time_to_switch = (k - reference.output_index) * reference.dt;
            break;
        }
    }
    reference.valid = true;
    reference.status = "valid";
    return reference;
}
} // namespace qd::auto_aim
