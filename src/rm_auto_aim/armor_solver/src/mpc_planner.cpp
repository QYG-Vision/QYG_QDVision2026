#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace qd::auto_aim {
void Solver::reset_selection() {
    handoff_planner_->reset();
    selection_target_id_.clear();
    selection_target_frame_.clear();
    selection_armors_num_ = 0;
    selection_target_stamp_.reset();
    selection_log_time_.reset();
}

void Solver::update_feedback(const rm_interfaces::msg::SerialReceiveData& feedback) {
    if (!std::isfinite(feedback.yaw) || !std::isfinite(feedback.pitch)
        || !std::isfinite(feedback.yaw_actual_vel_rad_s)
        || !std::isfinite(feedback.pitch_actual_vel_rad_s) || feedback.mode > 5) {
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
        reset_selection();
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
    const bool reachable_selection = selection_policy_ == "acceleration_reachable";
    GimbalSelectionState initial;
    if (reachable_selection) {
        if (!feedback_) return reject("selection_missing_feedback");
        if (feedback_->header.frame_id != target.header.frame_id) {
            return reject("selection_feedback_frame_mismatch");
        }
        const double feedback_age =
            (now - rclcpp::Time(feedback_->header.stamp, now.get_clock_type())).seconds();
        if (!std::isfinite(feedback_age) || feedback_age < 0.0
            || feedback_age > feedback_timeout_) {
            return reject("selection_stale_feedback");
        }
        if (!std::isfinite(predict2send_delay_) || predict2send_delay_ < 0.0
            || !std::isfinite(additional_prediction_time)) {
            return reject("selection_invalid_delay");
        }
        const auto stamp = rclcpp::Time(target.header.stamp, now.get_clock_type());
        if (selection_target_id_ != target.id || selection_armors_num_ != target.armors_num
            || selection_target_frame_ != target.header.frame_id
            || (selection_target_stamp_ && stamp < *selection_target_stamp_)) {
            reset_selection();
        }
        selection_target_id_ = target.id;
        selection_armors_num_ = target.armors_num;
        selection_target_frame_ = target.header.frame_id;
        selection_target_stamp_ = stamp;
        initial.velocity = {feedback_->yaw_actual_vel_rad_s, feedback_->pitch_actual_vel_rad_s};
        // 仅将实际反馈外推到命令时刻；飞行时间属于目标预测，不是云台捕获时间。
        const double extrapolation = feedback_age + predict2send_delay_;
        initial.angle = {feedback_->yaw + initial.velocity[0] * extrapolation,
                         feedback_->pitch + initial.velocity[1] * extrapolation};
    }
    auto candidates = get_armors_state(RosTarget(target));
    if (candidates.empty()) return reject("invalid_target");
    for (const auto& state: candidates) {
        if (!state.allFinite() || state[8] <= 0.0) return reject("invalid_target");
    }
    const int half = mpc_config_.horizon / 2;
    // 保留奇点图像年龄、发送延迟、额外前移及飞行时间求解；不叠加旧等板时间。
    for (auto& state: candidates) {
        state = solve_impact_state(state, age + predict2send_delay_
                                          + additional_prediction_time);
    }
    if (reachable_selection) {
        return plan_handoff_mpc(candidates, now, initial, command);
    }
    // 兼容策略保留逐采样最近板参考。
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
            yaw.push_back(reduced_angle(current.first.aim_ypd.yaw - yaw0));
            pitch.push_back(-current.first.aim_ypd.pitch);
            yaw_velocity.push_back(reduced_angle(next.first.aim_ypd.yaw
                - last.first.aim_ypd.yaw) / (2 * mpc_config_.dt));
            pitch_velocity.push_back(-(next.first.aim_ypd.pitch
                - last.first.aim_ypd.pitch) / (2 * mpc_config_.dt));
            if (k == half) {
                command.distance = current.second;
                command.exp_yaw = reduced_angle(current.first.aim_ypd.yaw);
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
    command.yaw = std::remainder(yaw_plan->position + yaw0, 2.0 * M_PI);
    command.pitch = pitch_plan->position;
    command.yaw_vel_rad_s = yaw_plan->velocity;
    command.pitch_vel_rad_s = pitch_plan->velocity;
    command.yaw_acc_rad_s2 = yaw_plan->acceleration;
    command.pitch_acc_rad_s2 = pitch_plan->acceleration;
    // 与济瞄 plan.fire 一致：中点后2拍的参考与规划误差，阈值单位 rad。
    const int fire_index = half + 2;
    command.fire_advice = std::hypot(
        yaw[fire_index] - yaw_mpc_->sample_at(fire_index).position,
        pitch[fire_index] - pitch_mpc_->sample_at(fire_index).position)
        < fire_thresh_;
    // 兼容策略仅使用反馈进行误差诊断。
    if (feedback_) {
        command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 2.0 * M_PI);
        command.pitch_diff = command.pitch - feedback_->pitch;
    }
    command.header.stamp = now + rclcpp::Duration::from_seconds(predict2send_delay_);
    command.reference_valid = true;
    command.reference_status = "valid_mpc";
    return true;
}
bool Solver::plan_handoff_mpc(
    const std::vector<State>& candidates,
    const rclcpp::Time& now,
    const GimbalSelectionState& initial,
    rm_interfaces::msg::GimbalCmd& command
) {
    const auto reject = [&](const char* reason) {
        command.reference_valid = false;
        command.fire_advice = false;
        command.reference_status = reason;
        reset_selection();
        return false;
    };
    std::vector<PlateTrajectory> trajectories(candidates.size());
    std::vector<std::string> failures(candidates.size(), "valid_prediction");
    const int first = -52;
    const int last =
        static_cast<int>(std::ceil(selection_config_.horizon / selection_config_.dt)) + 2;
    for (size_t i = 0; i < candidates.size(); ++i) {
        try {
            auto& trajectory = trajectories[i];
            trajectory.first_time = first * selection_config_.dt;
            trajectory.dt = selection_config_.dt;
            std::vector<PlateSelectionSample> raw(static_cast<size_t>(last - first + 5));
            for (int k = first - 2; k <= last + 2; ++k) {
                const auto state = state_predict(candidates[i], k * selection_config_.dt);
                const auto point = state_to_armor_pos(state);
                if (!point.allFinite() || point.norm() < 0.01) {
                    throw std::runtime_error("invalid_prediction");
                }
                const auto aim = target_pos_to_aim_ypd(point);
                if (!std::isfinite(aim.aim_ypd.yaw) || !std::isfinite(aim.aim_ypd.pitch)) {
                    throw std::runtime_error("invalid_prediction");
                }
                if (use_ballistic_compensation_
                    && (!aim.ballistic_debug || !aim.ballistic_debug->ballistic_success))
                {
                    throw std::runtime_error("ballistic_failed");
                }
                const size_t index = static_cast<size_t>(k - first + 2);
                auto& sample = raw[index];
                sample.angle = { aim.aim_ypd.yaw, -aim.aim_ypd.pitch };
                if (index > 0) {
                    sample.angle[0] = unwrap_radians(sample.angle[0], raw[index - 1].angle[0]);
                }
                sample.horizontal_distance = point.head<2>().norm();
                sample.facing_angle = PlateSelector::facing_angle(state[6], point.x(), point.y());
            }
            for (size_t k = 1; k + 1 < raw.size(); ++k) {
                for (size_t axis = 0; axis < 2; ++axis) {
                    raw[k].velocity[axis] = (raw[k + 1].angle[axis] - raw[k - 1].angle[axis])
                        / (2.0 * selection_config_.dt);
                }
            }
            for (size_t k = 2; k + 2 < raw.size(); ++k) {
                for (size_t axis = 0; axis < 2; ++axis) {
                    raw[k].acceleration[axis] =
                        (raw[k + 1].velocity[axis] - raw[k - 1].velocity[axis])
                        / (2.0 * selection_config_.dt);
                }
                trajectory.samples.push_back(raw[k]);
            }
        } catch (const std::runtime_error& error) {
            trajectories[i].samples.clear();
            failures[i] = error.what();
        }
    }
    const auto command_time = now + rclcpp::Duration::from_seconds(predict2send_delay_);
    const auto plan = handoff_planner_->plan(trajectories, initial, command_time.seconds());
    if (!plan)
        return reject("selection_no_valid_plan");
    const auto& reference = plan->references;
    const int half = mpc_config_.horizon / 2;
    const auto& center = reference[static_cast<size_t>(half)];
    const size_t aim_plate = center.aim_plate;
    const auto aim = trajectories[aim_plate].at(0.0);
    if (!aim)
        return reject("selection_invalid_aim");
    const double yaw0 = center.motion.angle[0];
    std::vector<double> yaw, pitch, yaw_velocity, pitch_velocity;
    for (const auto& value: reference) {
        yaw.push_back(value.motion.angle[0] - yaw0);
        pitch.push_back(value.motion.angle[1]);
        yaw_velocity.push_back(value.motion.velocity[0]);
        pitch_velocity.push_back(value.motion.velocity[1]);
    }
    const auto yaw_plan = yaw_mpc_->plan(yaw, &yaw_velocity);
    const auto pitch_plan = pitch_mpc_->plan(pitch, &pitch_velocity);
    if (!yaw_plan || !pitch_plan)
        return reject("invalid_mpc_output");
    command.yaw = std::remainder(yaw_plan->position + yaw0, 2.0 * M_PI);
    command.pitch = pitch_plan->position;
    command.yaw_vel_rad_s = yaw_plan->velocity;
    command.pitch_vel_rad_s = pitch_plan->velocity;
    command.yaw_acc_rad_s2 = yaw_plan->acceleration;
    command.pitch_acc_rad_s2 = pitch_plan->acceleration;
    // exp_* 始终记录真实板的瞄准角；转移曲线只进入 MPC 运动参考。
    command.exp_yaw = reduced_angle(aim->angle[0]);
    command.exp_pitch = aim->angle[1];
    command.yaw_vel = aim->velocity[0];
    command.pitch_vel = aim->velocity[1];
    command.distance = state_to_armor_pos(candidates[aim_plate]).norm();
    ballistic_debug_ =
        target_pos_to_aim_ypd(state_to_armor_pos(candidates[aim_plate])).ballistic_debug;
    const int fire_index = half + 2;
    const double fire_offset = 2.0 * mpc_config_.dt;
    const auto fire_plate =
        handoff_planner_->fire_target(*plan, trajectories, initial, fire_offset);
    command.fire_advice = false;
    if (fire_plate) {
        const auto target = trajectories[*fire_plate].at(fire_offset);
        if (target) {
            command.fire_advice =
                std::hypot(
                    std::remainder(
                        target->angle[0] - yaw_mpc_->sample_at(fire_index).position - yaw0,
                        2.0 * M_PI
                    ),
                    target->angle[1] - pitch_mpc_->sample_at(fire_index).position
                )
                < fire_thresh_;
        }
    }
    command.yaw_diff = std::remainder(command.yaw - feedback_->yaw, 2.0 * M_PI);
    command.pitch_diff = command.pitch - feedback_->pitch;
    command.header.stamp = command_time;
    command.reference_valid = true;
    const char* phase = "tracking";
    switch (center.phase) {
        case HandoffPhase::CAPTURING:
            phase = "capturing";
            break;
        case HandoffPhase::TRACKING:
            phase = "tracking";
            break;
        case HandoffPhase::SCHEDULED:
            phase = "scheduled";
            break;
        case HandoffPhase::TRANSFERRING:
            phase = "transferring";
            break;
    }
    std::ostringstream status;
    status << "valid_mpc_" << phase << ":plate=" << aim_plate << ":" << plan->reason;
    if (plan->active_plate)
        status << ":active=" << *plan->active_plate;
    if (plan->next_plate)
        status << ":next=" << *plan->next_plate;
    if (plan->departure_time)
        status << ":depart=" << *plan->departure_time - plan->zero_time;
    if (plan->arrival_time)
        status << ":arrive=" << *plan->arrival_time - plan->zero_time;
    command.reference_status = status.str();
    if (!selection_log_time_ || now < *selection_log_time_
        || (now - *selection_log_time_).seconds() >= 1.0)
    {
        std::ostringstream details;
        for (size_t i = 0; i < failures.size(); ++i) {
            details << " plate=" << i << " prediction=" << failures[i];
        }
        FYT_INFO("armor_solver", "handoff {}{}", command.reference_status, details.str());
        selection_log_time_ = now;
    }
    return true;
}
} // namespace qd::auto_aim
