#include "armor_solver/plate_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace qd::auto_aim {
namespace {
constexpr double TWO_PI = 6.28318530717958647692;

/** @brief 判断参数是否为有限正数。 @return 有限且大于零时为 true。 */
bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

/**
 * @brief 计算连接两端角度、角速度的三次曲线所需最大角加速度。
 * @param displacement 连续展开后的终点减起点角度，rad。
 * @param initial_velocity 起点角速度，rad/s。
 * @param final_velocity 终点角速度，rad/s。
 * @param duration 连接时间，s，必须大于零。
 * @return 曲线起终点角加速度绝对值的最大值，rad/s²。
 */
double connection_acceleration(double displacement, double initial_velocity,
                               double final_velocity, double duration) {
    const double start = 6.0 * displacement / (duration * duration)
        - (4.0 * initial_velocity + 2.0 * final_velocity) / duration;
    const double end = -6.0 * displacement / (duration * duration)
        + (2.0 * initial_velocity + 4.0 * final_velocity) / duration;
    return std::max(std::abs(start), std::abs(end));
}
} // namespace

PlateSelector::PlateSelector(const PlateSelectionConfig& config): config_(config) {
    if (!positive(config.dt) || std::abs(config.dt - 0.01) > 1e-9
        || !positive(config.horizon) || !positive(config.capture_deadline)
        || !positive(config.hold_time) || !positive(config.angle_tolerance)
        || !positive(config.velocity_tolerance) || !positive(config.max_facing_angle)
        || config.max_facing_angle >= TWO_PI / 4.0
        || !positive(config.max_acceleration[0]) || !positive(config.max_acceleration[1])
        || !positive(config.switch_hold_time) || !positive(config.switch_improvement)
        || config.switch_improvement >= 1.0 || config.capture_deadline < config.dt
        || config.hold_time < config.dt
        || config.capture_deadline + config.hold_time > config.horizon + 1e-9
        || config.horizon > 2.0) {
        throw std::invalid_argument("invalid solver.selection configuration");
    }
}

double PlateSelector::facing_angle(double phase, double x, double y) {
    return std::remainder(phase - std::atan2(y, x), TWO_PI);
}

PlateEvaluation PlateSelector::evaluate(const std::vector<PlateSelectionSample>& trajectory,
                                        const GimbalSelectionState& initial) const {
    PlateEvaluation result;
    const size_t steps = static_cast<size_t>(std::ceil(config_.horizon / config_.dt - 1e-9));
    const size_t hold_steps =
        static_cast<size_t>(std::ceil(config_.hold_time / config_.dt - 1e-9));
    if (trajectory.size() < steps + 1) {
        return result;
    }
    for (size_t axis = 0; axis < 2; ++axis) {
        if (!std::isfinite(initial.angle[axis]) || !std::isfinite(initial.velocity[axis])) {
            return result;
        }
    }
    for (const auto& sample: trajectory) {
        if (!std::isfinite(sample.facing_angle) || !positive(sample.horizontal_distance)) {
            return result;
        }
        for (size_t axis = 0; axis < 2; ++axis) {
            if (!std::isfinite(sample.angle[axis]) || !std::isfinite(sample.velocity[axis])
                || !std::isfinite(sample.acceleration[axis])) {
                return result;
            }
        }
    }
    result.valid = true;
    result.reason = "no_facing_window";
    result.horizontal_distance = trajectory.front().horizontal_distance;
    const double yaw_error = std::remainder(trajectory.front().angle[0] - initial.angle[0], TWO_PI);
    result.angular_error = std::hypot(yaw_error, trajectory.front().angle[1] - initial.angle[1]);
    // 对整段轨迹使用同一 yaw 分支，避免各终点独立取最短角差导致连接方向跳变。
    const double yaw_shift = initial.angle[0] + yaw_error - trajectory.front().angle[0];
    bool aligned = true;
    for (size_t axis = 0; axis < 2; ++axis) {
        const double error = axis == 0 ? yaw_error : trajectory.front().angle[1] - initial.angle[1];
        aligned = aligned && std::abs(error) <= config_.angle_tolerance
            && std::abs(trajectory.front().velocity[axis] - initial.velocity[axis])
                <= config_.velocity_tolerance;
    }
    for (size_t k = 0; k + hold_steps <= steps; ++k) {
        bool facing = true;
        double ratio = 0.0;
        for (size_t j = k; j <= k + hold_steps; ++j) {
            facing = facing && std::abs(trajectory[j].facing_angle) <= config_.max_facing_angle;
            for (size_t axis = 0; axis < 2; ++axis) {
                ratio = std::max(ratio, std::abs(trajectory[j].acceleration[axis])
                    / config_.max_acceleration[axis]);
            }
        }
        if (!facing) {
            continue;
        }
        const double time = static_cast<double>(k) * config_.dt;
        if (k == 0) {
            result.ready_to_fire = aligned && ratio <= 1.0;
            if (!aligned) {
                result.reason = "not_aligned";
                continue;
            }
        } else {
            for (size_t axis = 0; axis < 2; ++axis) {
                const double displacement = trajectory[k].angle[axis]
                    + (axis == 0 ? yaw_shift : 0.0) - initial.angle[axis];
                ratio = std::max(ratio, connection_acceleration(displacement,
                    initial.velocity[axis], trajectory[k].velocity[axis], time)
                    / config_.max_acceleration[axis]);
            }
        }
        result.demand_ratio = std::min(result.demand_ratio, ratio);
        result.reason = "acceleration_or_deadline";
        if (result.reachable || ratio > 1.0 || time > config_.capture_deadline + 1e-9) {
            continue;
        }
        result.reachable = true;
        result.capture_time = time;
        result.reason = "reachable";
        size_t end = k;
        for (; end <= steps; ++end) {
            const auto& sample = trajectory[end];
            if (std::abs(sample.facing_angle) > config_.max_facing_angle
                || std::abs(sample.acceleration[0]) > config_.max_acceleration[0]
                || std::abs(sample.acceleration[1]) > config_.max_acceleration[1]) {
                break;
            }
        }
        result.window_duration = static_cast<double>(end - k - 1) * config_.dt;
    }
    if (result.reachable) {
        result.reason = "reachable";
    }
    return result;
}

void PlateSelector::reset() {
    current_.reset();
    challenger_.reset();
    last_time_.reset();
}

PlateSelectionDecision PlateSelector::choose(const std::vector<PlateEvaluation>& evaluations,
                                            double now) {
    if (!std::isfinite(now)) {
        reset();
        return {};
    }
    if (last_time_ && now < *last_time_) {
        reset();
    }
    last_time_ = now;
    const auto valid = [&](size_t index) {
        return index < evaluations.size() && evaluations[index].valid;
    };
    const auto select = [&](size_t index, const char* reason) {
        current_ = index;
        challenger_.reset();
        return PlateSelectionDecision {index, reason};
    };
    if (current_ && valid(*current_) && evaluations[*current_].reachable) {
        return select(*current_, "keep_reachable");
    }
    std::optional<size_t> best;
    for (size_t i = 0; i < evaluations.size(); ++i) {
        const auto& value = evaluations[i];
        if (!value.valid || !value.reachable) {
            continue;
        }
        if (!best || std::make_tuple(value.capture_time, -value.window_duration,
                                     value.horizontal_distance, i)
            < std::make_tuple(evaluations[*best].capture_time,
                              -evaluations[*best].window_duration,
                              evaluations[*best].horizontal_distance, *best)) {
            best = i;
        }
    }
    if (best) {
        return select(*best, "earliest_capture");
    }
    for (size_t i = 0; i < evaluations.size(); ++i) {
        if (!valid(i) || !std::isfinite(evaluations[i].demand_ratio)) {
            continue;
        }
        if (!best || std::make_tuple(evaluations[i].demand_ratio, i)
            < std::make_tuple(evaluations[*best].demand_ratio, *best)) {
            best = i;
        }
    }
    if (!best) {
        if (current_ && valid(*current_)) {
            return select(*current_, "keep_without_window");
        }
        for (size_t i = 0; i < evaluations.size(); ++i) {
            if (valid(i) && (!best || std::make_tuple(evaluations[i].angular_error,
                    evaluations[i].horizontal_distance, i)
                < std::make_tuple(evaluations[*best].angular_error,
                    evaluations[*best].horizontal_distance, *best))) {
                best = i;
            }
        }
        if (!best) {
            reset();
            return {};
        }
    }
    if (!current_ || !valid(*current_)) {
        return select(*best, "initial_capture");
    }
    const double old_score = evaluations[*current_].demand_ratio;
    const double new_score = evaluations[*best].demand_ratio;
    if (*best == *current_ || !std::isfinite(new_score)
        || !(new_score < old_score * (1.0 - config_.switch_improvement))) {
        return select(*current_, "keep_capture");
    }
    if (challenger_ != best) {
        challenger_ = best;
        challenger_since_ = now;
    }
    if (now - challenger_since_ + 1e-9 >= config_.switch_hold_time) {
        return select(*best, "better_capture");
    }
    return {*current_, "wait_switch_hold"};
}

} // namespace qd::auto_aim
