#ifndef ARMOR_SOLVER_ARMOR_DECISION_HPP_
#define ARMOR_SOLVER_ARMOR_DECISION_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace qd::auto_aim {
using ArmorMotion = Eigen::Matrix<double, 9, 1>;

/** @brief 一次上游选板结果，等待时间单位为 s，状态单位为 m、rad。 */
struct ArmorChoice {
    int index = -1;
    ArmorMotion state = ArmorMotion::Zero();
    double wait_duration = 0.0;
    bool direct = false;
};

/**
 * @brief 按朝向窗口选择当前板或下一块即将进入窗口的板，不调用运动规划器。
 * @param states 同一预测时刻的全部板状态，索引仅在本次整车快照内有效。
 * @param gimbal_rpy odom 中的云台姿态，rad，pitch 向上为正。
 * @param orientation_limit 允许直接击打的板朝向半宽，rad，范围 [0, pi]。
 * @param allow_wait 没有直接板时是否允许选择未来等待位置。
 * @param preferred 本次快照内希望保持的板索引，仅在其仍满足窗口时保持。
 * @param prefer_waiting 上轮已决定等该板时，在同向进入边界前保持安排，避免边界回跳。
 * @return 直接追板或等板决定；坏状态、静止且没有直接板时返回空值。
 * @note 未来窗口采样不得改变跟踪器或全局选板状态；仅依赖给定快照。
 */
inline std::optional<ArmorChoice> choose_armor_at_time(
    const std::vector<ArmorMotion>& states, const Eigen::Vector3d& gimbal_rpy,
    double orientation_limit, bool allow_wait, int preferred = -1, bool prefer_waiting = false) {
    if (states.empty() || !gimbal_rpy.allFinite() || !std::isfinite(orientation_limit)
        || orientation_limit < 0.0 || orientation_limit > M_PI) {
        return std::nullopt;
    }
    const auto relative_angle = [&](const ArmorMotion& state) {
        return std::remainder(state[6] - gimbal_rpy[2], 2.0 * M_PI);
    };
    int selected = -1;
    bool preferred_is_direct = false;
    double min_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < states.size(); ++i) {
        const auto& state = states[i];
        if (!state.allFinite() || state[8] <= 0.0) return std::nullopt;
        if (std::abs(relative_angle(state)) > orientation_limit) continue;
        // 仅保持仍在配置窗口内的当前板，不再隐式增加旧策略的 15 度。
        if (static_cast<int>(i) == preferred) {
            preferred_is_direct = true;
        }
        const double x = state[0] - std::cos(state[6]) * state[8];
        const double y = state[2] - std::sin(state[6]) * state[8];
        const double yaw_error = std::remainder(std::atan2(y, x) - gimbal_rpy[2], 2 * M_PI);
        const double pitch_error = std::atan2(state[4], std::hypot(x, y)) - gimbal_rpy[1];
        const double cost = yaw_error * yaw_error + pitch_error * pitch_error;
        if (cost < min_cost) {
            min_cost = cost;
            selected = static_cast<int>(i);
        }
    }
    const auto wait_choice = [&](int index, double wait) {
        auto future = states[index];
        future[0] += future[1] * wait;
        future[2] += future[3] * wait;
        future[4] += future[5] * wait;
        future[6] = std::remainder(future[6] + future[7] * wait, 2.0 * M_PI);
        return ArmorChoice{index, future, wait, false};
    };
    if (allow_wait && prefer_waiting && !preferred_is_direct && preferred >= 0
        && static_cast<size_t>(preferred) < states.size()) {
        const auto& state = states[preferred];
        const double rel = relative_angle(state);
        const bool entering = (state[7] > 1e-9 && rel < -orientation_limit)
            || (state[7] < -1e-9 && rel > orientation_limit);
        const double angle = std::abs(rel) - orientation_limit;
        if (entering && angle <= 2.0 * M_PI / states.size()) {
            return wait_choice(preferred, angle / std::abs(state[7]));
        }
    }
    if (preferred_is_direct) selected = preferred;
    if (selected >= 0) return ArmorChoice{selected, states[selected], 0.0, true};
    if (!allow_wait) return std::nullopt;

    double min_wait = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < states.size(); ++i) {
        const auto& state = states[i];
        const double w = state[7];
        if (std::abs(w) < 1e-9) continue;
        // 根据旋转方向计算下一次进入边界的时间，不假设下一板等于索引 +1。
        double angle = w > 0.0 ? -orientation_limit - relative_angle(state)
                               : relative_angle(state) - orientation_limit;
        angle = std::fmod(angle + 2.0 * M_PI, 2.0 * M_PI);
        const double wait = angle / std::abs(w);
        if (wait < min_wait) {
            min_wait = wait;
            selected = static_cast<int>(i);
        }
    }
    if (selected < 0) return std::nullopt;
    return wait_choice(selected, min_wait);
}
} // namespace qd::auto_aim
#endif
