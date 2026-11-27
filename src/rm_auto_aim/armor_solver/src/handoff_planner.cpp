#include "armor_solver/handoff_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace qd::auto_aim {
namespace {
    constexpr double TWO_PI = 6.28318530717958647692;
    constexpr double EPSILON = 1e-6;
    constexpr size_t REFERENCE_COUNT = 100;
    constexpr size_t MIDPOINT = 50;

    /**
 * @brief 读取三次 Hermite 曲线及其一、二阶导数。
 * @param p0 起点角度，rad。
 * @param v0 起点角速度，rad/s。
 * @param p1 终点角度，rad，须预先连续展开。
 * @param v1 终点角速度，rad/s。
 * @param duration 两端时间差，s，须为正。
 * @param elapsed 距起点时间，s。
 * @return 角度、角速度和角加速度。
 */
    std::array<double, 3>
    cubic(double p0, double v0, double p1, double v1, double duration, double elapsed) {
        const double c2 = 3.0 * (p1 - p0) / (duration * duration) - (2.0 * v0 + v1) / duration;
        const double c3 =
            -2.0 * (p1 - p0) / (duration * duration * duration) + (v0 + v1) / (duration * duration);
        return { p0 + elapsed * (v0 + elapsed * (c2 + elapsed * c3)),
                 v0 + elapsed * (2.0 * c2 + 3.0 * elapsed * c3),
                 2.0 * c2 + 6.0 * elapsed * c3 };
    }

    /** @brief 检查单板参考是否为有限物理量。 @return 所有字段有效时为 true。 */
    bool finite_sample(const PlateSelectionSample& sample) {
        for (size_t axis = 0; axis < 2; ++axis) {
            if (!std::isfinite(sample.angle[axis]) || !std::isfinite(sample.velocity[axis])
                || !std::isfinite(sample.acceleration[axis]))
            {
                return false;
            }
        }
        return std::isfinite(sample.facing_angle) && std::isfinite(sample.horizontal_distance)
            && sample.horizontal_distance > 0.0;
    }

    /** @brief 从真实板参考提取角度及角速度。 @return 两轴运动状态。 */
    GimbalSelectionState motion_of(const PlateSelectionSample& sample) {
        return { sample.angle, sample.velocity };
    }
} // namespace

std::optional<PlateSelectionSample> PlateTrajectory::at(double time) const {
    if (samples.empty() || !std::isfinite(time) || !std::isfinite(dt) || dt <= 0.0
        || !std::isfinite(first_time))
    {
        return std::nullopt;
    }
    const double index = (time - first_time) / dt;
    if (index < -EPSILON / dt || index > static_cast<double>(samples.size() - 1) + EPSILON / dt) {
        return std::nullopt;
    }
    const double snapped = std::round(index);
    if (std::abs(index - snapped) < EPSILON / dt) {
        const auto& sample = samples[static_cast<size_t>(std::max(0.0, snapped))];
        return finite_sample(sample) ? std::optional<PlateSelectionSample>(sample) : std::nullopt;
    }
    const size_t left = static_cast<size_t>(std::floor(index));
    const auto& a = samples[left];
    const auto& b = samples[left + 1];
    if (!finite_sample(a) || !finite_sample(b))
        return std::nullopt;
    const double elapsed = (index - static_cast<double>(left)) * dt;
    auto result = a;
    for (size_t axis = 0; axis < 2; ++axis) {
        const auto value =
            cubic(a.angle[axis], a.velocity[axis], b.angle[axis], b.velocity[axis], dt, elapsed);
        result.angle[axis] = value[0];
        result.velocity[axis] = value[1];
        result.acceleration[axis] = value[2];
    }
    result.facing_angle += elapsed / dt * std::remainder(b.facing_angle - a.facing_angle, TWO_PI);
    result.horizontal_distance += elapsed / dt * (b.horizontal_distance - a.horizontal_distance);
    return result;
}

std::optional<HandoffReference> HandoffPlan::at(double absolute_time) const {
    if (references.size() != REFERENCE_COUNT || !std::isfinite(absolute_time)
        || !std::isfinite(zero_time) || !std::isfinite(dt) || dt <= 0.0)
    {
        return std::nullopt;
    }
    const double index = (absolute_time - zero_time) / dt + static_cast<double>(MIDPOINT);
    if (index < -EPSILON / dt || index > static_cast<double>(REFERENCE_COUNT - 1) + EPSILON / dt) {
        return std::nullopt;
    }
    for (auto it = curves.rbegin(); it != curves.rend(); ++it) {
        if (absolute_time < it->valid_begin - 1e-10 || absolute_time >= it->valid_end - 1e-10) {
            continue;
        }
        HandoffReference ref;
        ref.phase = it->phase;
        ref.aim_plate = it->aim_plate;
        for (size_t axis = 0; axis < 2; ++axis) {
            const auto value = cubic(
                it->start.angle[axis],
                it->start.velocity[axis],
                it->finish.angle[axis],
                it->finish.velocity[axis],
                it->end - it->begin,
                absolute_time - it->begin
            );
            ref.motion.angle[axis] = value[0];
            ref.motion.velocity[axis] = value[1];
            ref.acceleration[axis] = value[2];
        }
        return ref;
    }
    const double snapped = std::round(index);
    if (std::abs(index - snapped) < EPSILON / dt) {
        return references[static_cast<size_t>(std::max(0.0, snapped))];
    }
    const size_t left = static_cast<size_t>(std::floor(index));
    const auto& a = references[left];
    const auto& b = references[left + 1];
    auto result = a;
    for (size_t axis = 0; axis < 2; ++axis) {
        const auto value = cubic(
            a.motion.angle[axis],
            a.motion.velocity[axis],
            b.motion.angle[axis],
            b.motion.velocity[axis],
            dt,
            (index - static_cast<double>(left)) * dt
        );
        result.motion.angle[axis] = value[0];
        result.motion.velocity[axis] = value[1];
        result.acceleration[axis] = value[2];
    }
    if (a.tracking_plate != b.tracking_plate || a.phase != b.phase) {
        result.phase = HandoffPhase::TRANSFERRING;
        result.tracking_plate.reset();
        result.aim_plate = b.aim_plate;
    }
    return result;
}

HandoffPlanner::HandoffPlanner(const PlateSelectionConfig& selection, const HandoffConfig& handoff):
    selection_(selection),
    handoff_(handoff),
    capture_selector_(selection) {
    if (!std::isfinite(handoff.max_transfer_time) || handoff.max_transfer_time < selection.dt
        || handoff.max_transfer_time > 0.49 || !std::isfinite(handoff.window_margin)
        || handoff.window_margin < selection.dt
        || selection.hold_time + 2.0 * handoff.window_margin >= 0.49)
    {
        throw std::invalid_argument("invalid solver.selection.handoff configuration");
    }
}

bool HandoffPlanner::attack_valid(const PlateSelectionSample& sample) const {
    return finite_sample(sample) && std::abs(sample.facing_angle) <= selection_.max_facing_angle
        && std::abs(sample.acceleration[0]) <= selection_.max_acceleration[0]
        && std::abs(sample.acceleration[1]) <= selection_.max_acceleration[1];
}

bool HandoffPlanner::aligned(
    const PlateSelectionSample& sample,
    const GimbalSelectionState& initial
) const {
    for (size_t axis = 0; axis < 2; ++axis) {
        const double error = axis == 0
            ? std::remainder(sample.angle[axis] - initial.angle[axis], TWO_PI)
            : sample.angle[axis] - initial.angle[axis];
        if (!std::isfinite(initial.angle[axis]) || !std::isfinite(initial.velocity[axis])
            || std::abs(error) > selection_.angle_tolerance
            || std::abs(sample.velocity[axis] - initial.velocity[axis])
                > selection_.velocity_tolerance)
        {
            return false;
        }
    }
    return true;
}

std::vector<HandoffPlanner::Window>
HandoffPlanner::windows(const PlateTrajectory& trajectory) const {
    std::vector<Window> result;
    std::optional<Window> current;
    double min_angle = std::numeric_limits<double>::infinity();
    // 对窗口边界插值，避免 4 ms 重算相对 10 ms 网格移动时把固定接管时间判为过早。
    const auto boundary = [&](double left, double right, bool left_valid) {
        for (int iteration = 0; iteration < 20; ++iteration) {
            const double middle = (left + right) * 0.5;
            const auto value = trajectory.at(middle);
            if ((value && attack_valid(*value)) == left_valid)
                left = middle;
            else
                right = middle;
        }
        return (left + right) * 0.5;
    };
    for (size_t k = 0; k < trajectory.samples.size(); ++k) {
        const auto& sample = trajectory.samples[k];
        const double time = trajectory.first_time + static_cast<double>(k) * trajectory.dt;
        if (!attack_valid(sample)) {
            if (current) {
                current->end = boundary(time - trajectory.dt, time, true);
                result.push_back(*current);
            }
            current.reset();
            min_angle = std::numeric_limits<double>::infinity();
            continue;
        }
        if (!current) {
            const double begin = k > 0 ? boundary(time - trajectory.dt, time, false) : time;
            current = Window { begin, time, time };
        }
        current->end = time;
        if (std::abs(sample.facing_angle) < min_angle) {
            min_angle = std::abs(sample.facing_angle);
            current->center = time;
        }
        if (k > 0 && attack_valid(trajectory.samples[k - 1])
            && sample.facing_angle * trajectory.samples[k - 1].facing_angle < 0.0)
        {
            const double before = trajectory.samples[k - 1].facing_angle;
            current->center =
                time - trajectory.dt + trajectory.dt * (-before) / (sample.facing_angle - before);
            min_angle = 0.0;
        }
    }
    if (current)
        result.push_back(*current);
    return result;
}

double HandoffPlanner::demand(const Connection& connection) const {
    const double duration = connection.end - connection.begin;
    if (duration <= EPSILON)
        return std::numeric_limits<double>::infinity();
    double result = 0.0;
    for (size_t axis = 0; axis < 2; ++axis) {
        for (const double time: { 0.0, duration }) {
            const auto value = cubic(
                connection.start.angle[axis],
                connection.start.velocity[axis],
                connection.finish.angle[axis],
                connection.finish.velocity[axis],
                duration,
                time
            );
            result = std::max(result, std::abs(value[2]) / selection_.max_acceleration[axis]);
        }
    }
    return result;
}

HandoffReference HandoffPlanner::connection_at(const Connection& connection, double time) const {
    HandoffReference result;
    const double duration = connection.end - connection.begin;
    for (size_t axis = 0; axis < 2; ++axis) {
        const auto value = cubic(
            connection.start.angle[axis],
            connection.start.velocity[axis],
            connection.finish.angle[axis],
            connection.finish.velocity[axis],
            duration,
            std::clamp(time - connection.begin, 0.0, duration)
        );
        result.motion.angle[axis] = value[0];
        result.motion.velocity[axis] = value[1];
        result.acceleration[axis] = value[2];
    }
    return result;
}

void HandoffPlanner::reset() {
    capture_selector_.reset();
    active_.reset();
    capture_target_.reset();
    capture_locked_ = false;
    capture_not_before_ = 0.0;
    capture_connection_.reset();
    pending_.reset();
    challenger_.reset();
    previous_.reset();
    plate_count_ = 0;
}

std::optional<HandoffPlan> HandoffPlanner::plan(
    const std::vector<PlateTrajectory>& trajectories,
    const GimbalSelectionState& initial,
    double zero_time
) {
    if (!std::isfinite(zero_time)) {
        reset();
        return std::nullopt;
    }
    for (size_t axis = 0; axis < 2; ++axis) {
        if (!std::isfinite(initial.angle[axis]) || !std::isfinite(initial.velocity[axis])) {
            reset();
            return std::nullopt;
        }
    }
    if (previous_
        && (zero_time < previous_->zero_time - EPSILON || zero_time - previous_->zero_time > 0.50
            || plate_count_ != trajectories.size()))
    {
        reset();
    }
    plate_count_ = trajectories.size();
    const auto sample = [&](size_t plate, double time) -> std::optional<PlateSelectionSample> {
        return plate < trajectories.size() ? trajectories[plate].at(time) : std::nullopt;
    };
    const auto make_connection =
        [&](double begin, double end, GimbalSelectionState start, GimbalSelectionState finish) {
            finish.angle[0] =
                start.angle[0] + std::remainder(finish.angle[0] - start.angle[0], TWO_PI);
            return Connection { begin, end, start, finish };
        };
    const auto previous_zero = previous_ ? previous_->at(zero_time) : std::nullopt;
    GimbalSelectionState anchor = previous_zero ? previous_zero->motion : initial;
    const double future_end = std::min(selection_.horizon, 0.49);
    std::vector<std::vector<Window>> all_windows;
    for (const auto& trajectory: trajectories)
        all_windows.push_back(windows(trajectory));
    std::string reason = capture_locked_ ? "transfer_recovery" : "tracking";

    // 实际接管确认之前，不能仅凭计划时间把下一板变成当前板。
    if (pending_ && zero_time >= pending_->arrival - EPSILON) {
        const auto target = sample(pending_->target, 0.0);
        if (target && attack_valid(*target) && aligned(*target, initial)) {
            active_ = pending_->target;
            capture_target_.reset();
            capture_locked_ = false;
            capture_connection_.reset();
            reason = "takeover_confirmed";
        } else {
            active_.reset();
            capture_target_ = pending_->target;
            capture_locked_ = true;
            capture_not_before_ = pending_->arrival;
            capture_connection_.reset();
            reason = "takeover_unconfirmed";
        }
        pending_.reset();
        challenger_.reset();
    }
    if (!pending_ && !active_ && !capture_locked_) {
        double best_error = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < trajectories.size(); ++i) {
            const auto value = sample(i, 0.0);
            if (!value || !attack_valid(*value) || !aligned(*value, initial))
                continue;
            const double error = std::hypot(
                std::remainder(value->angle[0] - initial.angle[0], TWO_PI),
                value->angle[1] - initial.angle[1]
            );
            if (error < best_error) {
                active_ = i;
                best_error = error;
            }
        }
    }
    if (!pending_ && !active_ && capture_locked_ && capture_target_
        && zero_time + EPSILON >= capture_not_before_)
    {
        const auto target = sample(*capture_target_, 0.0);
        if (target && attack_valid(*target) && aligned(*target, initial)) {
            active_ = capture_target_;
            capture_locked_ = false;
            capture_connection_.reset();
            reason = "capture_confirmed";
        }
    }
    if (active_ && !pending_) {
        const auto current = sample(*active_, 0.0);
        if (!current || !attack_valid(*current) || !aligned(*current, initial))
            active_.reset();
    }
    if (!previous_ && active_)
        anchor = motion_of(*sample(*active_, 0.0));

    // 小幅模型修正通过有界的误差衰减曲线接回真实板轨迹，保持参考历史的 C1 连续。
    std::optional<Connection> correction;
    if (active_) {
        const auto current = sample(*active_, 0.0);
        if (current) {
            GimbalSelectionState error;
            for (size_t axis = 0; axis < 2; ++axis) {
                error.angle[axis] = axis == 0
                    ? std::remainder(anchor.angle[axis] - current->angle[axis], TWO_PI)
                    : anchor.angle[axis] - current->angle[axis];
                error.velocity[axis] = anchor.velocity[axis] - current->velocity[axis];
            }
            for (double duration = selection_.dt; duration <= 0.30 + EPSILON;
                 duration += selection_.dt)
            {
                const Connection candidate { 0.0, duration, error, {} };
                bool valid = true;
                for (double t = 0.0; t <= duration + EPSILON; t += selection_.dt) {
                    const auto target = sample(*active_, t);
                    if (!target) {
                        valid = false;
                        break;
                    }
                    const auto delta = connection_at(candidate, t);
                    for (size_t axis = 0; axis < 2; ++axis) {
                        valid = valid
                            && std::abs(target->acceleration[axis] + delta.acceleration[axis])
                                <= selection_.max_acceleration[axis] + 1e-8;
                    }
                }
                if (valid) {
                    correction = candidate;
                    break;
                }
            }
        }
        if (!correction && (!pending_ || zero_time < pending_->departure - EPSILON)) {
            active_.reset();
        }
    }
    const auto source_reference = [&](double t) -> std::optional<HandoffReference> {
        if (!active_)
            return std::nullopt;
        const auto value = sample(*active_, t);
        if (!value)
            return std::nullopt;
        HandoffReference ref;
        ref.motion = motion_of(*value);
        ref.motion.angle[0] =
            anchor.angle[0] + std::remainder(ref.motion.angle[0] - anchor.angle[0], TWO_PI);
        ref.acceleration = value->acceleration;
        if (correction && t < correction->end) {
            const auto delta = connection_at(*correction, t);
            for (size_t axis = 0; axis < 2; ++axis) {
                ref.motion.angle[axis] += delta.motion.angle[axis];
                ref.motion.velocity[axis] += delta.motion.velocity[axis];
                ref.acceleration[axis] += delta.acceleration[axis];
            }
        }
        ref.phase = HandoffPhase::TRACKING;
        ref.tracking_plate = active_;
        ref.aim_plate = *active_;
        return ref;
    };
    const auto target_window = [&](size_t plate, double arrival) -> std::optional<Window> {
        if (plate >= all_windows.size())
            return std::nullopt;
        for (const auto& window: all_windows[plate]) {
            if (arrival + EPSILON >= window.begin + handoff_.window_margin
                && arrival + selection_.hold_time + handoff_.window_margin
                    <= std::min(window.end, future_end) + EPSILON)
                return window;
        }
        return std::nullopt;
    };
    std::optional<Window> current_window;
    if (active_) {
        for (const auto& window: all_windows[*active_]) {
            if (window.begin <= EPSILON && window.end >= -EPSILON)
                current_window = window;
        }
    }
    // 未出发计划按原绝对时间校验；已出发计划只更新剩余连接，不推迟接管时间。
    if (pending_) {
        const double departure = pending_->departure - zero_time;
        const double arrival = pending_->arrival - zero_time;
        const auto target = sample(pending_->target, arrival);
        bool valid = target && target_window(pending_->target, arrival).has_value();
        GimbalSelectionState start = anchor;
        double begin = zero_time;
        if (departure > EPSILON) {
            const auto source = source_reference(departure);
            const auto current = sample(pending_->source, 0.0);
            valid = valid && source && current && aligned(*current, initial) && current_window
                && departure + handoff_.window_margin <= current_window->end + EPSILON;
            if (source)
                start = source->motion;
            begin = pending_->departure;
        }
        if (valid) {
            auto connection = make_connection(begin, pending_->arrival, start, motion_of(*target));
            valid = demand(connection) <= 1.0 + 1e-8;
            if (departure <= EPSILON) {
                valid = valid
                    && demand(make_connection(
                           zero_time,
                           pending_->arrival,
                           initial,
                           motion_of(*target)
                       ))
                        <= 1.0 + 1e-8;
            }
            if (valid)
                pending_->connection = connection;
        }
        if (!valid) {
            const bool started = departure <= EPSILON;
            if (started) {
                capture_target_ = pending_->target;
                capture_locked_ = true;
                capture_not_before_ = pending_->arrival;
                active_.reset();
                capture_connection_.reset();
            }
            pending_.reset();
            challenger_.reset();
            reason = started ? "transfer_recovery" : "replan_invalid_schedule";
        }
    }

    if (active_ && current_window && (!pending_ || zero_time < pending_->departure - EPSILON)) {
        std::optional<Pending> best;
        for (size_t target = 0; target < trajectories.size(); ++target) {
            if (target == *active_)
                continue;
            for (double arrival = selection_.dt;
                 arrival + selection_.hold_time + handoff_.window_margin <= future_end + EPSILON;
                 arrival += selection_.dt)
            {
                if (best && arrival > best->arrival - zero_time + EPSILON)
                    break;
                const auto window = target_window(target, arrival);
                if (!window || window->center <= current_window->center + selection_.dt)
                    continue;
                const auto finish = sample(target, arrival);
                if (!finish)
                    continue;
                for (double departure = 0.0; departure + selection_.dt <= arrival + EPSILON;
                     departure += selection_.dt)
                {
                    if (arrival - departure > handoff_.max_transfer_time + EPSILON
                        || departure + handoff_.window_margin > current_window->end + EPSILON)
                    {
                        continue;
                    }
                    const auto source = source_reference(departure);
                    if (!source)
                        continue;
                    const auto connection = make_connection(
                        zero_time + departure,
                        zero_time + arrival,
                        source->motion,
                        motion_of(*finish)
                    );
                    const double ratio = demand(connection);
                    if (ratio > 1.0 + 1e-8)
                        continue;
                    if (departure < EPSILON
                        && demand(make_connection(
                               zero_time,
                               zero_time + arrival,
                               initial,
                               motion_of(*finish)
                           ))
                            > 1.0 + 1e-8)
                        continue;
                    Pending candidate { *active_,
                                        target,
                                        zero_time + departure,
                                        zero_time + arrival,
                                        std::min(window->end, future_end) - arrival,
                                        ratio,
                                        finish->horizontal_distance,
                                        connection };
                    const auto rank = [](const Pending& value) {
                        return std::make_tuple(
                            value.arrival,
                            -value.window_duration,
                            value.demand,
                            -value.departure,
                            value.distance,
                            value.target
                        );
                    };
                    if (!best || rank(candidate) < rank(*best))
                        best = candidate;
                }
            }
        }
        if (!pending_ && best) {
            pending_ = best;
            reason = "earliest_handoff";
        } else if (
            pending_ && best && best->target != pending_->target
            && best->arrival - zero_time
                < (pending_->arrival - zero_time) * (1.0 - selection_.switch_improvement)
        )
        {
            if (challenger_ != best->target) {
                challenger_ = best->target;
                challenger_since_ = zero_time;
            }
            if (zero_time - challenger_since_ + EPSILON >= selection_.switch_hold_time) {
                pending_ = best;
                challenger_.reset();
                reason = "earlier_handoff";
            }
        } else {
            challenger_.reset();
            if (!pending_)
                reason = "no_feasible_handoff";
        }
    }

    if (!active_ && !pending_) {
        if (capture_locked_ && capture_target_) {
            const auto target = sample(*capture_target_, 0.0);
            if (!target) {
                capture_locked_ = false;
                capture_target_.reset();
            }
        }
        if (!active_ && !capture_locked_) {
            std::vector<PlateEvaluation> evaluations(trajectories.size());
            for (size_t i = 0; i < trajectories.size(); ++i) {
                std::vector<PlateSelectionSample> future;
                for (double t = 0.0; t <= selection_.horizon + EPSILON; t += selection_.dt) {
                    const auto value = sample(i, t);
                    if (!value) {
                        future.clear();
                        break;
                    }
                    future.push_back(*value);
                }
                evaluations[i] = capture_selector_.evaluate(future, initial);
            }
            const auto decision = capture_selector_.choose(evaluations, zero_time);
            if (capture_target_ != decision.index)
                capture_connection_.reset();
            capture_target_ = decision.index;
            if (!capture_target_) {
                reset();
                return std::nullopt;
            }
        }
        if (capture_connection_) {
            const double remaining = capture_connection_->end - zero_time;
            const auto finish = sample(*capture_target_, remaining);
            if (remaining > EPSILON && finish) {
                const auto connection = make_connection(
                    zero_time,
                    capture_connection_->end,
                    anchor,
                    motion_of(*finish)
                );
                if (demand(connection) <= 1.0 + 1e-8)
                    capture_connection_ = connection;
                else
                    capture_connection_.reset();
            } else {
                capture_connection_.reset();
            }
        }
        if (!active_ && !capture_connection_) {
            for (double arrival = selection_.dt; arrival <= selection_.capture_deadline + EPSILON;
                 arrival += selection_.dt)
            {
                const auto finish = sample(*capture_target_, arrival);
                if (!finish || !target_window(*capture_target_, arrival))
                    continue;
                const auto connection =
                    make_connection(zero_time, zero_time + arrival, anchor, motion_of(*finish));
                if (demand(connection) <= 1.0 + 1e-8
                    && demand(make_connection(
                           zero_time,
                           zero_time + arrival,
                           initial,
                           motion_of(*finish)
                       ))
                        <= 1.0 + 1e-8)
                {
                    capture_connection_ = connection;
                    break;
                }
            }
        }
    }

    HandoffPlan output;
    output.zero_time = zero_time;
    output.dt = selection_.dt;
    output.active_plate = active_;
    output.reason = reason;
    if (pending_) {
        output.next_plate = pending_->target;
        output.departure_time = pending_->departure;
        output.arrival_time = pending_->arrival;
    } else if (!active_) {
        output.next_plate = capture_target_;
        output.reason = capture_locked_ ? reason : "capturing";
    }
    if (previous_) {
        for (auto curve: previous_->curves) {
            curve.valid_begin = std::max(curve.valid_begin, zero_time - 0.50);
            curve.valid_end = std::min(curve.valid_end, zero_time);
            if (curve.valid_begin < curve.valid_end)
                output.curves.push_back(curve);
        }
    }
    if (pending_) {
        const auto& connection = pending_->connection;
        output.curves.push_back(
            { connection.begin,
              connection.end,
              std::max(zero_time, connection.begin),
              connection.end,
              connection.start,
              connection.finish,
              HandoffPhase::TRANSFERRING,
              pending_->target }
        );
    } else if (!active_ && capture_connection_ && capture_connection_->end > zero_time) {
        const auto& connection = *capture_connection_;
        output.curves.push_back(
            { connection.begin,
              connection.end,
              zero_time,
              connection.end,
              connection.start,
              connection.finish,
              HandoffPhase::CAPTURING,
              *capture_target_ }
        );
    }
    GimbalSelectionState capture_motion = anchor;
    for (size_t k = 0; k < REFERENCE_COUNT; ++k) {
        const double t = (static_cast<double>(k) - static_cast<double>(MIDPOINT)) * selection_.dt;
        const double absolute = zero_time + t;
        HandoffReference ref;
        if (k < MIDPOINT) {
            const auto history = previous_ ? previous_->at(absolute) : std::nullopt;
            if (history)
                ref = *history;
            else if (active_) {
                const auto value = sample(*active_, t);
                if (!value) {
                    reset();
                    return std::nullopt;
                }
                ref.motion = motion_of(*value);
                ref.acceleration = value->acceleration;
                ref.phase = HandoffPhase::TRACKING;
                ref.tracking_plate = active_;
                ref.aim_plate = *active_;
            } else {
                ref.motion = initial;
                for (size_t axis = 0; axis < 2; ++axis) {
                    ref.motion.angle[axis] += initial.velocity[axis] * t;
                }
                ref.aim_plate = *capture_target_;
            }
        } else if (pending_ && absolute + EPSILON >= pending_->departure) {
            if (absolute < pending_->arrival - EPSILON) {
                ref = connection_at(pending_->connection, absolute);
                ref.phase = HandoffPhase::TRANSFERRING;
            } else {
                const auto value = sample(pending_->target, t);
                const auto endpoint = sample(pending_->target, pending_->arrival - zero_time);
                if (!value || !endpoint) {
                    reset();
                    return std::nullopt;
                }
                ref.motion = motion_of(*value);
                ref.motion.angle[0] += pending_->connection.finish.angle[0] - endpoint->angle[0];
                ref.acceleration = value->acceleration;
                ref.phase = HandoffPhase::TRACKING;
                ref.tracking_plate = pending_->target;
            }
            ref.aim_plate = pending_->target;
        } else if (active_) {
            const auto value = source_reference(t);
            if (!value) {
                reset();
                return std::nullopt;
            }
            ref = *value;
            if (pending_)
                ref.phase = HandoffPhase::SCHEDULED;
        } else {
            const auto target = sample(*capture_target_, t);
            if (!target) {
                reset();
                return std::nullopt;
            }
            if (capture_connection_ && zero_time < capture_connection_->end - EPSILON) {
                if (absolute < capture_connection_->end - EPSILON) {
                    ref = connection_at(*capture_connection_, absolute);
                } else {
                    ref.motion = motion_of(*target);
                    ref.motion.angle[0] = capture_connection_->finish.angle[0]
                        + std::remainder(ref.motion.angle[0] - capture_connection_->finish.angle[0],
                                         TWO_PI);
                    ref.acceleration = target->acceleration;
                }
            } else {
                // 捕获期限已过或尚无完整可行连接时，继续输出加速度受限的转向参考。
                ref.motion = capture_motion;
                for (size_t axis = 0; axis < 2; ++axis) {
                    const double error = axis == 0
                        ? std::remainder(target->angle[axis] - capture_motion.angle[axis], TWO_PI)
                        : target->angle[axis] - capture_motion.angle[axis];
                    const double acceleration = std::clamp(
                        target->acceleration[axis]
                            + 40.0 * (target->velocity[axis] - capture_motion.velocity[axis])
                            + 400.0 * error,
                        -selection_.max_acceleration[axis],
                        selection_.max_acceleration[axis]
                    );
                    ref.acceleration[axis] = acceleration;
                    capture_motion.angle[axis] += capture_motion.velocity[axis] * selection_.dt
                        + 0.5 * acceleration * selection_.dt * selection_.dt;
                    capture_motion.velocity[axis] += acceleration * selection_.dt;
                }
            }
            ref.aim_plate = *capture_target_;
            ref.phase = HandoffPhase::CAPTURING;
            ref.tracking_plate.reset();
        }
        if (!output.references.empty()) {
            ref.motion.angle[0] = output.references.back().motion.angle[0]
                + std::remainder(ref.motion.angle[0] - output.references.back().motion.angle[0],
                                 TWO_PI);
        }
        output.references.push_back(ref);
    }
    previous_ = output;
    return output;
}

std::optional<size_t> HandoffPlanner::fire_target(
    const HandoffPlan& plan,
    const std::vector<PlateTrajectory>& trajectories,
    const GimbalSelectionState& initial,
    double fire_offset
) const {
    const auto now = plan.at(plan.zero_time);
    const auto fire = plan.at(plan.zero_time + fire_offset);
    if (!now || !fire || !now->tracking_plate || now->tracking_plate != fire->tracking_plate
        || now->phase == HandoffPhase::CAPTURING || now->phase == HandoffPhase::TRANSFERRING
        || fire->phase == HandoffPhase::CAPTURING || fire->phase == HandoffPhase::TRANSFERRING
        || *now->tracking_plate >= trajectories.size())
        return std::nullopt;
    const auto& trajectory = trajectories[*now->tracking_plate];
    const auto current = trajectory.at(0.0);
    const auto future = trajectory.at(fire_offset);
    if (!current || !future || !attack_valid(*current) || !attack_valid(*future)
        || !aligned(*current, initial))
        return std::nullopt;
    return now->tracking_plate;
}

} // namespace qd::auto_aim
