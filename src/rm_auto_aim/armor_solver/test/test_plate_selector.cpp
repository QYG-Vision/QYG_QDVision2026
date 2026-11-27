#include "armor_solver/handoff_planner.hpp"
#include "armor_solver/plate_selector.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace qd::auto_aim {
namespace {
    constexpr double PI = 3.14159265358979323846;

    /** @brief 检查独立算法测试断言，失败时携带上下文抛出异常。 */
    void require(bool condition, const std::string& message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    /**
 * @brief 生成覆盖默认时域的匀角速度单板参考。
 * @param yaw 零点 yaw，rad。
 * @param velocity yaw 角速度，rad/s。
 * @return 51 个连续参考点；pitch 为零，板面朝向己方。
 */
    std::vector<PlateSelectionSample> trajectory(double yaw, double velocity = 0.0) {
        std::vector<PlateSelectionSample> samples(51);
        for (size_t k = 0; k < samples.size(); ++k) {
            samples[k].angle[0] = yaw + static_cast<double>(k) * 0.01 * velocity;
            samples[k].velocity[0] = velocity;
            samples[k].horizontal_distance = 5.0;
        }
        return samples;
    }

    /** @brief 最近板不可达而稍远板可达时，选择可达板。 */
    void selects_reachable_instead_of_nearest() {
        PlateSelectionConfig config;
        config.max_acceleration = { 2.0, 2.0 };
        PlateSelector selector(config);
        auto near = trajectory(0.5);
        near.front().horizontal_distance = 3.0;
        const auto a = selector.evaluate(near, {});
        const auto b = selector.evaluate(trajectory(0.025), {});
        require(!a.reachable && b.reachable, "expected different reachability");
        require(selector.choose({ a, b }, 1.0).index == 1, "selected unreachable nearest plate");
    }

    /** @brief 相同角度差下，云台反向运动需要更长的捕获时间。 */
    void accounts_for_actual_velocity_direction() {
        PlateSelectionConfig config;
        config.max_acceleration = { 15.0, 15.0 };
        PlateSelector selector(config);
        GimbalSelectionState forward, backward;
        forward.velocity[0] = 0.3;
        backward.velocity[0] = -0.3;
        const auto a = selector.evaluate(trajectory(0.1), forward);
        const auto b = selector.evaluate(trajectory(0.1), backward);
        require(a.reachable && b.reachable, "both directions should be capturable");
        require(a.capture_time < b.capture_time, "reverse motion did not delay capture");
    }

    /** @brief yaw 可达但 pitch 不可达时，不得判为可追踪。 */
    void requires_both_axes() {
        PlateSelectionConfig config;
        config.max_acceleration = { 10.0, 1.0 };
        PlateSelector selector(config);
        auto samples = trajectory(0.0);
        for (auto& sample: samples) {
            sample.angle[1] = 1.0;
        }
        require(!selector.evaluate(samples, {}).reachable, "ignored pitch acceleration limit");
    }

    /** @brief 背面板未来转入攻击窗口时允许预选，但不得提前开火。 */
    void anticipates_future_facing_window() {
        PlateSelector selector(PlateSelectionConfig {});
        auto samples = trajectory(0.0);
        for (size_t k = 0; k < samples.size(); ++k) {
            samples[k].facing_angle = k < 20 ? PI : 0.0;
        }
        const auto evaluation = selector.evaluate(samples, {});
        require(evaluation.reachable, "future facing window was discarded");
        require(std::abs(evaluation.capture_time - 0.2) < 1e-9, "wrong future window start");
        require(!evaluation.ready_to_fire, "allowed fire before future window");
        require(selector.choose({ evaluation }, 1.0).index == 0, "did not preselect future plate");
    }

    /** @brief 短暂朝向合格或跟踪加速度过大，都不能满足连续保持条件。 */
    void requires_continuous_window() {
        PlateSelector selector(PlateSelectionConfig {});
        auto samples = trajectory(0.0);
        for (size_t k = 0; k < samples.size(); ++k) {
            samples[k].facing_angle = k < 5 ? 0.0 : PI;
        }
        require(!selector.evaluate(samples, {}).reachable, "accepted fleeting attack window");
        samples = trajectory(0.0);
        for (auto& sample: samples) {
            sample.acceleration[0] = 1000.0;
        }
        const auto evaluation = selector.evaluate(samples, {});
        require(
            !evaluation.reachable && !evaluation.ready_to_fire,
            "ignored sustained tracking acceleration"
        );
    }

    /** @brief 全部不可达仍选捕获目标；更优目标需满足改善比例及持续时间。 */
    void captures_with_hysteresis() {
        PlateSelector selector(PlateSelectionConfig {});
        PlateEvaluation a, b;
        a.valid = b.valid = true;
        a.demand_ratio = 2.0;
        b.demand_ratio = 3.0;
        require(selector.choose({ a, b }, 1.0).index == 0, "wrong initial capture");
        b.demand_ratio = 1.8;
        require(selector.choose({ a, b }, 1.01).index == 0, "switched for small improvement");
        b.demand_ratio = 1.5;
        require(selector.choose({ a, b }, 1.02).index == 0, "switched before hold");
        require(selector.choose({ a, b }, 1.06).index == 0, "hold elapsed too early");
        require(selector.choose({ a, b }, 1.07).index == 1, "did not switch after hold");
    }

    /** @brief 已可达的当前板保持不变，时间回退或显式重置后重新决策。 */
    void preserves_plate_and_resets() {
        PlateSelector selector(PlateSelectionConfig {});
        auto a = selector.evaluate(trajectory(0.01), {});
        auto b = a;
        a.capture_time = 0.2;
        b.capture_time = 0.3;
        require(selector.choose({ a, b }, 2.0).index == 0, "wrong first plate");
        b.capture_time = 0.0;
        require(selector.choose({ a, b }, 2.1).index == 0, "failed to hold reachable plate");
        require(selector.choose({ a, b }, 1.0).index == 1, "clock rewind did not reset");
        selector.reset();
        a.capture_time = 0.0;
        b.capture_time = 0.1;
        require(selector.choose({ a, b }, 3.0).index == 0, "explicit reset did not clear plate");
    }

    /** @brief yaw 跨界应沿同一连续分支评估，不应产生整圈运动。 */
    void handles_yaw_seam() {
        PlateSelector selector(PlateSelectionConfig {});
        GimbalSelectionState initial;
        initial.angle[0] = PI - 0.01;
        const auto evaluation = selector.evaluate(trajectory(-PI + 0.01), initial);
        require(evaluation.reachable, "seam caused artificial full turn");
        require(evaluation.angular_error < 0.021, "wrong seam error");
    }

    /** @brief 正面、侧面、背面的夹角遵守现有板位几何约定。 */
    void uses_correct_facing_convention() {
        require(
            std::abs(PlateSelector::facing_angle(0.0, 4.8, 0.0)) < 1e-12,
            "front normal reversed"
        );
        require(
            std::abs(std::abs(PlateSelector::facing_angle(PI, 5.2, 0.0)) - PI) < 1e-12,
            "rear plate treated as front"
        );
        require(
            std::abs(PlateSelector::facing_angle(PI / 2.0, 5.0, -0.2)) > PI / 2.0,
            "wrong side facing geometry"
        );
    }

    /** @brief 反馈角速度误差未收敛时不能因角度对准而开火。 */
    void gates_fire_on_velocity_error() {
        PlateSelector selector(PlateSelectionConfig {});
        GimbalSelectionState initial;
        initial.velocity[0] = 0.3;
        const auto evaluation = selector.evaluate(trajectory(0.0), initial);
        require(
            evaluation.reachable && !evaluation.ready_to_fire,
            "ignored initial velocity mismatch"
        );
        require(
            selector.evaluate(trajectory(0.0), {}).ready_to_fire,
            "stationary aligned plate should be ready"
        );
    }

    /** @brief 无朝向窗口时按角度误差捕获，并保持现有捕获板。 */
    void captures_without_facing_window() {
        PlateSelector selector(PlateSelectionConfig {});
        auto a = trajectory(0.2), b = trajectory(0.1);
        for (auto* samples: { &a, &b }) {
            for (auto& sample: *samples) {
                sample.facing_angle = PI;
            }
        }
        const auto ea = selector.evaluate(a, {}), eb = selector.evaluate(b, {});
        require(!ea.reachable && !eb.reachable, "rear plate was reachable");
        require(selector.choose({ ea, eb }, 1.0).index == 1, "wrong angular fallback");
        require(selector.choose({ eb, ea }, 1.1).index == 1, "failed to keep windowless capture");
    }

    /** @brief 非法配置、短轨迹及非有限输入应被拒绝。 */
    void rejects_invalid_inputs() {
        for (const double invalid: { 0.0, -1.0, std::numeric_limits<double>::quiet_NaN() }) {
            PlateSelectionConfig config;
            config.max_acceleration[0] = invalid;
            bool rejected = false;
            try {
                PlateSelector selector(config);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "accepted invalid acceleration config");
        }
        PlateSelector selector(PlateSelectionConfig {});
        require(!selector.evaluate({}, {}).valid, "accepted empty trajectory");
        auto samples = trajectory(0.0);
        samples[10].angle[0] = std::numeric_limits<double>::quiet_NaN();
        require(!selector.evaluate(samples, {}).valid, "accepted NaN trajectory");
        require(!selector.choose({ PlateEvaluation {} }, 1.0).index, "selected invalid plate");
    }

    /** @brief 第一版明确不施加角速度上限；匹配的高角速度不应被暗中拒绝。 */
    void models_acceleration_only() {
        PlateSelector selector(PlateSelectionConfig {});
        GimbalSelectionState initial;
        initial.velocity[0] = 20.0;
        const auto evaluation = selector.evaluate(trajectory(0.0, 20.0), initial);
        require(evaluation.ready_to_fire, "unexpected velocity bound in acceleration-only model");
    }

    /**
 * @brief 生成含历史的匀加速度板轨迹，用于验证换板时序及跨轮参考。
 * @param yaw 全局零点的 yaw，rad。
 * @param velocity 全局零点的 yaw 角速度，rad/s。
 * @param acceleration yaw 角加速度，rad/s²。
 * @param facing 全局零点板面夹角，rad。
 * @param facing_rate 板面夹角变化率，rad/s。
 * @param elapsed 本轮相对全局零点的时间，s。
 * @return 覆盖 [-0.52, 0.52] 的单板轨迹。
 */
    PlateTrajectory handoff_trajectory(
        double yaw,
        double velocity,
        double acceleration,
        double facing,
        double facing_rate,
        double elapsed = 0.0
    ) {
        PlateTrajectory result;
        for (int k = -52; k <= 52; ++k) {
            const double t = k * 0.01 + elapsed;
            PlateSelectionSample value;
            value.angle[0] = yaw + velocity * t + 0.5 * acceleration * t * t;
            value.velocity[0] = velocity + acceleration * t;
            value.acceleration[0] = acceleration;
            value.facing_angle = facing + facing_rate * t;
            value.horizontal_distance = 5.0;
            result.samples.push_back(value);
        }
        return result;
    }

    /** @brief 构造当前板即将离开、下一板正在进入的场景。 @return 两板连续预测。 */
    std::vector<PlateTrajectory> handoff_scene(double elapsed = 0.0) {
        return { handoff_trajectory(0.0, 0.0, 0.0, 0.6, 3.0, elapsed),
                 handoff_trajectory(0.03, 0.0, 0.0, -0.6, 3.0, elapsed) };
    }

    /** @brief 使用当前仓库 yaw 加速度限制。 @return 两轴评估配置。 */
    PlateSelectionConfig handoff_selection() {
        PlateSelectionConfig config;
        config.max_acceleration = { 15.0, 100.0 };
        return config;
    }

    /** @brief 当前板还能攻击时，窗口已包含转移及下一板。 */
    void previews_handoff_before_current_plate_exits() {
        HandoffPlanner planner(handoff_selection(), {});
        const auto plates = handoff_scene();
        const auto plan = planner.plan(plates, {}, 1.0);
        require(plan && plan->next_plate == 1 && plan->active_plate == 0, "no future takeover");
        require(
            plan->departure_time && *plan->departure_time < 1.14,
            "departed after current exit"
        );
        require(plan->arrival_time && *plan->arrival_time < 1.3, "missed next plate");
        bool old = false, transfer = false, next = false;
        for (const auto& ref: plan->references) {
            old = old || ref.tracking_plate == 0;
            transfer = transfer || ref.phase == HandoffPhase::TRANSFERRING;
            next = next || ref.tracking_plate == 1;
            require(std::abs(ref.acceleration[0]) <= 15.0 + 1e-7, "transfer exceeded yaw limit");
        }
        require(old && transfer && next, "MPC window does not contain all three segments");
        require(!planner.fire_target(*plan, plates, {}), "allowed firing during transfer");
    }

    /** @brief 连接角度和角速度在出发及接管处连续。 */
    void handoff_is_continuous() {
        HandoffPlanner planner(handoff_selection(), {});
        const auto plan = planner.plan(handoff_scene(), {}, 1.0);
        require(plan && plan->arrival_time && plan->departure_time, "no connection");
        const auto start = plan->at(*plan->departure_time);
        const auto end = plan->at(*plan->arrival_time);
        require(start && end, "missing endpoints");
        require(std::abs(start->motion.angle[0]) < 1e-9, "jump at departure");
        require(std::abs(start->motion.velocity[0]) < 1e-9, "velocity jump at departure");
        require(std::abs(end->motion.angle[0] - 0.03) < 1e-9, "jump at arrival");
        require(std::abs(end->motion.velocity[0]) < 1e-9, "velocity jump at arrival");
        const auto just_before = plan->at(*plan->arrival_time - 0.00001);
        require(
            just_before && std::abs(just_before->motion.angle[0] - 0.03) < 1e-6
                && std::abs(just_before->motion.velocity[0]) < 0.001,
            "connection is not C1"
        );
    }

    /** @brief 以 4 ms 重算时保持绝对接管时间，历史和转向目标不回跳。 */
    void rolling_handoff_keeps_deadline_and_history() {
        HandoffPlanner planner(handoff_selection(), {});
        auto plan = planner.plan(handoff_scene(), {}, 1.0);
        require(plan && plan->arrival_time, "no initial transfer");
        const double arrival = *plan->arrival_time;
        for (double elapsed = 0.004; 1.0 + elapsed < arrival - 0.001; elapsed += 0.004) {
            const auto actual = plan->at(1.0 + elapsed);
            require(actual.has_value(), "previous reference lost");
            const auto next = planner.plan(handoff_scene(elapsed), actual->motion, 1.0 + elapsed);
            require(
                next && next->next_plate == 1 && next->arrival_time,
                "transfer target or deadline discarded at " + std::to_string(elapsed)
                    + (next ? ": " + next->reason : ": no plan")
            );
            require(std::abs(*next->arrival_time - arrival) < 1e-8, "receding deadline");
            const auto old_history = plan->at(1.0 + elapsed - 0.1);
            const auto new_history = next->at(1.0 + elapsed - 0.1);
            require(
                old_history && new_history
                    && std::abs(old_history->motion.angle[0] - new_history->motion.angle[0]) < 1e-9,
                "rewrote committed reference history"
            );
            require(
                std::abs(next->references[50].motion.angle[0] - actual->motion.angle[0]) < 1e-8
                    && std::abs(
                           next->references[50].motion.velocity[0] - actual->motion.velocity[0]
                       ) < 1e-8,
                "reference jumped on replan"
            );
            plan = next;
        }
    }

    /** @brief 接管需要实际反馈；未确认时不能切回旧板或开火。 */
    void requires_actual_takeover_confirmation() {
        HandoffPlanner planner(handoff_selection(), {});
        const auto first = planner.plan(handoff_scene(), {}, 1.0);
        require(first && first->arrival_time, "no transfer");
        const double elapsed = *first->arrival_time - 1.0;
        const auto missed = planner.plan(handoff_scene(elapsed), {}, 1.0 + elapsed);
        require(
            missed && !missed->active_plate && missed->next_plate == 1,
            "took over without actual feedback or returned to old plate"
        );
        require(
            missed->references[50].phase == HandoffPhase::CAPTURING,
            "unconfirmed takeover not capturing"
        );
        require(!planner.fire_target(*missed, handoff_scene(elapsed), {}), "unconfirmed fire");
        GimbalSelectionState actual;
        actual.angle[0] = 0.03;
        const auto confirmed =
            planner.plan(handoff_scene(elapsed + 0.004), actual, 1.0 + elapsed + 0.004);
        require(confirmed && confirmed->active_plate == 1, "actual takeover was not confirmed");
        require(
            !confirmed->next_plate || *confirmed->next_plate != 0,
            "scheduled a reverse switch"
        );
    }

    /** @brief 下一板尚在背面时允许提前转移。 */
    void anticipates_unseen_next_plate() {
        HandoffPlanner planner(handoff_selection(), {});
        auto plates = handoff_scene();
        plates[1] = handoff_trajectory(0.03, 0.0, 0.0, -1.5, 5.0);
        const auto plan = planner.plan(plates, {}, 1.0);
        require(plan && plan->next_plate == 1, "discarded future incoming plate");
        require(*plan->departure_time < 1.09, "did not start preparing for future window");
        require(!planner.fire_target(*plan, plates, {}), "fired at future-only window");
    }

    /** @brief 下一板窗口过短或 pitch 不可达时，保留当前跟踪。 */
    void rejects_short_or_unreachable_takeover() {
        HandoffPlanner planner(handoff_selection(), {});
        auto plates = handoff_scene();
        for (auto& value: plates[1].samples)
            value.angle[1] = 10.0;
        auto plan = planner.plan(plates, {}, 1.0);
        require(plan && !plan->next_plate && plan->active_plate == 0, "ignored pitch limit");
        planner.reset();
        plates = handoff_scene();
        for (size_t k = 0; k < plates[1].samples.size(); ++k) {
            const double t = plates[1].first_time + k * plates[1].dt;
            plates[1].samples[k].facing_angle = t >= 0.1 && t <= 0.15 ? 0.0 : PI;
        }
        plan = planner.plan(plates, {}, 1.0);
        require(plan && !plan->next_plate, "accepted short takeover window");
    }

    /** @brief 静止目标没有后续窗口，不因邻板也能跟踪而反复换板。 */
    void stationary_target_does_not_handoff() {
        HandoffPlanner planner(handoff_selection(), {});
        const std::vector<PlateTrajectory> plates { handoff_trajectory(0.0, 0.0, 0.0, 0.0, 0.0),
                                                    handoff_trajectory(0.03, 0.0, 0.0, 0.5, 0.0) };
        const auto plan = planner.plan(plates, {}, 1.0);
        require(
            plan && plan->active_plate == 0 && !plan->next_plate,
            "unnecessary stationary switch"
        );
        require(planner.fire_target(*plan, plates, {}) == 0, "aligned static target cannot fire");
    }

    /** @brief 角度跨界时转移沿最短分支，不能出现整圈跳变。 */
    void handoff_handles_yaw_seam() {
        HandoffPlanner planner(handoff_selection(), {});
        auto plates = handoff_scene();
        plates[0] = handoff_trajectory(PI - 0.01, 0.0, 0.0, 0.6, 3.0);
        plates[1] = handoff_trajectory(-PI + 0.02, 0.0, 0.0, -0.6, 3.0);
        GimbalSelectionState actual;
        actual.angle[0] = PI - 0.01;
        const auto plan = planner.plan(plates, actual, 1.0);
        require(plan && plan->next_plate == 1, "no seam transfer");
        for (size_t i = 1; i < plan->references.size(); ++i) {
            require(
                std::abs(
                    plan->references[i].motion.angle[0] - plan->references[i - 1].motion.angle[0]
                ) < 0.01,
                "yaw seam jump"
            );
        }
    }

    /** @brief 时间回退、板数量改变及显式重置清除旧的待接管状态。 */
    void resets_pending_handoff() {
        HandoffPlanner planner(handoff_selection(), {});
        require(planner.plan(handoff_scene(), {}, 2.0).has_value(), "initial plan missing");
        const std::vector<PlateTrajectory> stationary { handoff_trajectory(0, 0, 0, 0, 0) };
        const auto reset = planner.plan(stationary, {}, 1.0);
        require(
            reset && reset->active_plate == 0 && !reset->next_plate,
            "failed clock/model reset"
        );
        planner.reset();
        require(!planner.plan({}, {}, 3.0), "accepted empty model");
    }

    /** @brief 只剩较短当前窗口时仍应提前转移，不应先按 hold_time 丢弃当前板。 */
    void handles_short_remaining_current_window() {
        HandoffPlanner planner(handoff_selection(), {});
        auto plates = handoff_scene();
        plates[0] = handoff_trajectory(0.0, 0.0, 0.0, 0.9, 3.0);
        const auto plan = planner.plan(plates, {}, 1.0);
        require(
            plan && plan->active_plate == 0 && plan->next_plate == 1,
            "lost short current window before planning handoff"
        );
        require(*plan->departure_time < 1.03, "departed too late for short current window");
    }

    /** @brief 临近窗口入口的接管计划不能因滚动采样网格变化而被不断推迟。 */
    void preserves_deadline_near_window_boundary() {
        HandoffPlanner planner(handoff_selection(), {});
        auto scene = [](double elapsed) {
            auto plates = handoff_scene(elapsed);
            plates[1] =
                handoff_trajectory(0.03, 0.0, 0.0, -1.0471975511965976 - 3.0 * 0.099, 3.0, elapsed);
            return plates;
        };
        auto plan = planner.plan(scene(0.0), {}, 1.0);
        require(plan && plan->arrival_time, "no boundary plan");
        const double arrival = *plan->arrival_time;
        for (double elapsed = 0.004; 1.0 + elapsed < arrival - 0.001; elapsed += 0.004) {
            const auto actual = plan->at(1.0 + elapsed);
            require(actual.has_value(), "missing boundary reference");
            plan = planner.plan(scene(elapsed), actual->motion, 1.0 + elapsed);
            require(
                plan && plan->arrival_time && std::abs(*plan->arrival_time - arrival) < 1e-8,
                "window quantization postponed the takeover"
            );
        }
    }

    /** @brief 实际 ROS 秒级时间戳下，参考索引及固定接管时间保持正确。 */
    void handles_epoch_timestamps() {
        HandoffPlanner planner(handoff_selection(), {});
        constexpr double EPOCH = 1795700000.0;
        const auto plan = planner.plan(handoff_scene(), {}, EPOCH);
        require(plan && plan->arrival_time, "no epoch plan");
        require(plan->at(EPOCH + 0.02).has_value(), "lost fire index at epoch timestamp");
        const auto actual = plan->at(EPOCH + 0.004);
        require(actual.has_value(), "lost fractional epoch reference");
        const auto next = planner.plan(handoff_scene(0.004), actual->motion, EPOCH + 0.004);
        require(
            next && next->arrival_time && *next->arrival_time == *plan->arrival_time,
            "epoch deadline changed"
        );
    }

    /** @brief 实际反向角速度会推迟可行接管，并产生出发前的计划阶段。 */
    void schedules_departure_from_actual_velocity() {
        HandoffPlanner forward_planner(handoff_selection(), {});
        HandoffPlanner backward_planner(handoff_selection(), {});
        GimbalSelectionState forward, backward;
        forward.velocity[0] = 0.09;
        backward.velocity[0] = -0.09;
        const auto a = forward_planner.plan(handoff_scene(), forward, 1.0);
        const auto b = backward_planner.plan(handoff_scene(), backward, 1.0);
        require(
            a && b && a->arrival_time && b->arrival_time && b->departure_time,
            "no velocity-dependent schedules"
        );
        require(*a->arrival_time < *b->arrival_time, "ignored actual reversal in handoff");
        require(*b->departure_time > 1.0, "missing scheduled phase before departure");
        require(
            b->references[50].phase == HandoffPhase::SCHEDULED
                && b->references[50].tracking_plate == 0,
            "replaced current plate before departure"
        );
        require(
            !backward_planner.fire_target(*b, handoff_scene(), backward),
            "allowed fire when fire checkpoint falls into transfer"
        );
        const auto current = b->at(1.004);
        require(current.has_value(), "missing scheduled reference");
        const auto next = backward_planner.plan(handoff_scene(0.004), current->motion, 1.004);
        require(
            next && next->departure_time == b->departure_time
                && next->arrival_time == b->arrival_time,
            "moved a scheduled absolute deadline"
        );
    }

    /**
 * @brief 生成绕车中心旋转的真实几何参考，导数使用解析表达式。
 * @param plate 模型板位下标。
 * @param yaw_rate 目标转速，rad/s。
 * @return 四板模型中的一块板轨迹。
 */
    PlateTrajectory rotating_plate(size_t plate, double yaw_rate) {
        PlateTrajectory trajectory;
        for (int k = -52; k <= 52; ++k) {
            const double phase = std::copysign(0.5, yaw_rate) + yaw_rate * k * 0.01
                + static_cast<double>(plate) * PI / 2.0;
            const double x = 5.2 - 0.2 * std::cos(phase), y = -0.2 * std::sin(phase);
            const double vx = 0.2 * std::sin(phase) * yaw_rate;
            const double vy = -0.2 * std::cos(phase) * yaw_rate;
            const double ax = 0.2 * std::cos(phase) * yaw_rate * yaw_rate;
            const double ay = 0.2 * std::sin(phase) * yaw_rate * yaw_rate;
            const double distance2 = x * x + y * y;
            PlateSelectionSample sample;
            sample.angle[0] = std::atan2(y, x);
            sample.velocity[0] = (x * vy - y * vx) / distance2;
            sample.acceleration[0] = (x * ay - y * ax) / distance2
                - (x * vy - y * vx) * 2.0 * (x * vx + y * vy) / (distance2 * distance2);
            sample.facing_angle = PlateSelector::facing_angle(phase, x, y);
            sample.horizontal_distance = std::sqrt(distance2);
            trajectory.samples.push_back(sample);
        }
        return trajectory;
    }

    /** @brief 真实旋转几何按运动方向选择后续窗口，且参考确实在当前窗口结束前转向。 */
    void hands_off_rotating_geometry_in_both_directions() {
        for (const double rate: { 3.0, -3.0 }) {
            HandoffPlanner planner(handoff_selection(), {});
            std::vector<PlateTrajectory> plates;
            for (size_t plate = 0; plate < 4; ++plate)
                plates.push_back(rotating_plate(plate, rate));
            const auto current = plates[0].at(0.0);
            require(current.has_value(), "missing geometry");
            const auto plan = planner.plan(plates, { current->angle, current->velocity }, 1.0);
            require(
                plan && plan->next_plate == (rate > 0.0 ? 3 : 1),
                "wrong rotation successor at " + std::to_string(rate)
                    + (plan ? ": " + plan->reason + " next="
                               + (plan->next_plate ? std::to_string(*plan->next_plate) : "none")
                            : ": no plan")
            );
            const double departure = *plan->departure_time - 1.0;
            const auto leaving = plates[0].at(departure);
            require(
                leaving && std::abs(leaving->facing_angle) < 1.0471975511965976,
                "waited until current plate left attack window"
            );
            const auto moving = plan->at(*plan->departure_time + 0.02);
            require(
                moving && moving->phase == HandoffPhase::TRANSFERRING,
                "no advance transfer in MPC reference"
            );
        }
    }

    /** @brief 已出发计划实际无法按时完成时，转为捕获原接管目标而不折返旧板。 */
    void recovers_missed_transfer_without_reversing_target() {
        HandoffPlanner planner(handoff_selection(), {});
        const auto first = planner.plan(handoff_scene(), {}, 1.0);
        require(first && first->arrival_time, "no transfer");
        GimbalSelectionState actual;
        actual.angle[0] = -0.5;
        actual.velocity[0] = -1.0;
        const auto recovery = planner.plan(handoff_scene(0.04), actual, 1.04);
        require(
            recovery && recovery->next_plate == 1 && !recovery->active_plate,
            "returned to old plate during failed transfer"
        );
        require(
            recovery->references[50].phase == HandoffPhase::CAPTURING
                && !planner.fire_target(*recovery, handoff_scene(0.04), actual),
            "failed transfer was not fire-gated"
        );
        const auto old_reference = first->at(1.04);
        require(
            old_reference
                && std::abs(
                       recovery->references[50].motion.angle[0] - old_reference->motion.angle[0]
                   ) < 1e-9,
            "recovery reference jumped"
        );
    }

    /** @brief 换板配置必须具有正转移时间及可容纳保持区间的余量。 */
    void rejects_invalid_handoff_configuration() {
        for (const double duration: { 0.0, -0.01, 0.50, std::numeric_limits<double>::quiet_NaN() })
        {
            HandoffConfig config;
            config.max_transfer_time = duration;
            bool rejected = false;
            try {
                HandoffPlanner planner(handoff_selection(), config);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "invalid handoff duration accepted");
        }
        HandoffPlanner planner(handoff_selection(), {});
        GimbalSelectionState invalid;
        invalid.velocity[1] = std::numeric_limits<double>::quiet_NaN();
        require(!planner.plan(handoff_scene(), invalid, 1.0), "invalid actual state accepted");
    }

} // namespace
} // namespace qd::auto_aim

int main() {
    using namespace qd::auto_aim;
    const std::vector<std::pair<std::string, std::function<void()>>> tests {
        { "reachable over nearest", selects_reachable_instead_of_nearest },
        { "actual velocity direction", accounts_for_actual_velocity_direction },
        { "both axes", requires_both_axes },
        { "future facing window", anticipates_future_facing_window },
        { "continuous window", requires_continuous_window },
        { "capture hysteresis", captures_with_hysteresis },
        { "plate persistence and reset", preserves_plate_and_resets },
        { "yaw seam", handles_yaw_seam },
        { "facing convention", uses_correct_facing_convention },
        { "velocity error fire gate", gates_fire_on_velocity_error },
        { "windowless capture", captures_without_facing_window },
        { "invalid input", rejects_invalid_inputs },
        { "acceleration only", models_acceleration_only },
        { "handoff preview", previews_handoff_before_current_plate_exits },
        { "handoff continuity", handoff_is_continuous },
        { "rolling deadline and history", rolling_handoff_keeps_deadline_and_history },
        { "actual takeover confirmation", requires_actual_takeover_confirmation },
        { "unseen incoming plate", anticipates_unseen_next_plate },
        { "invalid next window", rejects_short_or_unreachable_takeover },
        { "stationary target", stationary_target_does_not_handoff },
        { "handoff yaw seam", handoff_handles_yaw_seam },
        { "handoff reset", resets_pending_handoff },
        { "short current window", handles_short_remaining_current_window },
        { "rolling window boundary", preserves_deadline_near_window_boundary },
        { "epoch timestamps", handles_epoch_timestamps },
        { "scheduled actual velocity", schedules_departure_from_actual_velocity },
        { "rotating plate geometry", hands_off_rotating_geometry_in_both_directions },
        { "missed transfer recovery", recovers_missed_transfer_without_reversing_target },
        { "handoff configuration", rejects_invalid_handoff_configuration }
    };
    size_t failures = 0;
    for (const auto& test: tests) {
        try {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " passed\n";
    return failures == 0 ? 0 : 1;
}
