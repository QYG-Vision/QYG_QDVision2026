#ifndef ARMOR_SOLVER_HANDOFF_PLANNER_HPP_
#define ARMOR_SOLVER_HANDOFF_PLANNER_HPP_

#include "armor_solver/plate_selector.hpp"

namespace qd::auto_aim {

struct HandoffConfig {
    double max_transfer_time = 0.30;
    double window_margin = 0.02;
};

enum class HandoffPhase { CAPTURING, TRACKING, SCHEDULED, TRANSFERRING };

/** @brief 同一板位的连续参考，时间相对本轮命令时间戳，单位 s。 */
struct PlateTrajectory {
    double first_time = -0.52;
    double dt = 0.01;
    std::vector<PlateSelectionSample> samples;

    /**
     * @brief 在采样范围内以三次 Hermite 插值读取同一板位的参考。
     * @param time 相对命令时间，s。
     * @return 插值参考；越界或非法输入时为空。
     */
    std::optional<PlateSelectionSample> at(double time) const;
};

/** @brief MPC 的运动参考与真实装甲板瞄准目标分开保存。 */
struct HandoffReference {
    GimbalSelectionState motion;
    std::array<double, 2> acceleration {};
    HandoffPhase phase = HandoffPhase::CAPTURING;
    std::optional<size_t> tracking_plate;
    size_t aim_plate = 0;
};

/** @brief 保留三次段的解析表达式及已提交区间，防止重复重采样改变接管边界。 */
struct HandoffCurve {
    double begin = 0.0;
    double end = 0.0;
    double valid_begin = 0.0;
    double valid_end = 0.0;
    GimbalSelectionState start;
    GimbalSelectionState finish;
    HandoffPhase phase = HandoffPhase::TRANSFERRING;
    size_t aim_plate = 0;
};

/** @brief 一轮完整的 100 点中点输出参考及一次换板计划，时间均为绝对秒。 */
struct HandoffPlan {
    double zero_time = 0.0;
    double dt = 0.01;
    std::vector<HandoffReference> references;
    std::vector<HandoffCurve> curves;
    std::optional<size_t> active_plate;
    std::optional<size_t> next_plate;
    std::optional<double> departure_time;
    std::optional<double> arrival_time;
    std::string reason;

    /**
     * @brief 读取计划中的连续运动参考，并保守处理跨阶段的元数据。
     * @param absolute_time 与 zero_time 使用相同时钟的绝对时间，s。
     * @return 计划范围内的参考；越界时为空。
     */
    std::optional<HandoffReference> at(double absolute_time) const;
};

/** @brief 有状态的单次预测换板器；不改变底层 MPC 的模型和初始化方式。 */
class HandoffPlanner {
public:
    /**
     * @brief 初始化并校验选板与换板参数。
     * @param selection 两轴角加速度、攻击窗口及误差阈值。
     * @param handoff 单次转移时长和窗口余量，s。
     */
    HandoffPlanner(const PlateSelectionConfig& selection, const HandoffConfig& handoff);

    /**
     * @brief 生成当前板、转移段及下一板组成的滚动参考。
     * @param trajectories 固定板位顺序的轨迹，覆盖 [-0.50, horizon] 及差分端点。
     * @param initial 实际反馈外推到本轮命令时刻的状态，rad、rad/s。
     * @param zero_time 本轮命令的绝对参考时间，s；回退或中断超过 0.5 s 时重置。
     * @return 100 点计划；无有效候选时为空。
     * @note 转移开始后保持接管目标，接管须由实际反馈确认。
     */
    std::optional<HandoffPlan> plan(
        const std::vector<PlateTrajectory>& trajectories,
        const GimbalSelectionState& initial,
        double zero_time
    );

    /**
     * @brief 检查当前及火控检查点是否属于同一真实板的有效跟踪阶段。
     * @param plan 本轮计划。
     * @param trajectories 同轮候选板的真实瞄准轨迹。
     * @param initial 同轮命令时刻的实际状态外推值。
     * @param fire_offset 火控检查点相对命令时刻的时间，默认 0.02 s。
     * @return 满足阶段、朝向、反馈误差条件的真实板位；转移/捕获时为空。
     * @note 调用方还须比较 MPC 预测角与该板的几何瞄准角。
     */
    std::optional<size_t> fire_target(
        const HandoffPlan& plan,
        const std::vector<PlateTrajectory>& trajectories,
        const GimbalSelectionState& initial,
        double fire_offset = 0.02
    ) const;

    /** @brief 清除当前板、未完成转移、参考历史与切换滞回。 */
    void reset();

private:
    struct Window {
        double begin;
        double end;
        double center;
    };
    struct Connection {
        double begin = 0.0;
        double end = 0.0;
        GimbalSelectionState start;
        GimbalSelectionState finish;
    };
    struct Pending {
        size_t source = 0;
        size_t target = 0;
        double departure = 0.0;
        double arrival = 0.0;
        double window_duration = 0.0;
        double demand = 0.0;
        double distance = 0.0;
        Connection connection;
    };

    /** @brief 判断真实参考是否处于朝向及加速度窗口。 @return 合格时为 true。 */
    bool attack_valid(const PlateSelectionSample& sample) const;
    /** @brief 检查实际反馈是否跟住真实板。 @return 两轴误差均合格时为 true。 */
    bool aligned(const PlateSelectionSample& sample, const GimbalSelectionState& initial) const;
    /** @brief 提取完整可攻击区间及各区间最正面的采样时刻。 @return 时间相对本轮零点。 */
    std::vector<Window> windows(const PlateTrajectory& trajectory) const;
    /** @brief 求三次连接的最大加速度需求比。 @return 大于 1 表示超限。 */
    double demand(const Connection& connection) const;
    /** @brief 读取三次连接的运动状态。 @return 角度、速度、加速度参考。 */
    HandoffReference connection_at(const Connection& connection, double time) const;

    PlateSelectionConfig selection_;
    HandoffConfig handoff_;
    PlateSelector capture_selector_;
    std::optional<size_t> active_;
    std::optional<size_t> capture_target_;
    bool capture_locked_ = false;
    double capture_not_before_ = 0.0;
    std::optional<Connection> capture_connection_;
    std::optional<Pending> pending_;
    std::optional<size_t> challenger_;
    double challenger_since_ = 0.0;
    std::optional<HandoffPlan> previous_;
    size_t plate_count_ = 0;
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_HANDOFF_PLANNER_HPP_
