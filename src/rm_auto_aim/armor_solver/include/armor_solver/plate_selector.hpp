#ifndef ARMOR_SOLVER_PLATE_SELECTOR_HPP_
#define ARMOR_SOLVER_PLATE_SELECTOR_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace qd::auto_aim {

/** @brief 选板参数；角度为 rad，时间为 s，两轴顺序为 yaw、pitch。 */
struct PlateSelectionConfig {
    double dt = 0.01;
    double horizon = 0.50;
    double capture_deadline = 0.30;
    double hold_time = 0.10;
    double angle_tolerance = 0.02;
    double velocity_tolerance = 0.10;
    double max_facing_angle = 1.0471975511965976;
    double switch_improvement = 0.20;
    double switch_hold_time = 0.05;
    std::array<double, 2> max_acceleration {50.0, 100.0};
};

/** @brief 云台在命令参考时刻的实际状态外推值；pitch 向下为正。 */
struct GimbalSelectionState {
    std::array<double, 2> angle {};
    std::array<double, 2> velocity {};
};

/** @brief 同一物理板位的命中预测采样；以命令时刻为零点，等间隔排列。 */
struct PlateSelectionSample {
    std::array<double, 2> angle {};
    std::array<double, 2> velocity {};
    std::array<double, 2> acceleration {};
    double facing_angle = 0.0;
    double horizontal_distance = 0.0;
};

/** @brief 单板的加速度模型评估；不可达不代表不存在其他可行控制曲线。 */
struct PlateEvaluation {
    bool valid = false;
    bool reachable = false;
    bool ready_to_fire = false;
    double capture_time = std::numeric_limits<double>::infinity();
    double window_duration = 0.0;
    double demand_ratio = std::numeric_limits<double>::infinity();
    double angular_error = std::numeric_limits<double>::infinity();
    double horizontal_distance = std::numeric_limits<double>::infinity();
    std::string reason = "invalid_trajectory";
};

struct PlateSelectionDecision {
    std::optional<size_t> index;
    std::string reason = "no_valid_plate";
};

/** @brief 独立于 ROS/MPC 的单板可达性评估和有状态选板器。 */
class PlateSelector {
public:
    /**
     * @brief 创建评估器并校验参数。
     * @param config 评估时间、阈值及两轴角加速度限制。
     * @throws std::invalid_argument 参数非法时抛出。
     */
    explicit PlateSelector(const PlateSelectionConfig& config);

    /**
     * @brief 评估同一板位的未来捕获和连续攻击窗口。
     * @param trajectory 从命令时刻起、覆盖 horizon 的等间隔参考，yaw 已连续展开。
     * @param initial 同一命令时刻的云台状态，坐标系和正方向须与参考一致。
     * @return 可达性、连接加速度需求及当前是否具备开火附加条件。
     * @note 仅限制角加速度，不限制角速度；连接曲线不作为执行指令输出。
     */
    PlateEvaluation evaluate(const std::vector<PlateSelectionSample>& trajectory,
                             const GimbalSelectionState& initial) const;

    /**
     * @brief 优先保持可达板，并对不可达捕获目标的切换施加滞回。
     * @param evaluations 按固定模型板位排列的评估结果，包含无效板占位。
     * @param now 当前单调推进的求解时间，s；时间回退时自动重置。
     * @return 所选板位下标及选择原因，无有效候选时下标为空。
     */
    PlateSelectionDecision choose(const std::vector<PlateEvaluation>& evaluations, double now);

    /** @brief 清除当前板位、候选切换和时间状态；跟踪重建时须调用。 */
    void reset();

    /**
     * @brief 按现有整车模型计算板面偏离正面中心的水平夹角。
     * @param phase 板位在 odom 中的相位，rad；板位置为车中心减去半径方向。
     * @param x 预测命中位置的 odom x，m。
     * @param y 预测命中位置的 odom y，m。
     * @return 有符号最短夹角，rad；正面为零，背面为正负 pi。
     */
    static double facing_angle(double phase, double x, double y);

private:
    PlateSelectionConfig config_;
    std::optional<size_t> current_;
    std::optional<size_t> challenger_;
    double challenger_since_ = 0.0;
    std::optional<double> last_time_;
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_PLATE_SELECTOR_HPP_
