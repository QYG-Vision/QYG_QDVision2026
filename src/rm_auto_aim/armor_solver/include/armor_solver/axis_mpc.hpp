#ifndef ARMOR_SOLVER_AXIS_MPC_HPP_
#define ARMOR_SOLVER_AXIS_MPC_HPP_

#include <memory>
#include <limits>
#include <optional>
#include <vector>
#include <cmath>

namespace qd::auto_aim {
/** @brief 将 degree 角按最短角差展开至 anchor 附近。 */
inline double unwrap_degrees(double angle, double anchor) {
    return anchor + std::remainder(angle - anchor, 360.0);
}

/** @brief 使用同济 TinyMPC 双积分模型规划单轴运动，公共接口单位为 degree、s。 */
class AxisMpc {
public:
    struct Config {
        int horizon = 100;
        double dt = 0.01;
        double max_acceleration = 2864.7889756541163;
        double position_weight = 9e6;
        double velocity_weight = 0.0;
        double acceleration_weight = 1.0;
        int max_iterations = 10;
    };
    struct Debug {
        const char* stage = "invalid_input";
        int status = -1;
        int iterations = 0;
        bool x_finite = false;
        bool u_finite = false;
        int max_iterations = 0;
        double max_acceleration_deg_s2 = 0.0;
        double max_abs_reference_second_diff_deg = std::numeric_limits<double>::quiet_NaN();
        int second_diff_peak_index = -1;
        double primal_residual_state = std::numeric_limits<double>::quiet_NaN();
        double primal_residual_input = std::numeric_limits<double>::quiet_NaN();
        double dual_residual_state = std::numeric_limits<double>::quiet_NaN();
        double dual_residual_input = std::numeric_limits<double>::quiet_NaN();
    };
    struct Sample {
        double position;
        double velocity;
        double acceleration;
    };

    /** @brief 创建固定模型及约束的 TinyMPC 实例，非法配置使 plan 返回空值。 */
    explicit AxisMpc(const Config& config);
    ~AxisMpc();
    AxisMpc(const AxisMpc&) = delete;
    AxisMpc& operator=(const AxisMpc&) = delete;

    /**
     * @brief 求解参考轨迹并返回窗口中点的 p/v/a，与同济 Planner 取点一致。
     * @param reference horizon 个角度参考，窗口中点对应当前规划时刻。
     * @return 收敛且满足约束的参考；非法输入或求解失败时为空。
     * @note 实例由求解线程独占，每次清空上一轮对偶状态以隔离目标切换。
     */
    std::optional<Sample> plan(const std::vector<double>& reference, Debug* debug = nullptr) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif

