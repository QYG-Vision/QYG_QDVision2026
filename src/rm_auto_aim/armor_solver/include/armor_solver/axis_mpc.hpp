#ifndef ARMOR_SOLVER_AXIS_MPC_HPP_
#define ARMOR_SOLVER_AXIS_MPC_HPP_

#include <memory>
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
    std::optional<Sample> plan(const std::vector<double>& reference) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif

