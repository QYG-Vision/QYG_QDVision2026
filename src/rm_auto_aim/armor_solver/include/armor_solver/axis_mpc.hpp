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

/** @brief 使用济瞄 TinyMPC 规划单轴运动，公共接口单位为 degree、s。 */
class AxisMpc {
public:
    struct Config {
        int horizon = 20;
        double dt = 0.01;
        double max_velocity = 720.0;
        double max_acceleration = 6000.0;
        double position_weight = 1.0;
        double velocity_weight = 0.002;
        double acceleration_weight = 0.00001;
        int max_iterations = 3000;
        double tolerance = 1e-5;
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
     * @brief 求解未来加速度序列并返回首区间同一时刻的 p/v/a。
     * @param position 实际起始角，degree，yaw 已展开。
     * @param velocity 实际起始角速度，degree/s。
     * @param reference 初始点及 horizon 个未来角度，degree。
     * @param sample_time 首区间采样时间，s，范围 (0, dt]。
     * @return 收敛且满足约束的参考；非法输入或求解失败时为空。
     * @note 实例由求解线程独占，每次清空上一轮对偶状态以隔离目标切换。
     */
    std::optional<Sample> plan(double position, double velocity,
                               const std::vector<double>& reference, double sample_time) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif
