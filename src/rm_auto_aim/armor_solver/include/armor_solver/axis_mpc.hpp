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
     * @param velocities 可选的逐点角速度，degree/s；规划路径提供中心差分结果。
     * @return 有限迭代结果；非法输入或非有限输出时为空。
     * @note 实例由求解线程独占，沿用济瞄的求解器状态保留方式。
     */
    std::optional<Sample> plan(const std::vector<double>& reference,
                               const std::vector<double>* velocities = nullptr) const;
    /** @brief 读取最近规划的采样点，索引须位于 [0, horizon-2]，单位 degree、s。 */
    Sample sample_at(int index) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif

