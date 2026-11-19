#ifndef ARMOR_SOLVER_AXIS_MPC_HPP_
#define ARMOR_SOLVER_AXIS_MPC_HPP_

#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace qd::auto_aim {
/**
 * @brief 将弧度角按最短角差展开至 anchor 附近。
 * @param angle 待展开的角度，rad。
 * @param anchor 展开的参考角度，rad。
 * @return 与 angle 等价且位于 anchor 附近的弧度角。
 */
inline double unwrap_radians(double angle, double anchor) {
    return anchor + std::remainder(angle - anchor, 2.0 * M_PI);
}

/** @brief 使用同济 TinyMPC 双积分模型规划单轴运动，公共接口单位为 rad、s。 */
class AxisMpc {
public:
    struct Config {
        int horizon = 100;
        double dt = 0.01;
        double max_acceleration = 50.0; // rad/s²
        double position_weight = 9e6;
        double velocity_weight = 0.0;
        double acceleration_weight = 1.0;
        int max_iterations = 10;
    };
    struct Sample {
        double position; // rad
        double velocity; // rad/s
        double acceleration; // rad/s²
    };

    /**
     * @brief 创建固定模型及约束的 TinyMPC 实例，非法配置使 plan 返回空值。
     * @param config 时域、步长（s）、加速度上限（rad/s²）及优化权重。
     */
    explicit AxisMpc(const Config& config);
    ~AxisMpc();
    AxisMpc(const AxisMpc&) = delete;
    AxisMpc& operator=(const AxisMpc&) = delete;

    /**
     * @brief 求解参考轨迹并返回窗口中点的 p/v/a，与同济 Planner 取点一致。
     * @param reference horizon 个角度参考（rad），窗口中点对应当前规划时刻。
     * @param velocities 可选的逐点角速度，rad/s；规划路径提供中心差分结果。
     * @return 窗口中点的角度（rad）、角速度（rad/s）和角加速度（rad/s²）；非法时为空。
     * @note 实例由求解线程独占，沿用济瞄的求解器状态保留方式。
     */
    std::optional<Sample> plan(const std::vector<double>& reference,
                               const std::vector<double>* velocities = nullptr) const;
    /**
     * @brief 读取最近规划的采样点。
     * @param index 采样点索引，调用方须保证位于 [0, horizon-2]。
     * @return 角度（rad）、角速度（rad/s）和角加速度（rad/s²）。
     * @note 须在 plan 成功后调用，与 plan 由同一求解线程访问。
     */
    Sample sample_at(int index) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace qd::auto_aim
#endif

