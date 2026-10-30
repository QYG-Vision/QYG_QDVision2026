#ifndef ARMOR_SOLVER_AXIS_MPC_HPP_
#define ARMOR_SOLVER_AXIS_MPC_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace qd::auto_aim {
/**
 * @brief 将角度按最短角差展开到相邻参考附近。
 * @param angle 输入角度，degree。
 * @param anchor 相邻展开角度，degree。
 * @return 与 anchor 差值不超过 180 degree 的等价角度。
 */
inline double unwrap_degrees(double angle, double anchor) {
    return anchor + std::remainder(angle - anchor, 360.0);
}

/** @brief 一轴双积分模型的有限时域约束 MPC，内部单位 degree、s。 */
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
        int max_iterations = 300;
        double tolerance = 1e-5;
    };
    struct Sample {
        double position;
        double velocity;
        double acceleration;
    };

    /**
     * @brief 构造固定预测时域及约束矩阵，不引入外部 QP 运行库。
     * @param config 预测步长、时域、代价和轴的速度/加速度限制。
     * @note 非法配置会使 plan() 返回空值。
     */
    explicit AxisMpc(const Config& config) : config_(config) {
        const auto& c = config_;
        if (c.horizon < 2 || c.horizon > 100 || !std::isfinite(c.dt) || c.dt <= 0.0
            || !positive(c.max_velocity) || !positive(c.max_acceleration)
            || !positive(c.position_weight) || !positive(c.velocity_weight)
            || !positive(c.acceleration_weight) || c.max_iterations < 1
            || !positive(c.tolerance)) {
            return;
        }
        const int n = c.horizon;
        position_map_ = Eigen::MatrixXd::Zero(n, n);
        velocity_map_ = Eigen::MatrixXd::Zero(n, n);
        for (int k = 0; k < n; ++k) {
            for (int j = 0; j <= k; ++j) {
                position_map_(k, j) = (k - j + 0.5) * c.dt * c.dt * c.max_acceleration;
                velocity_map_(k, j) = c.dt * c.max_acceleration;
            }
        }
        // 决策量为 a/max_acceleration，约束也归一化以保持 活动集 QP 数值尺度。
        constraints_ = Eigen::MatrixXd::Zero(2 * n, n);
        constraints_.topRows(n).setIdentity();
        constraints_.bottomRows(n) = velocity_map_ / c.max_velocity;
        Eigen::MatrixXd hessian = c.position_weight * position_map_.transpose() * position_map_
            + c.velocity_weight * velocity_map_.transpose() * velocity_map_;
        hessian.diagonal().array() += c.acceleration_weight
            * c.max_acceleration * c.max_acceleration;
        scale_ = hessian.diagonal().maxCoeff();
        if (!positive(scale_) || !constraints_.allFinite() || !hessian.allFinite()) {
            return;
        }
        hessian /= scale_;
        hessian_ = hessian;
        factor_.compute(hessian);
        valid_ = factor_.info() == Eigen::Success;
        if (valid_) {
            inverse_ = factor_.solve(Eigen::MatrixXd::Identity(n, n));
            valid_ = inverse_.allFinite();
        }
    }

    /**
     * @brief 优化整个时域的加速度序列，返回首区间同一时刻的 p/v/a。
     * @param position 初始绝对角度，Yaw 已展开，degree。
     * @param velocity 初始实际姿态角速度，degree/s，不截断反馈。
     * @param reference 包含初始点及 horizon 个未来点的目标角度，degree。
     * @param sample_time 首个控制区间内的输出时刻，s，范围 (0, dt]。
     * @return 收敛且满足全部约束的参考；非法输入、不可行或不收敛时为空。
     */
    std::optional<Sample> plan(double position, double velocity,
                               const std::vector<double>& reference, double sample_time) const {
        const auto& c = config_;
        const int n = c.horizon;
        if (!valid_ || reference.size() != static_cast<size_t>(n + 1)
            || !std::isfinite(position) || !std::isfinite(velocity)
            || std::abs(velocity) > c.max_velocity || !std::isfinite(sample_time)
            || sample_time <= 0.0 || sample_time > c.dt
            || !std::all_of(reference.begin(), reference.end(), [](double x) {
                return std::isfinite(x);
            })) {
            return std::nullopt;
        }
        Eigen::VectorXd position_error(n), velocity_error(n);
        for (int k = 0; k < n; ++k) {
            position_error(k) = position + velocity * (k + 1) * c.dt - reference[k + 1];
            velocity_error(k) = velocity - (reference[k + 1] - reference[k]) / c.dt;
        }
        const Eigen::VectorXd gradient = (c.position_weight
            * position_map_.transpose() * position_error
            + c.velocity_weight * velocity_map_.transpose() * velocity_error) / scale_;
        Eigen::VectorXd lower = Eigen::VectorXd::Constant(2 * n, -1.0);
        Eigen::VectorXd upper = Eigen::VectorXd::Constant(2 * n, 1.0);
        lower.tail(n).array() -= velocity / c.max_velocity;
        upper.tail(n).array() -= velocity / c.max_velocity;
        // 从零加速度的可行轨迹开始，每次沿可行方向移动到最优点或下一个约束面。
        Eigen::MatrixXd inequalities(4 * n, n);
        inequalities.topRows(2 * n) = constraints_;
        inequalities.bottomRows(2 * n) = -constraints_;
        Eigen::VectorXd bounds(4 * n);
        bounds.head(2 * n) = upper;
        bounds.tail(2 * n) = -lower;
        Eigen::VectorXd controls = Eigen::VectorXd::Zero(n);
        std::vector<int> active;
        bool converged = false;
        for (int iteration = 0; iteration < c.max_iterations; ++iteration) {
            const Eigen::VectorXd grad = hessian_ * controls + gradient;
            Eigen::VectorXd direction = -inverse_ * grad;
            Eigen::VectorXd multipliers;
            if (!active.empty()) {
                Eigen::MatrixXd working(active.size(), n);
                for (size_t i = 0; i < active.size(); ++i) {
                    working.row(i) = inequalities.row(active[i]);
                }
                const Eigen::MatrixXd basis = inverse_ * working.transpose();
                const Eigen::MatrixXd schur = working * basis;
                const Eigen::LDLT<Eigen::MatrixXd> decomposition(schur);
                if (decomposition.info() != Eigen::Success) {
                    return std::nullopt;
                }
                multipliers = decomposition.solve(working * direction);
                direction -= basis * multipliers;
                if ((working * direction).lpNorm<Eigen::Infinity>() > c.tolerance) {
                    return std::nullopt;
                }
            }
            if (!direction.allFinite()) {
                return std::nullopt;
            }
            if (direction.lpNorm<Eigen::Infinity>() <= c.tolerance) {
                Eigen::Index index = 0;
                if (active.empty() || multipliers.minCoeff(&index) >= -c.tolerance) {
                    converged = true;
                    break;
                }
                active.erase(active.begin() + index);
                continue;
            }
            double step = 1.0;
            int blocker = -1;
            for (int i = 0; i < 4 * n; ++i) {
                if (std::find(active.begin(), active.end(), i) != active.end()) continue;
                const double slope = inequalities.row(i).dot(direction);
                if (slope <= 1e-12) continue;
                const double bound = (bounds(i) - inequalities.row(i).dot(controls)) / slope;
                if (bound < step) {
                    step = std::max(0.0, bound);
                    blocker = i;
                }
            }
            controls += step * direction;
            if (blocker >= 0) active.push_back(blocker);
        }
        const Eigen::VectorXd bounded = constraints_ * controls;
        // 最终再次验证全时域；不以截断首帧的方式伪装优化成功。
        if (!converged || (bounded.array() < lower.array() - c.tolerance).any()
            || (bounded.array() > upper.array() + c.tolerance).any()) {
            return std::nullopt;
        }
        // 数值容差内收紧首段，p/v 必须用同一个实际发送的加速度积分。
        const double min_acc = std::max(-c.max_acceleration,
                                        (-c.max_velocity - velocity) / c.dt);
        const double max_acc = std::min(c.max_acceleration,
                                        (c.max_velocity - velocity) / c.dt);
        const double acc = std::clamp(controls(0) * c.max_acceleration, min_acc, max_acc);
        const Sample sample{position + velocity * sample_time
                                + 0.5 * acc * sample_time * sample_time,
                            velocity + acc * sample_time, acc};
        if (!std::isfinite(sample.position) || !std::isfinite(sample.velocity)
            || !std::isfinite(sample.acceleration)) {
            return std::nullopt;
        }
        return sample;
    }

private:
    static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
    Config config_;
    bool valid_ = false;
    double scale_ = 1.0;
    Eigen::MatrixXd position_map_, velocity_map_, constraints_, hessian_, inverse_;
    Eigen::LDLT<Eigen::MatrixXd> factor_;
};
} // namespace qd::auto_aim
#endif
