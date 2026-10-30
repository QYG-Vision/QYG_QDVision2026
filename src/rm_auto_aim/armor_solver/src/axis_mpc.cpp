#include "armor_solver/axis_mpc.hpp"

#include <algorithm>
#include "tiny_api.hpp"

namespace qd::auto_aim {
struct AxisMpc::Impl {
    Config config;
    TinySolver* solver = nullptr;
    bool valid = false;
    explicit Impl(const Config& value) : config(value) {}
    ~Impl() {
        if (solver) {
            delete solver->solution;
            delete solver->cache;
            delete solver->settings;
            delete solver->work;
            delete solver;
        }
    }
};

AxisMpc::AxisMpc(const Config& config) : impl_(std::make_unique<Impl>(config)) {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
    const auto& c = config;
    if (c.horizon < 2 || c.horizon > 100 || !positive(c.dt)
        || !positive(c.max_velocity) || !positive(c.max_acceleration)
        || !positive(c.position_weight) || !positive(c.velocity_weight)
        || !positive(c.acceleration_weight) || c.max_iterations < 1 || !positive(c.tolerance)) {
        return;
    }
    // 按权重平方根缩放变量，速度与控制额外放大10倍以改善边界约束的 ADMM 条件数。
    const double ps = std::sqrt(c.position_weight);
    const double vs = 10.0 * std::sqrt(c.velocity_weight);
    const double us = 10.0 * std::sqrt(c.acceleration_weight);
    tinyMatrix a(2, 2), b(2, 1);
    a << 1, c.dt * ps / vs, 0, 1;
    b << 0.5 * c.dt * c.dt * ps / us, c.dt * vs / us;
    if (!a.allFinite() || !b.allFinite()) return;
    if (tiny_setup(&impl_->solver, a, b, tinyVector::Zero(2),
                   (tinyVector(2) << 1.0, 0.01).finished().asDiagonal(), tinyMatrix::Constant(1, 1, 0.01),
                   1.0, 2, 1, c.horizon + 1, 0) != 0) return;
    auto* solver = impl_->solver;
    tinyMatrix lower = tinyMatrix::Constant(2, c.horizon + 1, -1e17);
    tinyMatrix upper = -lower;
    lower.row(1).setConstant(-c.max_velocity * vs);
    upper.row(1).setConstant(c.max_velocity * vs);
    if (tiny_set_bound_constraints(solver, lower, upper,
            tinyMatrix::Constant(1, c.horizon, -c.max_acceleration * us),
            tinyMatrix::Constant(1, c.horizon, c.max_acceleration * us)) != 0) return;
    solver->settings->max_iter = c.max_iterations;
    solver->settings->abs_pri_tol = c.tolerance;
    solver->settings->abs_dua_tol = c.tolerance;
    impl_->valid = solver->cache->Pinf.allFinite() && solver->cache->Kinf.allFinite()
        && solver->cache->Quu_inv.allFinite() && lower.allFinite() && upper.allFinite()
        && solver->work->u_min.allFinite() && solver->work->u_max.allFinite();
}

AxisMpc::~AxisMpc() = default;

std::optional<AxisMpc::Sample> AxisMpc::plan(double position, double velocity,
    const std::vector<double>& reference, double sample_time) const {
    const auto& c = impl_->config;
    if (!impl_->valid || reference.size() != static_cast<size_t>(c.horizon + 1)
        || !std::isfinite(position) || !std::isfinite(velocity)
        || std::abs(velocity) > c.max_velocity || !std::isfinite(sample_time)
        || sample_time <= 0 || sample_time > c.dt
        || !std::all_of(reference.begin(), reference.end(), [](double x) {
            return std::isfinite(x);
        })) return std::nullopt;
    auto* solver = impl_->solver;
    auto* w = solver->work;
    const double ps = std::sqrt(c.position_weight);
    const double vs = 10.0 * std::sqrt(c.velocity_weight);
    const double us = 10.0 * std::sqrt(c.acceleration_weight);
    // 在参考末端切线的匀速坐标中规划，使终端参考为零，避免匀速轨迹引入虚假制动。
    const double base_velocity = std::clamp(
        (reference[c.horizon] - reference[c.horizon - 1]) / c.dt,
        -c.max_velocity, c.max_velocity);
    const double base_position = reference.back() - base_velocity * c.horizon * c.dt;
    if (!std::isfinite(base_velocity) || !std::isfinite(base_position)) return std::nullopt;
    w->x_min.row(1).setConstant((-c.max_velocity - base_velocity) * vs);
    w->x_max.row(1).setConstant((c.max_velocity - base_velocity) * vs);
    // 每次清空迭代状态，避免切板继承旧目标。
    for (int k = 0; k <= c.horizon; ++k) {
        w->Xref(0, k) = (reference[k] - base_position - base_velocity * k * c.dt) * ps;
        const int next = std::min(k + 1, c.horizon);
        const int previous = next - 1;
        w->Xref(1, k) = ((reference[next] - reference[previous]) / c.dt - base_velocity) * vs;
        w->x(0, k) = (position - base_position + (velocity - base_velocity) * k * c.dt) * ps;
        w->x(1, k) = (velocity - base_velocity) * vs;
    }
    if (!w->Xref.allFinite() || !w->x.allFinite()) return std::nullopt;
    w->u.setZero();
    w->Uref.setZero();
    w->v = w->x;
    w->vnew = w->x;
    w->z.setZero();
    w->znew.setZero();
    w->g.setZero();
    w->y.setZero();
    w->p.setZero();
    w->d.setZero();
    update_linear_cost(solver);
    tinyVector initial(2);
    initial << (position - base_position) * ps,
        (velocity - base_velocity) * vs;
    tiny_set_x0(solver, initial);
    const int status = tiny_solve(solver);
    if (status != 0 || !solver->solution->solved
        || !w->x.allFinite() || !w->u.allFinite()) return std::nullopt;
    // 复核未投影的动力学轨迹，不能仅凭投影后的变量声称满足约束。
    if ((w->x.row(1).array() + base_velocity * vs).abs().maxCoeff()
            > c.max_velocity * vs + c.tolerance
        || w->u.cwiseAbs().maxCoeff() > c.max_acceleration * us + c.tolerance) return std::nullopt;
    const double min_acc = std::max(-c.max_acceleration, (-c.max_velocity - velocity) / c.dt);
    const double max_acc = std::min(c.max_acceleration, (c.max_velocity - velocity) / c.dt);
    const double acc = std::clamp(w->u(0, 0) / us, min_acc, max_acc);
    const Sample result{position + velocity * sample_time + 0.5 * acc * sample_time * sample_time,
                        velocity + acc * sample_time, acc};
    if (!std::isfinite(result.position) || !std::isfinite(result.velocity)
        || !std::isfinite(result.acceleration)) return std::nullopt;
    return result;
}
} // namespace qd::auto_aim
