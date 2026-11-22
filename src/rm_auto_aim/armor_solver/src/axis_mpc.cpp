#include "armor_solver/axis_mpc.hpp"

#include <algorithm>
#include <chrono>
#include "tiny_api.hpp"

namespace qd::auto_aim {
struct AxisMpc::Impl {
    Config config;
    TinySolver* solver = nullptr;
    bool valid = false;
    DebugData debug;
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
    if (c.horizon < 3 || c.horizon > 100 || !positive(c.dt)
        || !positive(c.max_acceleration) || !positive(c.position_weight)
        || !std::isfinite(c.acceleration_tolerance) || c.acceleration_tolerance < 0.0
        || !std::isfinite(c.velocity_weight) || c.velocity_weight < 0.0
        || !positive(c.acceleration_weight) || c.max_iterations < 1) {
        return;
    }
    // 与同济 Planner 相同的离散模型：角度由上一拍角速度推进，控制量为角加速度。
    tinyMatrix a(2, 2), b(2, 1);
    a << 1, c.dt, 0, 1;
    b << 0, c.dt;
    if (!a.allFinite() || !b.allFinite()) return;
    if (tiny_setup(&impl_->solver, a, b, tinyVector::Zero(2),
                   (tinyVector(2) << c.position_weight, c.velocity_weight).finished().asDiagonal(),
                   tinyMatrix::Constant(1, 1, c.acceleration_weight),
                   1.0, 2, 1, c.horizon, 0) != 0) return;
    auto* solver = impl_->solver;
    tinyMatrix lower = tinyMatrix::Constant(2, c.horizon, -1e17);
    tinyMatrix upper = -lower;
    if (tiny_set_bound_constraints(solver, lower, upper,
            tinyMatrix::Constant(1, c.horizon - 1, -c.max_acceleration),
            tinyMatrix::Constant(1, c.horizon - 1, c.max_acceleration)) != 0) return;
    solver->settings->max_iter = c.max_iterations;
    impl_->valid = solver->cache->Pinf.allFinite() && solver->cache->Kinf.allFinite()
        && solver->cache->Quu_inv.allFinite() && lower.allFinite() && upper.allFinite()
        && solver->work->u_min.allFinite() && solver->work->u_max.allFinite();
}

AxisMpc::~AxisMpc() = default;

std::optional<AxisMpc::Sample> AxisMpc::plan(const std::vector<double>& reference,
                                             const std::vector<double>* velocities) const {
    const auto& c = impl_->config;
    auto& debug = impl_->debug;
    debug = DebugData {};
    debug.max_acceleration = c.max_acceleration;
    debug.acceleration_tolerance = c.acceleration_tolerance;
    debug.status = impl_->valid ? "invalid_input" : "invalid_configuration";
    if (!impl_->valid || reference.size() != static_cast<size_t>(c.horizon)
        || !std::all_of(reference.begin(), reference.end(), [](double x) {
            return std::isfinite(x);
        })) return std::nullopt;
    if (velocities && velocities->size() != reference.size()) return std::nullopt;
    auto* solver = impl_->solver;
    auto* w = solver->work;
    for (int k = 0; k < c.horizon; ++k) {
        const int left = k == 0 ? 0 : k - 1;
        const int right = k == c.horizon - 1 ? k : k + 1;
        w->Xref(0, k) = reference[k];
        const double velocity = velocities ? (*velocities)[k]
            : (reference[right] - reference[left]) / ((right - left) * c.dt);
        w->Xref(1, k) = velocity;
    }
    if (!w->Xref.allFinite() || !w->x.allFinite()) return std::nullopt;
    tinyVector initial(2);
    initial << w->Xref(0, 0), w->Xref(1, 0);
    tiny_set_x0(solver, initial);
    const auto solve_start = std::chrono::steady_clock::now();
    tiny_solve(solver);
    debug.solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_start).count();
    debug.attempted = true;
    debug.converged = solver->solution->solved != 0;
    debug.iterations = solver->solution->iter;
    debug.primal_residual_state = w->primal_residual_state;
    debug.primal_residual_input = w->primal_residual_input;
    debug.dual_residual_state = w->dual_residual_state;
    debug.dual_residual_input = w->dual_residual_input;
    debug.output = sample_at(c.horizon / 2);
    debug.positions.reserve(c.horizon);
    debug.velocities.reserve(c.horizon);
    debug.accelerations.reserve(c.horizon - 1);
    for (int k = 0; k < c.horizon; ++k) {
        debug.positions.push_back(w->x(0, k));
        debug.velocities.push_back(w->x(1, k));
        if (k < c.horizon - 1) debug.accelerations.push_back(w->u(0, k));
    }
    debug.status = "nonfinite_solution";
    if (!w->x.allFinite() || !w->u.allFinite()) return std::nullopt;
    const int output = c.horizon / 2;
    const Sample result = sample_at(output);
    debug.status = "acceleration_limit";
    if (!std::isfinite(result.position) || !std::isfinite(result.velocity)
        || !std::isfinite(result.acceleration)
        || std::abs(result.acceleration) - c.max_acceleration > c.acceleration_tolerance) {
        // 有限迭代可能返回越界解，超出配置的验收容限时拒绝该参考。
        return std::nullopt;
    }
    debug.accepted = true;
    debug.status = "accepted";
    return result;
}
const AxisMpc::DebugData& AxisMpc::get_debug() const noexcept {
    return impl_->debug;
}
AxisMpc::Sample AxisMpc::sample_at(int index) const {
    const auto* w = impl_->solver->work;
    return {w->x(0, index), w->x(1, index), w->u(0, index)};
}
} // namespace qd::auto_aim

