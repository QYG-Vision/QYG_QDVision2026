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
    if (c.horizon < 3 || c.horizon > 100 || !positive(c.dt)
        || !positive(c.max_acceleration) || !positive(c.position_weight)
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
                                             Debug* debug) const {
    const auto& c = impl_->config;
    if (debug) {
        *debug = Debug{};
        debug->max_iterations = c.max_iterations;
        debug->max_acceleration_deg_s2 = c.max_acceleration;
        debug->stage = impl_->valid ? "invalid_input" : "invalid_setup";
    }
    if (!impl_->valid || reference.size() != static_cast<size_t>(c.horizon)
        || !std::all_of(reference.begin(), reference.end(), [](double x) {
            return std::isfinite(x);
        })) return std::nullopt;
    auto* solver = impl_->solver;
    auto* w = solver->work;
    for (int k = 0; k < c.horizon; ++k) {
        const int left = k == 0 ? 0 : k - 1;
        const int right = k == c.horizon - 1 ? k : k + 1;
        w->Xref(0, k) = reference[k];
        w->Xref(1, k) = (reference[right] - reference[left]) / ((right - left) * c.dt);
    }
    if (debug) {
        debug->max_abs_reference_second_diff_deg = 0.0;
        debug->second_diff_peak_index = 1;
        for (int k = 1; k < c.horizon - 1; ++k) {
            const double delta = std::abs((reference[k + 1] - reference[k])
                                          - (reference[k] - reference[k - 1]));
            if (delta > debug->max_abs_reference_second_diff_deg) {
                debug->max_abs_reference_second_diff_deg = delta;
                debug->second_diff_peak_index = k;
            }
        }
        debug->x_finite = w->x.allFinite();
        debug->stage = "invalid_pre_solve_state";
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
    initial << w->Xref(0, 0), w->Xref(1, 0);
    tiny_set_x0(solver, initial);
    const int status = tiny_solve(solver);
    const bool x_finite = w->x.allFinite();
    const bool u_finite = w->u.allFinite();
    if (debug) {
        debug->status = status;
        debug->iterations = solver->solution->iter;
        debug->x_finite = x_finite;
        debug->u_finite = u_finite;
        debug->primal_residual_state = w->primal_residual_state;
        debug->primal_residual_input = w->primal_residual_input;
        debug->dual_residual_state = w->dual_residual_state;
        debug->dual_residual_input = w->dual_residual_input;
        debug->stage = "solve_failed";
    }
    if (status != 0 || !x_finite || !u_finite) return std::nullopt;
    if (debug) debug->stage = "invalid_output";
    const int output = c.horizon / 2;
    const Sample result{
        w->x(0, output), w->x(1, output),
        std::clamp(w->u(0, output), -c.max_acceleration, c.max_acceleration)};
    if (!std::isfinite(result.position) || !std::isfinite(result.velocity)
        || !std::isfinite(result.acceleration)) return std::nullopt;
    if (debug) debug->stage = "valid";
    return result;
}
} // namespace qd::auto_aim

