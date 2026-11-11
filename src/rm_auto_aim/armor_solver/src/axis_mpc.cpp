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
            tinyMatrix::Constant(1, c.horizon - 1, -c.max_acceleration * M_PI / 180.0),
            tinyMatrix::Constant(1, c.horizon - 1, c.max_acceleration * M_PI / 180.0)) != 0) return;
    solver->settings->max_iter = c.max_iterations;
    impl_->valid = solver->cache->Pinf.allFinite() && solver->cache->Kinf.allFinite()
        && solver->cache->Quu_inv.allFinite() && lower.allFinite() && upper.allFinite()
        && solver->work->u_min.allFinite() && solver->work->u_max.allFinite();
}

AxisMpc::~AxisMpc() = default;

std::optional<AxisMpc::Sample> AxisMpc::plan(const std::vector<double>& reference,
                                             const std::vector<double>* velocities) const {
    const auto& c = impl_->config;
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
        w->Xref(0, k) = reference[k] * M_PI / 180.0;
        const double velocity = velocities ? (*velocities)[k]
            : (reference[right] - reference[left]) / ((right - left) * c.dt);
        w->Xref(1, k) = velocity * M_PI / 180.0;
    }
    if (!w->Xref.allFinite() || !w->x.allFinite()) return std::nullopt;
    tinyVector initial(2);
    initial << w->Xref(0, 0), w->Xref(1, 0);
    tiny_set_x0(solver, initial);
    tiny_solve(solver);
    if (!w->x.allFinite() || !w->u.allFinite()) return std::nullopt;
    const int output = c.horizon / 2;
    const Sample result = sample_at(output);
    if (!std::isfinite(result.position) || !std::isfinite(result.velocity)
        || !std::isfinite(result.acceleration)) return std::nullopt;
    return result;
}
AxisMpc::Sample AxisMpc::sample_at(int index) const {
    const auto* w = impl_->solver->work;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    return {w->x(0, index) * RAD_TO_DEG, w->x(1, index) * RAD_TO_DEG,
            w->u(0, index) * RAD_TO_DEG};
}
} // namespace qd::auto_aim

