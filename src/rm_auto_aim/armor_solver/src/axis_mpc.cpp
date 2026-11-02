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
        || !positive(c.position_weight) || c.max_iterations < 1 || !positive(c.tolerance)) {
        return;
    }
    if (!positive(c.response_time) || !positive(c.command_weight)
        || !positive(c.max_command_lead) || !positive(c.velocity_weight)) return;
    // 临界阻尼位置闭环：角加速度 = (command-angle)/tau² - 2*velocity/tau。
    // 零阶保持精确离散化，状态为相对角度、实际角速度；输入仍为角度。
    const double tau = c.response_time;
    const double h = c.dt / tau;
    const double decay = std::exp(-h);
    tinyMatrix dynamics(2, 2);
    dynamics << decay * (1.0 + h), decay * c.dt,
        -decay * c.dt / (tau * tau), decay * (1.0 - h);
    tinyMatrix input(2, 1);
    input << 1.0 - dynamics(0, 0), -dynamics(1, 0);
    tinyMatrix cost = tinyMatrix::Zero(2, 2);
    cost(0, 0) = c.position_weight;
    cost(1, 1) = c.velocity_weight;
    if (tiny_setup(&impl_->solver, dynamics, input, tinyVector::Zero(2), cost,
            tinyMatrix::Constant(1, 1, c.command_weight),
            1.0, 2, 1, c.horizon + 1, 0) != 0) return;
    auto* solver = impl_->solver;
    if (tiny_set_bound_constraints(solver,
            tinyMatrix::Constant(2, c.horizon + 1, -1e17),
            tinyMatrix::Constant(2, c.horizon + 1, 1e17),
            tinyMatrix::Constant(1, c.horizon, -c.max_command_lead),
            tinyMatrix::Constant(1, c.horizon, c.max_command_lead)) != 0) return;
    solver->settings->max_iter = c.max_iterations;
    solver->settings->abs_pri_tol = c.tolerance;
    solver->settings->abs_dua_tol = c.tolerance;
    impl_->valid = solver->cache->Pinf.allFinite()
        && solver->cache->Kinf.allFinite() && solver->cache->Quu_inv.allFinite();
}

AxisMpc::~AxisMpc() = default;

std::optional<AxisMpc::Sample> AxisMpc::plan(double position, const std::vector<double>& reference,
                                              double velocity) const {
    const auto& c = impl_->config;
    if (!impl_->valid || reference.size() != static_cast<size_t>(c.horizon + 1)
        || !std::isfinite(position) || !std::isfinite(velocity)
        || !std::all_of(reference.begin(), reference.end(), [](double x) {
            return std::isfinite(x);
        })) return std::nullopt;
    auto* solver = impl_->solver;
    auto* w = solver->work;
    // 以当前姿态为局部原点，跨 yaw 接缝由调用方连续展开。
    w->x.setZero();
    w->u.setZero();
    w->v.setZero();
    w->vnew.setZero();
    w->z.setZero();
    w->znew.setZero();
    w->g.setZero();
    w->y.setZero();
    w->p.setZero();
    w->d.setZero();
    for (int k = 0; k <= c.horizon; ++k) {
        w->Xref(0, k) = reference[k] - position;
        // 参考角速度来自同一条未来瞄准轨迹，末点使用后向差分。
        const int left = k == 0 ? 0 : k - 1;
        const int right = k == c.horizon ? k : k + 1;
        w->Xref(1, k) = (reference[right] - reference[left]) / ((right - left) * c.dt);
        if (k < c.horizon) w->Uref(0, k) = reference[k] - position;
    }
    if (!w->Xref.allFinite() || !w->Uref.allFinite()) return std::nullopt;
    update_linear_cost(solver);
    tinyVector initial(2);
    initial << 0.0, velocity;
    tiny_set_x0(solver, initial);
    if (tiny_solve(solver) != 0 || !solver->solution->solved
        || !w->x.allFinite() || !w->u.allFinite()
        || w->u.cwiseAbs().maxCoeff() > c.max_command_lead + c.tolerance)
        return std::nullopt;
    const double command = position + std::clamp(w->u(0, 0),
        -c.max_command_lead, c.max_command_lead);
    if (!std::isfinite(command)) return std::nullopt;
    // 电控仅使用角度；不把模型内部导数当成可执行前馈。
    return Sample{command, 0.0, 0.0};
}
} // namespace qd::auto_aim
