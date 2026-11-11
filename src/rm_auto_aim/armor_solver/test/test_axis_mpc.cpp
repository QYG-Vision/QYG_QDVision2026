#include <cmath>
#include <limits>
#include <vector>

#include "armor_solver/axis_mpc.hpp"
#include "gtest/gtest.h"
#include "tiny_api.hpp"

namespace qd::auto_aim {
TEST(AxisMpc, holdsStationaryReferenceAtWindowCenter) {
    AxisMpc mpc(AxisMpc::Config{});
    const auto sample = mpc.plan(std::vector<double>(100, 10.0));
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, 10.0, 1e-6);
    EXPECT_NEAR(sample->velocity, 0.0, 1e-4);
    EXPECT_NEAR(sample->acceleration, 0.0, 1e-2);
}

TEST(AxisMpc, tracksConstantVelocityReference) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    std::vector<double> reference(config.horizon);
    for (int k = 0; k < config.horizon; ++k) {
        reference[k] = 5.0 + 30.0 * k * config.dt;
    }
    const auto sample = mpc.plan(reference);
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, reference[config.horizon / 2], 1e-5);
    EXPECT_NEAR(sample->velocity, 30.0, 1e-5);
    EXPECT_NEAR(sample->acceleration, 0.0, 1e-4);
}

TEST(AxisMpc, respectsAccelerationBound) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    std::vector<double> reference(config.horizon, 0.0);
    for (int k = config.horizon / 2; k < config.horizon; ++k) {
        reference[k] = 90.0;
    }
    const auto sample = mpc.plan(reference);
    ASSERT_TRUE(sample);
    EXPECT_LE(std::abs(sample->acceleration), config.max_acceleration + 1e-6);
}

TEST(AxisMpc, unwrapsYawAcrossSeam) {
    EXPECT_DOUBLE_EQ(unwrap_degrees(-179.0, 179.0), 181.0);
    EXPECT_DOUBLE_EQ(unwrap_degrees(179.0, -179.0), -181.0);
}

TEST(AxisMpc, rejectsBadInputAndConfiguration) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    EXPECT_FALSE(mpc.plan({}));
    auto reference = std::vector<double>(config.horizon, 0.0);
    reference[4] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(mpc.plan(reference));

    config.dt = 0.0;
    AxisMpc invalid(config);
    EXPECT_FALSE(invalid.plan(std::vector<double>(config.horizon, 0.0)));
}
} // namespace qd::auto_aim

namespace qd::auto_aim {
// 直接复现济瞄 setup/plan 调用，检验单位转换、保留工作状态和有限迭代输出。
TEST(AxisMpc, matchesTongjiDirectSolveAcrossRepeatedSwitches) {
    TinySolver* solver = nullptr;
    Eigen::MatrixXd a(2, 2), b(2, 1);
    a << 1, 0.01, 0, 1;
    b << 0, 0.01;
    Eigen::Vector2d q(9e6, 0);
    ASSERT_EQ(tiny_setup(&solver, a, b, Eigen::Vector2d::Zero(), q.asDiagonal(),
        Eigen::MatrixXd::Ones(1, 1), 1.0, 2, 1, 100, 0), 0);
    tiny_set_bound_constraints(solver, Eigen::MatrixXd::Constant(2, 100, -1e17),
        Eigen::MatrixXd::Constant(2, 100, 1e17),
        Eigen::MatrixXd::Constant(1, 99, -50), Eigen::MatrixXd::Constant(1, 99, 50));
    solver->settings->max_iter = 10;
    AxisMpc mpc(AxisMpc::Config{});
    for (int round = 0; round < 3; ++round) {
        std::vector<double> angles(100), velocities(100);
        const auto angle = [&](int k) {
            return (k < 55 - round * 5 ? 0.02 * k : -2.0 + 0.03 * k) * (round + 1);
        };
        for (int k = 0; k < 100; ++k) {
            angles[k] = angle(k);
            velocities[k] = (angle(k + 1) - angle(k - 1)) / 0.02;
            solver->work->Xref(0, k) = angles[k] * M_PI / 180;
            solver->work->Xref(1, k) = velocities[k] * M_PI / 180;
        }
        Eigen::Vector2d initial = solver->work->Xref.col(0);
        tiny_set_x0(solver, initial);
        tiny_solve(solver);
        const auto sample = mpc.plan(angles, &velocities);
        ASSERT_TRUE(sample);
        EXPECT_NEAR(sample->position, solver->work->x(0, 50) * 180 / M_PI, 1e-7);
        EXPECT_NEAR(sample->velocity, solver->work->x(1, 50) * 180 / M_PI, 1e-7);
        EXPECT_NEAR(sample->acceleration, solver->work->u(0, 50) * 180 / M_PI, 1e-6);
        EXPECT_NEAR(mpc.sample_at(52).position,
                    solver->work->x(0, 52) * 180 / M_PI, 1e-7);
    }
    delete solver->solution;
    delete solver->cache;
    delete solver->settings;
    delete solver->work;
    delete solver;
}
} // namespace qd::auto_aim
