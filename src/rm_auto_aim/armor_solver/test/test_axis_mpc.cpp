#include <cmath>
#include <limits>
#include <vector>

#include "armor_solver/axis_mpc.hpp"
#include "gtest/gtest.h"
#include "tiny_api.hpp"

namespace qd::auto_aim {
TEST(AxisMpc, holdsStationaryReferenceAtWindowCenter) {
    AxisMpc mpc(AxisMpc::Config{});
    const auto sample = mpc.plan(std::vector<double>(100, 0.17453292519943295));
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, 0.17453292519943295, 1e-6);
    EXPECT_NEAR(sample->velocity, 0.0, 1e-4);
    EXPECT_NEAR(sample->acceleration, 0.0, 1e-2);
}

TEST(AxisMpc, tracksConstantVelocityReference) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    std::vector<double> reference(config.horizon);
    for (int k = 0; k < config.horizon; ++k) {
        reference[k] = 0.08726646259971647 + 0.5235987755982988 * k * config.dt;
    }
    const auto sample = mpc.plan(reference);
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, reference[config.horizon / 2], 1e-5);
    EXPECT_NEAR(sample->velocity, 0.5235987755982988, 1e-5);
    EXPECT_NEAR(sample->acceleration, 0.0, 1e-4);
}

TEST(AxisMpc, respectsAccelerationBound) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    std::vector<double> reference(config.horizon, 0.0);
    for (int k = config.horizon / 2; k < config.horizon; ++k) {
        reference[k] = M_PI / 2.0;
    }
    const auto sample = mpc.plan(reference);
    ASSERT_TRUE(sample);
    EXPECT_LE(std::abs(sample->acceleration), config.max_acceleration + 1e-6);
}

TEST(AxisMpc, unwrapsYawAcrossSeam) {
    EXPECT_NEAR(unwrap_radians(-M_PI + 0.01, M_PI - 0.01), M_PI + 0.01, 1e-14);
    EXPECT_NEAR(unwrap_radians(M_PI - 0.01, -M_PI + 0.01), -M_PI - 0.01, 1e-14);
}

TEST(AxisMpc, acceptsSwitchingTrajectoryOnlyWithinConfiguredAccelerationTolerance) {
    AxisMpc::Config strict_config;
    strict_config.max_acceleration = 45.0;
    strict_config.max_iterations = 30;
    auto tolerant_config = strict_config;
    tolerant_config.acceleration_tolerance = 1.0;
    AxisMpc strict(strict_config);
    AxisMpc tolerant(tolerant_config);
    int newly_accepted = 0;
    // 平移切板时刻，覆盖切板前的加速和切板后的制动，并保留求解器工作状态。
    for (int switch_index = 80; switch_index >= 20; --switch_index) {
        std::vector<double> reference(100);
        const std::vector<double> velocities(100, -0.4);
        for (int k = 0; k < 100; ++k) {
            reference[k] = -0.4 * (k - 50) * 0.01 + (k >= switch_index ? 0.314 : 0.0);
        }
        const auto strict_sample = strict.plan(reference, &velocities);
        const auto tolerant_sample = tolerant.plan(reference, &velocities);
        const auto& diagnostic = strict.get_debug();
        EXPECT_TRUE(diagnostic.attempted);
        EXPECT_EQ(diagnostic.accepted, strict_sample.has_value());
        EXPECT_EQ(diagnostic.positions.size(), 100U);
        EXPECT_EQ(diagnostic.accelerations.size(), 99U);
        if (!strict_sample) {
            EXPECT_EQ(diagnostic.status, "acceleration_limit");
            EXPECT_GT(std::abs(diagnostic.output.acceleration),
                      strict_config.max_acceleration + strict_config.acceleration_tolerance);
        }
        if (strict_sample) {
            ASSERT_TRUE(tolerant_sample);
            EXPECT_DOUBLE_EQ(strict_sample->position, tolerant_sample->position);
            EXPECT_DOUBLE_EQ(strict_sample->velocity, tolerant_sample->velocity);
            EXPECT_DOUBLE_EQ(strict_sample->acceleration, tolerant_sample->acceleration);
        } else if (tolerant_sample) {
            ++newly_accepted;
            EXPECT_GT(std::abs(tolerant_sample->acceleration),
                      strict_config.max_acceleration + strict_config.acceleration_tolerance);
            EXPECT_LE(std::abs(tolerant_sample->acceleration),
                      tolerant_config.max_acceleration + tolerant_config.acceleration_tolerance);
        }
    }
    EXPECT_GT(newly_accepted, 0);
    EXPECT_FALSE(strict.plan({}));
    EXPECT_FALSE(strict.get_debug().attempted);
    EXPECT_TRUE(strict.get_debug().positions.empty());
    EXPECT_EQ(strict.get_debug().status, "invalid_input");
}

TEST(AxisMpc, validatesAccelerationTolerance) {
    for (const double tolerance : {-1.0, std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::quiet_NaN()}) {
        AxisMpc::Config config;
        config.acceleration_tolerance = tolerance;
        AxisMpc invalid(config);
        EXPECT_FALSE(invalid.plan(std::vector<double>(100, 0.0)));
    }
    AxisMpc::Config config;
    config.acceleration_tolerance = 0.0;
    AxisMpc exact(config);
    EXPECT_TRUE(exact.plan(std::vector<double>(100, 0.0)));
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
// 直接复现济瞄 setup/plan 调用，检验弧度接口、保留工作状态和有限迭代输出。
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
            return (k < 55 - round * 5 ? 0.00034906585039886593 * k
                : -0.03490658503988659 + 0.0005235987755982988 * k) * (round + 1);
        };
        for (int k = 0; k < 100; ++k) {
            angles[k] = angle(k);
            velocities[k] = (angle(k + 1) - angle(k - 1)) / 0.02;
            solver->work->Xref(0, k) = angles[k];
            solver->work->Xref(1, k) = velocities[k];
        }
        Eigen::Vector2d initial = solver->work->Xref.col(0);
        tiny_set_x0(solver, initial);
        tiny_solve(solver);
        const auto sample = mpc.plan(angles, &velocities);
        ASSERT_TRUE(sample);
        EXPECT_NEAR(sample->position, solver->work->x(0, 50), 1e-7);
        EXPECT_NEAR(sample->velocity, solver->work->x(1, 50), 1e-7);
        EXPECT_NEAR(sample->acceleration, solver->work->u(0, 50), 1e-6);
        EXPECT_NEAR(mpc.sample_at(52).position,
                    solver->work->x(0, 52), 1e-7);
    }
    delete solver->solution;
    delete solver->cache;
    delete solver->settings;
    delete solver->work;
    delete solver;
}
} // namespace qd::auto_aim
