#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "armor_solver/axis_mpc.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
TEST(AxisMpc, holdsStationaryReference) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    const auto sample = mpc.plan(10, 0, std::vector<double>(21, 10), 0.004);
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, 10, 1e-9);
    EXPECT_NEAR(sample->velocity, 0, 1e-9);
    EXPECT_NEAR(sample->acceleration, 0, 1e-9);
}

TEST(AxisMpc, stepUsesOneConsistentBoundedTrajectory) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    for (const auto goal : {-90.0, -10.0, 10.0, 90.0}) {
        const auto sample = mpc.plan(0, 0, std::vector<double>(21, goal), 0.004);
        ASSERT_TRUE(sample) << goal;
        EXPECT_GT(sample->acceleration * goal, 0);
        EXPECT_LE(std::abs(sample->acceleration), config.max_acceleration);
        EXPECT_LE(std::abs(sample->velocity), config.max_velocity);
        EXPECT_NEAR(sample->velocity, sample->acceleration * 0.004, 1e-9);
        EXPECT_NEAR(sample->position, 0.5 * sample->acceleration * 0.004 * 0.004, 1e-9);
    }
}

TEST(AxisMpc, tracksConstantVelocityWithoutInventedAcceleration) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    std::vector<double> reference(21);
    for (int i = 0; i <= 20; ++i) reference[i] = 5 + 30 * i * config.dt;
    const auto sample = mpc.plan(5, 30, reference, 0.004);
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, 5.12, 1e-8);
    EXPECT_NEAR(sample->velocity, 30, 1e-8);
    EXPECT_NEAR(sample->acceleration, 0, 1e-8);
}

TEST(AxisMpc, deceleratesAndSettlesInClosedLoop) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    double position = 0;
    double velocity = 0;
    const std::vector<double> reference(21, 20);
    for (int i = 0; i < 500; ++i) {
        const auto sample = mpc.plan(position, velocity, reference, 0.004);
        ASSERT_TRUE(sample) << i;
        EXPECT_LE(std::abs(sample->velocity), config.max_velocity);
        EXPECT_LE(std::abs(sample->acceleration), config.max_acceleration);
        position = sample->position;
        velocity = sample->velocity;
    }
    EXPECT_NEAR(position, 20, 0.05);
    EXPECT_NEAR(velocity, 0, 0.1);
}

TEST(AxisMpc, respectsVelocityBoundaryAndDifferentPitchLimits) {
    AxisMpc::Config config;
    config.max_velocity = 100;
    config.max_acceleration = 300;
    AxisMpc mpc(config);
    const auto sample = mpc.plan(0, 100, std::vector<double>(21, 90), 0.004);
    ASSERT_TRUE(sample);
    EXPECT_LE(sample->velocity, 100);
    EXPECT_LE(std::abs(sample->acceleration), 300);
}

TEST(AxisMpc, crossesYawSeamByShortestPath) {
    EXPECT_DOUBLE_EQ(unwrap_degrees(-179, 179), 181);
    EXPECT_DOUBLE_EQ(unwrap_degrees(179, -179), -181);
    AxisMpc::Config config;
    AxisMpc mpc(config);
    const auto sample = mpc.plan(179, 0, std::vector<double>(21, 181), 0.004);
    ASSERT_TRUE(sample);
    EXPECT_GT(sample->acceleration, 0);
    EXPECT_LT(sample->position - 179, 1);
}

TEST(AxisMpc, solvesVariedFeasibleReferences) {
    std::mt19937 random(2026);
    std::uniform_real_distribution<double> unit(-1, 1);
    for (const auto max_velocity : {100.0, 720.0}) {
        AxisMpc::Config config;
        config.max_velocity = max_velocity;
        config.max_acceleration = max_velocity * 5;
        AxisMpc mpc(config);
        for (int i = 0; i < 100; ++i) {
            const double position = 180 * unit(random);
            const double velocity = max_velocity * unit(random);
            const double goal = position + 180 * unit(random);
            const auto sample = mpc.plan(position, velocity, std::vector<double>(21, goal), 0.004);
            ASSERT_TRUE(sample) << i << " " << max_velocity;
            EXPECT_LE(std::abs(sample->velocity), max_velocity);
            EXPECT_LE(std::abs(sample->acceleration), config.max_acceleration);
        }
    }
}

TEST(AxisMpc, rejectsBadInputsAndUnconvergedOptimization) {
    AxisMpc::Config config;
    AxisMpc mpc(config);
    auto reference = std::vector<double>(21, 0);
    EXPECT_FALSE(mpc.plan(0, 721, reference, 0.004));
    EXPECT_FALSE(mpc.plan(0, 0, reference, 0.02));
    reference[4] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(mpc.plan(0, 0, reference, 0.004));
    EXPECT_FALSE(mpc.plan(0, 0, {}, 0.004));
    config.max_iterations = 1;
    AxisMpc limited(config);
    EXPECT_FALSE(limited.plan(0, 0, std::vector<double>(21, 90), 0.004));
    config.dt = 0;
    AxisMpc invalid(config);
    EXPECT_FALSE(invalid.plan(0, 0, std::vector<double>(21, 0), 0.004));
}
} // namespace qd::auto_aim
