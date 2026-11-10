#include <cmath>
#include <limits>
#include <vector>

#include "armor_solver/axis_mpc.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
TEST(AxisMpc, holdsStationaryReferenceAtWindowCenter) {
    AxisMpc mpc(AxisMpc::Config{});
    const auto sample = mpc.plan(std::vector<double>(100, 10.0));
    ASSERT_TRUE(sample);
    EXPECT_NEAR(sample->position, 10.0, 1e-8);
    EXPECT_NEAR(sample->velocity, 0.0, 1e-8);
    EXPECT_NEAR(sample->acceleration, 0.0, 1e-8);
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
