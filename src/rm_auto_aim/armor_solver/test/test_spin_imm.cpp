#include "armor_solver/spin_imm.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numeric>

namespace qd::auto_aim {
namespace {

TEST(SpinIMMTest, wraps_yaw_and_normalizes_model_probabilities) {
    SpinIMM imm;
    const double pi = std::acos(-1.0);
    imm.initialize(pi - 0.02, 1.0, 0.0);
    imm.predict(0.02);

    ASSERT_TRUE(imm.update(-pi + 0.02));
    EXPECT_LT(std::abs(std::remainder(imm.yaw() - (-pi + 0.02), 2.0 * pi)), 0.03);

    const auto probabilities = imm.model_probabilities();
    EXPECT_NEAR(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1.0, 1e-12);
    for (const double probability : probabilities) {
        EXPECT_GE(probability, 0.0);
        EXPECT_TRUE(std::isfinite(probability));
    }
}

TEST(SpinIMMTest, locks_to_the_selected_model) {
    SpinIMM imm;
    imm.initialize(0.0, 2.0, 0.0);
    imm.set_model_lock(1);
    imm.predict(0.01);

    ASSERT_TRUE(imm.update(0.02));
    const auto probabilities = imm.model_probabilities();
    EXPECT_DOUBLE_EQ(probabilities[0], 0.0);
    EXPECT_DOUBLE_EQ(probabilities[1], 1.0);
    EXPECT_DOUBLE_EQ(probabilities[2], 0.0);
}

TEST(SpinIMMTest, gates_large_yaw_innovations) {
    SpinIMM imm;
    imm.initialize(0.0, 1.0, 0.0, Eigen::Vector3d(1e-3, 1.0, 1.0));
    imm.predict(0.01);
    const double predicted_yaw = imm.yaw();

    EXPECT_FALSE(imm.update(2.0));
    EXPECT_TRUE(imm.last_update_gated());
    EXPECT_NEAR(imm.yaw(), predicted_yaw, 1e-12);
}

TEST(SpinIMMTest, rejects_non_finite_observations) {
    SpinIMM imm;
    imm.initialize(0.5, 0.0, 0.0);

    EXPECT_FALSE(imm.update(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_NEAR(imm.yaw(), 0.5, 1e-12);
}

}  // namespace
}  // namespace qd::auto_aim
