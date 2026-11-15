#include "armor_solver/spin_imm.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <gtest/gtest.h>

namespace qd::auto_aim {
TEST(SpinImm, predicts_constant_acceleration_and_full_elapsed_time) {
    SpinImm imm;
    imm.set_model_lock(2);
    imm.initialize(0.1, 2.0, 4.0, 0.01, 0.1, 1.0);
    imm.predict(0.45);
    EXPECT_NEAR(imm.yaw(), 0.1 + 2.0 * 0.45 + 2.0 * 0.45 * 0.45, 1e-10);
    EXPECT_NEAR(imm.v_yaw(), 3.8, 1e-10);
    EXPECT_NEAR(imm.alpha_yaw(), 4.0, 1e-10);
}

TEST(SpinImm, does_not_inflate_short_or_invalid_time_steps) {
    SpinImm imm;
    imm.set_model_lock(2);
    imm.initialize(0.0, 2.0, 0.0, 0.01, 0.1, 1.0);
    imm.predict(0.0001);
    EXPECT_NEAR(imm.yaw(), 0.0002, 1e-12);
    imm.predict(0.0);
    imm.predict(-0.1);
    imm.predict(std::numeric_limits<double>::quiet_NaN());
    EXPECT_NEAR(imm.yaw(), 0.0002, 1e-12);
}

TEST(SpinImm, decay_applied_once_matches_state_transition) {
    SpinImm imm;
    imm.set_model_lock(0);
    imm.initialize(0.0, 2.0, 4.0, 0.01, 0.1, 1.0);
    imm.predict(0.1);
    EXPECT_NEAR(imm.yaw(), 0.22, 1e-12);
    EXPECT_NEAR(imm.v_yaw(), 2.4, 1e-12);
    EXPECT_NEAR(imm.alpha_yaw(), 0.8, 1e-12);
}

TEST(SpinImm, rejects_outliers_without_correcting_state) {
    SpinImm imm;
    auto params = imm.params();
    params.q_v_variable = 0.0;
    params.q_alpha_variable = 0.0;
    imm.set_params(params);
    imm.set_model_lock(2);
    imm.initialize(0.0, 0.0, 0.0, 1e-5, 1e-5, 1e-5);
    imm.predict(0.01);
    ASSERT_TRUE(imm.update(2.0, 1e-5));
    EXPECT_TRUE(imm.last_update_gated());
    EXPECT_NEAR(imm.yaw(), 0.0, 1e-12);
    EXPECT_NEAR(imm.v_yaw(), 0.0, 1e-12);
    EXPECT_FALSE(imm.update(std::numeric_limits<double>::infinity(), 0.01));
}

TEST(SpinImm, circular_mixing_remains_finite_across_pi) {
    SpinImm imm;
    imm.initialize(3.10, 1.0, 0.0, 0.01, 0.1, 1.0);
    for (int i = 1; i <= 400; ++i) {
        imm.predict(0.01);
        const double observation = std::remainder(3.10 + i * 0.01, 2.0 * M_PI);
        ASSERT_TRUE(imm.update(observation, 0.002));
        EXPECT_LT(std::abs(std::remainder(imm.yaw() - observation, 2.0 * M_PI)), 0.05);
        EXPECT_GT(imm.yaw_cov(), 0.0);
        EXPECT_LT(imm.yaw_cov(), 0.2);
        const auto probabilities = imm.model_probabilities();
        EXPECT_NEAR(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1.0, 1e-10);
    }
}

TEST(SpinImm, variable_model_estimates_acceleration_from_angles) {
    SpinImm imm;
    imm.set_model_lock(2);
    imm.initialize(0.0, 0.0, 0.0, 0.01, 10.0, 100.0);
    for (int i = 1; i <= 500; ++i) {
        const double t = i * 0.01;
        imm.predict(0.01);
        ASSERT_TRUE(imm.update(std::remainder(0.5 * 2.0 * t * t, 2.0 * M_PI), 0.002));
    }
    EXPECT_NEAR(imm.v_yaw(), 10.0, 0.3);
    EXPECT_NEAR(imm.alpha_yaw(), 2.0, 0.5);
}

TEST(SpinImm, validates_params_and_resets_state) {
    SpinImm imm;
    auto params = imm.params();
    params.transition(0, 0) = 2.0;
    EXPECT_THROW(imm.set_params(params), std::invalid_argument);
    params = imm.params();
    params.q_v_variable = -1.0;
    EXPECT_THROW(imm.set_params(params), std::invalid_argument);
    imm.initialize(1.0, 2.0, 3.0, 1.0, 1.0, 1.0);
    imm.reset();
    EXPECT_FALSE(imm.initialized());
    EXPECT_DOUBLE_EQ(imm.alpha_yaw(), 0.0);
}
} // namespace qd::auto_aim

