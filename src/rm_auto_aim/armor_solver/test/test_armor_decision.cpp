#include "armor_solver/armor_decision.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace qd::auto_aim {
namespace {
std::vector<ArmorMotion> rotating(double phase, double speed, double time = 0.0) {
    std::vector<ArmorMotion> states;
    for (int i = 0; i < 4; ++i) {
        ArmorMotion state;
        state << 5.2, 0.0, 0.0, 0.0, 0.0, 0.0,
            std::remainder(phase + speed * time + i * M_PI / 2, 2 * M_PI), speed, 0.2;
        states.push_back(state);
    }
    return states;
}
const Eigen::Vector3d GIMBAL = Eigen::Vector3d::Zero();
constexpr double LIMIT = M_PI / 6;
}

TEST(ArmorDecision, predictsNextPlateBeforeCurrentDisappears) {
    const auto current = choose_armor_at_time(rotating(0.2, 3.0), GIMBAL, LIMIT, true);
    ASSERT_TRUE(current);
    EXPECT_EQ(current->index, 0);
    EXPECT_TRUE(current->direct);
    const auto waiting = choose_armor_at_time(rotating(0.2, 3.0, 0.15),
                                              GIMBAL, LIMIT, true, 0);
    ASSERT_TRUE(waiting);
    EXPECT_EQ(waiting->index, 3);
    EXPECT_FALSE(waiting->direct);
    EXPECT_NEAR(waiting->state[6], -LIMIT, 1e-10);
    const double entry_time = (M_PI / 2 - LIMIT - 0.2) / 3;
    EXPECT_NEAR(0.15 + waiting->wait_duration, entry_time, 1e-10);
    const auto arrived = choose_armor_at_time(rotating(0.2, 3.0, 0.30),
                                              GIMBAL, LIMIT, true, 3);
    ASSERT_TRUE(arrived);
    EXPECT_EQ(arrived->index, 3);
    EXPECT_TRUE(arrived->direct);
}

TEST(ArmorDecision, reverseRotationUsesOppositeNeighbor) {
    const auto choice = choose_armor_at_time(rotating(-0.2, -3.0, 0.15),
                                             GIMBAL, LIMIT, true, 0);
    ASSERT_TRUE(choice);
    EXPECT_EQ(choice->index, 1);
    EXPECT_FALSE(choice->direct);
    EXPECT_NEAR(choice->state[6], LIMIT, 1e-10);
}

TEST(ArmorDecision, arrayOrderDoesNotDefineNextPlate) {
    const auto original = rotating(0.2, 3.0, 0.15);
    const std::vector<ArmorMotion> reordered{original[2], original[0], original[3], original[1]};
    const auto choice = choose_armor_at_time(reordered, GIMBAL, LIMIT, true, 1);
    ASSERT_TRUE(choice);
    EXPECT_EQ(choice->index, 2);
    EXPECT_NEAR(choice->state[6], -LIMIT, 1e-10);
}

TEST(ArmorDecision, holdsPreferredOnlyInsideConfiguredWindow) {
    const auto held = choose_armor_at_time(rotating(0.7, 3), GIMBAL, M_PI / 3, true, 0);
    ASSERT_TRUE(held);
    EXPECT_EQ(held->index, 0);
    const auto released = choose_armor_at_time(rotating(0.6, 3), GIMBAL, LIMIT, true, 0);
    ASSERT_TRUE(released);
    EXPECT_EQ(released->index, 3);
    EXPECT_FALSE(released->direct);
}

TEST(ArmorDecision, committedWaitDoesNotJumpBackToOldPlateAtBoundary) {
    const auto choice = choose_armor_at_time(rotating(0.5, 3), GIMBAL, LIMIT, true, 3, true);
    ASSERT_TRUE(choice);
    EXPECT_EQ(choice->index, 3);
    EXPECT_FALSE(choice->direct);
    const auto reversed = choose_armor_at_time(rotating(0.5, -3), GIMBAL, LIMIT, true, 3, true);
    ASSERT_TRUE(reversed);
    EXPECT_EQ(reversed->index, 0);
    EXPECT_TRUE(reversed->direct);
}

TEST(ArmorDecision, noWaitModeAndZeroSpeedHaveNoFutureWindow) {
    EXPECT_FALSE(choose_armor_at_time(rotating(0.7, 3), GIMBAL, LIMIT, false));
    EXPECT_FALSE(choose_armor_at_time(rotating(0.7, 0), GIMBAL, LIMIT, true));
    const auto stationary = choose_armor_at_time(rotating(0, 0), GIMBAL, LIMIT, true);
    ASSERT_TRUE(stationary);
    EXPECT_TRUE(stationary->direct);
}

TEST(ArmorDecision, yawSeamDoesNotChangeEligibility) {
    const Eigen::Vector3d gimbal{0.0, 0.0, M_PI - 0.01};
    const auto choice = choose_armor_at_time(rotating(M_PI + 0.05, 3), gimbal, LIMIT, true);
    ASSERT_TRUE(choice);
    EXPECT_EQ(choice->index, 0);
    EXPECT_TRUE(choice->direct);
}

TEST(ArmorDecision, waitingPredictsTranslationWithoutMutatingInput) {
    auto states = rotating(0.7, 3);
    for (auto& state: states) state[1] = 1.0;
    const auto before = states;
    const auto choice = choose_armor_at_time(states, GIMBAL, LIMIT, true);
    ASSERT_TRUE(choice);
    EXPECT_FALSE(choice->direct);
    EXPECT_NEAR(choice->state[0], 5.2 + choice->wait_duration, 1e-10);
    for (size_t i = 0; i < states.size(); ++i) EXPECT_TRUE(states[i].isApprox(before[i]));
}

TEST(ArmorDecision, rejectsInvalidGeometryAndWindow) {
    EXPECT_FALSE(choose_armor_at_time({}, GIMBAL, LIMIT, true));
    EXPECT_FALSE(choose_armor_at_time(rotating(0, 3), GIMBAL, -1.0, true));
    auto states = rotating(0, 3);
    states[2][8] = 0;
    EXPECT_FALSE(choose_armor_at_time(states, GIMBAL, LIMIT, true));
    states = rotating(0, 3);
    states[1][6] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(choose_armor_at_time(states, GIMBAL, LIMIT, true));
}
} // namespace qd::auto_aim
