#include "rm_utils/math/trajectory_compensator.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace qd {

TEST(CeresCompensatorTest, RejectsUnreachableTargetAtLowSpeed) {
  CeresCompensator compensator;
  compensator.velocity = 9.7;
  compensator.gravity = 9.8;
  compensator.resistance = 0.022928514188;

  double pitch = 0.0;
  EXPECT_FALSE(compensator.compensate(Eigen::Vector3d(20.0, 0.0, 0.0), pitch));
}

TEST(CeresCompensatorTest, SolvesReachableTargetBelowTenMetersPerSecond) {
  CeresCompensator compensator;
  compensator.velocity = 9.7;
  compensator.gravity = 9.8;
  compensator.resistance = 0.022928514188;

  double pitch = 0.0;
  EXPECT_TRUE(compensator.compensate(Eigen::Vector3d(5.0, 0.0, 0.0), pitch));
}

} // namespace qd
