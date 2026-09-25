#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rm_serial_driver/gimbal_target_cli.hpp"

namespace qd::serial_driver {

TEST(GimbalTargetCli, parsesFiniteYawAndPitchDegrees) {
    const std::vector<std::string> arguments { "5.5", "-2.25" };

    const auto target = parseGimbalTargetArguments(arguments);

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(target->yaw, 5.5);
    EXPECT_DOUBLE_EQ(target->pitch, -2.25);
}

TEST(GimbalTargetCli, rejectsMissingNonNumericAndNonFiniteArguments) {
    EXPECT_FALSE(parseGimbalTargetArguments({ "5.0" }).has_value());
    EXPECT_FALSE(parseGimbalTargetArguments({ "yaw", "0.0" }).has_value());
    EXPECT_FALSE(parseGimbalTargetArguments({ "nan", "0.0" }).has_value());
}

} // namespace qd::serial_driver
