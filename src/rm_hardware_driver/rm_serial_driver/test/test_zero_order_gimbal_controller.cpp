#include <gtest/gtest.h>

#include "rm_serial_driver/zero_order_gimbal_controller.hpp"

namespace qd::serial_driver {

TEST(ZeroOrderGimbalController, holdsFirstFeedbackUntilTargetChanges) {
    ZeroOrderGimbalController controller(0.0, "0");

    EXPECT_FALSE(controller.makeCommand().has_value());

    controller.updateFeedback(12.5, -3.0);
    const auto initial_command = controller.makeCommand();

    ASSERT_TRUE(initial_command.has_value());
    EXPECT_DOUBLE_EQ(initial_command->yaw, 12.5);
    EXPECT_DOUBLE_EQ(initial_command->pitch, -3.0);
}

TEST(ZeroOrderGimbalController, repeatedlyOutputsLatestAbsoluteTargetWithoutFire) {
    ZeroOrderGimbalController controller(0.0, "0");
    controller.updateFeedback(0.0, 0.0);
    controller.updateTarget(5.0, 2.0);

    const auto first_command = controller.makeCommand();
    const auto repeated_command = controller.makeCommand();

    ASSERT_TRUE(first_command.has_value());
    ASSERT_TRUE(repeated_command.has_value());
    EXPECT_DOUBLE_EQ(first_command->yaw, 5.0);
    EXPECT_DOUBLE_EQ(first_command->pitch, 2.0);
    EXPECT_DOUBLE_EQ(repeated_command->yaw, 5.0);
    EXPECT_DOUBLE_EQ(repeated_command->pitch, 2.0);
    EXPECT_DOUBLE_EQ(first_command->distance, 0.0);
    EXPECT_EQ(first_command->id, "0");
    EXPECT_FALSE(first_command->fire_advice);
}

TEST(ZeroOrderGimbalController, waitsForFeedbackBeforeSendingEarlyTarget) {
    ZeroOrderGimbalController controller(0.0, "0");
    controller.updateTarget(5.0, 2.0);

    EXPECT_FALSE(controller.makeCommand().has_value());

    controller.updateFeedback(12.5, -3.0);
    const auto command = controller.makeCommand();

    ASSERT_TRUE(command.has_value());
    EXPECT_DOUBLE_EQ(command->yaw, 5.0);
    EXPECT_DOUBLE_EQ(command->pitch, 2.0);
}

} // namespace qd::serial_driver
