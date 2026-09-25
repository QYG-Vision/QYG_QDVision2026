#include <gtest/gtest.h>

#include "rm_serial_driver/mode_sync_config.hpp"

namespace qd::serial_driver {

TEST(ModeSyncConfig, DisabledSyncSkipsEveryModeService) {
    EXPECT_FALSE(should_create_mode_client(false, true, "/armor_detector/set_mode"));
    EXPECT_FALSE(should_create_mode_client(false, true, "/rune_solver/set_mode"));
}

TEST(ModeSyncConfig, EnabledSyncHonorsRuneFlag) {
    EXPECT_TRUE(should_create_mode_client(true, true, "/armor_detector/set_mode"));
    EXPECT_TRUE(should_create_mode_client(true, true, "/rune_solver/set_mode"));
    EXPECT_TRUE(should_create_mode_client(true, false, "/armor_solver/set_mode"));
    EXPECT_FALSE(should_create_mode_client(true, false, "/rune_detector/set_mode"));
}

} // namespace qd::serial_driver
