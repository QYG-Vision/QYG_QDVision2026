#include "armor_solver/bullet_speed_state.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace qd::auto_aim {

TEST(BulletSpeedStateTest, UsesLatestPositiveMeasurementWithoutExtraFiltering) {
    BulletSpeedState speed;
    EXPECT_DOUBLE_EQ(speed.value(), 0.0);

    speed.update(9.7);
    EXPECT_DOUBLE_EQ(speed.value(), 9.7);
    speed.update(9.5);
    EXPECT_DOUBLE_EQ(speed.value(), 9.5);
}

TEST(BulletSpeedStateTest, InvalidMeasurementsDoNotPoisonLastValidSpeed) {
    BulletSpeedState speed;
    speed.update(9.7);

    speed.update(0.0);
    speed.update(std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(speed.value(), 9.7);
}

} // namespace qd::auto_aim
