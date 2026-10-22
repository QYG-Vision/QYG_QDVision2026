#include "armor_solver/armor_tracker.hpp"

#include <gtest/gtest.h>

namespace qd::auto_aim {
namespace {

    class TrackerPredictionTest: public ::testing::Test {
    protected:
        /** @brief 构造带有已知速度的跟踪状态，测试不打开相机或串口。 */
        void SetUp() override {
            FYT_REGISTER_LOGGER("armor_solver", "/tmp/qd_tracker_prediction_test", INFO);
            tracker_ = std::make_unique<Tracker>(0.5, 0.7);
            tracker_->tracking_thres = 2;
            tracker_->lost_thres = 5;
            tracker_->ekf = std::make_unique<RobotStateEKF>(
                Predict { dt_ },
                Measure {},
                []() { return RobotStateEKF::MatrixXX::Identity() * 1e-4; },
                [](const RobotStateEKF::MatrixZ1&) { return RobotStateEKF::MatrixZZ::Identity(); },
                RobotStateEKF::MatrixXX::Identity()
            );
            tracker_->ekf_point = std::make_unique<ArmorStateEKF>(
                EkfPredict { dt_ },
                EkfMeasure {},
                []() { return ArmorStateEKF::MatrixXX::Identity() * 1e-4; },
                [](const ArmorStateEKF::MatrixZ1&) { return ArmorStateEKF::MatrixZZ::Identity(); },
                ArmorStateEKF::MatrixXX::Identity()
            );

            initial_frame_ = std::make_shared<Tracker::Armors>();
            Tracker::Armor armor;
            armor.number = "3";
            armor.type = "small";
            armor.pose.position.x = 2.0;
            armor.pose.position.z = 0.2;
            armor.pose.orientation.w = 1.0;
            armor.area = 100.0;
            initial_frame_->armors.push_back(armor);
            tracker_->init(initial_frame_);
            tracker_->tracker_state = TrackerState::TRACKING;
            tracker_->target_state(1) = 1.0;
            tracker_->target_state(3) = -0.5;
            tracker_->target_state(7) = 0.2;
            tracker_->armor_state(1) = 1.0;
            tracker_->armor_state(3) = -0.5;
            tracker_->ekf->setState(tracker_->target_state);
            tracker_->ekf_point->setState(tracker_->armor_state);
        }

        const double dt_ { 0.05 };
        std::unique_ptr<Tracker> tracker_;
        Tracker::Armors::SharedPtr initial_frame_;
    };

    TEST_F(TrackerPredictionTest, predicts_both_models_when_no_armor_is_detected) {
        const auto initial_target = tracker_->target_state;
        const auto initial_armor = tracker_->armor_state;
        const auto initial_measurement = tracker_->measurement;

        tracker_->update(std::make_shared<Tracker::Armors>());

        EXPECT_NEAR(tracker_->target_state(0), initial_target(0) + dt_, 1e-10);
        EXPECT_NEAR(tracker_->target_state(2), initial_target(2) - 0.5 * dt_, 1e-10);
        EXPECT_NEAR(tracker_->target_state(6), initial_target(6) + 0.2 * dt_, 1e-10);
        EXPECT_NEAR(tracker_->armor_state(0), initial_armor(0) + dt_, 1e-10);
        EXPECT_NEAR(tracker_->armor_state(2), initial_armor(2) - 0.5 * dt_, 1e-10);
        EXPECT_TRUE(tracker_->measurement.isApprox(initial_measurement));
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
    }

    TEST_F(TrackerPredictionTest, advances_state_across_consecutive_missing_frames) {
        const auto initial_target = tracker_->target_state;
        const auto initial_armor = tracker_->armor_state;
        auto missing_frame = std::make_shared<Tracker::Armors>();

        for (int i = 0; i < 3; ++i) {
            tracker_->update(missing_frame);
        }

        EXPECT_NEAR(tracker_->target_state(0), initial_target(0) + 3.0 * dt_, 1e-10);
        EXPECT_NEAR(tracker_->armor_state(0), initial_armor(0) + 3.0 * dt_, 1e-10);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
    }

    TEST_F(TrackerPredictionTest, predicts_armor_state_when_only_other_ids_are_visible) {
        const auto initial_armor = tracker_->armor_state;
        initial_frame_->armors.front().number = "4";

        tracker_->update(initial_frame_);

        EXPECT_NEAR(tracker_->armor_state(0), initial_armor(0) + dt_, 1e-10);
        EXPECT_NEAR(tracker_->armor_state(2), initial_armor(2) - 0.5 * dt_, 1e-10);
        EXPECT_EQ(tracker_->tracked_id, "3");
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
    }

    TEST_F(TrackerPredictionTest, preserves_existing_loss_timeout) {
        auto missing_frame = std::make_shared<Tracker::Armors>();
        for (int i = 0; i < tracker_->lost_thres; ++i) {
            tracker_->update(missing_frame);
            EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
        }
        tracker_->update(missing_frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::LOST);
    }

    TEST_F(TrackerPredictionTest, still_updates_a_stationary_matching_armor) {
        tracker_->target_state(1) = 0.0;
        tracker_->target_state(3) = 0.0;
        tracker_->target_state(7) = 0.0;
        tracker_->armor_state(1) = 0.0;
        tracker_->armor_state(3) = 0.0;
        tracker_->ekf->setState(tracker_->target_state);
        tracker_->ekf_point->setState(tracker_->armor_state);
        const auto initial_target = tracker_->target_state;
        const auto initial_armor = tracker_->armor_state;

        tracker_->update(initial_frame_);

        EXPECT_TRUE(tracker_->target_state.isApprox(initial_target, 1e-10));
        EXPECT_TRUE(tracker_->armor_state.isApprox(initial_armor, 1e-10));
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TRACKING);
    }

} // namespace
} // namespace qd::auto_aim
