#include <memory>

#include "armor_solver/armor_solver.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
class SolverMotionSwitches : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
        FYT_REGISTER_LOGGER("armor_solver", "/tmp/qyg-mpc-test-log", INFO);
    }
    void SetUp() override {
        rclcpp::NodeOptions options;
        std::vector<rclcpp::Parameter> parameters {
            rclcpp::Parameter("solver.use_armor_top", true),
            rclcpp::Parameter("solver.use_ballistic_compensation", false),
            rclcpp::Parameter("solver.compensator_type", "ideal")};
        const auto additional = selection_parameters();
        parameters.insert(parameters.end(), additional.begin(), additional.end());
        options.parameter_overrides(parameters);
        node_ = std::make_shared<rclcpp::Node>("solver_mpc_test", options);
        node_->declare_parameter("gimble_frame", "gimbal_link");
        solver_ = std::make_unique<Solver>(node_);
        buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now_;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "gimbal_link";
        transform.transform.rotation.w = 1;
        ASSERT_TRUE(buffer_->setTransform(transform, "test", true));
        target_.header = transform.header;
        target_.tracking = true;
        target_.id = "3";
        target_.armors_num = 4;
        target_.position.x = 5.2;
        target_.position_armor.x = 5;
        target_.radius_list = {0.2, 0.2, 0.2, 0.2};
        target_.dz_list = {0, 0, 0, 0};
        feedback_.header = target_.header;
        feedback_.mode = 0;
        feedback_.bullet_speed = 16;
    }
    rm_interfaces::msg::GimbalCmd solve() {
        return solver_->solve(target_, now_, buffer_);
    }
    /** @brief 提供派生测试的选板参数。 @return 默认不覆盖最近板兼容策略。 */
    virtual std::vector<rclcpp::Parameter> selection_parameters() const {
        return {};
    }
    rclcpp::Time now_{100, 0, RCL_ROS_TIME};
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<Solver> solver_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    rm_interfaces::msg::Target target_;
    rm_interfaces::msg::SerialReceiveData feedback_;
};

TEST_F(SolverMotionSwitches, plansWithoutFeedback) {
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_NEAR(command.yaw, 0, 1e-8);
    EXPECT_NEAR(command.pitch, 0, 1e-8);
    EXPECT_TRUE(command.fire_advice);
}

TEST_F(SolverMotionSwitches, usesWholeTargetInsteadOfTranslationArmor) {
    target_.position_armor.y = 100;
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_NEAR(command.yaw, 0, 1e-8);
}

TEST_F(SolverMotionSwitches, feedbackAgeModeAndFrameDoNotGatePlanner) {
    feedback_.mode = 2;
    feedback_.header.frame_id = "other_frame";
    solver_->update_feedback(feedback_);
    now_ += rclcpp::Duration::from_seconds(0.4);
    const auto command = solve();
    EXPECT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_TRUE(command.fire_advice);
}

TEST_F(SolverMotionSwitches, missingGimbalTransformDoesNotGatePlanner) {
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    const auto command = solve();
    EXPECT_TRUE(command.reference_valid) << command.reference_status;
}

TEST_F(SolverMotionSwitches, preservesBallisticCompensation) {
    node_->set_parameter(rclcpp::Parameter("solver.use_ballistic_compensation", true));
    solver_->update_bullet_speed(16);
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_LT(command.exp_pitch, 0);
    EXPECT_LT(command.pitch, 0);
}

TEST_F(SolverMotionSwitches, windowUsesNearestPlateAtEachSample) {
    target_.yaw = 0.5;
    target_.v_yaw = 3.0;
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    // 无弹速样本时飞行时间为0，独立计算四板最近板参考（含额外前后端点）。
    const auto angle = [&](int k) {
        double nearest = 1e10;
        double yaw = 0;
        for (int i = 0; i < 4; ++i) {
            const double phase = 0.5 + 3.0 * (k - 50) * 0.01 + i * M_PI / 2;
            const double x = 5.2 - 0.2 * std::cos(phase);
            const double y = -0.2 * std::sin(phase);
            if (std::hypot(x, y) < nearest) {
                nearest = std::hypot(x, y);
                yaw = std::atan2(y, x);
            }
        }
        return yaw;
    };
    std::vector<double> reference, velocity;
    const double anchor = angle(50);
    for (int k = 0; k < 100; ++k) {
        reference.push_back(std::remainder(angle(k) - anchor, 2.0 * M_PI));
        velocity.push_back(std::remainder(angle(k + 1) - angle(k - 1), 2.0 * M_PI) / 0.02);
    }
    AxisMpc expected(AxisMpc::Config{});
    const auto plan = expected.plan(reference, &velocity);
    ASSERT_TRUE(plan);
    EXPECT_NEAR(command.yaw, plan->position + anchor, 1e-5);
    EXPECT_NEAR(command.yaw_vel_rad_s, plan->velocity, 1e-4);
    EXPECT_NEAR(command.yaw_acc_rad_s2, plan->acceleration, 1e-3);
}

class SolverReachableSelection: public SolverMotionSwitches {
protected:
    /** @brief 启用可达选板。 @return 新策略的参数覆盖。 */
    std::vector<rclcpp::Parameter> selection_parameters() const override {
        return { rclcpp::Parameter("solver.selection.policy", "acceleration_reachable") };
    }
};

TEST_F(SolverReachableSelection, requires_fresh_feedback) {
    const auto missing = solve();
    EXPECT_FALSE(missing.reference_valid);
    EXPECT_FALSE(missing.fire_advice);
    EXPECT_EQ(missing.reference_status, "selection_missing_feedback");
    solver_->update_feedback(feedback_);
    EXPECT_TRUE(solve().reference_valid);
    now_ += rclcpp::Duration::from_seconds(0.11);
    const auto stale = solve();
    EXPECT_FALSE(stale.reference_valid);
    EXPECT_FALSE(stale.fire_advice);
    EXPECT_EQ(stale.reference_status, "selection_stale_feedback");
}

TEST_F(SolverReachableSelection, rejects_mismatched_frame_and_future_feedback) {
    feedback_.header.frame_id = "other_frame";
    solver_->update_feedback(feedback_);
    EXPECT_EQ(solve().reference_status, "selection_feedback_frame_mismatch");
    feedback_.header.frame_id = "odom";
    feedback_.header.stamp = now_ + rclcpp::Duration::from_seconds(0.01);
    solver_->update_feedback(feedback_);
    EXPECT_EQ(solve().reference_status, "selection_stale_feedback");
}

TEST_F(SolverReachableSelection, captures_without_firing_and_fires_when_aligned) {
    feedback_.yaw = 1.0;
    solver_->update_feedback(feedback_);
    const auto capture = solve();
    ASSERT_TRUE(capture.reference_valid) << capture.reference_status;
    EXPECT_FALSE(capture.fire_advice);
    EXPECT_NE(capture.reference_status.find("valid_mpc_capturing"), std::string::npos);
    // 两种初始状态分别检查；不在同一时间戳伪造瞬间完成的大角度转动。
    solver_->reset_selection();
    feedback_.yaw = 0.0;
    solver_->update_feedback(feedback_);
    const auto aligned = solve();
    ASSERT_TRUE(aligned.reference_valid) << aligned.reference_status;
    EXPECT_TRUE(aligned.fire_advice);
}

TEST_F(SolverReachableSelection, previews_next_plate_before_current_plate_exits) {
    target_.yaw = 0.5;
    target_.v_yaw = 3.0;
    const auto angle = [&](int k) {
        const double phase = 0.5 + 3.0 * (k - 50) * 0.01;
        return std::atan2(-0.2 * std::sin(phase), 5.2 - 0.2 * std::cos(phase));
    };
    feedback_.yaw = angle(50);
    feedback_.yaw_actual_vel_rad_s = (angle(51) - angle(49)) / 0.02;
    solver_->update_feedback(feedback_);
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    ASSERT_NE(command.reference_status.find(":active=0"), std::string::npos);
    ASSERT_NE(command.reference_status.find(":next=3"), std::string::npos);
    ASSERT_NE(command.reference_status.find(":arrive="), std::string::npos);
    // 当前板还朝向己方，未来换板已影响 MPC；不再等待当前板无法跟踪才重建窗口。
    std::vector<double> reference, velocity;
    for (int k = 0; k < 100; ++k) {
        reference.push_back(angle(k) - angle(50));
        velocity.push_back((angle(k + 1) - angle(k - 1)) / 0.02);
    }
    AxisMpc single_plate(AxisMpc::Config {});
    const auto baseline = single_plate.plan(reference, &velocity);
    ASSERT_TRUE(baseline);
    EXPECT_GT(command.yaw_acc_rad_s2, baseline->acceleration + 0.1);
    if (command.reference_status.find("valid_mpc_transferring") != std::string::npos) {
        EXPECT_FALSE(command.fire_advice);
        const double phase = 0.5 + 3.0 * M_PI / 2.0;
        const double next_yaw = std::atan2(-0.2 * std::sin(phase), 5.2 - 0.2 * std::cos(phase));
        EXPECT_NEAR(command.exp_yaw, next_yaw, 1e-10);
    }
}

TEST_F(SolverReachableSelection, preserves_stationary_tracking_and_resets) {
    target_.yaw = 0.6;
    const auto angle = [&](int plate) {
        const double phase = 0.6 + plate * M_PI / 2.0;
        return std::atan2(-0.2 * std::sin(phase), 5.2 - 0.2 * std::cos(phase));
    };
    feedback_.yaw = angle(0);
    solver_->update_feedback(feedback_);
    ASSERT_NE(solve().reference_status.find(":plate=0:"), std::string::npos);
    feedback_.yaw = angle(0) + 0.005;
    solver_->update_feedback(feedback_);
    EXPECT_NE(solve().reference_status.find(":active=0"), std::string::npos);
    solver_->reset_selection();
    feedback_.yaw = angle(3);
    solver_->update_feedback(feedback_);
    EXPECT_NE(solve().reference_status.find(":plate=3:"), std::string::npos);
    target_.tracking = false;
    EXPECT_FALSE(solve().reference_valid);
    target_.tracking = true;
    feedback_.yaw = angle(0);
    solver_->update_feedback(feedback_);
    EXPECT_NE(solve().reference_status.find(":plate=0:"), std::string::npos);
}

class SolverLimitedAcceleration: public SolverReachableSelection {
protected:
    /** @brief 使用低 yaw 加速度区分最近板和可达板。 @return 参数覆盖。 */
    std::vector<rclcpp::Parameter> selection_parameters() const override {
        return { rclcpp::Parameter("solver.selection.policy", "acceleration_reachable"),
                 rclcpp::Parameter("solver.mpc.yaw.max_acceleration", 0.01) };
    }
};

TEST_F(SolverLimitedAcceleration, selects_farther_reachable_plate) {
    target_.yaw = 0.6;
    const double phase = 0.6 + 3 * M_PI / 2.0;
    const double expected = std::atan2(-0.2 * std::sin(phase), 5.2 - 0.2 * std::cos(phase));
    feedback_.yaw = expected;
    solver_->update_feedback(feedback_);
    const auto command = solve();
    ASSERT_TRUE(command.reference_valid) << command.reference_status;
    EXPECT_NE(command.reference_status.find(":plate=3:"), std::string::npos);
    EXPECT_NEAR(command.exp_yaw, expected, 1e-10);
}
} // namespace qd::auto_aim
