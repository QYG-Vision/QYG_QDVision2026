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
        options.parameter_overrides({
            rclcpp::Parameter("solver.use_armor_top", true),
            rclcpp::Parameter("solver.use_ballistic_compensation", false),
            rclcpp::Parameter("solver.compensator_type", "ideal")});
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
                yaw = std::atan2(y, x) * 180 / M_PI;
            }
        }
        return yaw;
    };
    std::vector<double> reference, velocity;
    const double anchor = angle(50);
    for (int k = 0; k < 100; ++k) {
        reference.push_back(std::remainder(angle(k) - anchor, 360.0));
        velocity.push_back(std::remainder(angle(k + 1) - angle(k - 1), 360.0) / 0.02);
    }
    AxisMpc expected(AxisMpc::Config{});
    const auto plan = expected.plan(reference, &velocity);
    ASSERT_TRUE(plan);
    EXPECT_NEAR(command.yaw, plan->position + anchor, 1e-5);
    EXPECT_NEAR(command.yaw_vel_deg_s, plan->velocity, 1e-4);
    EXPECT_NEAR(command.yaw_acc_deg_s2, plan->acceleration, 1e-3);
}
} // namespace qd::auto_aim
