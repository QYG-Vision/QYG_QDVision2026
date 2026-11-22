#include <cmath>
#include <memory>
#include <vector>

#include "armor_solver/armor_yaw_solver.hpp"
#include "gtest/gtest.h"

namespace qd::auto_aim {
namespace {

TEST(DoubleArmorYaw, RecoversAdjacentBoardAnglesInsteadOfParallelBoardCompromise) {
    for (const double spacing : {M_PI / 2.0, 2.0 * M_PI / 3.0}) {
        SCOPED_TRACE(spacing);
        const double first_yaw = -1.2;
        const double gimbal_yaw = 0.23;
        const cv::Mat camera = (cv::Mat_<double>(3, 3)
            << 1200.0, 0.0, 800.0, 0.0, 1200.0, 620.0, 0.0, 0.0, 1.0);
        const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
        Eigen::Matrix3d world_to_camera;
        world_to_camera << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0;
        auto converter = std::make_shared<CoordConverter>();
        converter->set_camera_params(camera, distortion);
        converter->set_world2camera(world_to_camera, Eigen::Vector3d::Zero());
        converter->set_gimbal_rpy(Eigen::Vector3d(0.0, 0.0, gimbal_yaw));

        // 独立生成两块相邻板的理想像素观测，使用不同位置避免对称退化。
        const std::vector<cv::Point3f> corners{
            {0.0f, 0.0675f, 0.0275f}, {0.0f, 0.0675f, -0.0275f},
            {0.0f, -0.0675f, -0.0275f}, {0.0f, -0.0675f, 0.0275f}};
        std::vector<ArmorData> observations;
        for (int i = 0; i < 2; ++i) {
            rm_interfaces::msg::Armor armor;
            armor.number = "sentry";
            armor.type = "small";
            const Eigen::Vector3d position(2.0 + 0.1 * i, 0.2 - 0.4 * i, 0.05);
            const Eigen::Matrix3d rotation = world_to_camera
                * (Eigen::AngleAxisd(first_yaw + gimbal_yaw + i * spacing,
                                    Eigen::Vector3d::UnitZ())
                   * Eigen::AngleAxisd(ANGLE_UP_15, Eigen::Vector3d::UnitY())).toRotationMatrix();
            cv::Mat rotation_cv, translation_cv, rvec;
            cv::eigen2cv(rotation, rotation_cv);
            const Eigen::Vector3d translation = world_to_camera * position;
            cv::eigen2cv(translation, translation_cv);
            cv::Rodrigues(rotation_cv, rvec);
            std::vector<cv::Point2f> pixels;
            cv::projectPoints(corners, rvec, translation_cv, camera, distortion, pixels);
            armor.pose.position.x = position.x();
            armor.pose.position.y = position.y();
            armor.pose.position.z = position.z();
            for (size_t j = 0; j < pixels.size(); ++j) {
                armor.image_points[j].x = pixels[j].x;
                armor.image_points[j].y = pixels[j].y;
            }
            observations.emplace_back(armor);
        }

        DoubleArmor cost(observations, first_yaw, converter, spacing);
        EXPECT_NEAR(cost(first_yaw), 0.0, 1e-5);
        SolveYawPnP solver{std::weak_ptr<rclcpp::Node>{}};
        const auto result = solver.find(-M_PI / 2.0, M_PI / 2.0 - spacing, cost, 40);
        EXPECT_NEAR(result.first, first_yaw, 0.01);
        EXPECT_LT(cost(first_yaw), cost(first_yaw + M_PI / 4.0));
    }
}

}  // namespace
}  // namespace qd::auto_aim
