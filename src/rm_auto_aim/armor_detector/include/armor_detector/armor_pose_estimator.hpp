// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_

// std
#include <array>
#include <memory>
#include <vector>
// OpenCV
#include <opencv2/opencv.hpp>
// Eigen
#include <Eigen/Dense>
// ros2
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
// project
// #include "armor_detector/ba_solver.hpp"
#include "armor_detector/types.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_utils/math/pnp_solver.hpp"

namespace qd::auto_aim {
class ArmorPoseEstimator {
public:
    explicit ArmorPoseEstimator(sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
    explicit ArmorPoseEstimator();

    std::vector<rm_interfaces::msg::Armor>
    extractArmorPoses(const std::vector<Armor>& armors, Eigen::Matrix3d R_imu_camera);

    // void enableBA(bool enable) { use_ba_ = enable; }

    // 为 pnp_solver 设置相机参数，无参构造才需要
    void setCameraParame(cv::Mat& camera_matrix, cv::Mat& distortion_coefficients);

    // 设置最大偏航角阈值（单位：度）
    void setMaxYawAngle(double max_yaw_angle) {
        max_yaw_angle_ = max_yaw_angle;
    }

private:
    // Select the best PnP solution according to the armor's direction in image, only available for SOLVEPNP_IPPE
    void sortPnPResult(const Armor& armor, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
        const;

    // Convert a rotation matrix to RPY
    static Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R);

    // bool use_ba_;

    Eigen::Matrix3d R_gimbal_camera_;

    // 最大偏航角阈值（单位：度），用于过滤大朝向角装甲板
    double max_yaw_angle_;

    // std::unique_ptr<BaSolver> ba_solver_;
    std::unique_ptr<PnPSolver> pnp_solver_;
};
} // namespace qd::auto_aim
#endif // ARMOR_POSE_ESTIMATOR_HPP_