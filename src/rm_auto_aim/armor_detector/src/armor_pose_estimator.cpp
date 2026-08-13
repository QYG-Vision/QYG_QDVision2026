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

#include "armor_detector/armor_pose_estimator.hpp"

#include "armor_detector/types.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace qd::auto_aim {
ArmorPoseEstimator::ArmorPoseEstimator(sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
    // Setup pnp solver
    pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
    pnp_solver_->setObjectPoints(
        "small",
        Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)
    );
    pnp_solver_->setObjectPoints(
        "large",
        Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT)
    );
    // BA solver
    // ba_solver_ = std::make_unique<BaSolver>(camera_info->k, camera_info->d);

    R_gimbal_camera_ = Eigen::Matrix3d::Identity();
    R_gimbal_camera_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    // 默认最大偏航角为60度
    max_yaw_angle_ = 60.0;
}

ArmorPoseEstimator::ArmorPoseEstimator() {
    pnp_solver_ = std::make_unique<PnPSolver>();
    pnp_solver_->setObjectPoints(
        "small",
        Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)
    );
    pnp_solver_->setObjectPoints(
        "large",
        Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT)
    );

    R_gimbal_camera_ = Eigen::Matrix3d::Identity();
    R_gimbal_camera_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    // 默认最大偏航角为60度
    max_yaw_angle_ = 60.0;
}

void ArmorPoseEstimator::setCameraParame(cv::Mat& camera_matrix, cv::Mat& distortion_coefficients) {
    this->pnp_solver_->setCameraParame(camera_matrix, distortion_coefficients);
}

/**
 * @brief 进行 pnp 求解和补充 armor_msg 数据
 * 
 * @param armors 
 * @param R_imu_camera 
 * @return std::vector<rm_interfaces::msg::Armor> 填充完数据的 rm_interfaces::msg::Armor 向量
 */
std::vector<rm_interfaces::msg::Armor> ArmorPoseEstimator::extractArmorPoses(
    const std::vector<Armor>& armors,
    Eigen::Matrix3d R_imu_camera
) {
    std::vector<rm_interfaces::msg::Armor> armors_msg;

    for (const auto& armor: armors) {
        std::vector<cv::Mat> rvecs, tvecs;

        // Use PnP to get the initial pose information
        if (pnp_solver_->solvePnPGeneric(
                armor.landmarks(),
                rvecs,
                tvecs,
                (armor.type == ArmorType::SMALL ? "small" : "large")
            ))
        {
            sortPnPResult(armor, rvecs, tvecs);
            cv::Mat rmat;
            cv::Rodrigues(rvecs[0], rmat);

            Eigen::Matrix3d R = utils::cvToEigen(rmat);
            Eigen::Vector3d t = utils::cvToEigen(tvecs[0]);

            // 计算装甲板在云台坐标系下的RPY角
            auto rpy = rotationMatrixToRPY(R_gimbal_camera_ * R);
            double yaw_deg = std::abs(rpy[2] * 180.0 / M_PI);

            // 过滤大朝向角装甲板
            if (yaw_deg > max_yaw_angle_) {
                FYT_DEBUG(
                    "armor_detector",
                    "Filtered armor with yaw angle: {:.2f} deg (threshold: {:.2f} deg)",
                    yaw_deg,
                    max_yaw_angle_
                );
                continue;
            }

            /*       double armor_roll =
          rotationMatrixToRPY(R_gimbal_camera_ * R)[0] * 180 / M_PI;

      if (use_ba_ && armor_roll < 15) {
        // Use BA alogorithm to optimize the pose from PnP
        // solveBa() will modify the rotation_matrix
        R = ba_solver_->solveBa(armor, t, R, R_imu_camera);
      } */
            Eigen::Quaterniond q(R);

            // Fill the armor message
            rm_interfaces::msg::Armor armor_msg;

            // Fill basic info
            armor_msg.type = armorTypeToString(armor.type);
            armor_msg.number = armor.number;

            // Fill pose
            armor_msg.pose.position.x = t(0);
            armor_msg.pose.position.y = t(1);
            armor_msg.pose.position.z = t(2);
            armor_msg.pose.orientation.x = q.x();
            armor_msg.pose.orientation.y = q.y();
            armor_msg.pose.orientation.z = q.z();
            armor_msg.pose.orientation.w = q.w();

            // Fill the distance to image center
            armor_msg.distance_to_image_center =
                pnp_solver_->calculateDistanceToCenter(armor.center);
            //计算装甲板像素面积
            armor_msg.area = [&]() {
                auto points = armor.landmarks();
                // 使用鞋带公式计算面积
                double area = 0.0;
                for (int i = 0; i < 4; i++) {
                    int j = (i + 1) % 4; // 循环到第一个点
                    area += points[i].x * points[j].y;
                    area -= points[j].x * points[i].y;
                }

                // 取绝对值并除以2
                area = abs(area) / 2.0;
                return area;
            }();
            //输入像素坐标点
            armor_msg.image_points = [&]() {
                auto image_points = armor_msg.image_points;
                auto ps = armor.landmarks();

                int map[4] = { 2, 0, 5, 3 };
                for (int i = 0; i < 4; i++) {
                    int index = map[i];
                    image_points[i].x = ps[index].x;
                    image_points[i].y = ps[index].y;
                }

                return image_points;
            }();

            armors_msg.push_back(std::move(armor_msg));
        } else {
            FYT_WARN("armor_detector", "PnP Failed!");
        }
    }

    // 从左到右排序
    std::sort(armors_msg.begin(), armors_msg.end(), [](const auto& a, const auto& b) {
        // 返回 true 表示 a 排在 b 前面 (即从小到大)
        return a.image_points[0].x < b.image_points[0].x;
    });

    return armors_msg;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    // Transform to camera frame
    Eigen::Quaterniond q(R);
    // Get armor yaw
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    Eigen::Vector3d rpy;
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

void ArmorPoseEstimator::sortPnPResult(
    const Armor& armor,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs
) const {
    constexpr float PROJECT_ERR_THRES = 3.0;

    // 获取这两个解
    cv::Mat& rvec1 = rvecs.at(0);
    cv::Mat& tvec1 = tvecs.at(0);
    cv::Mat& rvec2 = rvecs.at(1);
    cv::Mat& tvec2 = tvecs.at(1);

    // 将旋转向量转换为旋转矩阵
    cv::Mat R1_cv, R2_cv;
    cv::Rodrigues(rvec1, R1_cv);
    cv::Rodrigues(rvec2, R2_cv);

    // 转换为Eigen矩阵
    Eigen::Matrix3d R1 = utils::cvToEigen(R1_cv);
    Eigen::Matrix3d R2 = utils::cvToEigen(R2_cv);

    // 计算云台系下装甲板的RPY角
    auto rpy1 = rotationMatrixToRPY(R_gimbal_camera_ * R1);
    auto rpy2 = rotationMatrixToRPY(R_gimbal_camera_ * R2);

    std::string coord_frame_name = (armor.type == ArmorType::SMALL ? "small" : "large");
    double error1 =
        pnp_solver_->calculateReprojectionError(armor.landmarks(), rvec1, tvec1, coord_frame_name);
    double error2 =
        pnp_solver_->calculateReprojectionError(armor.landmarks(), rvec2, tvec2, coord_frame_name);

    // 两个解的重投影误差差距较大或者roll角度较大时，不做选择
    if ((error2 / error1 > PROJECT_ERR_THRES) || (rpy1[0] > 10 * 180 / M_PI)
        || (rpy2[0] > 10 * 180 / M_PI))
    {
        return;
    }

    // 计算灯条在图像中的倾斜角度
    double l_angle = std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180 / M_PI;
    double r_angle = std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180 / M_PI;
    double angle = (l_angle + r_angle) / 2;
    angle += 90.0;

    if (armor.number == "outpost")
        angle = -angle;

    // 根据倾斜角度选择解
    // 如果装甲板左倾（angle > 0），选择Yaw为负的解
    // 如果装甲板右倾（angle < 0），选择Yaw为正的解
    if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) || (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
        std::swap(rvec1, rvec2);
        std::swap(tvec1, tvec2);
        // FYT_DEBUG("armor_detector", "PnP Solution 2 Selected");
    }
}

} // namespace qd::auto_aim
