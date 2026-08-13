// Created by Chengfu Zou on 2024.1.19
// Copyright(C) FYT Vision Group. All rights resevred.
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

#ifndef RM_UTILS_UTILS_HPP_
#define RM_UTILS_UTILS_HPP_

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
// project
#include "rm_utils/math/math.hpp"
#include "rm_utils/logger/log.hpp"

// util functions
namespace qd::utils {
// Convert euler angles to rotation matrix
enum class EulerOrder { XYZ, XZY, YXZ, YZX, ZXY, ZYX };
template <typename Vec3Like>
Eigen::Matrix3d eulerToMatrix(const Vec3Like &euler, EulerOrder order = EulerOrder::XYZ) {
  auto r = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
  auto p = Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY());
  auto y = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
  switch (order) {
    case EulerOrder::XYZ:
      return (y * p * r).matrix();
    case EulerOrder::XZY:
      return (p * y * r).matrix();
    case EulerOrder::YXZ:
      return (y * r * p).matrix();
    case EulerOrder::YZX:
      return (r * y * p).matrix();
    case EulerOrder::ZXY:
      return (p * r * y).matrix();
    case EulerOrder::ZYX:
      return (r * p * y).matrix();
  }
}

inline Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d &R,
                                     EulerOrder order = EulerOrder::XYZ) noexcept {
  switch (order) {
    case EulerOrder::XYZ:
      return R.eulerAngles(0, 1, 2);
    case EulerOrder::XZY:
      return R.eulerAngles(0, 2, 1);
    case EulerOrder::YXZ:
      return R.eulerAngles(1, 0, 2);
    case EulerOrder::YZX:
      return R.eulerAngles(1, 2, 0);
    case EulerOrder::ZXY:
      return R.eulerAngles(2, 0, 1);
    case EulerOrder::ZYX:
      return R.eulerAngles(2, 1, 0);
  }
}

inline Eigen::Vector3d getRPY(const Eigen::Matrix3d &R) {
  double yaw = atan2(R(0, 1), R(0, 0));
  double c2 = Eigen::Vector2d(R(2, 2), R(1, 2)).norm();
  double pitch = atan2(-R(0, 2), c2);

  double s1 = sin(yaw);
  double c1 = cos(yaw);
  double roll = atan2(s1 * R(2, 0) - c1 * R(2, 1), c1 * R(1, 1) - s1 * R(1, 0));

  return -Eigen::Vector3d(roll, pitch, yaw);
}

template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
cv::Mat eigenToCv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &eigen_mat) {
  cv::Mat cv_mat;
  cv::eigen2cv(eigen_mat, cv_mat);
  return cv_mat;
}

inline Eigen::MatrixXd cvToEigen(const cv::Mat &cv_mat) noexcept {
  Eigen::MatrixXd eigen_mat = Eigen::MatrixXd::Zero(cv_mat.rows, cv_mat.cols);
  cv::cv2eigen(cv_mat, eigen_mat);
  return eigen_mat;
}

// 定义一个简单的计时器类
class AutoTimer {
public:
    AutoTimer(std::string node_name, const std::string& name) 
        : name_(name), node_name_(node_name), start_(std::chrono::high_resolution_clock::now()), is_stopped_(false) {}

    // 析构函数：如果还没停止，就自动停止
    ~AutoTimer() {
        if (!is_stopped_) {
            stop();
        }
    }

    // 主动停止方法
    void stop() {
        if (is_stopped_) return; // 防止重复停止

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start_;
        FYT_DEBUG(node_name_, "[{}] latency : {:.2f} ms", name_, elapsed.count());
        
        is_stopped_ = true; // 标记为已停止
    }

private:
    std::string name_;
    std::string node_name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    bool is_stopped_; // 状态标志位
};

/** 单块装甲板在 odom 下的位姿：位置 xyz + 朝向 yaw */
struct ArmorPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

/**
 * 从 EKF 状态量得到 armor 0（参考装甲板）的位置
 * 状态量约定：xc(0), v_xc(1), yc(2), v_yc(3), zc(4), v_zc(5),
 *            yaw(6), v_yaw(7), r(8), dr(9), dz1(10), dz2(11)
 */
inline Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x) noexcept {
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * std::cos(yaw);
    double ya = yc - r * std::sin(yaw);
    return Eigen::Vector3d(xa, ya, za);
}

/**
 * 从整车状态得到所有装甲板在 odom 下的位姿（xyz + yaw）
 * @param state EKF 状态量
 * @param armors_num 装甲板数量
 * @param radius_list 各装甲板半径
 * @param dz_list 各装甲板高度 z 
 * @return 各装甲板位姿
 */
inline std::vector<ArmorPose> getArmorPosesFromState(
    Eigen::VectorXd state,
    int armors_num,
    const std::vector<double>& radius_list,
    const std::vector<double>& dz_list
) noexcept {
    std::vector<ArmorPose> poses(armors_num);
    for (int i = 0; i < armors_num; ++i) {
        double armor_yaw = auto_aim::reduced_angle(state(6) + i * (2.0 * M_PI / armors_num));
        double r = radius_list[i];
        double dz = dz_list[i];
        ArmorPose pose;
        pose.position = Eigen::Vector3d(
            state(0) - r * std::cos(armor_yaw),
            state(2) - r * std::sin(armor_yaw),
             dz
        );
        pose.yaw = armor_yaw;
        poses[i] = pose;
    }
    return poses;
}


} // namespace qd::utils

#endif
