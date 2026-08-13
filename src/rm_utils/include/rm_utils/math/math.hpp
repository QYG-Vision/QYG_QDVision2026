#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <string>
namespace qd::auto_aim {

enum class ShootMode { IDLE = 0, TRACKING = 1, SHOOT_NOW = 2 };

struct YpdCoord {
    double yaw;
    double pitch;
    double distance;
};

/**
 * @brief 归一化角度到 [-π, π]
 * 
 * @param x 
 * @return double 
 */
inline double reduced_angle(const double& x) {
    return std::atan2(std::sin(x), std::cos(x));
}

// 计算在距离 dis 处，两个方向之间（夹角为 angle）的直线距离（弦长）
inline double get_termination_dis(const double& dis, const double& angle) {
    return std::fabs(2. * dis * std::sin(reduced_angle(angle) / 2.));
}

// 2 维向量 vec 逆时针旋转 angle
inline Eigen::Vector2d rotate(const Eigen::Vector2d& vec, const double& angle) {
    Eigen::Matrix2d mat;
    double sin_angle = std::sin(angle);
    double cos_angle = std::cos(angle);
    mat << cos_angle, -sin_angle, sin_angle, cos_angle;
    return mat * vec;
}

/**
* @brief 求平方
*/
template<typename T>
T sq(const T& x) {
    return x * x;
}

// 总是返回 0 ~ pi
inline double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2){
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
        return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

// 获取 tar 最近的 cur。获取结果可能不在 -pi 到 pi 之间
inline double get_closest_angle(const double& cur, const double& tar) {
    const double delta = reduced_angle(cur - tar);
    return tar + delta; // tar + cur - tar
}

}