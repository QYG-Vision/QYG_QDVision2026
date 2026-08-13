// Created by Chengfu Zou
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

#include "rm_utils/math/trajectory_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <ceres/ceres.h>

namespace qd {
namespace {

constexpr double kPitchLimit = M_PI / 2.5; // 最大仰角限制 72 deg

class LinearResistanceResidual {
public:
  LinearResistanceResidual(const double distance,
                           const double target_height,
                           const double velocity,
                           const double gravity,
                           const double resistance)
  : distance_(distance),
    target_height_(target_height),
    velocity_(velocity),
    gravity_(gravity),
    resistance_(resistance) {}

  template<typename T>
  bool operator()(const T *const angle, T *residual) const {
    const T cos_angle = ceres::cos(angle[0]);
    const T sin_angle = ceres::sin(angle[0]);
    residual[0] =
      (resistance_ * velocity_ * sin_angle + gravity_) * distance_
        / (resistance_ * velocity_ * cos_angle)
      + gravity_ * ceres::log(T(1.0) - (resistance_ * distance_) / (velocity_ * cos_angle))
          / (resistance_ * resistance_)
      - target_height_;
    return true;
  }

private:
  const double distance_;
  const double target_height_;
  const double velocity_;
  const double gravity_;
  const double resistance_;
};

}  // namespace

bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);
  // The iterative_height is used to calculate angle in each iteration
  double iterative_height = target_height;
  double impact_height = 0;
  double distance = target_position.head(2).norm();
  double angle = std::atan2(target_height, distance);
  double dh = 0;
  // Iterate to find the right angle, which makes the impact height equal to the
  // target height
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > kPitchLimit) {
      break;
    }
    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;
    if (std::abs(dh) < 0.01) {
      break;
    }
    iterative_height += dh;
  }
  if (std::abs(dh) > 0.01 || std::abs(angle) > kPitchLimit) {
    return false;
  }
  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.2) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  double t = x / (velocity * std::cos(angle));
  double y = velocity * std::sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance = target_position.head(2).norm();
  double angle = std::atan2(target_position(2), distance);
  double t = distance / (velocity * std::cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  double r = resistance;
  double t = (std::exp(r * x) - 1) / (r * velocity * std::cos(angle));
  double y = velocity * std::sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double r = resistance;
  double distance = target_position.head(2).norm();
  double angle = std::atan2(target_position(2), distance);
  double t = (std::exp(r * distance) - 1) / (r * velocity * std::cos(angle));
  return t;
}
// RK4 helpers
static inline Eigen::Vector3d rk4_acc(const Eigen::Vector3d &v, double g, double k) {
  const double v_norm = v.norm();
  Eigen::Vector3d drag_acc = -k * v_norm * v;
  return {drag_acc.x(), drag_acc.y(), drag_acc.z() - g};
}

static inline void rk4_integrate_step(
  Eigen::Vector3d &vel,
  Eigen::Vector3d &pos,
  double gravity,
  double resistance,
  double dt) {
  Eigen::Vector3d k1_v = rk4_acc(vel, gravity, resistance) * dt;
  Eigen::Vector3d k1_p = vel * dt;

  Eigen::Vector3d k2_v = rk4_acc(vel + 0.5 * k1_v, gravity, resistance) * dt;
  Eigen::Vector3d k2_p = (vel + 0.5 * k1_v) * dt;

  Eigen::Vector3d k3_v = rk4_acc(vel + 0.5 * k2_v, gravity, resistance) * dt;
  Eigen::Vector3d k3_p = (vel + 0.5 * k2_v) * dt;

  Eigen::Vector3d k4_v = rk4_acc(vel + k3_v, gravity, resistance) * dt;
  Eigen::Vector3d k4_p = (vel + k3_v) * dt;

  vel += (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) / 6.0;
  pos += (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p) / 6.0;
}

Eigen::Vector3d RK4Compensator::simulate(double pitch, double yaw, double speed,
                                         double t_total) const {
  Eigen::Vector3d velocity;
  velocity.x() = speed * std::cos(pitch) * std::cos(yaw);
  velocity.y() = speed * std::cos(pitch) * std::sin(yaw);
  velocity.z() = speed * std::sin(pitch);

  Eigen::Vector3d pos(0, 0, 0);
  double t = 0.0;
  const double dt_cfg = std::max(1e-4, rk4_time_step);

  while (t < t_total - 1e-6) {
    double dt = std::min(dt_cfg, t_total - t);
    rk4_integrate_step(velocity, pos, gravity, resistance, dt);
    t += dt;
  }
  return pos;
}

std::vector<Eigen::Vector3d> RK4Compensator::sampleTrajectory(
    double pitch, double yaw, double speed, double t_total, std::size_t steps) const {
  std::vector<Eigen::Vector3d> pts;
  if (t_total <= 0 || speed <= 0 || steps < 2) return pts;

  const double dt = std::max(1e-4, rk4_time_step);
  std::size_t total_steps = static_cast<std::size_t>(std::ceil(t_total / dt));
  std::size_t stride = std::max<std::size_t>(1, total_steps / (steps - 1));

  Eigen::Vector3d velocity;
  velocity.x() = speed * std::cos(pitch) * std::cos(yaw);
  velocity.y() = speed * std::cos(pitch) * std::sin(yaw);
  velocity.z() = speed * std::sin(pitch);
  Eigen::Vector3d pos(0, 0, 0);

  pts.reserve(steps);
  pts.push_back(pos);

  double t = 0.0;
  for (std::size_t i = 0; i < total_steps; ++i) {
    double remain = t_total - t;
    double h = std::min(dt, remain);
    if (h <= 1e-9) break;

    rk4_integrate_step(velocity, pos, gravity, resistance, h);
    t += h;

    if (i % stride == 0 && pts.size() + 1 < steps) pts.push_back(pos);
  }
  if (pts.size() < steps) pts.push_back(pos);
  return pts;
}

// 使用 simulate 近似求解 x 处高度：沿 x 正方向，yaw=0
double RK4Compensator::calculateTrajectory(const double x, const double angle) const noexcept {
  // 粗略估计时间：t = x / (v*cos(angle))
  double t_guess = std::max(1e-4, x / (velocity * std::max(1e-4, std::cos(angle))));
  Eigen::Vector3d pos = simulate(angle, 0.0, velocity, t_guess);
  if (pos.x() <= 0) return pos.z();
  // 如果超过，则比例内插
  double scale = x / std::max(1e-6, pos.x());
  return pos.z() * scale;
}

double RK4Compensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  Eigen::Vector3d target = target_position;
  const double dist = target.head<2>().norm();
  const double yaw = std::atan2(target.y(), target.x());
  double pitch = std::atan2(target.z(), dist);
  const double hit_th = rk4_hit_distance_threshold;
  const int max_steps = rk4_max_steps;
  const double dt = rk4_time_step;

  Eigen::Vector3d velocity;
  double v0 = this->velocity;
  velocity.x() = v0 * std::cos(pitch) * std::cos(yaw);
  velocity.y() = v0 * std::cos(pitch) * std::sin(yaw);
  velocity.z() = v0 * std::sin(pitch);

  Eigen::Vector3d pos(0, 0, 0);
  double t = 0.0;

  for (int i = 0; i < max_steps; ++i) {
    Eigen::Vector3d diff = target - pos;
    if (diff.norm() < hit_th) return t;

    rk4_integrate_step(velocity, pos, gravity, resistance, dt);

    t += dt;
    if (i % 100 == 0 && pos.norm() > target.norm() * 1.5) break; // 远离目标则提前退出
  }
  return std::numeric_limits<double>::quiet_NaN();
}

bool CeresCompensator::compensate(const Eigen::Vector3d &target_position, double &pitch) const noexcept {
  const double distance = target_position.head(2).norm();
  if (distance <= 1e-6 || velocity <= 1e-6) {
    return false;
  }

  double shoot_angle =
    std::clamp(std::atan2(target_position.z(), distance), -kPitchLimit, kPitchLimit);
  ceres::Problem problem;
  problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<LinearResistanceResidual, 1, 1>(
      new LinearResistanceResidual(
        distance, target_position.z(), velocity, gravity, resistance)),
    nullptr,
    &shoot_angle);
  problem.SetParameterLowerBound(&shoot_angle, 0, -kPitchLimit);
  problem.SetParameterUpperBound(&shoot_angle, 0, kPitchLimit);

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!summary.IsSolutionUsable() || !std::isfinite(shoot_angle)) {
    return false;
  }

  pitch = shoot_angle;
  return true;
}

double CeresCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  const double r = resistance;
  const double cos_angle = std::cos(angle);
  if (velocity <= 1e-6 || std::abs(cos_angle) <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double remaining_ratio = 1.0 - (r * x) / (velocity * cos_angle);
  if (remaining_ratio <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return (r * velocity * std::sin(angle) + gravity) * x / (r * velocity * cos_angle)
       + gravity * std::log(remaining_ratio) / (r * r);
}

double CeresCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  const double r = resistance;
  const double distance = target_position.head(2).norm();
  const double angle = std::atan2(target_position.z(), distance);
  const double cos_angle = std::cos(angle);
  if (velocity <= 1e-6 || std::abs(cos_angle) <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double remaining_ratio = 1.0 - (r * distance) / (velocity * cos_angle);
  if (remaining_ratio <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return -std::log(remaining_ratio) / r;
}
}  // namespace qd
