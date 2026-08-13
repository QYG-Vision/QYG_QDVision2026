// Copyright Chen Jun 2023. Licensed under the MIT License.
// Copyright xinyang 2021.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#ifndef RM_UTILS_KALMAN_FILTER_HPP_
#define RM_UTILS_KALMAN_FILTER_HPP_

// std
#include <functional>
// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/jet.h>

namespace qd {

// Extended Kalman Filter with auto differentiation
// N_X: state vector dimension
// N_Z: measurement vector dimension
// PredicFunc: process nonlinear vector function
// MeasureFunc: observation nonlinear vector function
template <int N_X, int N_Z, class PredicFunc, class MeasureFunc>
class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter() = default;

  using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
  using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
  using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
  using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
  using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
  using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;

  using UpdateQFunc = std::function<MatrixXX()>;
  using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1 &z)>;
  using UpdateInnovationFunc = std::function<MatrixZ1(const MatrixZ1 &z, const MatrixZ1 &z_pri)>;

  explicit ExtendedKalmanFilter(const PredicFunc &f,
                                const MeasureFunc &h,
                                const UpdateQFunc &u_q,
                                const UpdateRFunc &u_r,
                                const MatrixXX &P0,
                                const UpdateInnovationFunc &u_i = nullptr) noexcept
  : f(f), h(h), update_Q(u_q), update_R(u_r), update_innovation(u_i), P_init(P0), P_post(P0) {
    F = MatrixXX::Zero();
    H = MatrixZX::Zero();
  }

  // Set the initial state
  void init(const MatrixX1 &x0) noexcept {
    x_post = x0;
    x_pri = x0;
    P_post = P_init;
    P_pri = P_init;
  }

  void setState(const MatrixX1 &x0) noexcept { x_post = x0; x_pri = x0;}

  void setPredictFunc(const PredicFunc &f) noexcept { this->f = f; }

  void setMeasureFunc(const MeasureFunc &h) noexcept { this->h = h; }

  void setInnovationFunc(const UpdateInnovationFunc &u_i) noexcept {
    this->update_innovation = u_i;
  }

  // Compute a predicted state
  MatrixX1 predict() noexcept {
    ceres::Jet<double, N_X> x_e_jet[N_X];
    for (int i = 0; i < N_X; ++i) {
      x_e_jet[i].a = x_post[i];
      x_e_jet[i].v[i] = 1.;
      // a 对自己的偏导数为 1.
    }
    ceres::Jet<double, N_X> x_p_jet[N_X];
    f(x_e_jet, x_p_jet);

    for (int i = 0; i < N_X; ++i) {
      x_pri[i] = x_p_jet[i].a;
      F.block(i, 0, 1, N_X) = x_p_jet[i].v.transpose();
    }

    Q = update_Q();
    P_pri = F * P_post * F.transpose() + Q;
    x_post = x_pri;

    return x_pri;
  }

  // Update the estimated state based on measurement
  MatrixX1 update(const MatrixZ1 &z) noexcept {
    ceres::Jet<double, N_X> x_p_jet[N_X];
    for (int i = 0; i < N_X; i++) {
      x_p_jet[i].a = x_pri[i];
      x_p_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> z_p_jet[N_Z];
    h(x_p_jet, z_p_jet);

    MatrixZ1 z_pri;
    for (int i = 0; i < N_Z; i++) {
      z_pri[i] = z_p_jet[i].a;
      H.block(i, 0, 1, N_X) = z_p_jet[i].v.transpose();
    }

    R = update_R(z);
    MatrixZZ S = H * P_pri * H.transpose() + R;
    K = P_pri * H.transpose() * S.inverse();
    MatrixZ1 innovation = z - z_pri;

    // 角度保护
    if (update_innovation) {
      innovation = update_innovation(z, z_pri);
    }
    x_post = x_post + K * innovation;
    P_post = (MatrixXX::Identity() - K * H) * P_pri;

    // NIS (Normalized Innovation Squared) for chi-square consistency test
    nis_ = (innovation.transpose() * S.inverse() * innovation)(0, 0);

    // Support sequential updates: align prior to posterior so that
    // the next update() in the same frame linearizes at the latest estimate
    x_pri = x_post;
    P_pri = P_post;

    return x_post;
  }

  double getNIS() const noexcept { return nis_; }

  // Evaluate measurement association quality under the current priori state.
  double calculateMahalanobisDistance(const MatrixZ1 &z) noexcept {
    ceres::Jet<double, N_X> x_p_jet[N_X];
    for (int i = 0; i < N_X; i++) {
      x_p_jet[i].a = x_pri[i];
      x_p_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> z_p_jet[N_Z];
    h(x_p_jet, z_p_jet);

    MatrixZ1 z_pri;
    MatrixZX H_eval = MatrixZX::Zero();
    for (int i = 0; i < N_Z; i++) {
      z_pri[i] = z_p_jet[i].a;
      H_eval.block(i, 0, 1, N_X) = z_p_jet[i].v.transpose();
    }

    MatrixZ1 innovation = z - z_pri;
    if (update_innovation) {
      innovation = update_innovation(z, z_pri);
    }

    MatrixZZ R_eval = update_R(z);
    MatrixZZ S = H_eval * P_pri * H_eval.transpose() + R_eval;
    return (innovation.transpose() * S.ldlt().solve(innovation))(0, 0);
  }

private:
  // Process nonlinear vector function
  PredicFunc f;
  MatrixXX F;
  // Observation nonlinear vector function
  MeasureFunc h;
  MatrixZX H;
  // Process noise covariance matrix
  UpdateQFunc update_Q;
  MatrixXX Q;
  // Measurement noise covariance matrix
  UpdateRFunc update_R;
  MatrixZZ R;
  // Optional measurement residual post-process, e.g. angle wrap-around handling
  UpdateInnovationFunc update_innovation;

  // Priori error estimate covariance matrix
  MatrixXX P_pri;
  // Posteriori error estimate covariance matrix
  MatrixXX P_init;
  MatrixXX P_post;

  // Kalman gain
  MatrixXZ K;

  // Priori state
  MatrixX1 x_pri;
  // Posteriori state
  MatrixX1 x_post;

  // NIS (Normalized Innovation Squared)
  double nis_{0.0};
};

}  // namespace qd

#endif  // RM_UTILS_KALMAN_FILTER_HPP_
