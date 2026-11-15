// Adapted from HDU-PHOENIX/Sc_vision, commit e0e7c00e1a97ccf6a17df89d782b05e26711890d.
// Copyright (c) 2025 TongjiSuperPower. MIT License.
// See third_party/hdu_imm/LICENSE for the full license.
// 本工程适配：匹配预测接线、圆周协方差修正、状态转移一致性与真实时间推进。
#ifndef ARMOR_SOLVER_SPIN_IMM_HPP_
#define ARMOR_SOLVER_SPIN_IMM_HPP_

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <optional>

namespace qd::auto_aim
{

class SpinImm
{
public:
  static constexpr int kModelCount = 3;
  static constexpr int kStateDim = 3;  // [yaw, v_yaw, alpha_yaw]

  struct ModelState
  {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    double mu = 0.0;
    double likelihood = 0.0;
    double innovation = 0.0;
    double innovation_var = 0.0;
    bool gated = false;  // this model's last update was rejected by the chi-square gate
  };

  struct Params
  {
    Eigen::Matrix3d transition;
    double r_yaw;               // observation noise for yaw position (rad^2)
    double q_v_slow;
    double q_v_constant;
    double q_v_variable;
    double q_alpha_slow;
    double q_alpha_constant;
    double q_alpha_variable;
    double alpha_decay_slow;
    double alpha_decay_constant;
    double dt_min;
    double dt_max;
    double mu_min;
    double nis_gate;  // chi-square gate on innovation^2/S; gated observations do not correct the model
  };

  SpinImm();

  /** @brief 校验并设置三模型参数；非法参数抛出 invalid_argument。 */
  void set_params(const Params & params);
  /** @brief 返回当前滤波参数，供节点声明默认值。 */
  const Params & params() const { return params_; }
  /**
   * @brief 用同一时刻的旋转状态初始化三个模型。
   * @param yaw 参考相位，rad。
   * @param v_yaw 旋转角速度，rad/s。
   * @param alpha_yaw 旋转角加速度，rad/s²。
   * @param P_yaw 相位方差，rad²。
   * @param P_v 角速度方差，(rad/s)²。
   * @param P_alpha 角加速度方差，(rad/s²)²。
   */
  void initialize(double yaw, double v_yaw, double alpha_yaw, double P_yaw, double P_v, double P_alpha);
  void reset();

  void set_model_lock(std::optional<size_t> model_index);
  void clear_model_lock();
  bool model_locked() const { return locked_model_index_.has_value(); }
  std::optional<size_t> locked_model_index() const { return locked_model_index_; }

  bool initialized() const { return initialized_; }

  /**
   * @brief 将三模型推进完整的实际经过时间，大间隔分步处理。
   * @param dt 相邻图像时间差，秒；非正数或非有限值不推进。
   */
  void predict(double dt);
  /**
   * @brief 使用消除板号偏移后的整车相位进行量测更新。
   * @param observed_yaw 世界坐标系参考相位，rad。
   * @param r_yaw 朝向观测方差，rad²。
   * @return 已初始化且输入有限时返回 true；门控拒绝量测时仍保留预测状态。
   */
  bool update(double observed_yaw, double r_yaw);

  double yaw() const;
  double v_yaw() const;
  double alpha_yaw() const;
  double yaw_cov() const;
  double v_yaw_cov() const;
  double alpha_yaw_cov() const;

  // Innovation covariance S of the highest-probability model (for monitoring)
  double innovation_var() const;

  // Innovation and gate status of the highest-probability model (for monitoring)
  double innovation() const;
  bool last_update_gated() const;
  std::array<double, kModelCount> model_probabilities() const;
  std::array<double, kModelCount> model_angular_velocities() const;
  std::array<double, kModelCount> model_angular_accelerations() const;

private:
  static constexpr int kYawIdx = 0;
  static constexpr int kVIdx = 1;
  static constexpr int kAlphaIdx = 2;

  bool initialized_;
  Params params_;
  std::optional<size_t> locked_model_index_;
  std::array<ModelState, kModelCount> models_;
  double fused_yaw_;
  double fused_v_yaw_;
  double fused_alpha_yaw_;
  double fused_P_yaw_;
  double fused_P_v_;
  double fused_P_alpha_;
  std::array<double, kModelCount> c_bar_;

  static double normalize_angle(double angle);
  static double safe_det(const Eigen::MatrixXd & matrix);

  double model_q_v(size_t model_index) const;
  double model_q_alpha(size_t model_index) const;
  double alpha_decay(size_t model_index) const;

  void mix_states();
  void fuse_output();
};



}  // namespace qd::auto_aim

#endif  // ARMOR_SOLVER_SPIN_IMM_HPP_
