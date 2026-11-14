#ifndef ARMOR_SOLVER_SPIN_IMM_HPP_
#define ARMOR_SOLVER_SPIN_IMM_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

#include <Eigen/Dense>

namespace qd::auto_aim {

/** @brief 用三个旋转模型估计目标的 yaw、角速度和角加速度。 */
class SpinIMM {
public:
    static constexpr std::size_t MODEL_COUNT = 3;

    struct Params {
        Eigen::Matrix3d transition = (Eigen::Matrix3d() <<
            0.93, 0.05, 0.02,
            0.04, 0.93, 0.03,
            0.03, 0.07, 0.90).finished();
        double measurement_noise = 2e-3;
        double q_velocity[MODEL_COUNT] {8e-2, 2e-2, 2e-1};
        double q_acceleration[MODEL_COUNT] {1e-1, 5e-2, 8e-1};
        double alpha_decay[MODEL_COUNT] {0.2, 0.5, 1.0};
        double dt_min = 1e-3;
        double dt_max = 0.2;
        double minimum_probability = 1e-4;
        double nis_gate = 9.0;
    };

    SpinIMM() = default;

    void set_params(const Params & params);
    void initialize(double yaw, double velocity, double acceleration);
    void reset() noexcept;
    void predict(double dt);
    bool update(double observed_yaw, double measurement_noise = -1.0);

    bool initialized() const noexcept { return initialized_; }
    double yaw() const noexcept { return fused_[0]; }
    double velocity() const noexcept { return fused_[1]; }
    double acceleration() const noexcept { return fused_[2]; }
    std::array<double, MODEL_COUNT> model_probabilities() const noexcept;

    void set_model_lock(std::optional<std::size_t> index);
    void clear_model_lock();

private:
    struct Model {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
        double probability = 1.0 / static_cast<double>(MODEL_COUNT);
        double likelihood = 1.0;
    };

    static double normalize_angle(double angle) noexcept;
    void mix_states();
    void fuse_states();
    void predict_model(Model & model, std::size_t index, double dt) const;

    Params params_;
    std::array<Model, MODEL_COUNT> models_;
    Eigen::Vector3d fused_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d fused_covariance_ = Eigen::Matrix3d::Identity();
    std::array<double, MODEL_COUNT> mixing_prior_ {};
    bool initialized_ = false;
    std::optional<std::size_t> locked_model_;
};

}  // namespace qd::auto_aim

#endif  // ARMOR_SOLVER_SPIN_IMM_HPP_
