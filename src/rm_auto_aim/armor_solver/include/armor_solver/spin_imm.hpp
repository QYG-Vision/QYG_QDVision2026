#ifndef ARMOR_SOLVER_SPIN_IMM_HPP_
#define ARMOR_SOLVER_SPIN_IMM_HPP_

#include <array>
#include <cstddef>
#include <optional>

#include <Eigen/Dense>

namespace qd::auto_aim {

/** @brief 通过慢转、匀速和变速模型估计目标旋转状态。 */
class SpinIMM {
public:
    static constexpr std::size_t MODEL_COUNT = 3;

    struct Params {
        Eigen::Matrix3d transition = (Eigen::Matrix3d() <<
            0.93, 0.05, 0.02,
            0.04, 0.93, 0.03,
            0.03, 0.07, 0.90).finished();
        double r_yaw = 2e-3;
        std::array<double, MODEL_COUNT> q_v {8e-2, 2e-2, 2e-1};
        std::array<double, MODEL_COUNT> q_alpha {1e-1, 5e-2, 8e-1};
        std::array<double, MODEL_COUNT> alpha_decay {0.2, 0.5, 1.0};
        double dt_min = 1e-3;
        double dt_max = 0.2;
        double mu_min = 1e-4;
        double nis_gate = 9.0;
    };

    SpinIMM() = default;

    void set_params(const Params & params);
    void initialize(
        double yaw, double v_yaw, double alpha_yaw,
        const Eigen::Vector3d & covariance_diagonal = Eigen::Vector3d(0.4, 100.0, 100.0));
    void reset() noexcept;
    void predict(double dt);
    bool update(double observed_yaw, double r_yaw = -1.0);

    bool initialized() const noexcept { return initialized_; }
    double yaw() const noexcept { return fused_[0]; }
    double v_yaw() const noexcept { return fused_[1]; }
    double alpha_yaw() const noexcept { return fused_[2]; }
    std::array<double, MODEL_COUNT> model_probabilities() const noexcept;
    std::array<double, MODEL_COUNT> model_angular_velocities() const noexcept;
    std::array<double, MODEL_COUNT> model_angular_accelerations() const noexcept;
    double innovation() const noexcept;
    double innovation_variance() const noexcept;
    bool last_update_gated() const noexcept;

    void set_model_lock(std::optional<std::size_t> model_index);
    void clear_model_lock();

private:
    struct Model {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
        double probability = 1.0 / static_cast<double>(MODEL_COUNT);
        double likelihood = 1.0;
        double innovation = 0.0;
        double innovation_variance = 0.0;
        bool gated = false;
    };

    static double normalize_angle(double angle) noexcept;
    void mix_states();
    void predict_model(Model & model, std::size_t index, double dt) const;
    void update_model(Model & model, double observed_yaw, double r_yaw);
    void fuse_states();
    void set_locked_probabilities() noexcept;
    std::size_t most_likely_model() const noexcept;

    Params params_;
    std::array<Model, MODEL_COUNT> models_;
    Eigen::Vector3d fused_ = Eigen::Vector3d::Zero();
    std::array<double, MODEL_COUNT> mixing_prior_ {};
    bool initialized_ = false;
    std::optional<std::size_t> locked_model_;
};

}  // namespace qd::auto_aim

#endif  // ARMOR_SOLVER_SPIN_IMM_HPP_
