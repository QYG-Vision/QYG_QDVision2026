#include "armor_solver/spin_imm.hpp"

#include <algorithm>
#include <cmath>

namespace qd::auto_aim {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON = 1e-12;
}

double SpinIMM::normalize_angle(double angle) noexcept {
    return std::remainder(angle, 2.0 * PI);
}

void SpinIMM::set_params(const Params & params) {
    params_ = params;
    params_.r_yaw = std::isfinite(params_.r_yaw) ? std::max(EPSILON, params_.r_yaw) : 2e-3;
    params_.dt_min = std::isfinite(params_.dt_min) ? std::max(1e-6, params_.dt_min) : 1e-3;
    params_.dt_max = std::isfinite(params_.dt_max) ? std::max(params_.dt_min, params_.dt_max) : 0.2;
    params_.mu_min = std::isfinite(params_.mu_min)
                         ? std::clamp(params_.mu_min, 0.0, 1.0 / 3.0)
                         : 1e-4;
    params_.nis_gate = std::isfinite(params_.nis_gate) ? std::max(1.0, params_.nis_gate) : 9.0;

    for (int row = 0; row < static_cast<int>(MODEL_COUNT); ++row) {
        double sum = 0.0;
        for (int col = 0; col < static_cast<int>(MODEL_COUNT); ++col) {
            double & value = params_.transition(row, col);
            value = std::isfinite(value) ? std::max(0.0, value) : 0.0;
            sum += value;
        }
        if (sum <= EPSILON) {
            params_.transition.row(row).setConstant(1.0 / static_cast<double>(MODEL_COUNT));
        } else {
            params_.transition.row(row) /= sum;
        }
    }

    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        params_.q_v[i] = std::isfinite(params_.q_v[i]) ? std::max(0.0, params_.q_v[i]) : 0.0;
        params_.q_alpha[i] =
            std::isfinite(params_.q_alpha[i]) ? std::max(0.0, params_.q_alpha[i]) : 0.0;
        params_.alpha_decay[i] = std::isfinite(params_.alpha_decay[i])
                                     ? std::clamp(params_.alpha_decay[i], 0.0, 1.0)
                                     : 1.0;
    }
}

void SpinIMM::initialize(
    double yaw, double v_yaw, double alpha_yaw, const Eigen::Vector3d & covariance_diagonal) {
    if (!std::isfinite(yaw)) yaw = 0.0;
    if (!std::isfinite(v_yaw)) v_yaw = 0.0;
    if (!std::isfinite(alpha_yaw)) alpha_yaw = 0.0;
    fused_ << normalize_angle(yaw), v_yaw, alpha_yaw;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
        const double variance = covariance_diagonal[i];
        covariance(i, i) = std::isfinite(variance) ? std::max(EPSILON, variance) : 1.0;
    }

    for (auto & model : models_) {
        model.x = fused_;
        model.covariance = covariance;
        model.likelihood = 1.0;
        model.innovation = 0.0;
        model.innovation_variance = 0.0;
        model.gated = false;
    }
    initialized_ = true;
    mixing_prior_.fill(1.0 / static_cast<double>(MODEL_COUNT));
    set_locked_probabilities();
    fuse_states();
}

void SpinIMM::reset() noexcept {
    initialized_ = false;
    locked_model_.reset();
    fused_.setZero();
    mixing_prior_.fill(1.0 / static_cast<double>(MODEL_COUNT));
    for (auto & model : models_) model = Model {};
}

void SpinIMM::set_locked_probabilities() noexcept {
    const double uniform = 1.0 / static_cast<double>(MODEL_COUNT);
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        models_[i].probability = locked_model_ && *locked_model_ == i ? 1.0 : uniform;
    }
    if (locked_model_) {
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            models_[i].probability = i == *locked_model_ ? 1.0 : 0.0;
        }
    }
}

std::size_t SpinIMM::most_likely_model() const noexcept {
    std::size_t best = 0;
    for (std::size_t i = 1; i < MODEL_COUNT; ++i) {
        if (models_[i].probability > models_[best].probability) best = i;
    }
    return best;
}

void SpinIMM::set_model_lock(std::optional<std::size_t> model_index) {
    if (model_index && *model_index >= MODEL_COUNT) model_index.reset();
    locked_model_ = model_index;
    if (initialized_) set_locked_probabilities();
}

void SpinIMM::clear_model_lock() {
    locked_model_.reset();
    if (initialized_) set_locked_probabilities();
}

void SpinIMM::mix_states() {
    const auto prior = models_;
    for (std::size_t j = 0; j < MODEL_COUNT; ++j) {
        double normalizer = 0.0;
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            normalizer += params_.transition(static_cast<int>(i), static_cast<int>(j)) *
                          std::max(0.0, prior[i].probability);
        }
        normalizer = std::max(EPSILON, normalizer);
        mixing_prior_[j] = normalizer;

        std::array<double, MODEL_COUNT> weights {};
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            weights[i] = params_.transition(static_cast<int>(i), static_cast<int>(j)) *
                         std::max(0.0, prior[i].probability) / normalizer;
        }

        std::size_t reference = 0;
        for (std::size_t i = 1; i < MODEL_COUNT; ++i) {
            if (weights[i] > weights[reference]) reference = i;
        }
        const double reference_yaw = prior[reference].x[0];
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            mean[0] += weights[i] *
                       (reference_yaw + normalize_angle(prior[i].x[0] - reference_yaw));
            mean[1] += weights[i] * prior[i].x[1];
            mean[2] += weights[i] * prior[i].x[2];
        }
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            Eigen::Vector3d delta = prior[i].x - mean;
            delta[0] = normalize_angle(delta[0]);
            covariance += weights[i] * (prior[i].covariance + delta * delta.transpose());
        }
        models_[j].x = mean;
        models_[j].x[0] = normalize_angle(models_[j].x[0]);
        models_[j].covariance = 0.5 * (covariance + covariance.transpose());
    }
}

void SpinIMM::predict_model(Model & model, std::size_t index, double dt) const {
    Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
    transition(0, 1) = dt;
    transition(0, 2) = 0.5 * dt * dt;
    transition(1, 2) = dt;
    transition(2, 2) = params_.alpha_decay[index];

    Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();
    process_noise(0, 0) = params_.q_v[index] * dt;
    process_noise(1, 1) = params_.q_v[index];
    process_noise(2, 2) = params_.q_alpha[index];

    model.x = transition * model.x;
    model.x[0] = normalize_angle(model.x[0]);
    model.covariance = transition * model.covariance * transition.transpose() + process_noise;
    model.covariance = 0.5 * (model.covariance + model.covariance.transpose());
}

void SpinIMM::predict(double dt) {
    if (!initialized_) return;
    if (!std::isfinite(dt)) dt = params_.dt_min;
    dt = std::clamp(dt, params_.dt_min, params_.dt_max);

    if (!locked_model_) {
        mix_states();
    } else {
        mixing_prior_.fill(0.0);
        mixing_prior_[*locked_model_] = 1.0;
    }

    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        if (locked_model_ && i != *locked_model_) {
            models_[i].probability = 0.0;
            continue;
        }
        predict_model(models_[i], i, dt);
        models_[i].probability = locked_model_ ? 1.0 : mixing_prior_[i];
        models_[i].likelihood = 1.0;
    }
    fuse_states();
}

void SpinIMM::update_model(Model & model, double observed_yaw, double r_yaw) {
    const double innovation = normalize_angle(observed_yaw - model.x[0]);
    const double innovation_variance = std::max(EPSILON, model.covariance(0, 0) + r_yaw);
    const double nis = innovation * innovation / innovation_variance;
    model.innovation = innovation;
    model.innovation_variance = innovation_variance;
    model.gated = nis > params_.nis_gate;

    if (model.gated) {
        model.likelihood = EPSILON;
        return;
    }

    const Eigen::Vector3d gain = model.covariance.col(0) / innovation_variance;
    model.x += gain * innovation;
    model.x[0] = normalize_angle(model.x[0]);

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction.col(0) -= gain;
    model.covariance = correction * model.covariance * correction.transpose() +
                       r_yaw * gain * gain.transpose();
    model.covariance = 0.5 * (model.covariance + model.covariance.transpose());

    model.likelihood = std::exp(-0.5 * nis) / std::sqrt(2.0 * PI * innovation_variance);
    if (!std::isfinite(model.likelihood) || model.likelihood < EPSILON) {
        model.likelihood = EPSILON;
    }
}

bool SpinIMM::update(double observed_yaw, double r_yaw) {
    if (!initialized_ || !std::isfinite(observed_yaw)) return false;
    const double measurement_noise = std::isfinite(r_yaw) && r_yaw > 0.0
                                        ? std::max(EPSILON, r_yaw)
                                        : params_.r_yaw;

    if (locked_model_) {
        const auto index = *locked_model_;
        update_model(models_[index], observed_yaw, measurement_noise);
        models_[index].probability = 1.0;
        fuse_states();
        return !models_[index].gated;
    }

    double posterior_sum = 0.0;
    bool any_model_updated = false;
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        update_model(models_[i], observed_yaw, measurement_noise);
        any_model_updated = any_model_updated || !models_[i].gated;
        models_[i].probability = mixing_prior_[i] * models_[i].likelihood;
        posterior_sum += models_[i].probability;
    }

    if (!any_model_updated) {
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            models_[i].probability = mixing_prior_[i];
        }
        fuse_states();
        return false;
    }

    if (!std::isfinite(posterior_sum) || posterior_sum <= EPSILON) {
        for (auto & model : models_) model.probability = 1.0 / static_cast<double>(MODEL_COUNT);
    } else {
        for (auto & model : models_) model.probability /= posterior_sum;
        if (params_.mu_min > 0.0) {
            double probability_sum = 0.0;
            for (auto & model : models_) {
                model.probability = std::max(params_.mu_min, model.probability);
                probability_sum += model.probability;
            }
            for (auto & model : models_) model.probability /= probability_sum;
        }
    }
    fuse_states();
    return any_model_updated;
}

void SpinIMM::fuse_states() {
    std::size_t reference = 0;
    for (std::size_t i = 1; i < MODEL_COUNT; ++i) {
        if (models_[i].probability > models_[reference].probability) reference = i;
    }
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    fused_[1] = 0.0;
    fused_[2] = 0.0;
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        const double weight = std::max(0.0, models_[i].probability);
        sin_sum += weight * std::sin(models_[i].x[0]);
        cos_sum += weight * std::cos(models_[i].x[0]);
        fused_[1] += weight * models_[i].x[1];
        fused_[2] += weight * models_[i].x[2];
    }
    if (std::hypot(sin_sum, cos_sum) <= EPSILON) {
        fused_[0] = models_[reference].x[0];
    } else {
        fused_[0] = normalize_angle(std::atan2(sin_sum, cos_sum));
    }

}

std::array<double, SpinIMM::MODEL_COUNT> SpinIMM::model_probabilities() const noexcept {
    std::array<double, MODEL_COUNT> values {};
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) values[i] = models_[i].probability;
    return values;
}

std::array<double, SpinIMM::MODEL_COUNT> SpinIMM::model_angular_velocities() const noexcept {
    std::array<double, MODEL_COUNT> values {};
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) values[i] = models_[i].x[1];
    return values;
}

std::array<double, SpinIMM::MODEL_COUNT> SpinIMM::model_angular_accelerations() const noexcept {
    std::array<double, MODEL_COUNT> values {};
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) values[i] = models_[i].x[2];
    return values;
}

double SpinIMM::innovation() const noexcept {
    if (locked_model_) return models_[*locked_model_].innovation;
    return models_[most_likely_model()].innovation;
}

double SpinIMM::innovation_variance() const noexcept {
    if (locked_model_) return models_[*locked_model_].innovation_variance;
    return models_[most_likely_model()].innovation_variance;
}

bool SpinIMM::last_update_gated() const noexcept {
    if (locked_model_) return models_[*locked_model_].gated;
    return models_[most_likely_model()].gated;
}

}  // namespace qd::auto_aim
