#include "armor_solver/spin_imm.hpp"

#include <algorithm>
#include <limits>

namespace qd::auto_aim {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON = 1e-12;
}

double SpinIMM::normalize_angle(double angle) noexcept {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

void SpinIMM::set_params(const Params & params) {
    params_ = params;
    params_.measurement_noise = std::max(1e-9, params_.measurement_noise);
    params_.dt_min = std::max(1e-6, params_.dt_min);
    params_.dt_max = std::max(params_.dt_min, params_.dt_max);
    params_.minimum_probability = std::clamp(
        params_.minimum_probability, 0.0, 1.0 / static_cast<double>(MODEL_COUNT));
    params_.nis_gate = std::max(1.0, params_.nis_gate);
    for (auto & decay : params_.alpha_decay) decay = std::clamp(decay, 0.0, 1.0);
}

void SpinIMM::initialize(double yaw, double velocity, double acceleration) {
    initialized_ = true;
    fused_ << normalize_angle(yaw), velocity, acceleration;
    fused_covariance_.setIdentity();
    fused_covariance_.diagonal() << 0.4, 100.0, 100.0;
    const double probability = 1.0 / static_cast<double>(MODEL_COUNT);
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        models_[i].x = fused_;
        models_[i].covariance = fused_covariance_;
        models_[i].probability = locked_model_ && *locked_model_ == i ? 1.0 : probability;
        models_[i].likelihood = 1.0;
    }
    if (locked_model_) {
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            models_[i].probability = i == *locked_model_ ? 1.0 : 0.0;
        }
    }
}

void SpinIMM::reset() noexcept {
    initialized_ = false;
    fused_.setZero();
    fused_covariance_.setIdentity();
    for (auto & model : models_) {
        model.x.setZero();
        model.covariance.setIdentity();
        model.probability = 1.0 / static_cast<double>(MODEL_COUNT);
        model.likelihood = 1.0;
    }
}

void SpinIMM::set_model_lock(std::optional<std::size_t> index) {
    if (index && *index >= MODEL_COUNT) index.reset();
    locked_model_ = index;
    if (!initialized_ || !locked_model_) return;
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        models_[i].probability = i == *locked_model_ ? 1.0 : 0.0;
    }
}

void SpinIMM::clear_model_lock() {
    locked_model_.reset();
    if (!initialized_) return;
    for (auto & model : models_) model.probability = 1.0 / static_cast<double>(MODEL_COUNT);
}

void SpinIMM::mix_states() {
    const auto prior = models_;
    for (std::size_t j = 0; j < MODEL_COUNT; ++j) {
        double c = 0.0;
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            c += params_.transition(static_cast<int>(i), static_cast<int>(j)) *
                 prior[i].probability;
        }
        c = std::max(EPSILON, c);
        mixing_prior_[j] = c;
        std::array<double, MODEL_COUNT> weights {};
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            weights[i] = params_.transition(static_cast<int>(i), static_cast<int>(j)) *
                         prior[i].probability / c;
        }

        std::size_t reference = 0;
        for (std::size_t i = 1; i < MODEL_COUNT; ++i) {
            if (weights[i] > weights[reference]) reference = i;
        }
        const double reference_yaw = prior[reference].x[0];
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            const double yaw = reference_yaw + normalize_angle(prior[i].x[0] - reference_yaw);
            mean[0] += weights[i] * yaw;
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
        models_[j].covariance = covariance;
    }
}

void SpinIMM::predict_model(Model & model, std::size_t index, double dt) const {
    const double dt2 = dt * dt;
    Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
    transition(0, 1) = dt;
    transition(0, 2) = 0.5 * dt2;
    transition(1, 2) = dt;
    transition(2, 2) = params_.alpha_decay[index];
    Eigen::Matrix3d noise = Eigen::Matrix3d::Zero();
    const double qv = std::max(0.0, params_.q_velocity[index]);
    const double qa = std::max(0.0, params_.q_acceleration[index]);
    noise(0, 0) = 0.25 * qv * dt2 * dt2;
    noise(1, 1) = qv * dt2;
    noise(2, 2) = qa * std::max(dt, params_.dt_min);
    model.x = transition * model.x;
    model.x[0] = normalize_angle(model.x[0]);
    model.covariance = transition * model.covariance * transition.transpose() + noise;
    model.covariance = 0.5 * (model.covariance + model.covariance.transpose());
}

void SpinIMM::predict(double dt) {
    if (!initialized_) return;
    dt = std::clamp(dt, params_.dt_min, params_.dt_max);
    mix_states();
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) predict_model(models_[i], i, dt);
    fuse_states();
}

bool SpinIMM::update(double observed_yaw, double measurement_noise) {
    if (!initialized_) return false;
    const double r = std::max(
        1e-9, measurement_noise > 0.0 ? measurement_noise : params_.measurement_noise);
    double probability_sum = 0.0;
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        auto & model = models_[i];
        const double innovation = normalize_angle(observed_yaw - model.x[0]);
        const double innovation_variance = std::max(EPSILON, model.covariance(0, 0) + r);
        const double nis = innovation * innovation / innovation_variance;
        model.likelihood = std::exp(-0.5 * nis) / std::sqrt(2.0 * PI * innovation_variance);
        if (nis <= params_.nis_gate) {
            const Eigen::Vector3d gain = model.covariance.col(0) / innovation_variance;
            model.x += gain * innovation;
            model.x[0] = normalize_angle(model.x[0]);
            model.covariance -= gain * model.covariance.row(0);
            model.covariance = 0.5 * (model.covariance + model.covariance.transpose());
        } else {
            model.likelihood *= 0.05;
        }
        probability_sum += mixing_prior_[i] * model.likelihood;
    }
    if (probability_sum <= EPSILON) probability_sum = EPSILON;
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        models_[i].probability = mixing_prior_[i] * models_[i].likelihood / probability_sum;
    }
    if (locked_model_) {
        for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
            models_[i].probability = i == *locked_model_ ? 1.0 : 0.0;
        }
    }
    fuse_states();
    return true;
}

void SpinIMM::fuse_states() {
    std::size_t reference = 0;
    for (std::size_t i = 1; i < MODEL_COUNT; ++i) {
        if (models_[i].probability > models_[reference].probability) reference = i;
    }
    const double reference_yaw = models_[reference].x[0];
    fused_.setZero();
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        fused_[0] += models_[i].probability *
                     (reference_yaw + normalize_angle(models_[i].x[0] - reference_yaw));
        fused_[1] += models_[i].probability * models_[i].x[1];
        fused_[2] += models_[i].probability * models_[i].x[2];
    }
    fused_[0] = normalize_angle(fused_[0]);
    fused_covariance_.setZero();
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) {
        Eigen::Vector3d delta = models_[i].x - fused_;
        delta[0] = normalize_angle(delta[0]);
        fused_covariance_ += models_[i].probability *
                             (models_[i].covariance + delta * delta.transpose());
    }
}

std::array<double, SpinIMM::MODEL_COUNT> SpinIMM::model_probabilities() const noexcept {
    std::array<double, MODEL_COUNT> result {};
    for (std::size_t i = 0; i < MODEL_COUNT; ++i) result[i] = models_[i].probability;
    return result;
}

}  // namespace qd::auto_aim
