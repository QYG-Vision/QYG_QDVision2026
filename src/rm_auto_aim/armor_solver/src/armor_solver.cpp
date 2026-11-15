// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>

namespace qd::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n): node_(n) {
    auto node = node_.lock();
    // 装甲板大小
    shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
    shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
    // 延迟时间
    predict2send_delay_ = node->declare_parameter("solver.predict2send_delay", 0.0);
    controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
    additional_prediction_time = node->declare_parameter("solver.additional_prediction_time", 0.0);
    gravity = node->declare_parameter("solver.gravity", 9.8);
    // 空气阻力
    resistance = node->declare_parameter("solver.resistance", 0.001);

    use_armor_top_ = node->declare_parameter("solver.use_armor_top", false);
    use_ballistic_compensation_ =
        node->declare_parameter("solver.use_ballistic_compensation", true);

    // 补偿器
    std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
    trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
    trajectory_compensator_->iteration_times =
        node->declare_parameter("solver.iteration_times", 20);
    trajectory_compensator_->velocity = get_bullet_speed();
    trajectory_compensator_->gravity = gravity;
    trajectory_compensator_->resistance = resistance;
    trajectory_compensator_->rk4_time_step =
        node->declare_parameter("solver.rk4.time_step", 0.003);
    trajectory_compensator_->rk4_max_steps =
        static_cast<int>(node->declare_parameter("solver.rk4.max_steps", 8000));
    trajectory_compensator_->rk4_hit_distance_threshold =
        node->declare_parameter("solver.rk4.hit_distance_threshold", 0.02);

    // 角度补偿器
    manual_compensator_ = std::make_unique<ManualCompensator>();
    auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string> {});
    if (!manual_compensator_->updateMapFlow(angle_offset)) {
        FYT_WARN("armor_solver", "Manual compensator update failed!");
    }

    mpc_enabled_ = node->declare_parameter("solver.mpc.enabled", true);
    mpc_config_.horizon = node->declare_parameter("solver.mpc.horizon", 100);
    mpc_config_.dt = node->declare_parameter("solver.mpc.dt", 0.01);
    mpc_config_.position_weight = node->declare_parameter("solver.mpc.position_weight", 9e6);
    mpc_config_.velocity_weight = node->declare_parameter("solver.mpc.velocity_weight", 0.0);
    mpc_config_.acceleration_weight =
        node->declare_parameter("solver.mpc.acceleration_weight", 1.0);
    mpc_config_.max_iterations = node->declare_parameter("solver.mpc.max_iterations", 10);
    fire_thresh_ = node->declare_parameter("solver.mpc.fire_thresh", 0.003);
    feedback_timeout_ = node->declare_parameter("solver.mpc.feedback_timeout", 0.1);
    target_timeout_ = node->declare_parameter("solver.mpc.target_timeout", 0.3);
    mpc_config_.max_acceleration =
        node->declare_parameter("solver.mpc.yaw.max_acceleration", 2864.7889756541163);
    yaw_mpc_ = std::make_unique<AxisMpc>(mpc_config_);
    auto pitch_config = mpc_config_;
    pitch_config.max_acceleration =
        node->declare_parameter("solver.mpc.pitch.max_acceleration", 5729.5779513082325);
    pitch_mpc_ = std::make_unique<AxisMpc>(pitch_config);

    node.reset();
}

/**
 * @brief 获得瞄准信息
 * 
 * @param target 目标状态信息
 * @param current_time 当前时间
 * @param tf2_buffer_ ros的坐标监听，用于获取云台姿态
 * @return rm_interfaces::msg::GimbalCmd 
 */
rm_interfaces::msg::GimbalCmd Solver::solve(
    rm_interfaces::msg::Target& target,
    const rclcpp::Time& current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_
) {
    ballistic_debug_.reset();
    // Get newest parameters
    // 更新参数
    try {
        auto node = node_.lock();
        predict2send_delay_ = node->get_parameter("solver.predict2send_delay").as_double();
        controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
        additional_prediction_time =
            node->get_parameter("solver.additional_prediction_time").as_double();
        use_armor_top_ = node->get_parameter("solver.use_armor_top").as_bool();
        use_ballistic_compensation_ =
            node->get_parameter("solver.use_ballistic_compensation").as_bool();
        gimble_frame_ = node->get_parameter("gimble_frame").as_string();
        node.reset();

        prediction_delay_ = (current_time - target.header.stamp).seconds();
    } catch (const std::runtime_error& e) {
        FYT_ERROR("armor_solver", "{}", e.what());
    }

    // Get current roll, yaw and pitch of gimbal
    // 获得云台当前姿态
    try {
        auto gimbal_tf =
            tf2_buffer_->lookupTransform(target.header.frame_id, gimble_frame_, tf2::TimePointZero);
        auto msg_q = gimbal_tf.transform.rotation;

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(gimbal_rpy_[0], gimbal_rpy_[1], gimbal_rpy_[2]);
        gimbal_rpy_[1] = -gimbal_rpy_[1]; // 转成向上为正
    } catch (tf2::TransformException& ex) {
        FYT_ERROR("armor_solver", "{}", ex.what());
        // 姿态只供可视化使用，不因缺少实际云台反馈阻断济瞄规划路径。
    }

    // 保留消息兼容字段；当前规划不再使用旧旋转分级。
    target.top_level = 0;

    // 在访问模型数组之前验证所用输入，避免坏目标被送入选板器或规划器。
    const auto finite_xyz = [](const auto& value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    bool valid_target = target.tracking && std::isfinite(target.v_yaw)
        && std::isfinite(target.yaw) && finite_xyz(target.position) && finite_xyz(target.velocity)
        && target.armors_num > 0
        && static_cast<size_t>(target.armors_num) <= target.radius_list.size()
        && static_cast<size_t>(target.armors_num) <= target.dz_list.size();
    if (valid_target) {
        for (int i = 0; i < target.armors_num; ++i) {
            valid_target = valid_target && std::isfinite(target.radius_list[i])
                && target.radius_list[i] > 0.0 && std::isfinite(target.dz_list[i]);
        }
    }
    if (!valid_target) {
        rm_interfaces::msg::GimbalCmd invalid;
        invalid.header = target.header;
        invalid.id = target.id;
        invalid.distance = -1.0;
        invalid.reference_status = "invalid_target";
        return invalid;
    }

    // 对齐济瞄 Planner：不进入旧追板、等板与火控链。
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;
    gimbal_cmd.id = target.id;
    gimbal_cmd.bullet_speed = get_bullet_speed();
    gimbal_cmd.target_v_yaw = target.v_yaw;
    if (!plan_mpc(target, current_time, gimbal_cmd)) {
        gimbal_cmd.reference_valid = false;
        gimbal_cmd.fire_advice = false;
    }
    return gimbal_cmd;
}

std::optional<BallisticDebugData> Solver::get_ballistic_debug() const noexcept {
    return ballistic_debug_;
}

/**
 * @brief 根据 pitch 模拟弹道轨迹
 * 
 * @return std::vector<std::pair<double, double>> 
 */
std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
    if (bullet_speed_state_.value() <= 0.0) {
        return {};
    }
    auto trajectory = trajectory_compensator_->getTrajectory(15, gimbal_rpy_[1]);
    // Rotate
    for (auto& p: trajectory) {
        double x = p.first;
        double y = p.second;
        p.first = x * cos(gimbal_rpy_[1]) + y * sin(gimbal_rpy_[1]);
        p.second = -x * sin(gimbal_rpy_[1]) + y * cos(gimbal_rpy_[1]);
    }
    return trajectory;
}

/**
  @brief 基于当前弹道模型计算飞行时间
  @param target_pos [IN] 目标三维坐标 xyz
  @return 飞行所需时间，单位秒
*/
double Solver::get_ballistic_flying_time(const Eigen::Vector3d& target_pos) {
    const double bullet_speed = get_bullet_speed();
    if (bullet_speed <= 0.0) {
        return 0.0;
    }
    const double distance_xy = target_pos.head(2).norm();

    double pitch = 0.0;
    if (!trajectory_compensator_ || !trajectory_compensator_->compensate(target_pos, pitch)) {
        return target_pos.norm() / bullet_speed;
    }

    Eigen::Vector3d aim_pos = target_pos;
    aim_pos.z() = distance_xy * std::tan(pitch);
    const double flying_time = trajectory_compensator_->getFlyingTime(aim_pos);

    if (!std::isfinite(flying_time) || flying_time < 0.0) {
        return target_pos.norm() / bullet_speed;
    }

    return flying_time;
}

/**
  @brief 迭代求解命中时刻对应的装甲板状态
  @param state [IN] 当前装甲板状态
  @param base_delay [IN] 开火前固定延迟
  @return 命中时刻的装甲板状态
*/
State Solver::solve_impact_state(const State& state, double base_delay) {
    constexpr int kMaxIterations = 8;
    constexpr double kTimeTolerance = 1e-3;
    constexpr double kRelaxation = 0.5;

    double dt = std::max(0.0, base_delay);
    State predicted_state = state_predict(state, dt);
    dt += get_ballistic_flying_time(state_to_armor_pos(predicted_state));
    predicted_state = state_predict(state, dt);

    for (int i = 0; i < kMaxIterations; ++i) {
        const double solved_dt = std::max(
            0.0,
            base_delay + get_ballistic_flying_time(state_to_armor_pos(predicted_state))
        );
        if (std::abs(solved_dt - dt) < kTimeTolerance) {
            dt = solved_dt;
            break;
        }

        dt = (1.0 - kRelaxation) * dt + kRelaxation * solved_dt;
        predicted_state = state_predict(state, dt);
    }

    return state_predict(state, dt);
}

/**
  @brief 获得各个装甲板的向量，xyz是中心点，便于后面想要直接算出当前板子的位置
*/
std::vector<State> Solver::get_armors_state(const RosTarget& target_msg) {
    auto target_yaw = target_msg.yaw_;
    auto v_yaw = target_msg.v_yaw_;
    auto c_pos = target_msg.center_position_;
    auto c_vel = target_msg.center_velocity_;
    // auto zc = c_pos.z();

    auto armors_num = target_msg.armors_num_;
    auto r_list = target_msg.radius_list_;
    auto dz_list = target_msg.dz_list_;
    auto armors_state = std::vector<State>(
        armors_num,
        { c_pos.x(), c_vel.x(), c_pos.y(), c_vel.y(), c_pos.z(), c_vel.z(), 0.0, v_yaw, 0.0 }
    );
    // Calculate the position of each armor
    double r = 0., target_dz = 0.;
    for (int i = 0; i < armors_num; i++) {
        double temp_yaw = reduced_angle(target_yaw + i * (2 * M_PI / armors_num));

        r = r_list[i]; // 使用每块装甲板的半径
        target_dz = dz_list[i];
        armors_state[i](4) = target_dz;
        armors_state[i](6) = temp_yaw;
        armors_state[i](8) = r;
    }
    return armors_state;
}

/**
  @brief 弹道求解，在于计算 pitch
*/
ShootParam Solver::target_pos_to_shoot_param(const Eigen::Vector3d& target_pos) {
    Eigen::Vector3d target_xyz_i_barrel = target_pos;
    const double bs = this->get_bullet_speed();
    const double target_xy = std::hypot(target_xyz_i_barrel.x(), target_xyz_i_barrel.y());
    const double geometric_pitch = std::atan2(target_xyz_i_barrel.z(), target_xy);
    double shoot_angle = geometric_pitch;
    Eigen::Vector3d aim_xyz_i_barrel = target_xyz_i_barrel;

    double compensated_pitch = shoot_angle;
    // 仅切换发射角；命中时刻的飞行时间预测仍使用原有弹道模型。
    const bool ballistic_success =
        use_ballistic_compensation_
        && bs > 0.0
        && trajectory_compensator_->compensate(target_xyz_i_barrel, compensated_pitch)
        && std::isfinite(compensated_pitch);
    if (ballistic_success)
    {
        shoot_angle = compensated_pitch;
        aim_xyz_i_barrel.z() = target_xy * std::tan(shoot_angle);
    }

    return ShootParam {
        bs, shoot_angle, aim_xyz_i_barrel, target_pos, geometric_pitch, ballistic_success
    };
}

/** @brief 使用电控最新弹速更新弹道模型。 */
void Solver::update_bullet_speed(double raw_bullet_speed) {
    if (!BulletSpeedState::is_valid_sample(raw_bullet_speed)) {
        return;
    }

    bullet_speed_state_.update(raw_bullet_speed);

    if (trajectory_compensator_) {
        trajectory_compensator_->velocity = get_bullet_speed();
    }
}

/**
  @brief 获得子弹速度
*/
double Solver::get_bullet_speed() {
    return bullet_speed_state_.value();
}

/**
  @brief 预测装甲板dt后的向量
*/
State Solver::state_predict(const State& state, double dt) {
    State state_predict = state;
    state_predict(0) += state_predict(1) * dt;
    state_predict(2) += state_predict(3) * dt;
    state_predict(4) += state_predict(5) * dt;
    state_predict(6) = reduced_angle(state_predict(6) + state_predict(7) * dt);

    return state_predict;
}

/**
  @brief 获得装甲板的xyz
*/
Eigen::Vector3d Solver::state_to_armor_pos(const State& state) {
    return Eigen::Vector3d { state[0] - (std::cos(state[6]) * state[8]),
                             state[2] - (std::sin(state[6]) * state[8]),
                             state[4] };
}

/**
  @brief 将xyz坐标转化成ypd
*/
YpdCoord Solver::xyz_to_ypd(const Eigen::Vector3d& xyz) {
    YpdCoord ypd;
    ypd.yaw = std::atan2(xyz(1, 0), xyz(0, 0));
    ypd.pitch = std::atan2(xyz(2, 0), std::sqrt(xyz(0, 0) * xyz(0, 0) + xyz(1, 0) * xyz(1, 0)));
    ypd.distance = xyz.norm();

    return ypd;
}

/**
  @brief 将xyz转成ypd，带手动角度补偿
*/
YpdCoord Solver::aim_xyz_to_aim_ypd(const Eigen::Vector3d& xyz) {
    YpdCoord aim_ypd = xyz_to_ypd(xyz);

    auto angle_offset = manual_compensator_->angleHardCorrect(xyz.head(2).norm(), xyz.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;

    // NEED DEBUG
    return { reduced_angle(aim_ypd.yaw + yaw_offset),
             aim_ypd.pitch + pitch_offset,
             aim_ypd.distance };
}

/**
 * @brief 计算目标瞄准角及同一次解算的弹道角度。
 * @param xyz 目标在 odom 坐标系中的位置，单位 m。
 * @return 带手动角度补偿的瞄准角，以及手动补偿前的弹道角度。
 */
AimResult Solver::target_pos_to_aim_ypd(const Eigen::Vector3d& xyz) {
    const ShootParam shoot_param = this->target_pos_to_shoot_param(xyz);
    return {
        this->aim_xyz_to_aim_ypd(shoot_param.aim_xyz),
        BallisticDebugData {
            shoot_param.geometric_pitch_rad,
            shoot_param.pitch_angle,
            use_ballistic_compensation_,
            shoot_param.ballistic_success
        }
    };
}

} // namespace qd::auto_aim
