#ifndef RM_SERIAL_DRIVER__ZERO_ORDER_GIMBAL_CONTROLLER_HPP_
#define RM_SERIAL_DRIVER__ZERO_ORDER_GIMBAL_CONTROLLER_HPP_

#include <optional>
#include <string>

#include "rm_interfaces/msg/gimbal_cmd.hpp"

namespace qd::serial_driver {

/**
 * @brief 保持最新绝对云台目标，并生成禁止开火的零阶控制指令。
 */
class ZeroOrderGimbalController {
public:
    /**
     * @brief 构造零阶云台命令保持器。
     * @param distance 发送给电控的有效目标距离，非负值使 QYG 协议进入云台控制模式。
     * @param id 发送给电控的测试目标编号。
     */
    ZeroOrderGimbalController(double distance, std::string id);

    /**
     * @brief 使用第一帧云台反馈初始化保持目标。
     * @param yaw 当前云台 yaw 绝对角度，单位为度。
     * @param pitch 当前云台 pitch 绝对角度，单位为度。
     * @note 后续反馈仅供节点观测，不会覆盖已建立的保持目标。
     */
    void updateFeedback(double yaw, double pitch) noexcept;

    /**
     * @brief 设置新的零阶绝对角度目标。
     * @param yaw 世界坐标系下的目标 yaw，单位为度。
     * @param pitch 世界坐标系下的目标 pitch，单位为度。
     */
    void updateTarget(double yaw, double pitch) noexcept;

    /**
     * @brief 获取当前应重复发送的云台命令。
     * @return 已获得首帧反馈时返回固定 yaw/pitch 命令，否则返回空值。
     * @note 返回的命令始终将 `fire_advice` 设置为 false。
     */
    std::optional<rm_interfaces::msg::GimbalCmd> makeCommand() const;

private:
    double distance_;
    std::string id_;
    bool has_feedback_ { false };
    bool has_explicit_target_ { false };
    double target_yaw_ { 0.0 };
    double target_pitch_ { 0.0 };
};

} // namespace qd::serial_driver

#endif // RM_SERIAL_DRIVER__ZERO_ORDER_GIMBAL_CONTROLLER_HPP_
