#ifndef RM_SERIAL_DRIVER__GIMBAL_TARGET_CLI_HPP_
#define RM_SERIAL_DRIVER__GIMBAL_TARGET_CLI_HPP_

#include <optional>
#include <string>
#include <vector>

namespace qd::serial_driver {

struct GimbalTargetDegrees {
    double yaw;
    double pitch;
};

/**
 * @brief 解析命令行中的绝对 yaw/pitch 目标。
 * @param arguments 两个按顺序给出的角度参数：yaw、pitch，单位为度。
 * @return 参数完整且为有限数时返回目标，否则返回空值。
 */
std::optional<GimbalTargetDegrees>
parseGimbalTargetArguments(const std::vector<std::string>& arguments) noexcept;

} // namespace qd::serial_driver

#endif // RM_SERIAL_DRIVER__GIMBAL_TARGET_CLI_HPP_
