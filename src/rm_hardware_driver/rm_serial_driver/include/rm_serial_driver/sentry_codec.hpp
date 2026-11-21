#ifndef RM_SERIAL_DRIVER_SENTRY_CODEC_HPP_
#define RM_SERIAL_DRIVER_SENTRY_CODEC_HPP_

#include "rm_serial_driver/v2_codec.hpp"

namespace qd::serial_driver::sentry {
using FeedbackFrame = std::array<uint8_t, 36>;

/**
 * @brief 解码哨兵两路实测绝对 yaw，失败时保留原快照。
 * @param frame 大端 36 字节反馈帧。
 * @param data 输出同一采样时刻的状态，小 yaw 同步至旧 yaw 字段供相机 TF 使用。
 * @return 帧头尾、模式和全部浮点量合法时为 true。
 */
template <typename Feedback>
bool decode(const FeedbackFrame& frame, Feedback& data) {
    if (frame.front() != 0xfe || frame.back() != 0x0d || frame[1] > 5) {
        return false;
    }
    constexpr std::array<unsigned, 7> OFFSETS{2, 6, 10, 14, 18, 26, 30};
    std::array<float, 7> values{};
    for (unsigned i = 0; i < values.size(); ++i) {
        values[i] = v2::getFloat(frame.data() + OFFSETS[i]);
        if (!std::isfinite(values[i])) {
            return false;
        }
    }
    auto decoded = data;
    decoded.mode = frame[1];
    decoded.roll = values[0];
    decoded.pitch = values[1];
    decoded.yaw_large = values[2];
    decoded.yaw_small = values[3];
    decoded.yaw = decoded.yaw_small;
    decoded.bullet_speed = values[4];
    decoded.mcu_timestamp = v2::get(frame.data() + 22, 4);
    decoded.yaw_actual_vel_rad_s = values[5];
    decoded.pitch_actual_vel_rad_s = values[6];
    decoded.dual_yaw_feedback = true;
    data = decoded;
    return true;
}
} // namespace qd::serial_driver::sentry

#endif
