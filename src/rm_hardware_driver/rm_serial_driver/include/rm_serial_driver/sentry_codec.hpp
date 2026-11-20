#ifndef RM_SERIAL_DRIVER_SENTRY_CODEC_HPP_
#define RM_SERIAL_DRIVER_SENTRY_CODEC_HPP_

#include "rm_serial_driver/v2_codec.hpp"

namespace qd::serial_driver::sentry {
using CommandFrame = std::array<uint8_t, 52>;
using FeedbackFrame = std::array<uint8_t, 36>;

/**
 * @brief 编码哨兵两路 yaw 和共用 pitch 的同一时刻参考。
 * @param data 三轴角度及导数，单位 rad、rad/s、rad/s²。
 * @param id 目标编号，范围 0～9。
 * @return 大端 52 字节帧，无效参考的距离为 −1 m。
 */
template <typename Command>
CommandFrame encode(const Command& data, uint8_t id) {
    CommandFrame out{};
    out.front() = 0xfe;
    out.back() = 0x0d;
    v2::putFloat(out.data() + 14, -1.0f);
    const std::array<double, 11> values{
        data.yaw_large, data.yaw_small, data.pitch, data.distance, data.target_v_yaw,
        data.yaw_large_vel_rad_s, data.yaw_small_vel_rad_s, data.pitch_vel_rad_s,
        data.yaw_large_acc_rad_s2, data.yaw_small_acc_rad_s2, data.pitch_acc_rad_s2
    };
    const bool valid = data.reference_valid && data.dual_yaw_reference
        && id <= 9 && data.distance > 0.0
        && std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value)
                && std::abs(value) <= std::numeric_limits<float>::max();
        });
    if (!valid || static_cast<float>(data.distance) <= 0.0f) {
        return out;
    }
    out[1] = data.fire_advice ? 1 : 0;
    out[18] = id;
    out[23] = 1;
    constexpr std::array<unsigned, 11> OFFSETS{2, 6, 10, 14, 19, 26, 30, 34, 38, 42, 46};
    for (unsigned i = 0; i < values.size(); ++i) {
        v2::putFloat(out.data() + OFFSETS[i], static_cast<float>(values[i]));
    }
    return out;
}

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
