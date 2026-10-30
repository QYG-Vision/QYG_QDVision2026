#ifndef RM_SERIAL_DRIVER_V2_CODEC_HPP_
#define RM_SERIAL_DRIVER_V2_CODEC_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace qd::serial_driver::v2 {
using CommandFrame = std::array<uint8_t, 32>;
using FeedbackFrame = std::array<uint8_t, 24>;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

/** @brief 按大端写入整数的低 size 字节。 */
inline void put(uint8_t* out, uint32_t value, unsigned size) {
    for (unsigned i = 0; i < size; ++i) {
        out[size - 1 - i] = static_cast<uint8_t>(value >> (8 * i));
    }
}
/** @brief 读取大端无符号整数。 */
inline uint32_t get(const uint8_t* in, unsigned size) {
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value = (value << 8) | in[i];
    }
    return value;
}
/** @brief 写入 IEEE 754 binary32 的大端位模式。 */
inline void putFloat(uint8_t* out, float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    put(out, bits, 4);
}
/** @brief 读取 IEEE 754 binary32 的大端位模式。 */
inline float getFloat(const uint8_t* in) {
    const uint32_t bits = get(in, 4);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
/** @brief 定点值先限幅再截断，非有限值归零。 */
inline uint16_t fixed(double value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    const auto scaled = std::clamp(value * 100.0, -32768.0, 32767.0);
    return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}
/** @brief 读取有符号大端定点值，避免依赖主机端序。 */
inline float readFixed(const uint8_t* in) {
    const auto bits = get(in, 2);
    const int32_t value = bits >= 0x8000 ? static_cast<int32_t>(bits) - 65536
                                       : static_cast<int32_t>(bits);
    return value / 100.0f;
}

/**
 * @brief 编码完整规划参考；没有完整参考时只编码无目标帧。
 * @param data 含绝对角度（degree）和同一时刻规划导数的消息。
 * @param id 目标编号，0～9。
 * @return 大端 32 字节 V2 命令。
 */
template <typename Command>
CommandFrame encode(const Command& data, uint8_t id) {
    CommandFrame out{};
    out[0] = 0xfe;
    out[31] = 0x0d;
    put(out.data() + 6, fixed(-1.0), 2);
    const std::array<double, 4> derivatives{data.yaw_vel_deg_s, data.pitch_vel_deg_s,
                                         data.yaw_acc_deg_s2, data.pitch_acc_deg_s2};
    const bool valid = data.reference_valid && id <= 9 && data.distance > 0.0
        && std::isfinite(data.distance) && fixed(data.distance) > 0
        && std::isfinite(data.yaw)
        && std::isfinite(data.pitch) && std::isfinite(data.target_v_yaw)
        && std::all_of(derivatives.begin(), derivatives.end(), [](double value) {
            return std::isfinite(value)
                && std::abs(value) <= std::numeric_limits<float>::max();
        });
    if (!valid) {
        return out;
    }
    out[1] = data.fire_advice ? 1 : 0;
    put(out.data() + 2, fixed(data.yaw), 2);
    put(out.data() + 4, fixed(data.pitch), 2);
    put(out.data() + 6, fixed(data.distance), 2);
    out[8] = id;
    put(out.data() + 9, fixed(data.target_v_yaw), 2);
    out[11] = 1;
    for (unsigned i = 0; i < derivatives.size(); ++i) {
        putFloat(out.data() + 14 + 4 * i, static_cast<float>(derivatives[i]));
    }
    return out;
}

/**
 * @brief 校验并解码 V2 反馈，失败时不修改输出消息。
 * @param frame 完整的 24 字节状态帧。
 * @param data 输出原协议正方向的角度和实际角速度，单位 degree、degree/s。
 * @return 帧头尾、模式及实际角速度均合法时为 true。
 */
template <typename Feedback>
bool decode(const FeedbackFrame& frame, Feedback& data) {
    if (frame[0] != 0xfe || frame[23] != 0x0d || frame[1] > 5) {
        return false;
    }
    const float yaw_vel = getFloat(frame.data() + 14);
    const float pitch_vel = getFloat(frame.data() + 18);
    if (!std::isfinite(yaw_vel) || !std::isfinite(pitch_vel)) {
        return false;
    }
    auto decoded = data;
    decoded.mode = frame[1];
    decoded.roll = readFixed(frame.data() + 2);
    decoded.pitch = readFixed(frame.data() + 4);
    decoded.yaw = readFixed(frame.data() + 6);
    decoded.bullet_speed = readFixed(frame.data() + 8);
    decoded.mcu_timestamp = get(frame.data() + 10, 4);
    decoded.yaw_actual_vel_deg_s = yaw_vel;
    decoded.pitch_actual_vel_deg_s = pitch_vel;
    data = decoded;
    return true;
}
} // namespace qd::serial_driver::v2
#endif
