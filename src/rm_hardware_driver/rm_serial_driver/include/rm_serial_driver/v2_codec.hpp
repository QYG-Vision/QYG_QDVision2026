#ifndef RM_SERIAL_DRIVER_V2_CODEC_HPP_
#define RM_SERIAL_DRIVER_V2_CODEC_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace qd::serial_driver::v2 {
using CommandFrame = std::array<uint8_t, 40>;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

/**
 * @brief 按大端写入整数的低 size 字节。
 * @param out 至少 size 字节的输出缓冲区。
 * @param value 待编码的无符号值。
 * @param size 字节数，范围 1～4。
 */
inline void put(uint8_t* out, uint32_t value, unsigned size) {
    for (unsigned i = 0; i < size; ++i) {
        out[size - 1 - i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

/**
 * @brief 读取大端无符号整数。
 * @param in 至少 size 字节的输入缓冲区。
 * @param size 字节数，范围 1～4。
 * @return 解码后的无符号整数。
 */
inline uint32_t get(const uint8_t* in, unsigned size) {
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value = (value << 8) | in[i];
    }
    return value;
}

/**
 * @brief 写入 IEEE 754 binary32 的大端位模式，不进行定点缩放。
 * @param out 至少 4 字节的输出缓冲区。
 * @param value 已使用约定物理单位的单精度数值。
 */
inline void putFloat(uint8_t* out, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    put(out, bits, 4);
}

/**
 * @brief 读取 IEEE 754 binary32 的大端位模式。
 * @param in 至少 4 字节的输入缓冲区。
 * @return 解码后的直接物理值，可能为 NaN 或 Inf，由调用方校验。
 */
inline float getFloat(const uint8_t* in) {
    const uint32_t bits = get(in, 4);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
 * @brief 编码完整规划参考；没有完整参考或输入非法时编码无目标帧。
 * @param data 同一规划时刻的绝对角（rad）、导数（rad/s、rad/s²）、距离（m）。
 * @param id 目标编号，0～9。
 * @return 大端 40 字节命令，物理量均为直接 float32 值；无效时距离为 −1 m。
 */
template <typename Command>
CommandFrame encode(const Command& data, uint8_t id) {
    CommandFrame out{};
    out.front() = 0xfe;
    out.back() = 0x0d;
    putFloat(out.data() + 10, -1.0f);
    const std::array<double, 8> values{
        data.yaw, data.pitch, data.distance, data.target_v_yaw,
        data.yaw_vel_rad_s, data.pitch_vel_rad_s, data.yaw_acc_rad_s2, data.pitch_acc_rad_s2
    };
    const bool valid = data.reference_valid && id <= 9 && data.distance > 0.0
        && std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value)
                && std::abs(value) <= std::numeric_limits<float>::max();
        });
    if (!valid || static_cast<float>(data.distance) <= 0.0f) {
        return out;
    }
    out[1] = data.fire_advice ? 1 : 0;
    out[14] = id;
    out[19] = 1;
    constexpr std::array<unsigned, 8> OFFSETS{2, 6, 10, 15, 22, 26, 30, 34};
    for (unsigned i = 0; i < values.size(); ++i) {
        putFloat(out.data() + OFFSETS[i], static_cast<float>(values[i]));
    }
    return out;
}

} // namespace qd::serial_driver::v2
#endif
