#ifndef RM_SERIAL_DRIVER_FLOAT_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_FLOAT_PROTOCOL_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace qd::serial_driver::wire {

using Frame = std::array<uint8_t, 24>;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

/**
 * @brief 按大端编码无符号整数的低 size 字节。
 * @param out 至少 size 字节的输出缓冲区。
 * @param value 待编码的无符号值。
 * @param size 字节数，范围 1～4。
 */
inline void put_integer(uint8_t* out, uint32_t value, unsigned size) {
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
inline uint32_t get_integer(const uint8_t* in, unsigned size) {
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value = (value << 8) | in[i];
    }
    return value;
}

/**
 * @brief 按大端写入 IEEE 754 binary32 位模式，不进行定点缩放。
 * @param out 至少 4 字节的输出缓冲区。
 * @param value 已使用约定物理单位的单精度数值。
 */
inline void put_float(uint8_t* out, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    put_integer(out, bits, 4);
}

/**
 * @brief 从大端 IEEE 754 binary32 位模式读取浮点值。
 * @param in 至少 4 字节的输入缓冲区。
 * @return 解码后的直接物理值，可能包含 NaN 或 Inf，由调用方校验。
 */
inline float get_float(const uint8_t* in) {
    const uint32_t bits = get_integer(in, 4);
    float value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
 * @brief 转换为线路 float32；非有限值或不可表示的输入归零。
 * @param value 已使用约定物理单位的双精度数值。
 * @return 转为 float32 的值；非有限值或超出 float32 范围时为 0。
 */
inline float wire_float(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
        return 0.0f;
    }
    return static_cast<float>(value);
}

/**
 * @brief 创建 24 字节帧，载荷与预留位初始化为零。
 * @return 帧头为 0xFF、帧尾为 0x0D 的空帧。
 */
inline Frame make_frame() {
    Frame frame{};
    frame.front() = 0xff;
    frame.back() = 0x0d;
    return frame;
}

/**
 * @brief 将完整云台指令编码为一个 24 字节帧。
 * @param data 云台绝对角（rad）、距离（m）、目标自转速度（rad/s）及开火建议。
 * @param id 目标编号，0～9。
 * @return 大端 float32 指令帧，无目标时保留原有独立字段编码规则。
 */
template <typename Command>
Frame encode_command(const Command& data, uint8_t id) {
    auto frame = make_frame();
    frame[1] = data.fire_advice ? 1 : 0;
    put_float(frame.data() + 2, wire_float(data.yaw));
    put_float(frame.data() + 6, wire_float(data.pitch));
    put_float(frame.data() + 10, wire_float(data.distance));
    frame[14] = id;
    put_float(frame.data() + 15, wire_float(data.target_v_yaw));
    frame[19] = std::isfinite(data.distance) && data.distance > 0.0 ? 1 : 0;
    return frame;
}

struct Feedback {
    float roll = 0.0f; // rad
    float pitch = 0.0f; // rad，向下为正
    float yaw = 0.0f; // rad，向左为正
    float bullet_speed = 0.0f; // m/s
    uint32_t mcu_timestamp = 0; // ms，同一姿态快照的采样时间
    uint8_t mode = 0;
};

/**
 * @brief 将同一次电控采样编码为一个 24 字节帧，供电控对接与字节回归核对。
 * @param data 完整姿态（rad）、弹速（m/s）、采样时间（ms）和模式。
 * @return 大端 float32 姿态、大端 uint32 时间戳及整数模式组成的完整帧。
 */
inline Frame encode_feedback(const Feedback& data) {
    auto frame = make_frame();
    frame[1] = data.mode;
    put_float(frame.data() + 2, data.roll);
    put_float(frame.data() + 6, data.pitch);
    put_float(frame.data() + 10, data.yaw);
    put_float(frame.data() + 14, data.bullet_speed);
    put_integer(frame.data() + 18, data.mcu_timestamp, 4);
    return frame;
}

/**
 * @brief 校验并解码完整电控反馈，非法帧不修改输出。
 * @param frame 一个完整的 24 字节帧，预留校验位尚未启用。
 * @param output 合法时写入的姿态（rad）、弹速（m/s）、时间戳（ms）及模式。
 * @return 帧头尾正确、模式在 0～5 且所有浮点值有限时为 true。
 */
inline bool decode_feedback(const Frame& frame, Feedback& output) {
    if (frame.front() != 0xff || frame.back() != 0x0d || frame[1] > 5) {
        return false;
    }
    Feedback decoded;
    decoded.mode = frame[1];
    decoded.roll = get_float(frame.data() + 2);
    decoded.pitch = get_float(frame.data() + 6);
    decoded.yaw = get_float(frame.data() + 10);
    decoded.bullet_speed = get_float(frame.data() + 14);
    decoded.mcu_timestamp = get_integer(frame.data() + 18, 4);
    if (!std::isfinite(decoded.roll) || !std::isfinite(decoded.pitch)
        || !std::isfinite(decoded.yaw) || !std::isfinite(decoded.bullet_speed))
    {
        return false;
    }
    output = decoded;
    return true;
}

} // namespace qd::serial_driver::wire
#endif // RM_SERIAL_DRIVER_FLOAT_PROTOCOL_HPP_
