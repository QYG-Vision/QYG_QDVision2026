#ifndef RM_SERIAL_DRIVER__PROTOCOL__QYG_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL__QYG_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace qd::serial_driver::protocol::qyg
{

#pragma pack(push, 1)
/** @brief QYG 电控回传的固定长度串口帧。 */
struct QygReceiveFrame
{
  uint8_t header[2]{'G', 'D'};
  uint8_t current_mode{0};
  float actual_vx{0.0F};
  float actual_vy{0.0F};
  float actual_wz{0.0F};
  uint16_t sentry_state{0};
  float vyaw{0.0F};
  float vpitch{0.0F};
  float vroll{0.0F};
  float bullet_speed{0.0F};      // 实时弹速，单位为米每秒
  uint32_t mcu_timestamp{0};     // 云台角采样时刻，单位为毫秒
  uint16_t crc16{0};
};

/** @brief QYG 视觉指令下发的固定长度串口帧。 */
struct QygSendFrame
{
  uint8_t header[2]{'Q', 'Y'};
  uint8_t mode{0};
  float yaw{0.0F};
  float pitch{0.0F};
  float linear_x{0.0F};
  float linear_y{0.0F};
  float angular_z{0.0F};
  float distance{0.0F};          // 目标距离，单位为米；负值表示无有效目标
  uint8_t target_id{0};          // 目标装甲板类型编号
  float target_v_yaw{0.0F};      // 目标自转角速度，单位为弧度每秒
  uint16_t crc16{0};
};
#pragma pack(pop)

/** @brief QYG 回传角转换到视觉内部约定后的云台角度，单位为度。 */
struct GimbalFeedbackAngles {
    float roll_degrees { 0.0F };
    float pitch_degrees { 0.0F };
    float yaw_degrees { 0.0F };
};

static_assert(sizeof(float) == 4, "QYG protocol requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559, "QYG protocol requires IEEE-754 float");
static_assert(sizeof(QygReceiveFrame) == 39, "QYG receive frame must be 39 bytes");
static_assert(offsetof(QygReceiveFrame, current_mode) == 2, "Invalid current_mode offset");
static_assert(offsetof(QygReceiveFrame, actual_vx) == 3, "Invalid actual_vx offset");
static_assert(offsetof(QygReceiveFrame, actual_vy) == 7, "Invalid actual_vy offset");
static_assert(offsetof(QygReceiveFrame, actual_wz) == 11, "Invalid actual_wz offset");
static_assert(offsetof(QygReceiveFrame, sentry_state) == 15, "Invalid sentry_state offset");
static_assert(offsetof(QygReceiveFrame, vyaw) == 17, "Invalid vyaw offset");
static_assert(offsetof(QygReceiveFrame, vpitch) == 21, "Invalid vpitch offset");
static_assert(offsetof(QygReceiveFrame, vroll) == 25, "Invalid vroll offset");
static_assert(offsetof(QygReceiveFrame, bullet_speed) == 29, "Invalid bullet_speed offset");
static_assert(offsetof(QygReceiveFrame, mcu_timestamp) == 33, "Invalid mcu_timestamp offset");
static_assert(offsetof(QygReceiveFrame, crc16) == 37, "Invalid receive CRC offset");
static_assert(sizeof(QygSendFrame) == 34, "QYG send frame must be 34 bytes");
static_assert(offsetof(QygSendFrame, mode) == 2, "Invalid mode offset");
static_assert(offsetof(QygSendFrame, yaw) == 3, "Invalid yaw offset");
static_assert(offsetof(QygSendFrame, pitch) == 7, "Invalid pitch offset");
static_assert(offsetof(QygSendFrame, linear_x) == 11, "Invalid linear_x offset");
static_assert(offsetof(QygSendFrame, linear_y) == 15, "Invalid linear_y offset");
static_assert(offsetof(QygSendFrame, angular_z) == 19, "Invalid angular_z offset");
static_assert(offsetof(QygSendFrame, distance) == 23, "Invalid distance offset");
static_assert(offsetof(QygSendFrame, target_id) == 27, "Invalid target_id offset");
static_assert(offsetof(QygSendFrame, target_v_yaw) == 28, "Invalid target_v_yaw offset");
static_assert(offsetof(QygSendFrame, crc16) == 32, "Invalid send CRC offset");

/**
 * @brief 计算 QYG 使用的 CRC-16/DECT 校验值。
 * @param data 待校验的字节序列。
 * @param length 待校验字节数。
 * @return CRC-16 校验值。
 */
uint16_t crc16(const uint8_t * data, size_t length);

/**
 * @brief 校验并解码与 QD `VisionMode` 直接对位的模式编号。
 * @param current_mode QYG 回传的当前视觉模式，合法范围为 0～5。
 * @return 合法时返回原模式编号，否则返回空值。
 */
std::optional<uint8_t> decodeQdVisionMode(uint8_t current_mode);

/**
 * @brief 将角度从度转换为弧度。
 * @param degrees 角度值，单位为度。
 * @return 弧度值。
 */
float degreesToRadians(float degrees);

/**
 * @brief 将 QYG 回传云台角映射为视觉内部使用的角度。
 * @param frame QYG 电控回传帧，其中 pitch 以抬头为正。
 * @return 单位为度且 pitch 以抬头为正的云台角度。
 */
GimbalFeedbackAngles decodeGimbalFeedback(const QygReceiveFrame& frame);

/**
 * @brief 构造并校验 QYG 下发帧。
 * @param control 是否允许云台控制。
 * @param fire 是否允许开火，仅在 `control` 为 true 时生效。
 * @param yaw 云台 yaw，单位为弧度。
 * @param pitch 云台 pitch，单位为弧度。
 * @param linear_x 底盘 x 速度，限幅到 [-1, 1]。
 * @param linear_y 底盘 y 速度，限幅到 [-1, 1]。
 * @param angular_z 底盘角速度，限幅到 [-1, 1]。
 * @param distance 目标距离，单位为米；负值表示当前无有效目标。
 * @param target_id 目标装甲板类型编号。
 * @param target_v_yaw 目标自转角速度，单位为弧度每秒。
 * @return 带有 CRC 的 QYG 下发帧。
 */
QygSendFrame makeSendFrame(
  bool control, bool fire, float yaw, float pitch, float linear_x, float linear_y, float angular_z,
  float distance, uint8_t target_id, float target_v_yaw);

/**
 * @brief 校验并解析一帧 QYG 回传数据。
 * @param data 串口数据缓冲区。
 * @param length 缓冲区长度。
 * @return 校验成功时返回回传帧，否则返回空值。
 */
std::optional<QygReceiveFrame> parseReceiveFrame(const uint8_t * data, size_t length);

/** @brief 从可能包含噪声和分片数据的字节流中提取 QYG 回传帧。 */
class QygStreamParser
{
public:
  /** @brief 追加一段串口数据到解析缓存。 */
  void append(const uint8_t * data, size_t length);
  /** @brief 弹出下一帧完整且 CRC 正确的回传帧。 */
  std::optional<QygReceiveFrame> popFrame();
  /** @brief 返回当前缓存中的字节数。 */
  size_t bufferedSize() const;

private:
  std::vector<uint8_t> buffer_;
};

}  // namespace qd::serial_driver::protocol::qyg

#endif  // RM_SERIAL_DRIVER__PROTOCOL__QYG_PROTOCOL_HPP_
