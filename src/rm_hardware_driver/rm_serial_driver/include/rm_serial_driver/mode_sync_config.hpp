#ifndef RM_SERIAL_DRIVER__MODE_SYNC_CONFIG_HPP_
#define RM_SERIAL_DRIVER__MODE_SYNC_CONFIG_HPP_

#include <string_view>

namespace qd::serial_driver {

/**
 * @brief 判断是否创建指定视觉模式切换服务的客户端。
 * @param enable_mode_sync 是否启用串口反馈到视觉节点的模式同步。
 * @param has_rune 是否启用打符节点。
 * @param service_name 候选模式切换服务的完整名称。
 * @return 需要创建客户端时返回 true。
 */
inline bool should_create_mode_client(
    const bool enable_mode_sync,
    const bool has_rune,
    const std::string_view service_name
) {
    if (!enable_mode_sync) {
        return false;
    }
    return has_rune || service_name.find("/rune_") == std::string_view::npos;
}

} // namespace qd::serial_driver

#endif // RM_SERIAL_DRIVER__MODE_SYNC_CONFIG_HPP_
