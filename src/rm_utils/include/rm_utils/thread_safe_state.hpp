#ifndef RM_UTILS_THREAD_SAFE_STATE_HPP_
#define RM_UTILS_THREAD_SAFE_STATE_HPP_

#include <atomic>
#include <mutex>

namespace qd::utils {

/**
 * @brief 线程安全的状态变量模板类
 * 
 * 使用原子操作和双重检查锁定模式，确保高效的线程安全访问
 * 
 * @tparam T 状态变量类型，必须支持原子操作
 */
template<typename T>
class ThreadSafeState {
public:
    /**
     * @brief 构造函数
     * @param initial_value 初始值
     */
    explicit ThreadSafeState(T initial_value = T {}): value_(initial_value) {}

    /**
     * @brief 设置状态值（带双重检查锁定）
     * @param new_value 新值
     * @return 如果值发生了改变返回 true，否则返回 false
     */
    bool set(T new_value) {
        // 快速检查：如果值相同，直接返回，避免不必要的锁操作
        T current_value = value_.load(std::memory_order_acquire);
        if (current_value == new_value) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        // 双重检查：获取锁后再次检查，防止在等待锁期间值被其他线程修改
        current_value = value_.load(std::memory_order_acquire);
        if (current_value == new_value) {
            return false;
        }

        value_.store(new_value, std::memory_order_release);
        return true;
    }

    /**
     * @brief 获取当前状态值
     * @return 当前值
     */
    T get() const {
        // 使用 memory_order_acquire 确保读取到最新的值
        return value_.load(std::memory_order_acquire);
    }

    /**
     * @brief 比较并交换（CAS操作）
     * @param expected 期望值
     * @param desired 目标值
     * @return 如果交换成功返回 true，否则返回 false
     */
    bool compare_exchange(T& expected, T desired) {
        return value_.compare_exchange_strong(
            expected,
            desired,
            std::memory_order_release,
            std::memory_order_acquire
        );
    }

    /**
     * @brief 原子交换值
     * @param new_value 新值
     * @return 旧值
     */
    T exchange(T new_value) {
        return value_.exchange(new_value, std::memory_order_acq_rel);
    }

    /**
     * @brief 类型转换运算符，允许显式转换为 T 类型
     */
    explicit operator T() const {
        return get();
    }

    /**
     * @brief 赋值运算符
     */
    ThreadSafeState& operator=(T new_value) {
        set(new_value);
        return *this;
    }

    /**
     * @brief 相等比较运算符
     */
    bool operator==(T other) const {
        return get() == other;
    }

    /**
     * @brief 不等比较运算符
     */
    bool operator!=(T other) const {
        return get() != other;
    }

private:
    std::atomic<T> value_; ///< 原子状态变量
    mutable std::mutex mutex_; ///< 用于双重检查锁定的互斥锁
};

} // namespace qd::utils

#endif // RM_UTILS_THREAD_SAFE_STATE_HPP_
