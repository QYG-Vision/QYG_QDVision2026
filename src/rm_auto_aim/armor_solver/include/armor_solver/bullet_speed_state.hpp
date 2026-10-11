#ifndef ARMOR_SOLVER_BULLET_SPEED_STATE_HPP_
#define ARMOR_SOLVER_BULLET_SPEED_STATE_HPP_

#include <cmath>

namespace qd::auto_aim {

/** @brief 保存电控最近一次有效弹速，单位为米每秒。 */
class BulletSpeedState {
public:
    /** @brief 判断弹速样本能否用于弹道计算。 */
    static bool is_valid_sample(double raw_speed) {
        return std::isfinite(raw_speed) && raw_speed > 0.0;
    }

    /**
     * @brief 处理一条新的弹速样本。
     * @param raw_speed 弹速样本，单位米每秒；非正值或非有限值会被忽略。
     */
    void update(double raw_speed) {
        if (!is_valid_sample(raw_speed)) {
            return;
        }
        speed_ = raw_speed;
    }

    /** @brief 返回最近有效弹速；尚未收到时返回 0。 */
    double value() const { return speed_; }

private:
    double speed_ = 0.0;
};

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_BULLET_SPEED_STATE_HPP_
