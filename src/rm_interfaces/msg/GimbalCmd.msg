# 所有角度使用 rad，角速度使用 rad/s，角加速度使用 rad/s²。
std_msgs/Header header                 # 指令生成时间戳及瞄准角所采用的参考坐标系
string id                              # 当前目标编号：1～5、outpost、sentry、base 或 negative
float64 yaw                            # 发送给云台的最终 yaw 目标角，包含超调/预测补偿
float64 pitch                          # 发送给云台的最终 pitch 目标角，包含超调/弹道补偿，向下为正
float64 exp_yaw                        # 不含控制超调量的期望 yaw 角
float64 exp_pitch                      # 不含控制超调量的期望 pitch 角，向下为正
float64 yaw_diff                       # 当前 yaw 与期望 yaw 的角度误差
float64 pitch_diff                     # 期望 pitch 与当前 pitch 的角度误差，向下为正
float64 distance                       # 云台到预测瞄准点的空间距离，单位 m
bool fire_advice                       # 当前角度误差和弹速等条件是否满足建议开火要求
float64 bullet_speed                   # 解算实际采用的电控弹速，单位 m/s；未收到有效值时为 0
float64 yaw_vel                        # 几何瞄准角的 yaw 角速度，单位 rad/s
float64 pitch_vel                      # 几何瞄准角的 pitch 角速度，单位 rad/s，向下为正
float64 target_v_yaw                   # EKF 估计的目标自身旋转角速度，单位 rad/s

# V2 完整规划参考；完整规划参考有效时为 true。
bool reference_valid                   # 下列完整规划速度、加速度参考是否有效
float32 yaw_vel_rad_s                  # 最终 yaw 轨迹在规划时刻的角速度，单位 rad/s
float32 pitch_vel_rad_s                # 最终 pitch 轨迹在规划时刻的角速度，单位 rad/s，向下为正，含弹道补偿
float32 yaw_acc_rad_s2                 # 最终 yaw 轨迹在规划时刻的角加速度，单位 rad/s²
float32 pitch_acc_rad_s2               # 最终 pitch 轨迹在规划时刻的角加速度，单位 rad/s²，向下为正

string reference_status                # MPC 参考状态：valid 或无效原因；仅用于 ROS 调试，不占串口字节
