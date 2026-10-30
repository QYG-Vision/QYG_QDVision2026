std_msgs/Header header
string id # 1 2 3 4 5 outpost sentry base negative
float64 yaw         # 发送给云台要达到的 yaw 角度，含超调量
float64 pitch       # 发送给云台要达到的 pitch 角度，含超调量，向下为正
float64 exp_yaw     # 实际期望到达的 yaw 角度
float64 exp_pitch   # 实际期望到达的 pitch 角度，向下为正
float64 yaw_diff    # 云台当前 yaw 与期望 yaw 的差值
float64 pitch_diff  # 期望 pitch 与当前 pitch 的差值，向下为正
float64 distance    # 目标距离
bool fire_advice    # 是否建议开火
float64 bullet_speed    # 实际使用的电控弹速，单位 m/s；未收到有效值时为 0
float64 yaw_vel     # 超调量的 yaw 角速度
float64 pitch_vel   # 超调量的 pitch 角速度，向下为正
float64 target_v_yaw # EKF 状态量 v_yaw，目标自转角速度，单位弧度每秒

# V2 完整规划参考；只有 MPC 求解成功且反馈新鲜时为 true。
bool reference_valid
float32 yaw_vel_deg_s # degree/s，与最终 yaw 同方向、同一规划时刻
float32 pitch_vel_deg_s # degree/s，向下为正，包含弹道补偿
float32 yaw_acc_deg_s2 # degree/s²
float32 pitch_acc_deg_s2 # degree/s²，向下为正

string reference_status # MPC 状态：valid 或无效原因，仅 ROS 调试，不占串口字节
