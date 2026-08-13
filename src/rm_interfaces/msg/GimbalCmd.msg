std_msgs/Header header
string id # 1 2 3 4 5 outpost sentry base negative
float64 yaw         # 发送给云台要达到的 yaw 角度，含超调量
float64 pitch       # 发送给云台要达到的 pitch 角度，含超调量
float64 exp_yaw     # 实际期望到达的 yaw 角度
float64 exp_pitch   # 实际期望到达的 pitch 角度
float64 yaw_diff    # 云台当前 yaw 与期望 yaw 的差值
float64 pitch_diff  # 云台当前 pitch 与期望 pitch 的差值
float64 distance    # 目标距离
bool fire_advice    # 是否建议开火
float64 bullet_speed    # 子弹速度，滤波后实际使用的弹速
float64 yaw_vel     # 超调量的 yaw 角速度
float64 pitch_vel   # 超调量的 pitch 角速度
float64 target_v_yaw # EKF 状态量 v_yaw，目标自转角速度，单位弧度每秒