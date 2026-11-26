std_msgs/Header header                     # 电控反馈对应的同步后时间戳及云台参考坐标系
uint8 mode                                 # 电控指定的视觉工作模式码，如自瞄颜色或大小符模式
float32 bullet_speed                       # 电控反馈的当前弹速，单位 m/s
float32 roll                               # 云台当前 roll 姿态角，单位 rad
float32 pitch                              # 云台当前 pitch 姿态角，单位 rad，与电控一致，向下为正
float32 yaw                                # 云台当前 yaw 姿态角，单位 rad
uint32 mcu_timestamp                       # MCU 采样时刻的 32 位计时值，供与 PC 时钟同步和回绕展开
JudgeSystemData judge_system_data          # 电控转发的裁判系统与操作手指令数据

float32 yaw_actual_vel_rad_s               # 本帧 yaw 实际姿态角速度，单位 rad/s
float32 pitch_actual_vel_rad_s             # 本帧 pitch 实际姿态角速度，单位 rad/s，向下为正
