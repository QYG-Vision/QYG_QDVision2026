std_msgs/Header header
uint8 mode
float32 bullet_speed
float32 roll # rad
float32 pitch # 单位 rad，与电控一致，向下为正
float32 yaw # rad
uint32 mcu_timestamp
JudgeSystemData judge_system_data

float32 yaw_actual_vel_rad_s # 本帧 yaw 的实际姿态角速度，rad/s
float32 pitch_actual_vel_rad_s # 本帧 pitch 的实际姿态角速度，rad/s，向下为正
