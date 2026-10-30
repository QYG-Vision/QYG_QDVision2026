std_msgs/Header header
uint8 mode
float32 bullet_speed
float32 roll 
float32 pitch # 单位 degree，与电控一致，向下为正
float32 yaw
uint32 mcu_timestamp
JudgeSystemData judge_system_data

float32 yaw_actual_vel_deg_s # 本帧 yaw 的实际姿态角速度，degree/s
float32 pitch_actual_vel_deg_s # 本帧 pitch 的实际姿态角速度，degree/s，向下为正
