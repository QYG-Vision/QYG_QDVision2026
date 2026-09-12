std_msgs/Header header
uint8 mode
float32 bullet_speed
float32 roll 
float32 pitch
float32 yaw
uint32 mcu_timestamp
JudgeSystemData judge_system_data

# QYG 哨兵专用回传字段；其他协议保持默认值 0
uint8 current_mode
float32 actual_vx
float32 actual_vy
float32 actual_wz
float32 imu_yaw
float32 imu_pitch
float32 yaw_angular
float32 pitch_angular
float32 odom_x
uint16 sentry_state
