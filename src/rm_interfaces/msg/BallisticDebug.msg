std_msgs/Header header

# 原始几何瞄准角，单位 degree
float64 geometric_pitch
# 弹道解算器输出的完整发射角，单位 degree
float64 compensated_pitch
# compensated_pitch - geometric_pitch，单位 degree
float64 delta_pitch
# 当前帧是否启用发射角弹道补偿
bool ballistic_enabled
# 弹道解算是否成功；关闭或失败时使用几何角
bool ballistic_success
