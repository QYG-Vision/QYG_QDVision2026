std_msgs/Header header                 # 本次弹道解算对应的时间戳及坐标系
float64 geometric_pitch               # 未补偿弹道下坠的几何 pitch 瞄准角，单位 rad
float64 compensated_pitch             # 弹道解算器输出的完整 pitch 发射角，单位 rad
float64 delta_pitch                   # 弹道补偿量：compensated_pitch - geometric_pitch，单位 rad
bool ballistic_enabled                # 当前帧是否配置为启用发射角弹道补偿
bool ballistic_success                # 本次弹道解算是否成功；关闭或失败时使用几何角
