# header 是本轮规划时刻，target_header 是输入 EKF 状态的图像时刻。
std_msgs/Header header
std_msgs/Header target_header
bool tracking
string target_id
float64 target_yaw
float64 target_v_yaw
bool reference_valid
string status
float64 dt
int32 output_index
# 编号对应输入 Target 的半径/高度数组下标，不是识别数字或当前可见板编号。
int32 selected_armor_index -1
float64 reference_yaw
float64 reference_pitch
# 时间相对命中预测状态，窗口中点为 0；编号在求解无效时仍保留。
float64[] time_offsets
int32[] armor_indices
float64[] yaw_reference
float64[] pitch_reference
MpcAxisDebug yaw
MpcAxisDebug pitch
