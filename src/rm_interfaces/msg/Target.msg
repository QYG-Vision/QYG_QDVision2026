std_msgs/Header header                      # 目标状态时间戳及状态量所在坐标系，通常为 odom
bool tracking                              # 跟踪器当前是否已稳定跟踪目标
string id                                  # 被跟踪机器人的装甲板编号/类别
int32 armors_num                           # 目标机器人一圈的装甲板数量
geometry_msgs/Point position               # EKF 估计的目标旋转中心位置，单位 m
geometry_msgs/Vector3 velocity             # EKF 估计的目标旋转中心线速度，单位 m/s
geometry_msgs/Point position_armor         # 独立装甲板模型估计的当前装甲板位置，单位 m
geometry_msgs/Vector3 velocity_armor       # 独立装甲板模型估计的当前装甲板线速度，单位 m/s
float64 yaw                                # 目标旋转中心到当前装甲板的方位角，单位 rad
float64 v_yaw                              # 目标自身旋转角速度，单位 rad/s
int32 top_level                            # 旧版旋转分级兼容字段；当前固定为 0，不参与控制
float64[] dz_list                          # 各装甲板相对目标中心高度的偏移列表，单位 m，与装甲板序号对应
float64[] radius_list                      # 各装甲板到目标旋转中心的半径列表，单位 m，与装甲板序号对应
float64 nis                                # EKF 新息平方统计量（NIS），无量纲，用于判断观测与预测的一致性


