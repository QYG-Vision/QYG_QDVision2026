std_msgs/Header header                 # 底盘控制指令的时间戳及参考坐标系
bool is_spining                        # 底盘是否处于小陀螺旋转状态（字段名保留了历史拼写）
bool is_navigating                     # 底盘是否处于自主导航状态
geometry_msgs/Twist twist              # 底盘期望线速度和角速度；具体方向由 header.frame_id 指定
