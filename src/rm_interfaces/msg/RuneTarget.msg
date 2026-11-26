std_msgs/Header header                  # 能量机关检测帧的时间戳及相机坐标系
Point2d[5] pts                          # 图像特征点：R 标中心、左下、左上、右上、右下，单位 px
bool is_lost                           # 本帧是否未找到可击打的未激活扇叶
bool is_big_rune                       # 当前是否为大能量机关模式；false 表示小能量机关
