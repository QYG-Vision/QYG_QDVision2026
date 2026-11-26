string number                         # 装甲板编号：1、2、3、4、5、outpost、sentry、base 或 negative
string type                           # 装甲板尺寸类型：small（小装甲）或 large（大装甲）
float32 distance_to_image_center      # 装甲板中心到图像中心的像素距离，单位 px，用于选择优先跟踪目标
float32 area                          # 四个装甲板角点围成的图像面积，单位 px²
geometry_msgs/Pose pose               # PnP 解算出的装甲板位姿；所属坐标系由 Armors.header.frame_id 指定，位置单位 m
Point2d[4] image_points               # 装甲板四个 PnP 角点的图像像素坐标，顺序与 PnP 模型点一一对应
