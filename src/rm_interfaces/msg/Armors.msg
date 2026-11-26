std_msgs/Header header                 # 本帧时间戳及装甲板位姿所在坐标系
sensor_msgs/Image image                # 与检测结果对应的原始图像；未随消息传输时可为空
Armor[] armors                         # 本帧检测并成功完成 PnP 解算的装甲板列表
