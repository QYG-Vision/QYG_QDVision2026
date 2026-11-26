int32 center_x                       # 候选装甲板中心的图像横坐标，单位 px
string type                         # 分类结果：small、large 或 invalid
float32 light_ratio                 # 左右灯条长度比（短边/长边），无量纲，越接近 1 越对称
float32 center_distance             # 两灯条中心距与平均灯条长度之比，无量纲
float32 angle                       # 两灯条中心连线相对图像水平线的倾角绝对值，单位 rad
