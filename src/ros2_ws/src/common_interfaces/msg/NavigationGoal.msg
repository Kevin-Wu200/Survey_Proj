# UGV 导航目标点消息
std_msgs/Header header

# 目标纬度
float64 target_lat

# 目标经度
float64 target_lon

# 目标偏航角 (弧度)
float64 target_yaw

# 目标位姿 (map坐标系)
geometry_msgs/PoseStamped target_pose

# 速度限制 (0=使用默认值)
float32 max_linear_speed
float32 max_angular_speed
