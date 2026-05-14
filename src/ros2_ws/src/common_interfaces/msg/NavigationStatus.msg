# UGV 导航状态反馈
std_msgs/Header header

# 导航状态: 0=空闲 1=规划中 2=执行中 3=到达目标 4=失败 5=取消
uint8 state

# 到目标点的剩余距离 (米)
float32 distance_remaining

# 到目标点的偏航角误差 (弧度)
float32 yaw_error

# 路径长度 (米)
float32 path_length

# 当前路径点索引
uint32 current_path_index

# 总路径点数
uint32 total_path_points

# 状态文本描述
string status_text
