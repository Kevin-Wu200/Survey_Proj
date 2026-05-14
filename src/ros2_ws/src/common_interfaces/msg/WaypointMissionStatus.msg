# 航点任务状态反馈
std_msgs/Header header

# 任务状态: 0=空闲 1=上传中 2=就绪 3=执行中 4=暂停 5=完成 6=失败 7=取消
uint8 state

# 当前航点索引 (从0开始)
uint32 current_waypoint_index

# 总航点数
uint32 total_waypoints

# 任务完成百分比
float32 progress

# 当前航点纬度
float64 current_waypoint_lat

# 当前航点经度
float64 current_waypoint_lon

# 当前航点高度 (米)
float64 current_waypoint_alt

# 已拍照数量
uint32 photos_taken

# 任务ID
string mission_id

# 状态文本描述
string status_text
