# 心跳消息
# 用于节点存活检测

std_msgs/Header header           # 时间戳
string  node_name                # 节点名称
string  node_type                # 节点类型: uav/ugv/ground_station
float64 uptime_seconds           # 运行时长 (秒)
bool    emergency_stop           # 紧急停止标志
