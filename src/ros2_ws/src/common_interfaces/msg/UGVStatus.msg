# UGV 状态消息
# 无人车实时状态，由 UGV 仿真节点发布

std_msgs/Header header           # 时间戳和坐标系
float64 latitude                 # 纬度 (度, WGS84)
float64 longitude                # 经度 (度, WGS84)
float64 altitude                 # 海拔高度 (米)
float64 heading                  # 航向角 (弧度)
float64 linear_velocity          # 线速度 (m/s)
float64 angular_velocity         # 角速度 (rad/s)
float64 battery_voltage          # 电池电压 (V)
float64 battery_percentage       # 电池电量百分比
bool    remote_control           # 遥控模式激活
string  status_text              # 状态文本描述
