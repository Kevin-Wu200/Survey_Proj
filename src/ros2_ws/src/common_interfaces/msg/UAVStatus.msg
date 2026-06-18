# UAV 状态消息
# 无人机实时状态，由 UAV 仿真节点发布

std_msgs/Header header           # 时间戳和坐标系
float64 latitude                 # 纬度 (度, WGS84)
float64 longitude                # 经度 (度, WGS84)
float64 altitude                 # 海拔高度 (米)
float64 roll                     # 横滚角 (弧度)
float64 pitch                    # 俯仰角 (弧度)
float64 yaw                      # 偏航角 (弧度)
float64 ground_speed             # 地速 (m/s)
float64 battery_voltage          # 电池电压 (V)
float64 battery_percentage       # 电池电量百分比
uint8   flight_mode              # 飞行模式: 0=待机 1=起飞 2=悬停 3=航线 4=降落 5=返航
bool    armed                    # 解锁状态
bool    camera_trigger           # 相机触发标志
string  status_text              # 状态文本描述
