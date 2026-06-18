# 系统告警消息
# 用于异常通知

std_msgs/Header header           # 时间戳
string  source                   # 告警来源 (uav/ugv/ground_station)
uint8   level                    # 告警级别: 0=info 1=warning 2=error 3=critical
string  message                  # 告警内容
