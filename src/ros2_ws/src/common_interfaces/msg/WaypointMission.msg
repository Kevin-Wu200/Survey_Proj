# 航点任务消息 - 用于 UAV 航点任务指令下发
std_msgs/Header header

# 任务动作: 0=上传航点 1=启动任务 2=暂停 3=恢复 4=停止 5=查询状态
uint8 action

# 航点数据 (JSON 字符串，格式: [{"lat": 30.0, "lon": 120.0, "alt": 50.0, "speed": 8.0, "heading": 0.0, "action": "photo"}, ...])
string waypoints_json

# 任务ID
string mission_id

# 航线类型: 0=自定义 1=多边形 2=蛇形
uint8 route_type

# 拍照触发模式: 0=等距触发 1=等时触发 2=手动
uint8 camera_trigger_mode

# 拍照间隔 (等距模式: 米; 等时模式: 秒)
float32 camera_trigger_interval

# 任务参数 (JSON 字符串，多边形/蛇形航线用)
string mission_params_json
