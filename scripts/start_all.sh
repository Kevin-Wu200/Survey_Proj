#!/bin/bash
# =============================================================================
# 空地协同无人化智能测绘系统 - 一键启动脚本
# 配置: 优先从项目根目录 env.txt 读取，其次使用环境变量，最后使用默认值
# 启动顺序:
#   1. FastAPI 后端服务
#   2. Web 前端开发服务器
#   3. ROS2 地面站节点
#   4. UAV 仿真节点
#   5. UGV 仿真节点
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# -----------------------------------------------------------------------------
# 加载 env.txt 配置 (优先级: env.txt > 默认值)
# -----------------------------------------------------------------------------
load_env() {
    local env_file="$PROJECT_DIR/env.txt"
    if [ -f "$env_file" ]; then
        echo "加载配置文件: $env_file"
        while IFS='=' read -r key value; do
            # 跳过空行和注释行
            [[ -z "$key" || "$key" =~ ^[[:space:]]*# ]] && continue
            key="${key// /}"       # 去除空格
            value="${value// /}"   # 去除空格
            [[ -n "$key" ]] && export "$key"="$value"
        done < "$env_file"
    else
        echo "提示: env.txt 未找到，使用默认配置 (可复制 env.txt.example 创建)"
    fi

    # 设置默认值 (如果未通过 env.txt 或环境变量设置)
    export FRONTEND_HOST="${FRONTEND_HOST:-0.0.0.0}"
    export FRONTEND_PORT="${FRONTEND_PORT:-3000}"
    export BACKEND_HOST="${BACKEND_HOST:-0.0.0.0}"
    export BACKEND_PORT="${BACKEND_PORT:-8000}"
}

load_env

echo "=============================================="
echo " 空地协同无人化智能测绘系统 v0.1.0"
echo "=============================================="
echo ""

# 激活虚拟环境
if [ -f "$PROJECT_DIR/venv/bin/activate" ]; then
    source "$PROJECT_DIR/venv/bin/activate"
else
    echo "错误: 虚拟环境未找到，请先运行 scripts/setup_env.sh"
    exit 1
fi

# 杀掉已有进程
CLEANUP_DONE=0
cleanup() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return 0
    fi
    CLEANUP_DONE=1
    echo "正在关闭所有服务..."
    kill $BACKEND_PID $FRONTEND_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "所有服务已关闭"
}
trap cleanup EXIT INT TERM

# 1. 启动后端 (FastAPI + WebSocket)
echo "[1/3] 启动后端服务 (FastAPI + WebSocket)..."
cd "$PROJECT_DIR"
python3 -m uvicorn src.backend.main:app --host "$BACKEND_HOST" --port "$BACKEND_PORT" --reload &
BACKEND_PID=$!
sleep 2
echo "  后端服务已启动: http://${BACKEND_HOST}:${BACKEND_PORT}"
echo "  API 文档: http://${BACKEND_HOST}:${BACKEND_PORT}/docs"

# 2. 启动前端 (Vite 开发服务器)
echo "[2/3] 启动前端开发服务器 (Vue3 + Vite)..."
cd "$PROJECT_DIR/src/web"
npm run dev -- --host "$FRONTEND_HOST" --port "$FRONTEND_PORT" &
FRONTEND_PID=$!
sleep 3
echo "  前端服务已启动: http://${FRONTEND_HOST}:${FRONTEND_PORT}"

# 3. 提示 ROS2 启动 (需要手动在另一个终端执行)
echo ""
echo "[3/3] ROS2 节点启动说明:"
echo "  在另一个终端执行以下命令:"
echo ""
echo "  # 激活 ROS2 环境"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $PROJECT_DIR/src/ros2_ws/install/setup.bash"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  export CYCLONEDDS_URI=file://$PROJECT_DIR/config/dds/cyclonedds.xml"
echo ""
echo "  # 启动地面站"
echo "  ros2 launch ground_station ground_station.launch.py"
echo ""
echo "  # 新终端: 启动 UAV 仿真"
echo "  ros2 launch uav_sim uav_sim.launch.py use_gazebo:=false"
echo ""
echo "  # 新终端: 启动 UGV 仿真"
echo "  ros2 launch ugv_sim ugv_sim.launch.py use_gazebo:=false"
echo ""
echo "  # 发送起飞指令"
echo "  ros2 topic pub /uav/cmd std_msgs/msg/String \"data: 'arm'\""
echo "  ros2 topic pub /uav/cmd std_msgs/msg/String \"data: 'takeoff'\""
echo ""
echo "=============================================="
echo " Web 监测页面: http://${FRONTEND_HOST}:${FRONTEND_PORT}"
echo " 按 Ctrl+C 关闭所有服务"
echo "=============================================="

# 等待用户终止
wait $BACKEND_PID $FRONTEND_PID 2>/dev/null
