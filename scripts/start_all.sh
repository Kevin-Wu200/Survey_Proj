#!/bin/bash
# =============================================================================
# 空地协同无人化智能测绘系统 - 一键启动脚本
# 启动顺序:
#   1. FastAPI 后端服务
#   2. Web 前端开发服务器
#   3. ROS2 地面站节点
#   4. UAV 仿真节点
#   5. UGV 仿真节点
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " 空地协同无人化智能测绘系统 v0.1.0"
echo "=============================================="
echo ""

# 激活虚拟环境
source "$PROJECT_DIR/venv/bin/activate"

# 杀掉已有进程
cleanup() {
    echo "正在关闭所有服务..."
    kill $BACKEND_PID $FRONTEND_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "所有服务已关闭"
}
trap cleanup EXIT INT TERM

# 1. 启动后端 (FastAPI + WebSocket)
echo "[1/3] 启动后端服务 (FastAPI + WebSocket)..."
cd "$PROJECT_DIR"
python -m uvicorn src.backend.main:app --host 0.0.0.0 --port 8000 --reload &
BACKEND_PID=$!
sleep 2
echo "  后端服务已启动: http://localhost:8000"
echo "  API 文档: http://localhost:8000/docs"

# 2. 启动前端 (Vite 开发服务器)
echo "[2/3] 启动前端开发服务器 (Vue3 + Vite)..."
cd "$PROJECT_DIR/src/web"
npm run dev -- --host 0.0.0.0 &
FRONTEND_PID=$!
sleep 3
echo "  前端服务已启动: http://localhost:3000"

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
echo " Web 监测页面: http://localhost:3000"
echo " 按 Ctrl+C 关闭所有服务"
echo "=============================================="

# 等待用户终止
wait $BACKEND_PID $FRONTEND_PID 2>/dev/null
