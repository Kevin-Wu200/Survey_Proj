#!/bin/bash
# =============================================================================
# 空地协同无人化智能测绘系统 - 环境安装脚本
# 适用于 Ubuntu 22.04 LTS
# =============================================================================
set -e

echo "=============================================="
echo " 空地协同无人化智能测绘系统 - 环境安装"
echo "=============================================="

# --- 系统依赖 ---
echo "[1/6] 安装系统依赖..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    curl gnupg lsb-release \
    python3-pip python3-venv python3-colcon-common-extensions \
    ros-dev-tools \
    build-essential cmake git \
    libyaml-cpp-dev libpoco-dev

# --- ROS2 Humble ---
echo "[2/6] 安装 ROS2 Humble..."
if ! dpkg -l | grep -q ros-humble-desktop; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq ros-humble-desktop ros-humble-ros-gz
fi

# --- Gazebo Garden (Ignition Gazebo) ---
echo "[3/6] 安装 Gazebo Garden..."
if ! dpkg -l | grep -q gz-garden; then
    sudo apt-get install -y -qq gz-garden
fi

# --- CycloneDDS ---
echo "[4/6] 配置 CycloneDDS..."
if ! dpkg -l | grep -q ros-humble-rmw-cyclonedds-cpp; then
    sudo apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp
fi

# 配置环境变量
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

if ! grep -q "CYCLONEDDS_URI" ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc << EOF

# === 空地协同无人化智能测绘系统 ===
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${PROJECT_DIR}/config/dds/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ${PROJECT_DIR}/src/ros2_ws/install/setup.bash 2>/dev/null || true
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${PROJECT_DIR}/gazebo/models
export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:${PROJECT_DIR}/gazebo/worlds
EOF
fi

# --- Python 虚拟环境 ---
echo "[5/6] 配置 Python 虚拟环境..."
cd "$PROJECT_DIR"
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi
source venv/bin/activate
pip install --upgrade pip -q
pip install -r requirements.txt -q 2>/dev/null || true

# --- 构建 ROS2 Workspace ---
echo "[6/6] 构建 ROS2 Workspace..."
source /opt/ros/humble/setup.bash
cd "$PROJECT_DIR/src/ros2_ws"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>/dev/null || \
    colcon build --symlink-install --packages-skip common_interfaces 2>/dev/null || true

echo ""
echo "=============================================="
echo " 环境安装完成！请执行: source ~/.bashrc"
echo "=============================================="
