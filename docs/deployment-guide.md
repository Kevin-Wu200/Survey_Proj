# 空地协同无人化智能测绘系统 — 部署文档

---

## 环境要求

| 组件       | 最低版本                    | 说明                         |
|-----------|----------------------------|------------------------------|
| 操作系统   | Ubuntu 22.04 LTS           | ROS2 Humble 运行环境          |
| Python    | 3.10+                      | venv 虚拟环境                 |
| Node.js   | 18+                        | 前端构建和开发服务器           |
| ROS2      | Humble Hawksbill           | 仿真和通信中间件              |
| PostgreSQL | 15 + PostGIS 3            | 数据持久化存储                |

---

## 一、基础环境安装

### 1.1 安装 ROS2 Humble

```bash
# 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble 桌面版
sudo apt update
sudo apt install ros-humble-desktop-full
```

### 1.2 安装 Python 虚拟环境与依赖

```bash
cd /path/to/空地协同无人化智能测绘系统
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 1.3 安装 Node.js 和前端依赖

```bash
# 使用 nvm 安装 Node.js 18+
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install 18
nvm use 18

# 安装前端依赖
cd src/web
npm install
```

### 1.4 安装 PostgreSQL 15 + PostGIS 3

详见 `docs/postgresql-setup.md`

```bash
# 快速安装
sudo apt install postgresql-15 postgresql-15-postgis-3 postgis

# 创建数据库
sudo -u postgres createdb aerial_ground_mapping
sudo -u postgres psql -c "CREATE USER mapper WITH PASSWORD 'your_password';"
sudo -u postgres psql -c "GRANT ALL PRIVILEGES ON DATABASE aerial_ground_mapping TO mapper;"
sudo -u postgres psql -d aerial_ground_mapping -c "CREATE EXTENSION postgis;"
```

---

## 二、环境配置

### 2.1 初始化配置文件

```bash
cd /path/to/空地协同无人化智能测绘系统
cp env.txt.example env.txt
```

### 2.2 编辑环境配置

编辑 `env.txt`，按需调整：

| 配置项         | 说明                   | 默认值       |
|---------------|-----------------------|-------------|
| FRONTEND_HOST | 前端监听地址            | 0.0.0.0    |
| FRONTEND_PORT | 前端端口               | 3000        |
| BACKEND_HOST  | 后端监听地址            | 0.0.0.0    |
| BACKEND_PORT  | 后端端口               | 8000        |
| CENTER_LAT    | 地图默认中心纬度         | 30.0        |
| CENTER_LNG    | 地图默认中心经度         | 120.0       |
| DB_HOST       | 数据库主机             | localhost   |
| DB_PORT       | 数据库端口             | 5432        |
| DB_NAME       | 数据库名               | aerial_ground_mapping |
| DB_USER       | 数据库用户             | mapper      |
| DB_PASSWORD   | 数据库密码             | —           |

配置优先级：**环境变量 > env.txt > 默认值**

---

## 三、编译与构建

### 3.1 构建 ROS2 Workspace

```bash
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译
cd src/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载工作空间环境
source install/setup.bash
```

### 3.2 构建前端

```bash
cd src/web
npm run build
# 构建产物位于 src/web/dist/
```

---

## 四、启动系统

### 4.1 启动后端服务

```bash
cd /path/to/空地协同无人化智能测绘系统
source venv/bin/activate
python3 -m uvicorn src.backend.main:app --host 0.0.0.0 --port 8000
```

### 4.2 启动前端开发服务器

```bash
cd src/web
npm run dev
# 访问 http://localhost:3000
```

### 4.3 启动 ROS2 仿真（可选，需 ROS2 环境）

```bash
source /opt/ros/humble/setup.bash
source src/ros2_ws/install/setup.bash

# 启动 UAV 仿真
ros2 launch uav_sim uav_sim.launch.py

# 启动 UGV 仿真
ros2 launch ugv_sim ugv_sim.launch.py

# 启动地面站
ros2 launch ground_station ground_station.launch.py

# 启动数据融合管道
ros2 launch fusion_pipeline fusion_pipeline.launch.py
```

---

## 五、运行测试

### 5.1 运行集成测试

```bash
bash scripts/integration_test.sh
```

### 5.2 运行单元测试

```bash
source venv/bin/activate
pip install pytest pytest-cov
python -m pytest tests/ -v --cov=src/slam --cov-report=term-missing
```

### 5.3 运行仿真实验

```bash
source venv/bin/activate
python scripts/simulation_experiments.py --output ./experiment_results
```

### 5.4 运行对比实验

```bash
source venv/bin/activate
python scripts/comparative_experiments.py --output ./comparative_results
```

### 5.5 运行鲁棒性测试

```bash
source venv/bin/activate
python scripts/robustness_tests.py --output ./robustness_results
```

---

## 六、CI/CD 流水线

项目配置了 GitHub Actions CI 流水线（`.github/workflows/ci.yml`）：

```yaml
触发条件:
  - push 到 main/master/develop 分支
  - pull_request 到 main/master 分支

流水线 Job:
  1. Python 测试 — 语法检查 + pytest + 覆盖率
  2. 集成测试 — 端到端数据流验证
  3. 前端构建 — TypeScript 类型检查 + Vite 构建
  4. ROS2 编译 — colcon 编译检查 (Docker)
```

---

## 七、局域网访问

服务默认监听 `0.0.0.0`，局域网内其他设备可通过：

```
http://{服务器IP}:3000/
```

**防火墙设置**（如有需要）：

```bash
sudo ufw allow 3000/tcp
sudo ufw allow 8000/tcp
```

---

## 八、常见问题

### Q: venv 虚拟环境激活失败？
A: 确保在 Linux Ubuntu 22.04 环境中创建本地 venv，而非跨平台复制。

### Q: ROS2 colcon build 失败？
A: 确保已 `source /opt/ros/humble/setup.bash`，检查 `rosdep` 依赖是否完整。

### Q: PostgreSQL 连接失败？
A: 检查 `env.txt` 中的数据库配置是否正确，PostgreSQL 服务是否运行：`sudo systemctl status postgresql`

### Q: 前端构建报错？
A: 检查 Node.js 版本 ≥ 18，运行 `npm install` 确保依赖完整。
