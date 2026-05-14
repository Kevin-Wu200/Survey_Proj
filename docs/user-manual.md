# 空地协同无人化智能测绘系统 — 使用手册

---

## 1. 系统概述

空地协同无人化智能测绘系统是一个集成无人机（UAV）和无人车（UGV）的智能化测绘平台。通过 Web 界面可实时查看 UAV/UGV 位置、作业状态、采集进度和融合成果。

---

## 2. 界面说明

### 2.1 主界面布局

```
┌─────────────────────────────────────────────────┐
│                   顶部工具栏                     │
│  [航点工具] [告警] [状态]                        │
├──────────┬───────────────────┬──────────────────┤
│          │                   │                  │
│  2D 地图  │    3D 地形场景    │   右侧信息面板    │
│  (叠加层) │   (Three.js)     │  - UAV 状态      │
│          │                   │  - UGV 状态      │
│          │                   │  - 系统告警      │
├──────────┴───────────────────┴──────────────────┤
│                底部状态栏 / 时间线                │
└─────────────────────────────────────────────────┘
```

### 2.2 核心组件

| 组件              | 功能                                    |
|-------------------|----------------------------------------|
| **3D 场景视图**   | 地形展示、UAV/UGV 3D 标绘、点云可视化    |
| **2D 地图叠加**   | 卫星地图底图、航线规划、航点标识          |
| **状态面板**      | 实时显示 UAV/UGV 的飞行/行驶状态          |
| **告警面板**      | 系统告警信息（电池低、信号弱、碰撞预警等） |
| **航点工具栏**    | UAV 航线规划、航点添加/编辑/删除          |
| **时间线控制**    | 回放历史数据、快进/快退                   |
| **数据面板**      | 采集进度、任务历史、融合成果、PGW 工具    |

---

## 3. 操作流程

### 3.1 系统启动

1. **启动后端服务**：
   ```bash
   cd 空地协同无人化智能测绘系统
   source venv/bin/activate
   python3 -m uvicorn src.backend.main:app --host 0.0.0.0 --port 8000
   ```

2. **启动前端**：
   ```bash
   cd src/web
   npm run dev
   ```

3. **访问系统**：浏览器打开 `http://localhost:3000`

### 3.2 创建测绘任务

1. 在数据面板选择「任务管理」
2. 点击「新建任务」
3. 填写任务信息：
   - 任务名称
   - 测绘区域（地图上框选）
   - 飞行高度和速度
   - UGV 行驶路线
4. 确认创建

### 3.3 UAV 航线规划

1. 打开「航点工具栏」
2. 在地图上点击添加航点
3. 系统自动生成蛇形航线
4. 可调整航点位置、高度、速度
5. 点击「上传至 UAV」发送航线

### 3.4 实时监控

1. 主界面 3D 场景实时显示 UAV 和 UGV 位置
2. 右侧状态面板显示：
   - UAV：飞行模式、高度、速度、电池电量
   - UGV：行驶速度、电池电量、遥控状态
3. 告警面板显示系统异常信息
4. 数据面板显示采集进度

### 3.5 数据查看

1. **采集进度**：已采集面积 / 总面积、照片数、里程
2. **任务历史**：已完成任务列表、状态过滤、分页
3. **融合成果**：点云数、面片数、粗/精配准 RMSE
4. **PGW 工具**：生成模型对应的世界文件

---

## 4. 运行仿真实验

### 4.1 场景实验

```bash
# 运行全部四类场景
python scripts/simulation_experiments.py

# 运行单个场景
python scripts/simulation_experiments.py --scene E1
```

### 4.2 对比实验

```bash
python scripts/comparative_experiments.py
```

### 4.3 鲁棒性测试

```bash
# 全部测试
python scripts/robustness_tests.py

# 单项测试
python scripts/robustness_tests.py --test gnss
python scripts/robustness_tests.py --test comm
python scripts/robustness_tests.py --test texture
python scripts/robustness_tests.py --test duration
```

---

## 5. ROS2 仿真操作

### 5.1 启动完整仿真

```bash
# 终端 1: 启动 UAV 仿真
source /opt/ros/humble/setup.bash
source src/ros2_ws/install/setup.bash
ros2 launch uav_sim uav_sim.launch.py

# 终端 2: 启动 UGV 仿真
ros2 launch ugv_sim ugv_sim.launch.py

# 终端 3: 启动地面站
ros2 launch ground_station ground_station.launch.py

# 终端 4: 启动融合管道
ros2 launch fusion_pipeline fusion_pipeline.launch.py
```

### 5.2 查看运行状态

```bash
# 查看活跃节点
ros2 node list

# 查看活跃 Topic
ros2 topic list

# 查看 UAV 位姿
ros2 topic echo /uav/pose

# 查看 UGV LiDAR 点云统计
ros2 topic echo /ugv/lidar/points --once
```

### 5.3 录制和回放数据

```bash
# 录制所有 Topic
ros2 bag record -a -o my_recording

# 回放录制数据
ros2 bag play my_recording
```

---

## 6. 常见问题

### Q: 前端页面加载缓慢？
A: 检查网络连接，3D 场景需要加载地形数据。确保后端服务正常运行。

### Q: UAV/UGV 位置不更新？
A: 检查 ROS2 节点是否正常启动，使用 `ros2 node list` 确认节点运行状态。

### Q: 数据面板显示为空？
A: 检查 PostgreSQL 数据库连接，确认数据库服务运行正常：`sudo systemctl status postgresql`

### Q: 仿真场景看不到 Gazebo 画面？
A: 确认 Gazebo Garden 已安装，或使用无 GUI 模式（添加 `headless:=true` 参数）。

### Q: 如何切换 2D/3D 视角？
A: 界面默认同时显示 2D 地图和 3D 场景，可通过右侧面板调整各区域大小。
