# 0002 — 二阶段基础功能增强架构说明

## 概述

二阶段在一阶段主链路闭环基础上，完善各子系统核心功能，使系统具备基本的自动化作业能力。

## 功能清单

### 2.1 无人机航点任务模块

- **新增 ROS2 节点**: `uav_mission_controller` (uav_sim 包)
- **新增 ROS2 消息**: `WaypointMission.msg`, `WaypointMissionStatus.msg`
- **核心类**: `WaypointMissionManager` — 模拟 DJI OSDK Waypoint V2 接口
- **航线类型**: 自定义航点 / 多边形航线 / 蛇形航线
- **拍照模式**: 等距触发（相机快门与航点绑定）
- **Mission 状态机**: 空闲 → 上传中 → 就绪 → 执行中 → 暂停 → 完成/失败/取消
- **后端 API**:
  - `POST /api/uav/mission/upload` — 上传航点任务
  - `POST /api/uav/mission/start` — 启动任务
  - `POST /api/uav/mission/pause` — 暂停任务
  - `POST /api/uav/mission/resume` — 恢复任务
  - `POST /api/uav/mission/stop` — 停止任务
  - `GET /api/uav/mission/status` — 获取任务状态
- **前端**: WaypointToolbar 组件支持可视化航线设计和任务控制

### 2.2 无人车自主导航

- **新增 ROS2 节点**: `ugv_nav_controller` (ugv_sim 包)
- **新增 ROS2 消息**: `NavigationGoal.msg`, `NavigationStatus.msg`
- **核心类**: `NavController` + `PathPlanner` — 仿真模式模拟 Nav2 行为
- **规划器配置**: Smac Hybrid-A* 全局规划器 + Regulated Pure Pursuit 局部规划器
- **代价地图**: 分辨率 0.05m，10m×10m
- **Cartographer 2D SLAM**: 配置文件 `cartographer_2d.lua`
- **Nav2 参数**: 完整 `nav2_params.yaml` (AMCL定位、全局/局部代价地图、规划器、行为树)
- **后端 API**:
  - `POST /api/ugv/nav/goal` — 发送导航目标点
  - `POST /api/ugv/nav/cancel` — 取消导航
  - `GET /api/ugv/nav/status` — 获取导航状态
- **前端**: 地图 Ctrl+点击下发 UGV 导航目标点

### 2.3 Web 前端功能增强

- **WebSocket 告警推送**: 实时推送电量低、连接断开、任务完成等告警
- **航点绘制**: WaypointToolbar 组件支持在地图上绘制航线，生成航点并发送至 UAV
- **UGV 目标下发**: 地图点击（Ctrl+Click）发送导航目标位姿至 UGV
- **告警面板**: AlertPanel 组件实时展示系统告警，支持分级显示（info/warning/error/critical）
- **模式指示器**: 顶部栏显示仿真/实物模式状态

### 2.4 任务回放功能

- **后端**: ROS2 Bag (SQLite3) 解析器，提取位姿/图像/点云时间序列
- **后端 API**:
  - `GET /api/replay/bags` — 列出可回放的 Bag 文件
  - `POST /api/replay/load` — 加载回放会话
  - `POST /api/replay/control` — 回放控制 (play/pause/stop/seek)
  - `GET /api/replay/frame` — 获取当前回放帧
- **前端**: TimelineControl 组件支持自定义时间轴，高德地图回放 UAV/UGV 历史轨迹
- **倍速回放**: 支持 0.5× / 1× / 2× / 4× / 8× 多档速度

### 2.5 仿真/实物模式切换

- **SystemMode 枚举**: SIMULATION(0) / REAL(1)
- **后端 API**:
  - `GET /api/mode` — 获取当前模式
  - `POST /api/mode/switch` — 切换运行模式
- **前端**: 顶部栏实时显示模式状态（蓝色=仿真，绿色=实物）
- **架构设计**: 传感器输入接口抽象 + 控制输出接口抽象，同一套代码在两种模式下切换

### 2.6 容错机制

- **通信超时检测**: 3s 超时，最多重试 3 次
- **UGV 断连处理**:
  - 30s → 原地等待
  - 60s → 原路返回
- **UAV 断连处理**: 60s → 执行 DJI Return to Home（返航）
- **电量低告警**: UAV/UGV 电量低于 20% 自动告警
- **后端 API**: `GET /api/fault/status` — 容错监控状态查询
- **后台任务**: `fault_monitor_loop` — 每秒检查连接状态

## 新增文件清单

### ROS2 消息 (common_interfaces)
- `msg/WaypointMission.msg`
- `msg/WaypointMissionStatus.msg`
- `msg/NavigationGoal.msg`
- `msg/NavigationStatus.msg`

### ROS2 节点
- `src/ros2_ws/src/uav_sim/uav_sim/uav_mission_controller.py`
- `src/ros2_ws/src/ugv_sim/ugv_sim/ugv_nav_controller.py`

### Nav2 配置
- `src/ros2_ws/src/ugv_sim/config/nav2/nav2_params.yaml`
- `src/ros2_ws/src/ugv_sim/config/nav2/cartographer_2d.lua`

### 后端增强
- `src/backend/main.py` — 大幅扩展（新增22个API端点，3个后台任务）

### 前端新增组件
- `src/web/src/components/AlertPanel.vue` — 告警面板
- `src/web/src/components/WaypointToolbar.vue` — 航线工具栏
- `src/web/src/components/TimelineControl.vue` — 回放时间轴

### 前端修改
- `src/web/src/types/index.ts` — 新增11个类型定义
- `src/web/src/stores/system.ts` — 新增6个状态字段
- `src/web/src/composables/useWebSocket.ts` — 新增告警推送处理
- `src/web/src/App.vue` — 集成新组件 + 模式指示器
- `src/web/src/views/MapView.vue` — 航点绘制 + UGV目标 + 回放轨迹

## 启动方式

```bash
# 后端 (所有功能集成)
cd src/backend
uvicorn main:app --host 0.0.0.0 --port 8000 --reload

# 前端
cd src/web
npm run dev

# ROS2 (仿真模式)
ros2 launch uav_sim uav_sim.launch.py
ros2 launch ugv_sim ugv_sim.launch.py
ros2 run uav_sim uav_mission_controller
ros2 run ugv_sim ugv_nav_controller

# ROS2 (实物模式)
# 切换至实物模式: POST /api/mode/switch {"mode": 1}
# 启动对应实物控制节点
```
