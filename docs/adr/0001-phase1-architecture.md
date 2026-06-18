# ADR-0001: 一阶段主链路闭环架构设计

## 状态
已采纳 (2026-06-18)

## 背景
一阶段（MVP）目标是打通"无人机采集 → 无人车采集 → 通信传输 → 地面站展示"的全流程闭环，验证系统基本可行性。

## 决策

### 1. ROS2 通信选型
- **决策**: 使用 ROS2 Humble + CycloneDDS 作为通信中间件
- **理由**: CycloneDDS 是 ROS2 Humble 默认 RMW 实现，性能优异，零配置即可工作
- **替代方案**: Fast-DDS（配置复杂）、RTI Connext（商业授权）

### 2. 仿真平台
- **决策**: Gazebo Garden (Ignition Gazebo) 作为仿真环境
- **理由**: 与 ROS2 Humble 深度集成，支持 ros_gz_bridge 桥接
- **替代方案**: Gazebo Classic（ROS1 时代产物，不推荐）

### 3. 消息接口设计
- **决策**: 定义自定义 ROS2 消息 (common_interfaces 包)
  - UAVStatus / UGVStatus: 载具实时状态
  - Heartbeat: 节点存活检测
  - SystemAlert: 系统告警
- **理由**: 统一数据结构，确保 UAV/UGV/地面站三方数据兼容

### 4. 地面站数据处理
- **决策**: 三节点架构（data_receiver / bag_recorder / cache_manager）
- **理由**: 职责分离，各节点可独立启停，便于调试和维护
- **SSD 缓存目录结构**: `~/airunway_cache/{bags,images,lidar,telemetry,logs}`

### 5. Web 监测方案
- **决策**: FastAPI + WebSocket 后端 + Vue3 + 天地图前端
- **理由**:
  - FastAPI: 原生支持 WebSocket，异步高性能
  - Vue3: Composition API + TypeScript 类型安全
  - 天地图: 国内合规地图服务，免费 API Key
- **替代方案**: Cesium（三维地球，资源消耗大）、Leaflet（需要自行配置瓦片源）

### 6. 数据桥接
- **决策**: ROS2 → HTTP REST → FastAPI → WebSocket → 前端
- **理由**: 解耦 ROS2 和 Web 层，Web 服务可独立于 ROS2 运行和测试
- **延迟要求**: 端到端延迟 < 500ms

## 影响
- 所有包依赖 common_interfaces 自定义消息
- Web 后端通过 HTTP POST 接口接收 ROS2 数据
- 前端通过 WebSocket 获取实时状态推送

## 风险
- macOS 无法完整运行 ROS2 仿真，需在 Ubuntu 22.04 环境中进行运行时验证
- 网络延迟可能影响实时性，需在实际部署环境中进一步测试
