# 空地协同无人化智能测绘系统 - 项目上下文

## 系统概述

空地协同无人化智能测绘系统是一个集无人机（UAV）、无人车（UGV）与地面站于一体的智能化测绘平台。系统通过 ROS2 实现 UAV 与 UGV 的协同数据采集，地面站负责数据接收、处理、存储与 Web 可视化展示。

## 核心术语

| 术语 | 定义 |
|------|------|
| UAV | 无人机（Unmanned Aerial Vehicle），使用 DJI M300 RTK 四旋翼模型 |
| UGV | 无人车（Unmanned Ground Vehicle），使用四轮差速底盘模型 |
| 地面站 | Ground Station，负责数据汇聚、存储、处理和 Web 服务 |
| DDS | Data Distribution Service，ROS2 底层通信中间件 |
| 高德地图 | AMap，高德软件有限公司，提供地图 JS API |
| ros2 bag | ROS2 数据录制工具，用于存储传感器数据包 |

## 系统架构

```
┌─────────────────────┐    ┌─────────────────────┐
│   UAV 仿真节点       │    │   UGV 仿真节点       │
│  - 位姿发布          │    │  - 位姿发布          │
│  - 相机图像          │    │  - LiDAR 点云        │
│  - 飞行状态          │    │  - 双目相机          │
│  - 心跳              │    │  - IMU              │
└──────┬──────────────┘    └──────┬──────────────┘
       │   ROS2 DDS (CycloneDDS)  │
       └──────────┬───────────────┘
                  │
       ┌──────────▼───────────────┐
       │     地面站 (Ground Station) │
       │  - 数据接收节点            │
       │  - Bag 录制               │
       │  - 缓存管理               │
       │  - WebSocket 桥接         │
       └──────────┬───────────────┘
                  │
       ┌──────────▼───────────────┐
       │    Web 服务层             │
       │  - FastAPI REST API      │
       │  - WebSocket 实时推送     │
       └──────────┬───────────────┘
                  │
       ┌──────────▼───────────────┐
       │    Web 前端 (Vue3)        │
       │  - 高德地图地图展示          │
       │  - UAV/UGV 实时标绘       │
       │  - 状态面板               │
       └──────────────────────────┘
```

## 技术栈

- **仿真**: ROS2 Humble + Gazebo Garden
- **通信**: CycloneDDS (RMW_IMPLEMENTATION)
- **后端**: Python FastAPI + WebSocket + uvicorn
- **前端**: Vue 3 + TypeScript + Pinia + 高德地图 JS API v2.0
- **构建**: colcon (ROS2) + Vite (前端)
- **数据存储**: ros2 bag (SQLite3) + 本地 SSD 缓存

## 开发环境

- 操作系统: Ubuntu 22.04 LTS (ROS2 运行时) / macOS (Web 服务开发)
- Python: 3.10+ (venv 虚拟环境)
- Node.js: 18+
- ROS2: Humble Hawksbill

## 关键设计决策

详见 `docs/adr/0001-phase1-architecture.md`
