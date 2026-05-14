# ADR 0004 — 四阶段：数据融合与性能优化

**状态**: 已实现  
**日期**: 2026-05-14  
**决策者**: 开发团队  

---

## 背景

三阶段协同 SLAM 已实现空地位姿对齐。四阶段需在此基础上搭建完整的数据融合管道，将 UAV SfM 稀疏点云与 UGV LiDAR 稠密点云融合，输出三维网格模型，并提供 Web 端的数据管理和成果展示能力。

## 决策

### 4.1 数据融合管道

**决策**: 使用 ROS2 Component 节点管道实现端到端数据流。

管道顺序：`image_rectify_node` → `pointcloud_filter_node` + `sfm_node` → `icp_registration_node` → `meshing_node`

- `image_rectify_node`: 基于 OpenCV 的相机去畸变，预计算映射表优化性能
- `pointcloud_filter_node`: VoxelGrid 降采样 + Statistical Outlier Removal + 直通滤波
- `sfm_node`: 增量式 SfM，支持 slam 模块 `IncrementalSfM` 和内置 `SimpleSfM` 双模式
- `icp_registration_node`: Open3D point-to-plane ICP（主）+ numpy SVD ICP（备用）
- `meshing_node`: Open3D 泊松重建（主）+ numpy 2.5D 网格（备用）

**参数配置**: YAML 格式，ROS2 命名空间按节点名分离，通过 launch 文件统一加载。

### 4.2 粗配准

**决策**: 复用三阶段 `CollaborativeOptimizer` 的手眼标定结果 `T_uav2ugv`。

- 使用 `CoarseRegistrator.align_point_clouds()` 将 UAV 点云变换到 UGV 坐标系
- 目标精度: RMSE < 20cm
- 回退策略: 标定失败时使用单位变换

### 4.3 精配准

**决策**: ICP point-to-plane 为主算法，NDT 为备选。

- 主算法: Open3D `registration_icp` with `TransformationEstimationPointToPlane`
- 备选: 纯 numpy NDT (Normal Distributions Transform) 高斯-牛顿优化
- 降级方案: numpy SVD ICP (无 Open3D 时)
- 目标精度: RMSE < 5cm
- `FineRegistrator` 支持配置化切换算法

### 4.4 数据持久化

**决策**: PostgreSQL 15 + PostGIS 3，系统直接安装（非 Docker）。

- 已有完整 `DatabaseManager` 实现 (src/slam/persistence.py)
- 支持的 CRUD: 任务、航点、轨迹、模型元数据、融合成果
- 三维模型文件存储于文件系统，数据库维护索引
- 安装文档: docs/postgresql-setup.md
- 安装脚本: scripts/setup_postgresql.sh

### 4.5 Web 前端功能增强

**决策**: 在现有 Vue3 + Three.js 前端基础上新增数据管理面板。

新增功能：
1. **采集进度展示**: 后端 `/api/collection/progress/{task_id}` API + 内存缓存
   - 展示已采集面积 / 总面积
   - 进度百分比 + 进度条
   - UAV 照片数、UGV 里程、已用时间、预计完成时间

2. **融合成果预览**: 后端 `/api/tasks/{task_id}/fusion` API
   - 列表展示融合成果（点云数、面片数、RMSE）
   - 粗/精配准 RMSE 对比
   - 缩略图预览（V2.0 简略实现，通过 URL 加载）

3. **任务历史查看**: 后端 `/api/tasks` API（已有）
   - 状态过滤 + 分页
   - 任务详情含融合成果概览

4. **PGW 世界文件工具**: 后端 `/api/pgw/generate` + `/api/pgw/list` API
   - 读取 GLB 模型包围盒，自动计算 .pgw 6行参数
   - 支持自定义图像分辨率和边距
   - 前端提供可视化生成界面，支持一键复制

前端实现: `DataPanel.vue` 组件，4 个 Tab（采集进度 / 任务历史 / 融合成果 / PGW工具）

### 4.6 全链路性能调优

**决策**: 监控指标 + 目标阈值对比。

- 后端 `/api/performance` 端点提供性能指标查询和更新
- 关键指标: 采集效率 (ha/min)、融合延迟 (s)、点云压缩比、通信带宽
- 目标: 采集效率 > 0.2 ha/min (即 < 5 min/ha)，融合延迟 < 600s (10 min)

## 影响

- 前端新增 DataPanel 组件，Vue 构建通过
- 后端新增 4 个 API 端点组 (采集进度、PGW、性能指标)
- 融合管道 YAML 配置重构为 ROS2 标准格式
- 所有 Python 文件语法检查通过

## 依赖

- 三阶段协同 SLAM ✅（空地位姿对齐）
- PostgreSQL 15 + PostGIS 3
- OpenCV、Open3D（可选，有 numpy 回退）
