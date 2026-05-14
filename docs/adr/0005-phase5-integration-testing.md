# ADR 0005 — 五阶段：系统集成与全面测试

**状态**: 已实现
**日期**: 2026-05-14
**决策者**: 开发团队

---

## 背景

前四阶段已完成主链路闭环、基础功能增强、协同 SLAM 实现和数据融合优化。五阶段需完成系统集成、全面测试和交付文档，确保系统达到可交付状态。

## 决策

### 5.1 GitHub Workflows CI 搭建

**决策**: 使用 GitHub Actions 构建四级流水线。

流水线 Job：
1. **Python 测试** — 语法检查 + pytest 单元测试 + 覆盖率报告
2. **集成测试** — bash 脚本端到端数据流验证
3. **前端构建** — TypeScript 类型检查 + Vite 生产构建
4. **ROS2 编译** — Docker 容器内 colcon 构建检查

配置文件: `.github/workflows/ci.yml`

### 5.2 单元测试覆盖

**决策**: 为全部核心模块编写 pytest 单元测试，目标覆盖率 ≥ 70%。

测试文件：

| 测试文件 | 覆盖模块 |
|---------|---------|
| `tests/test_imu_preintegration.py` | IMU 预积分器（初始化、积分精度、信息矩阵、异常处理） |
| `tests/test_scan_to_map_icp.py` | Scan-to-Map ICP 配准（恒等、已知变换、收敛性、分数指标） |
| `tests/test_factor_graph_optimization.py` | 因子图构建/优化、UGV 融合、滑动窗口、PGO |
| `tests/test_pipeline_nodes.py` | 点云滤波/ICP/SfM 节点独立逻辑、手眼标定、协同优化 |
| `tests/test_registration_and_consistency.py` | 粗配准、一致性校验、回环检测、鲁棒性增强器 |

### 5.3 四类场景仿真实验

**决策**: 创建统一仿真实验框架，支持四大类场景的自动化评测。

场景定义：
- **E1 城市街区**: 密集建筑 + 弱GNSS，核心指标 ATE/RPE
- **E2 矿区地形**: 陡坡 + 不规则堆体，核心指标 点云配准 RMSE/密度比
- **E3 山地丘陵**: 大高差 + 植被覆盖，核心指标 模型完整性/SSIM
- **E4 地质灾害应急**: 滑坡区域快速测绘，核心指标 全流程耗时

实验脚本: `scripts/simulation_experiments.py`
评测工具: `MetricsComputer` 类（ATE/RPE/RMSE/SSIM/密度比/完整性）

### 5.4 对比实验

**决策**: 实现四种基线方案对比框架。

基线方案：
1. **纯无人机 (UAV Only)**: 倾斜摄影 + SfM
2. **纯无人车 (UGV Only)**: LiDAR SLAM
3. **离线融合**: 独立采集后离线配准
4. **协同方案**: 空地实时协同优化

预期目标：
- ATE 降低 ≥ 40%
- 空洞率降低 ≥ 60%
- 全流程耗时减少 ≥ 50%

对比脚本: `scripts/comparative_experiments.py`
输出: JSON + Markdown 对比报告

### 5.5 鲁棒性测试

**决策**: 实现四类鲁棒性测试。

测试项：
1. **GNSS 信号丢失** — 定位保持能力（漂移 < 5m）
2. **通信断连** — 容错机制验证
3. **弱纹理环境** — V-SLAM 降级验证（FULL_FUSION → LIDAR_IMU_ONLY）
4. **长时间运行** — ≥ 4 小时稳定性

测试脚本: `scripts/robustness_tests.py`
输出: JSON + Markdown 鲁棒性测试报告

### 5.6 文档与演示

**决策**: 完成全套项目文档。

文档清单：

| 文档 | 路径 | 内容 |
|------|------|------|
| 技术报告 | `docs/technical-report.md` | 系统架构、核心算法、实验验证、指标总结 |
| 部署文档 | `docs/deployment-guide.md` | 环境安装、配置、编译、启动、测试全流程 |
| 使用手册 | `docs/user-manual.md` | 界面说明、操作流程、常见问题 |
| ROS2 功能包说明 | `docs/ros2-packages.md` | 5 个功能包 API、Topic、参数、降级策略 |

## 影响

- 新增 `.github/workflows/ci.yml` CI 流水线配置
- 新增 `tests/` 目录（6 个测试文件，覆盖核心模块）
- 新增 `scripts/simulation_experiments.py` 仿真实验框架
- 新增 `scripts/comparative_experiments.py` 对比实验框架
- 新增 `scripts/robustness_tests.py` 鲁棒性测试框架
- 新增 4 个文档文件（技术报告/部署/使用手册/ROS2包说明）

## 依赖

- 前四阶段全部完成 ✅
- GitHub Actions 运行环境
- pytest + pytest-cov 测试框架
