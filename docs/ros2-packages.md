# 空地协同无人化智能测绘系统 — ROS2 功能包说明文档

---

## 概述

本项目包含 5 个 ROS2 功能包，组织在 `src/ros2_ws/src/` 下，使用 colcon 构建系统。

---

## 功能包一览

| 包名              | 类型     | 说明                                   |
|-------------------|----------|---------------------------------------|
| `common_interfaces` | CMake  | 自定义消息接口定义（.msg）               |
| `uav_sim`         | Python   | UAV 仿真节点（飞行控制、相机、任务管理）    |
| `ugv_sim`         | Python   | UGV 仿真节点（底盘控制、传感器、导航）      |
| `ground_station`   | Python   | 地面站数据处理（接收、录制、缓存、桥接）    |
| `fusion_pipeline`  | Python   | 数据融合管道（滤波、SfM、ICP、网格重建）   |

---

## 构建方法

```bash
source /opt/ros/humble/setup.bash
cd src/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 1. common_interfaces（消息接口包）

### 1.1 构建系统

C++ CMake（`ament_cmake`）

### 1.2 自定义消息

| 消息文件                  | 用途                     |
|--------------------------|-------------------------|
| `UAVStatus.msg`          | UAV 飞行状态（位置、姿态、电池等）|
| `UGVStatus.msg`          | UGV 行驶状态              |
| `Heartbeat.msg`          | 心跳包（在线状态检测）       |
| `SystemAlert.msg`        | 系统告警信息              |
| `WaypointMission.msg`    | 航点任务定义              |
| `WaypointMissionStatus.msg` | 航点任务执行状态         |
| `NavigationGoal.msg`     | 导航目标点                |
| `NavigationStatus.msg`   | 导航执行状态              |

---

## 2. uav_sim（UAV 仿真包）

### 2.1 节点列表

| 节点                       | 功能                         |
|---------------------------|------------------------------|
| `uav_controller`          | 飞行姿态与位置控制器            |
| `uav_camera_sim`          | 仿真相机（发布图像 Topic）       |
| `uav_mission_controller`  | 任务控制器（航线执行、状态管理）   |

### 2.2 发布 Topic

| Topic                     | 类型                     | 频率   |
|---------------------------|-------------------------|--------|
| `/uav/pose`               | PoseStamped             | 10 Hz  |
| `/uav/status`             | UAVStatus               | 5 Hz   |
| `/uav/heartbeat`          | Heartbeat               | 1 Hz   |
| `/uav/image_raw/compressed` | CompressedImage       | 30 Hz  |

### 2.3 参数配置

配置文件：`config/uav_params.yaml`

| 参数                  | 默认值    | 说明          |
|----------------------|----------|--------------|
| `max_speed`          | 15.0     | 最大飞行速度 (m/s) |
| `max_altitude`       | 120.0    | 最大飞行高度 (m)   |
| `camera_width`       | 3840     | 图像宽度 (px)     |
| `camera_height`      | 2160     | 图像高度 (px)     |
| `camera_fps`         | 30       | 帧率            |

### 2.4 启动

```bash
ros2 launch uav_sim uav_sim.launch.py
```

---

## 3. ugv_sim（UGV 仿真包）

### 3.1 节点列表

| 节点                  | 功能                        |
|----------------------|-----------------------------|
| `ugv_controller`     | 四轮差速底盘控制器             |
| `ugv_sensor_pub`     | 传感器数据发布（LiDAR/IMU/相机）|
| `ugv_nav_controller` | 导航控制器（基于 Nav2 路径规划） |

### 3.2 发布 Topic

| Topic                              | 类型             | 频率   |
|------------------------------------|-----------------|--------|
| `/ugv/pose`                        | PoseStamped     | 10 Hz  |
| `/ugv/status`                      | UGVStatus       | 5 Hz   |
| `/ugv/heartbeat`                   | Heartbeat       | 1 Hz   |
| `/ugv/lidar/points`                | PointCloud2     | 10 Hz  |
| `/ugv/imu`                         | Imu             | 100 Hz |
| `/ugv/camera/left/image_raw/compressed`  | CompressedImage | 30 Hz |
| `/ugv/camera/right/image_raw/compressed` | CompressedImage | 30 Hz |
| `/ugv/odom`                        | Odometry        | 10 Hz  |

### 3.3 参数配置

配置文件：`config/ugv_params.yaml`

| 参数            | 默认值  | 说明              |
|----------------|--------|-------------------|
| `max_speed`    | 3.0    | 最大行驶速度 (m/s)  |
| `wheel_base`   | 0.5    | 轴距 (m)          |
| `lidar_lines`  | 64     | LiDAR 线数         |
| `lidar_range`  | 100.0  | LiDAR 最大距离 (m)  |
| `imu_freq`     | 100    | IMU 频率 (Hz)      |

### 3.4 自主导航配置

Nav2 配置位于 `config/nav2/`，Cartographer 配置位于 `config/nav2/cartographer/`。

### 3.5 启动

```bash
ros2 launch ugv_sim ugv_sim.launch.py
```

---

## 4. ground_station（地面站包）

### 4.1 节点列表

| 节点                       | 功能                              |
|---------------------------|-----------------------------------|
| `data_receiver`           | 数据接收（汇聚 UAV/UGV Topic 数据）  |
| `bag_recorder`            | ros2 bag 录制（存储传感器数据包）     |
| `cache_manager`           | 本地 SSD 缓存管理                  |
| `ros2_websocket_bridge`   | ROS2 ↔ WebSocket 桥接             |

### 4.2 发布 Topic

| Topic                        | 类型          | 说明          |
|------------------------------|--------------|--------------|
| `/ground_station/alert`      | SystemAlert  | 系统告警       |

### 4.3 WebSocket 桥接

`ros2_websocket_bridge` 节点将 ROS2 Topic 数据实时转发至 Web 前端：

- UAV 位姿 → WebSocket `/ws/uav`
- UGV 位姿 → WebSocket `/ws/ugv`
- 系统告警 → WebSocket `/ws/alert`

### 4.4 参数配置

配置文件：`config/ground_station_params.yaml`

### 4.5 启动

```bash
ros2 launch ground_station ground_station.launch.py
```

---

## 5. fusion_pipeline（数据融合管道包）

### 5.1 节点列表

| 节点                       | 功能                                  |
|---------------------------|---------------------------------------|
| `pointcloud_filter_node`  | 点云滤波（体素降采样/离群点去除/直通滤波）   |
| `image_rectify_node`      | 图像校正（相机去畸变）                     |
| `sfm_node`                | 增量式 Structure from Motion           |
| `icp_registration_node`   | ICP 精配准（UAV SfM ↔ UGV LiDAR）       |
| `meshing_node`            | 网格重建（泊松重建/2.5D网格）              |

### 5.2 Topic 数据流

```
/ugv/lidar/points          ──→ pointcloud_filter ──→ /fusion/pointcloud_filtered
/uav/image_raw/compressed   ──→ image_rectify      ──→ /fusion/image_rectified
/uav/pose                   ──→ sfm                ──→ /fusion/sfm_points
/fusion/pointcloud_filtered ──→ icp_registration   ──→ /fusion/registered_cloud
/fusion/sfm_points          ──→ icp_registration
/fusion/registered_cloud    ──→ meshing            ──→ /fusion/mesh
```

### 5.3 参数配置

配置文件：`config/fusion_pipeline_params.yaml`

| 节点          | 关键参数                              |
|--------------|--------------------------------------|
| pointcloud_filter | voxel_leaf_size=0.05, sor_mean_k=50 |
| image_rectify | camera_matrix, dist_coeffs           |
| sfm          | keyframe_distance_thresh=5.0, min_inliers=20 |
| icp_registration | max_iterations=50, max_correspondence_distance=0.5 |
| meshing      | reconstruction_depth=8, mesh_resolution=0.1 |

### 5.4 降级策略

各节点在缺少可选依赖时自动降级：

| 依赖       | 主方案          | 降级方案      |
|-----------|----------------|--------------|
| Open3D    | point-to-plane ICP / 泊松重建 | numpy SVD ICP / 2.5D 网格 |
| OpenCV    | ORB 特征 + BFMatcher | 模拟匹配     |
| cv_bridge | CvBridge       | 跳过图像处理  |
| sensor_msgs_py | pc2 点云转换 | 跳过点云处理  |

### 5.5 启动

```bash
ros2 launch fusion_pipeline fusion_pipeline.launch.py
```

---

## 跨包通信矩阵

| 发布者          | Topic                         | 订阅者                   |
|----------------|-------------------------------|-------------------------|
| uav_sim        | /uav/pose                     | ground_station, sfm     |
| uav_sim        | /uav/status                   | ground_station          |
| uav_sim        | /uav/heartbeat                | ground_station          |
| uav_sim        | /uav/image_raw/compressed     | image_rectify           |
| ugv_sim        | /ugv/pose                     | ground_station          |
| ugv_sim        | /ugv/lidar/points             | pointcloud_filter       |
| ugv_sim        | /ugv/imu                      | —                       |
| ground_station | /ground_station/alert         | Web (WebSocket)         |
| image_rectify  | /fusion/image_rectified       | sfm                     |
| pointcloud_filter | /fusion/pointcloud_filtered | icp_registration        |
| sfm            | /fusion/sfm_points            | icp_registration        |
| icp_registration | /fusion/registered_cloud    | meshing                 |

---

## 依赖关系图

```
common_interfaces
    ├── uav_sim
    ├── ugv_sim
    ├── ground_station
    └── fusion_pipeline

uav_sim ────→ common_interfaces
ugv_sim ────→ common_interfaces
ground_station ─→ common_interfaces
fusion_pipeline → common_interfaces
```

---

## 开发指南

### 添加新消息类型

1. 在 `common_interfaces/msg/` 下创建 `.msg` 文件
2. 更新 `CMakeLists.txt` 中 `msg_files` 列表
3. 重新构建：`colcon build --packages-select common_interfaces`

### 添加新节点

1. 在对应包的 Python 目录下创建节点脚本
2. 在 `setup.py` 中注册 entry point：
   ```python
   entry_points={
       'console_scripts': [
           'my_node = my_package.my_node:main',
       ],
   }
   ```
3. 重新构建包

### 单元测试

测试文件位于 `tests/` 目录，覆盖：
- IMU 预积分模块
- Scan-to-Map ICP 匹配
- 因子图优化
- 数据管道各节点

运行测试：
```bash
python -m pytest tests/ -v
```
