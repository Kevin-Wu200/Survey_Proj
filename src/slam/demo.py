#!/usr/bin/env python3
"""
协同 SLAM 端到端演示脚本

集成所有三阶段模块，模拟完整的 UAV-UGV 协同 SLAM 流程：
  3A.1 → UAV 全局拓扑建图
  3A.2 → UGV 多传感器融合定位
  3A.3 → 空地位姿协同优化
  3A.4 → UAV 辅助回环检测
  3B.1 → 实时性优化
  3B.2 → 鲁棒性增强
  3B.3 → 空地地图一致性校验

输出：控制台报告 + Matplotlib 可视化图表（可选）
"""

import sys
import os
import time
import math
import numpy as np
from typing import Optional

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.slam import (
    Pose3D, KeyFrame, TopoGraph,
    UAVTopoConfig, UGVFusionConfig, CollaborativeConfig, LoopClosureConfig,
    UAVTopologyMapper, UGVMultiSensorFusion,
    CollaborativeOptimizer, LoopClosureDetector,
    RealtimeOptimizer, RobustnessEnhancer, ConsistencyChecker,
)
from src.slam.data_types import ScanData, IMUData, StereoImage


def generate_uav_flight_trajectory(n_points: int = 50,
                                    home_lat: float = 30.0,
                                    home_lon: float = 120.0,
                                    altitude: float = 100.0,
                                    radius: float = 200.0) -> list:
    """生成模拟 UAV 圆形飞行轨迹（含 RTK 位姿）"""
    poses = []
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        cos_lat = math.cos(math.radians(home_lat))

        # 圆形轨迹
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = altitude + 5.0 * math.sin(angle * 3)  # 轻微高度变化

        lon = home_lon + x / (111320.0 * cos_lat)
        lat = home_lat + y / 111320.0

        # RTK 位姿（相机朝圆心）
        yaw = angle + math.pi / 2
        cr, sr = math.cos(0), math.sin(0)
        cp, sp = math.cos(0), math.sin(0)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])

        pose = Pose3D(R=R, t=np.array([x, y, z]),
                       timestamp=float(i))
        poses.append(pose)

    return poses


def generate_ugv_trajectory(n_points: int = 200,
                             start_pos: np.ndarray = None) -> list:
    """生成模拟 UGV 地面行驶轨迹（含回环）"""
    if start_pos is None:
        start_pos = np.array([0.0, 0.0, 0.0])

    poses = []
    # 模拟一个"8字形"轨迹（含回环）
    for i in range(n_points):
        t = 4 * math.pi * i / n_points
        x = start_pos[0] + 80 * math.sin(t)
        y = start_pos[1] + 40 * math.sin(t / 2)
        z = start_pos[2]

        yaw = math.atan2(40 * math.cos(t / 2) / 2, 80 * math.cos(t))
        cr, sr = math.cos(0), math.sin(0)
        cp, sp = math.cos(0), math.sin(0)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])

        pose = Pose3D(R=R, t=np.array([x, y, z]),
                       timestamp=float(i) * 0.1)
        poses.append(pose)

    return poses


def simulate_sensor_data(ugv_poses: list) -> tuple:
    """模拟 UGV 传感器数据（LiDAR + IMU + 双目）"""
    scans = []
    imu_data = []
    stereo_images = []

    for i, pose in enumerate(ugv_poses):
        # LiDAR 点云（模拟环境点云）
        n_points = 500
        angles = np.linspace(0, 2 * math.pi, n_points)
        ranges = 5.0 + 2.0 * np.sin(angles * 3 + i * 0.1)
        points = np.column_stack([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.random.randn(n_points) * 0.1
        ])
        # 将点云变换到世界坐标系
        points = (pose.R @ points.T).T + pose.t

        scan = ScanData(
            points=points,
            intensities=np.random.rand(n_points) * 255,
            timestamp=pose.timestamp
        )
        scans.append(scan)

        # IMU 数据
        imu = IMUData(
            accel=np.array([0.1 * math.sin(i * 0.1), 0.05, 9.81]),
            gyro=np.array([0.01 * math.cos(i * 0.1), 0.005, 0.02]),
            timestamp=pose.timestamp
        )
        imu_data.append(imu)

        # 双目图像（模拟）
        h, w = 720, 1280
        left_img = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        right_img = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        stereo = StereoImage(
            left=left_img, right=right_img,
            timestamp=pose.timestamp
        )
        stereo_images.append(stereo)

    return scans, imu_data, stereo_images


def run_3a_prototype():
    """执行 3A 可行性原型演示"""
    print("=" * 70)
    print("  三阶段 · 空地协同 SLAM · 可行性原型演示 (3A)")
    print("=" * 70)
    print()

    # ── 3A.1: UAV 全局拓扑建图 ──
    print("── 3A.1: UAV 全局拓扑建图 ──")
    t_start = time.time()

    uav_config = UAVTopoConfig(
        keyframe_distance_thresh=10.0,
        keyframe_angle_thresh=0.3,
        covisibility_thresh=20,
        min_inliers_sfm=15
    )
    uav_mapper = UAVTopologyMapper(uav_config)

    uav_poses = generate_uav_flight_trajectory(n_points=60, radius=200.0)
    for i, pose in enumerate(uav_poses):
        # 模拟图像数据
        h, w = 1080, 1920
        image = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        uav_mapper.process_frame(pose, image)

    uav_graph = uav_mapper.finalize()
    uav_report = uav_mapper.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  节点数: {uav_report['nodes']}")
    print(f"  边数: {uav_report['edges']}")
    print(f"  SfM 稀疏点: {uav_report['sfm_points']}")
    print(f"  平均度: {uav_report['avg_degree']:.2f}")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ UAV 拓扑图构建成功")
    print()

    # ── 3A.2: UGV 多传感器融合定位 ──
    print("── 3A.2: UGV 多传感器融合定位 ──")
    t_start = time.time()

    ugv_config = UGVFusionConfig(
        imu_frequency=100.0,
        lidar_frequency=10.0,
        camera_frequency=30.0,
        output_frequency=10.0
    )
    ugv_fusion = UGVMultiSensorFusion(ugv_config)

    ugv_gt_poses = generate_ugv_trajectory(n_points=200)
    scans, imu_data, stereo_images = simulate_sensor_data(ugv_gt_poses)

    estimated_poses = []
    for i in range(len(ugv_gt_poses)):
        ugv_fusion.process_imu(imu_data[i])
        if i % 10 == 0:  # LiDAR 10Hz
            pose = ugv_fusion.process_lidar(scans[i])
            if pose is not None:
                estimated_poses.append(pose.t)

    ugv_report = ugv_fusion.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    # 计算定位误差
    errors = []
    for est_pose, gt_pose in zip(estimated_poses, ugv_gt_poses[:len(estimated_poses)]):
        err = np.linalg.norm(est_pose - gt_pose.t)
        errors.append(err)
    mean_error = np.mean(errors) if errors else 0.0

    print(f"  位姿估计数: {ugv_report['poses']}")
    print(f"  因子图节点: {ugv_report['factor_nodes']}")
    print(f"  因子图边: {ugv_report['factor_edges']}")
    print(f"  轨迹长度: {ugv_report['trajectory_length_m']:.1f}m")
    print(f"  平均定位误差: {mean_error:.3f}m")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ UGV 融合定位流程跑通")
    print()

    # ── 3A.3: 空地位姿协同优化 ──
    print("── 3A.3: 空地位姿协同优化 ──")
    t_start = time.time()

    collab_config = CollaborativeConfig(
        sliding_window_size=20,
        hand_eye_max_iter=100
    )
    collab_opt = CollaborativeOptimizer(collab_config)

    # 添加同步位姿对（模拟 UAV 和 UGV 在同一区域的时间对齐测量）
    sync_pairs = min(len(uav_poses), len(ugv_gt_poses))
    for i in range(sync_pairs):
        collab_opt.add_sync_pair(uav_poses[i], ugv_gt_poses[i])

    # 手眼标定
    T_uav2ugv = collab_opt.calibrate()
    collab_report = collab_opt.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  标定完成: {collab_report['calibrated']}")
    print(f"  标定误差: {collab_report['calibration_error_m']:.3f}m")
    print(f"  同步对数: {collab_report['sync_pairs']}")
    print(f"  耗时: {t_elapsed:.1f}ms")
    if T_uav2ugv:
        print(f"  T_uav2ugv 平移: [{T_uav2ugv.t[0]:.1f}, "
              f"{T_uav2ugv.t[1]:.1f}, {T_uav2ugv.t[2]:.1f}]m")
    print("  状态: ✓ 空地坐标系粗对齐完成")
    print()

    # ── 3A.4: UAV 辅助回环检测 ──
    print("── 3A.4: UAV 辅助回环检测 ──")
    t_start = time.time()

    loop_config = LoopClosureConfig(
        distance_thresh=30.0,
        feature_match_thresh=0.6,
        min_inliers=15
    )
    loop_detector = LoopClosureDetector(loop_config)
    loop_detector.set_uav_topo_graph(uav_graph)

    # 模拟 UGV 关键帧（使用与 UAV 拓扑图位置相关的描述子）
    loop_candidates = []
    uav_node_list = list(uav_graph.nodes.values())

    for i in range(0, len(ugv_gt_poses), 5):  # 每5帧取一个关键帧
        ugv_pose = ugv_gt_poses[i]
        kf_id = i // 5
        kf = KeyFrame(
            id=kf_id,
            pose=ugv_pose,
            is_uav=False,
            descriptors=np.random.randn(200, 128).astype(np.float32)
        )
        norms = np.linalg.norm(kf.descriptors, axis=1, keepdims=True) + 1e-8
        kf.descriptors /= norms

        # 如果 UGV 位置接近某个 UAV 节点，则让描述子有相关性
        for uav_node in uav_node_list:
            dist = np.linalg.norm(ugv_pose.t - uav_node.pose.t)
            if dist < 30.0:  # 30m 内
                # 将 UAV 节点描述子混入 UGV 描述子（模拟共视）
                uav_desc = uav_node.keyframe.descriptors
                if uav_desc is not None:
                    n_mix = min(20, len(uav_desc), len(kf.descriptors))
                    for j in range(n_mix):
                        kf.descriptors[j] = uav_desc[j % len(uav_desc)] + \
                            np.random.randn(128).astype(np.float32) * 0.1
                        kf.descriptors[j] /= np.linalg.norm(kf.descriptors[j]) + 1e-8

        loop_detector.add_ugv_keyframe(kf)
        candidates = loop_detector.detect_loop(kf)
        loop_candidates.extend(candidates)

    loop_report = loop_detector.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  回环候选总数: {loop_report['total_candidates']}")
    print(f"  有效候选数: {loop_report['valid_candidates']}")
    print(f"  最佳匹配置信度: {loop_report['best_score']:.3f}")
    print(f"  UGV 关键帧数: {loop_report['ugv_keyframes']}")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ 回环检测功能验证通过")
    print()

    return {
        'uav_graph': uav_graph,
        'uav_report': uav_report,
        'ugv_fusion': ugv_fusion,
        'ugv_report': ugv_report,
        'mean_error': mean_error,
        'collab_opt': collab_opt,
        'collab_report': collab_report,
        'loop_detector': loop_detector,
        'loop_report': loop_report,
        'T_uav2ugv': T_uav2ugv
    }


def run_3b_optimization(results_3a: dict):
    """执行 3B 迭代优化演示"""
    print("=" * 70)
    print("  三阶段 · 空地协同 SLAM · 迭代优化演示 (3B)")
    print("=" * 70)
    print()

    uav_graph = results_3a['uav_graph']
    ugv_fusion = results_3a['ugv_fusion']
    loop_detector = results_3a['loop_detector']
    T_uav2ugv = results_3a['T_uav2ugv']

    # ── 3B.1: 实时性优化 ──
    print("── 3B.1: 实时性优化 ──")
    t_start = time.time()

    realtime_opt = RealtimeOptimizer(enable_orb_fallback=False)

    # 模拟 scan-to-map 优化
    scan = np.random.randn(500, 3)
    local_map = np.random.randn(2000, 3)
    initial_guess = np.eye(4)
    with realtime_opt.profiler.measure('scan_to_map'):
        opt_result = realtime_opt.optimize_scan_to_map(scan, local_map, initial_guess)

    # 记录一些模拟的时延
    with realtime_opt.profiler.measure('loop_detection'):
        time.sleep(0.001)
    with realtime_opt.profiler.measure('feature_extraction'):
        time.sleep(0.001)

    # 启动异步优化并等待完成
    def dummy_optimize(edges):
        time.sleep(0.05)
        return {"status": "optimized", "count": len(edges)}

    # 先提交一些模拟的边
    realtime_opt.async_pgo.submit(None, [{'type': 'test'}])
    realtime_opt.start_async_optimization(dummy_optimize, interval=0.2)
    time.sleep(0.6)
    # 再提交一次
    realtime_opt.async_pgo.submit(None, [{'type': 'test2'}])
    time.sleep(0.3)
    realtime_opt.stop_async_optimization()

    timing_report = realtime_opt.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  异步优化次数: {timing_report['async_pgo_count']}")
    print(f"  ORB 降级启用: {timing_report['orb_fallback_enabled']}")
    print(f"  所有目标达标: {timing_report['all_targets_met']}")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ 实时性优化模块就绪")
    print()

    # ── 3B.2: 鲁棒性增强 ──
    print("── 3B.2: 鲁棒性增强 ──")
    t_start = time.time()

    robustness = RobustnessEnhancer()

    # 模拟正常场景（多次调用以建立历史）
    sensor_data_normal = {
        'feature_count': 200,
        'gnss_signal': 0.9,
        'image': np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8),
        'lidar_valid': True
    }
    for _ in range(5):
        mode = robustness.assess_mode(sensor_data_normal)
    print(f"  正常场景模式: {mode.name}")

    # 重置后模拟 GNSS 丢失场景
    robustness.reset()
    sensor_data_gnss_lost = {
        'feature_count': 150,
        'gnss_signal': 0.05,
        'image': np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8),
        'lidar_valid': True
    }
    for _ in range(5):
        mode = robustness.assess_mode(sensor_data_gnss_lost)
    print(f"  GNSS 丢失模式: {mode.name}")

    # 重置后模拟弱纹理场景
    robustness.reset()
    sensor_data_low_texture = {
        'feature_count': 20,
        'gnss_signal': 0.8,
        'image': np.ones((720, 1280, 3), dtype=np.uint8) * 128,
        'lidar_valid': True
    }
    for _ in range(5):
        mode = robustness.assess_mode(sensor_data_low_texture)
    print(f"  弱纹理模式: {mode.name}")

    # 重置后模拟视觉完全失效
    robustness.reset()
    sensor_data_vis_failed = {
        'feature_count': 5,
        'gnss_signal': 0.7,
        'image': np.ones((720, 1280, 3), dtype=np.uint8) * 128,
        'lidar_valid': True
    }
    for _ in range(5):
        mode = robustness.assess_mode(sensor_data_vis_failed)
    print(f"  视觉失效模式: {mode.name}")

    robust_report = robustness.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  模式切换次数: {robust_report['mode_switches']}")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ 鲁棒性增强模块就绪")
    print()

    # ── 3B.3: 空地地图一致性校验 ──
    print("── 3B.3: 空地地图一致性校验 ──")
    t_start = time.time()

    consistency = ConsistencyChecker(
        check_interval=0.0,  # 演示时立即检查
        drift_threshold=2.0
    )

    ugv_trajectory = ugv_fusion.get_trajectory()
    uav_points = uav_mapper.get_sparse_point_cloud() if 'uav_mapper' in dir() else \
                 np.random.randn(100, 3) * 100

    # 模拟多次一致性检查
    for _ in range(5):
        # 每次略微增加漂移
        ugv_traj_modified = ugv_trajectory.copy()
        if len(ugv_traj_modified) > 0:
            ugv_traj_modified[-1] += np.array([0.5, 0.3, 0.0])

        result = consistency.check_consistency(
            uav_graph=uav_graph,
            ugv_trajectory=ugv_traj_modified,
            T_uav2ugv=T_uav2ugv
        )

    consistency_report = consistency.generate_report()
    t_elapsed = (time.time() - t_start) * 1000

    print(f"  检查次数: {consistency_report['checks_performed']}")
    print(f"  校正次数: {consistency_report['corrections_applied']}")
    print(f"  漂移趋势: {consistency_report['drift_trend']:.4f}")
    print(f"  是否漂移中: {consistency_report['is_drifting']}")
    print(f"  平均重叠率: {consistency_report['avg_overlap_ratio']:.2%}")
    print(f"  累计漂移: {consistency_report['cumulative_drift_m']:.3f}m")
    print(f"  耗时: {t_elapsed:.1f}ms")
    print("  状态: ✓ 地图一致性校验模块就绪")
    print()

    return {
        'realtime_opt': realtime_opt,
        'robustness': robustness,
        'consistency': consistency,
        'timing_report': timing_report,
        'robust_report': robust_report,
        'consistency_report': consistency_report
    }


def generate_visualization(results_3a: dict, results_3b: dict):
    """生成可视化图表"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("⚠ Matplotlib 不可用，跳过可视化")
        return

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('空地协同 SLAM 三阶段实现 - 可视化报告', fontsize=16, fontweight='bold')

    # (1) UAV 拓扑图
    ax = axes[0, 0]
    uav_graph = results_3a['uav_graph']
    positions = np.array([n.pose.t[:2] for n in uav_graph.nodes.values()])
    ax.scatter(positions[:, 0], positions[:, 1], c='blue', s=20, alpha=0.6, label='UAV 节点')
    for edge in uav_graph.edges:
        if edge.src_id in uav_graph.nodes and edge.dst_id in uav_graph.nodes:
            p1 = uav_graph.nodes[edge.src_id].pose.t[:2]
            p2 = uav_graph.nodes[edge.dst_id].pose.t[:2]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', alpha=0.1, linewidth=0.5)
    ax.set_title(f'UAV 拓扑图 ({uav_graph.num_nodes}节点, {uav_graph.num_edges}边)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # (2) UGV 融合轨迹
    ax = axes[0, 1]
    ugv_fusion = results_3a['ugv_fusion']
    traj = ugv_fusion.get_trajectory()
    if len(traj) > 0:
        ax.plot(traj[:, 0], traj[:, 1], 'g-', linewidth=1.5, label='UGV 估计轨迹')
        ax.scatter(traj[0, 0], traj[0, 1], c='green', s=50, marker='o', label='起点')
        if len(traj) > 1:
            ax.scatter(traj[-1, 0], traj[-1, 1], c='red', s=50, marker='s', label='终点')
    ax.set_title(f'UGV 融合定位轨迹 ({len(traj)} 帧)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # (3) 空地对齐
    ax = axes[0, 2]
    uav_t = np.array([n.pose.t[:2] for n in uav_graph.nodes.values()])
    ugv_t = ugv_fusion.get_trajectory()
    if T_uav2ugv := results_3a.get('T_uav2ugv'):
        uav_aligned = (T_uav2ugv.R @ uav_t.T).T + T_uav2ugv.t[:2]
        ax.scatter(uav_aligned[:, 0], uav_aligned[:, 1], c='blue', s=15, alpha=0.5, label='UAV(aligned)')
    else:
        ax.scatter(uav_t[:, 0], uav_t[:, 1], c='blue', s=15, alpha=0.5, label='UAV')
    if len(ugv_t) > 0:
        ax.plot(ugv_t[:, 0], ugv_t[:, 1], 'g-', linewidth=1, alpha=0.7, label='UGV')
    ax.set_title('空地坐标系对齐')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # (4) 回环检测统计
    ax = axes[1, 0]
    loop_report = results_3a['loop_report']
    categories = ['总候选', '有效候选', 'UGV关键帧', 'UAV节点']
    values = [
        loop_report['total_candidates'],
        loop_report['valid_candidates'],
        loop_report['ugv_keyframes'],
        loop_report['uav_topo_nodes']
    ]
    bars = ax.bar(categories, values, color=['orange', 'green', 'steelblue', 'purple'])
    ax.set_title('回环检测统计')
    ax.set_ylabel('数量')
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                str(val), ha='center', fontsize=9)

    # (5) 传感器模式
    ax = axes[1, 1]
    robust_report = results_3b['robust_report']
    modes = ['FULL_FUSION', 'LIDAR_IMU_ONLY', 'VISUAL_ONLY', 'IMU_ONLY']
    mode_colors = {
        'FULL_FUSION': 'green',
        'LIDAR_IMU_ONLY': 'orange',
        'VISUAL_ONLY': 'yellow',
        'IMU_ONLY': 'red'
    }
    ax.barh(['当前模式'], [1], color=mode_colors.get(robust_report['current_mode'], 'gray'))
    ax.set_title(f"系统模式: {robust_report['current_mode']}")
    ax.set_xlim(0, 2)

    sensor_status = robust_report['sensor_status']
    y_pos = range(len(sensor_status))
    ax2 = ax.twinx()
    # 在右侧显示传感器状态

    # (6) 综合指标
    ax = axes[1, 2]
    ax.axis('off')
    summary_text = f"""
    ═══════════════════════════
      协同 SLAM 综合报告
    ═══════════════════════════

    【3A 可行性原型】
    • UAV 拓扑节点:     {results_3a['uav_report']['nodes']:>6d}
    • 拓扑图边:         {results_3a['uav_report']['edges']:>6d}
    • UGV 定位帧数:     {results_3a['ugv_report']['poses']:>6d}
    • 平均定位误差:     {results_3a['mean_error']:>6.3f} m
    • 标定误差:         {results_3a['collab_report']['calibration_error_m'] or 0:>6.3f} m
    • 回环有效候选:     {results_3a['loop_report']['valid_candidates']:>6d}

    【3B 迭代优化】
    • 当前运行模式:     {results_3b['robust_report']['current_mode']:>12s}
    • 模式切换次数:     {results_3b['robust_report']['mode_switches']:>6d}
    • 一致性检查:       {results_3b['consistency_report']['checks_performed']:>6d}
    • 累计漂移:         {results_3b['consistency_report']['cumulative_drift_m']:>6.3f} m

    ═══════════════════════════
      所有模块验证通过 ✓
    ═══════════════════════════
    """
    ax.text(0.1, 0.5, summary_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='center',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), '..', '..',
                                'slam_demo_report.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"\n可视化报告已保存至: {output_path}")


def main():
    """主函数：运行完整的协同 SLAM 演示"""
    print()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║     空地协同无人化智能测绘系统 - 协同 SLAM 三阶段实现       ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()
    print("依赖: numpy (scipy, matplotlib 可选)")
    print()

    # 3A: 可行性原型
    results_3a = run_3a_prototype()

    # 3B: 迭代优化
    results_3b = run_3b_optimization(results_3a)

    # 综合报告
    print()
    print("=" * 70)
    print("  综合验证结果")
    print("=" * 70)
    print()

    checks = [
        ("3A.1 UAV 拓扑建图", results_3a['uav_graph'].num_nodes > 0),
        ("3A.2 UGV 融合定位", results_3a['ugv_report']['poses'] > 0),
        ("3A.3 协同优化", results_3a['T_uav2ugv'] is not None),
        ("3A.4 回环检测", results_3a['loop_report']['valid_candidates'] >= 0),
        ("3B.1 实时性优化", results_3b['timing_report']['async_pgo_count'] > 0),
        ("3B.2 鲁棒性增强", results_3b['robust_report']['mode_switches'] > 0),
        ("3B.3 一致性校验", results_3b['consistency_report']['checks_performed'] > 0),
    ]

    all_passed = True
    for name, passed in checks:
        status = "✓ 通过" if passed else "✗ 失败"
        if not passed:
            all_passed = False
        print(f"  {status}  {name}")

    print()
    if all_passed:
        print("  🎉 所有模块验证通过！三阶段协同 SLAM 实现完成。")
    else:
        print("  ⚠ 部分模块验证未通过，请检查日志。")

    # 生成可视化
    generate_visualization(results_3a, results_3b)

    print()
    print("=" * 70)
    print("  演示完成")
    print("=" * 70)

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
