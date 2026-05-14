#!/usr/bin/env python3
"""
四类场景仿真实验框架 (5.3)

场景定义：
  E1 - 城市街区（密集建筑+弱GNSS峡谷区）
  E2 - 矿区地形（陡坡+植被+不规则堆体）
  E3 - 山地丘陵（大高差+植被覆盖）
  E4 - 地质灾害应急（滑坡区域快速测绘）

评测指标：
  - ATE (Absolute Trajectory Error)
  - RPE (Relative Pose Error)
  - 点云配准 RMSE
  - 重叠区域密度比
  - 模型完整性 (完整性占比)
  - SSIM (结构相似性)
  - 全流程耗时
  - 成果交付时间
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import json
import time
import argparse
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from pathlib import Path

# ── 导入项目模块 ─────────────────────────────────────────────────
from src.slam.data_types import Pose3D, UAVTopoConfig, UGVFusionConfig
from src.slam.ugv_fusion import UGVMultiSensorFusion, IMUData, ScanData
from src.slam.collaborative_optimizer import CollaborativeOptimizer, CollaborativeConfig
from src.slam.coarse_registration import CoarseRegistrator
from src.slam.consistency import ConsistencyChecker
from src.slam.robustness import RobustnessEnhancer


# ══════════════════════════════════════════════════════════════════════
# 场景配置
# ══════════════════════════════════════════════════════════════════════

@dataclass
class SceneConfig:
    """仿真场景配置"""
    name: str                           # 场景名
    id: str                             # 场景编号 (E1-E4)
    description: str                    # 场景描述
    # 地形参数
    terrain_size: Tuple[float, float]   # (宽, 长) 米
    terrain_height_range: Tuple[float, float]  # (min, max)
    building_density: float = 0.0       # 建筑密度 [0, 1]
    vegetation_density: float = 0.0     # 植被密度 [0, 1]
    # GNSS 条件
    gnss_availability: float = 1.0      # GNSS 可用性 [0, 1]
    gnss_noise_m: float = 1.0           # GNSS 噪声 (m)
    # UAV 航线参数
    uav_altitude_m: float = 100.0       # 飞行高度
    uav_speed_ms: float = 8.0           # 飞行速度
    uav_coverage_area: float = 50000    # 覆盖面积 (m²)
    # UGV 路径参数
    ugv_path_length_m: float = 500.0    # 路径长度
    ugv_speed_ms: float = 2.0           # 行驶速度
    # 视觉条件
    texture_richness: float = 0.8       # 纹理丰富度 [0, 1]
    # 核心评估指标
    primary_metrics: List[str] = field(default_factory=list)


# ══════════════════════════════════════════════════════════════════════
# 四类场景定义
# ══════════════════════════════════════════════════════════════════════

SCENES = {
    "E1": SceneConfig(
        name="城市街区",
        id="E1",
        description="密集建筑+弱GNSS峡谷区",
        terrain_size=(500, 500),
        terrain_height_range=(0, 80),
        building_density=0.7,
        vegetation_density=0.1,
        gnss_availability=0.4,
        gnss_noise_m=5.0,
        uav_altitude_m=120,
        uav_speed_ms=10.0,
        uav_coverage_area=80000,
        ugv_path_length_m=800,
        ugv_speed_ms=2.5,
        texture_richness=0.6,
        primary_metrics=["ATE", "RPE"]
    ),
    "E2": SceneConfig(
        name="矿区地形",
        id="E2",
        description="陡坡+植被+不规则堆体",
        terrain_size=(400, 400),
        terrain_height_range=(-20, 100),
        building_density=0.05,
        vegetation_density=0.4,
        gnss_availability=0.7,
        gnss_noise_m=2.0,
        uav_altitude_m=80,
        uav_speed_ms=6.0,
        uav_coverage_area=40000,
        ugv_path_length_m=600,
        ugv_speed_ms=1.8,
        texture_richness=0.5,
        primary_metrics=["点云配准 RMSE", "重叠区域密度比"]
    ),
    "E3": SceneConfig(
        name="山地丘陵",
        id="E3",
        description="大高差+植被覆盖",
        terrain_size=(600, 600),
        terrain_height_range=(0, 300),
        building_density=0.02,
        vegetation_density=0.7,
        gnss_availability=0.6,
        gnss_noise_m=3.0,
        uav_altitude_m=150,
        uav_speed_ms=8.0,
        uav_coverage_area=100000,
        ugv_path_length_m=700,
        ugv_speed_ms=2.0,
        texture_richness=0.4,
        primary_metrics=["模型完整性", "SSIM"]
    ),
    "E4": SceneConfig(
        name="地质灾害应急",
        id="E4",
        description="滑坡区域快速测绘",
        terrain_size=(300, 300),
        terrain_height_range=(-30, 80),
        building_density=0.0,
        vegetation_density=0.3,
        gnss_availability=0.3,
        gnss_noise_m=8.0,
        uav_altitude_m=60,
        uav_speed_ms=12.0,
        uav_coverage_area=20000,
        ugv_path_length_m=300,
        ugv_speed_ms=3.0,
        texture_richness=0.3,
        primary_metrics=["全流程耗时", "成果交付时间"]
    ),
}


# ══════════════════════════════════════════════════════════════════════
# 仿真数据生成器
# ══════════════════════════════════════════════════════════════════════

class SimulationDataGenerator:
    """为每个场景生成仿真数据集"""

    def __init__(self, scene: SceneConfig, seed: int = 42):
        self.scene = scene
        self.rng = np.random.RandomState(seed)

    def generate_uav_trajectory(self) -> List[Pose3D]:
        """生成 UAV 飞行轨迹（蛇形航线）"""
        traj = []
        area = self.scene.uav_coverage_area
        side = np.sqrt(area)
        altitude = self.scene.uav_altitude_m
        speed = self.scene.uav_speed_ms

        # 蛇形航线：往返覆盖区域
        line_spacing = 30  # 航线间距 (m)
        num_lines = int(side / line_spacing)

        for i in range(num_lines):
            y = i * line_spacing
            x_start = 0 if i % 2 == 0 else side
            x_end = side if i % 2 == 0 else 0
            steps = int(side / (speed * 1.0))

            for s in range(steps + 1):
                t = s / steps
                x = x_start + (x_end - x_start) * t
                z = altitude + self.rng.randn() * 2.0  # 气压高度噪声

                # 姿态（机头朝向飞行方向）
                heading = 0 if i % 2 == 0 else np.pi
                R = np.array([
                    [np.cos(heading), -np.sin(heading), 0],
                    [np.sin(heading),  np.cos(heading), 0],
                    [0, 0, 1]
                ])
                traj.append(Pose3D(R=R, t=np.array([x, y, z])))

        return traj

    def generate_ugv_trajectory(self) -> List[Pose3D]:
        """生成 UGV 行驶轨迹（沿地面路径）"""
        traj = []
        path_length = self.scene.ugv_path_length_m
        speed = self.scene.ugv_speed_ms
        steps = int(path_length / (speed * 0.1))

        for i in range(steps + 1):
            t = i / steps
            x = path_length * t * 0.7 + self.rng.randn() * 0.5
            y = path_length * t * 0.5 * np.sin(t * np.pi * 2)
            z = 0.0 + self.rng.randn() * 0.1  # 地面高度噪声

            heading = np.arctan2(
                np.cos(t * np.pi * 2) * 0.5,
                0.7
            )
            R = np.array([
                [np.cos(heading), -np.sin(heading), 0],
                [np.sin(heading),  np.cos(heading), 0],
                [0, 0, 1]
            ])
            traj.append(Pose3D(R=R, t=np.array([x, y, z])))

        return traj

    def generate_ugv_sensor_data(self, ugv_traj: List[Pose3D]) -> Tuple[
        List[ScanData], List[IMUData], List[Pose3D]]:
        """为 UGV 轨迹生成 IMU + LiDAR 传感器数据"""
        scans = []
        imus = []
        gnss = []

        for i, pose in enumerate(ugv_traj):
            # LiDAR: 生成地面扫描点云
            n_points = 2000
            half_size = 30.0
            points = self.rng.rand(n_points, 3) * half_size * 2 - half_size
            points[:, 2] = np.abs(points[:, 2]) * 0.1  # 地面附近
            # 变换到全局坐标
            points = (pose.R @ points.T).T + pose.t
            scans.append(ScanData(points=points, timestamp=i * 0.1))

            # IMU: 生成加速度和角速度
            accel = np.array([0.0, 0.0, 9.81]) + self.rng.randn(3) * 0.01
            gyro = self.rng.randn(3) * 0.001
            imus.append(IMUData(accel=accel, gyro=gyro, timestamp=i * 0.01))

            # GNSS: 生成带噪声的绝对位置
            gnss_noise = self.rng.randn(3) * self.scene.gnss_noise_m
            gnss_pose = Pose3D(
                R=np.eye(3),
                t=pose.t + gnss_noise * (1 - self.scene.gnss_availability + 0.01)
            )
            gnss.append(gnss_pose)

        return scans, imus, gnss

    def generate_point_clouds(self) -> Tuple[np.ndarray, np.ndarray]:
        """生成模拟 UAV SfM 和 UGV LiDAR 点云"""
        n_uav = 5000
        n_ugv = 10000

        area = np.sqrt(self.scene.uav_coverage_area)

        # UAV SfM 点云（从空中视角）
        uav_points = self.rng.rand(n_uav, 3)
        uav_points[:, 0] *= area
        uav_points[:, 1] *= area
        uav_points[:, 2] = self.scene.terrain_height_range[0] + \
            self.rng.rand(n_uav) * (self.scene.terrain_height_range[1] -
                                     self.scene.terrain_height_range[0]) * 0.3

        # UGV LiDAR 点云（地面视角，更密集的地表信息）
        ugv_points = self.rng.rand(n_ugv, 3)
        ugv_points[:, 0] *= area * 0.8
        ugv_points[:, 1] *= area * 0.8
        ugv_points[:, 2] = np.abs(self.rng.randn(n_ugv)) * 5.0  # 地表附近

        return uav_points, ugv_points


# ══════════════════════════════════════════════════════════════════════
# 评测指标计算
# ══════════════════════════════════════════════════════════════════════

class MetricsComputer:
    """评测指标计算器"""

    def compute_ate(self, estimated: np.ndarray, ground_truth: np.ndarray) -> float:
        """计算绝对轨迹误差 (ATE)"""
        if len(estimated) == 0 or len(ground_truth) == 0:
            return float('inf')
        n = min(len(estimated), len(ground_truth))
        errors = np.linalg.norm(estimated[:n] - ground_truth[:n], axis=1)
        return float(np.sqrt(np.mean(errors ** 2)))

    def compute_rpe(self, estimated: np.ndarray, ground_truth: np.ndarray,
                     delta: int = 1) -> float:
        """计算相对位姿误差 (RPE)"""
        if len(estimated) < delta + 1 or len(ground_truth) < delta + 1:
            return float('inf')
        n = min(len(estimated), len(ground_truth)) - delta
        errors = []
        for i in range(n):
            est_delta = np.linalg.norm(estimated[i + delta] - estimated[i])
            gt_delta = np.linalg.norm(ground_truth[i + delta] - ground_truth[i])
            errors.append(abs(est_delta - gt_delta))
        return float(np.mean(errors))

    def compute_pointcloud_rmse(self, source: np.ndarray, target: np.ndarray,
                                 max_samples: int = 2000) -> float:
        """计算点云配准 RMSE"""
        if len(source) == 0 or len(target) == 0:
            return float('inf')

        n = min(len(source), max_samples)
        idx = np.random.choice(len(source), n, replace=False)
        src = source[idx]

        min_dists = []
        chunk_size = 500
        for i in range(0, n, chunk_size):
            chunk = src[i:i + chunk_size]
            diffs = chunk[:, None, :] - target[None, :, :]
            dists = np.linalg.norm(diffs, axis=2)
            min_dists.append(np.min(dists, axis=1))

        all_dists = np.concatenate(min_dists)
        return float(np.sqrt(np.mean(all_dists ** 2)))

    def compute_overlap_density_ratio(self, cloud_a: np.ndarray,
                                       cloud_b: np.ndarray,
                                       radius: float = 1.0) -> float:
        """计算重叠区域密度比"""
        if len(cloud_a) == 0 or len(cloud_b) == 0:
            return 0.0

        # 对 cloud_a 中每个点，检查 cloud_b 中是否有近邻
        n_samples = min(500, len(cloud_a))
        idx = np.random.choice(len(cloud_a), n_samples, replace=False)
        samples = cloud_a[idx]

        overlap_count = 0
        for pt in samples:
            dists = np.linalg.norm(cloud_b - pt, axis=1)
            if np.min(dists) < radius:
                overlap_count += 1

        return overlap_count / n_samples

    def compute_model_completeness(self, filled_area: float,
                                    total_area: float) -> float:
        """计算模型完整性（填充比例）"""
        if total_area == 0:
            return 0.0
        return min(1.0, filled_area / total_area)

    def compute_ssim(self, reference: np.ndarray, reconstructed: np.ndarray) -> float:
        """计算结构相似性指数（简化版）

        对于点云的 2D 投影进行 SSIM 计算。
        """
        try:
            # 投影到 2D 高度图
            size = 256
            ref_map = self._points_to_heightmap(reference, size)
            rec_map = self._points_to_heightmap(reconstructed, size)

            # 计算 SSIM
            c1, c2 = 0.01, 0.03
            mu_x = np.mean(ref_map)
            mu_y = np.mean(rec_map)
            sigma_x = np.var(ref_map)
            sigma_y = np.var(rec_map)
            sigma_xy = np.mean((ref_map - mu_x) * (rec_map - mu_y))

            numerator = (2 * mu_x * mu_y + c1) * (2 * sigma_xy + c2)
            denominator = (mu_x ** 2 + mu_y ** 2 + c1) * (sigma_x + sigma_y + c2)
            return float(numerator / denominator) if denominator > 0 else 0.0
        except Exception:
            return 0.0

    @staticmethod
    def _points_to_heightmap(points: np.ndarray, size: int) -> np.ndarray:
        """点云 → 2D 高度图"""
        if len(points) == 0:
            return np.zeros((size, size))

        heightmap = np.zeros((size, size))
        # 归一化坐标到 [0, size-1]
        x_min, x_max = points[:, 0].min(), points[:, 0].max()
        y_min, y_max = points[:, 1].min(), points[:, 1].max()

        if x_max - x_min < 1e-6 or y_max - y_min < 1e-6:
            return heightmap

        xi = ((points[:, 0] - x_min) / (x_max - x_min) * (size - 1)).astype(int)
        yi = ((points[:, 1] - y_min) / (y_max - y_min) * (size - 1)).astype(int)
        zi = points[:, 2]

        np.add.at(heightmap, (yi, xi), zi)
        count = np.zeros((size, size))
        np.add.at(count, (yi, xi), 1)
        count[count == 0] = 1
        heightmap /= count

        return heightmap


# ══════════════════════════════════════════════════════════════════════
# 实验执行器
# ══════════════════════════════════════════════════════════════════════

class ExperimentRunner:
    """场景实验执行器"""

    def __init__(self, scene_id: str, output_dir: str = "./experiment_results"):
        self.scene_id = scene_id
        self.scene = SCENES[scene_id]
        self.generator = SimulationDataGenerator(self.scene)
        self.metrics = MetricsComputer()
        self.output_dir = Path(output_dir) / scene_id
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.results: Dict = {}

    def run(self) -> Dict:
        """执行完整实验流程"""
        print(f"\n{'='*60}")
        print(f"  场景 {self.scene.id}: {self.scene.name}")
        print(f"  描述: {self.scene.description}")
        print(f"{'='*60}\n")

        t0 = time.time()
        results = {
            "scene": self.scene.id,
            "name": self.scene.name,
            "description": self.scene.description,
            "timestamp": time.time(),
            "config": {
                "terrain_size": self.scene.terrain_size,
                "gnss_availability": self.scene.gnss_availability,
                "uav_coverage_area": self.scene.uav_coverage_area,
                "ugv_path_length": self.scene.ugv_path_length_m,
            }
        }

        # ── 1. 生成仿真数据 ────────────────────────────────────────
        print("  1. 生成仿真数据...")
        uav_traj = self.generator.generate_uav_trajectory()
        ugv_traj = self.generator.generate_ugv_trajectory()
        scans, imus, gnss = self.generator.generate_ugv_sensor_data(ugv_traj)
        uav_points, ugv_points = self.generator.generate_point_clouds()

        results["data"] = {
            "uav_trajectory_points": len(uav_traj),
            "ugv_trajectory_points": len(ugv_traj),
            "scans": len(scans),
            "imus": len(imus),
            "uav_point_cloud_size": len(uav_points),
            "ugv_point_cloud_size": len(ugv_points),
        }

        # ── 2. UGV 多传感器融合 ────────────────────────────────────
        print("  2. 执行 UGV 多传感器融合...")
        ugv_fusion = UGVMultiSensorFusion()

        estimated_ugv_traj = []
        for i in range(min(len(scans), len(imus))):
            ugv_fusion.process_imu(imus[i])
            if i % 10 == 0:  # 10Hz LiDAR
                pose = ugv_fusion.process_lidar(scans[i])
                if pose is not None:
                    estimated_ugv_traj.append(pose.t.copy())

        estimated_ugv_traj = np.array(estimated_ugv_traj) if estimated_ugv_traj else np.empty((0, 3))
        gt_ugv_traj = np.array([p.t for p in ugv_traj])

        # ── 3. 协同优化 ─────────────────────────────────────────────
        print("  3. 执行空地协同优化...")
        collab_optimizer = CollaborativeOptimizer()
        for i in range(min(len(uav_traj), len(ugv_traj), 100)):
            collab_optimizer.add_sync_pair(uav_traj[i], ugv_traj[i])

        T_calib = collab_optimizer.calibrate()

        # ── 4. 粗配准 ──────────────────────────────────────────────
        print("  4. 执行粗配准...")
        coarse_reg = CoarseRegistrator()
        uav_aligned, ugv_aligned, T_coarse = coarse_reg.align_point_clouds(
            uav_points, ugv_points, uav_traj, ugv_traj, T_uav2ugv=T_calib
        )

        # ── 5. 计算评测指标 ────────────────────────────────────────
        print("  5. 计算评测指标...")
        metrics_result = {}

        # ATE
        if len(estimated_ugv_traj) > 0 and len(gt_ugv_traj) > 0:
            n = min(len(estimated_ugv_traj), len(gt_ugv_traj))
            ate = self.metrics.compute_ate(estimated_ugv_traj[:n], gt_ugv_traj[:n])
            metrics_result["ATE_m"] = round(ate, 3)
        else:
            metrics_result["ATE_m"] = None

        # RPE
        if len(estimated_ugv_traj) > 10:
            rpe = self.metrics.compute_rpe(estimated_ugv_traj, gt_ugv_traj[:len(estimated_ugv_traj)])
            metrics_result["RPE_m"] = round(rpe, 3)
        else:
            metrics_result["RPE_m"] = None

        # 点云配准 RMSE
        rmse = self.metrics.compute_pointcloud_rmse(uav_aligned, ugv_points)
        metrics_result["pointcloud_rmse_m"] = round(rmse, 3)
        metrics_result["pointcloud_rmse_cm"] = round(rmse * 100, 1)

        # 重叠区域密度比
        overlap_ratio = self.metrics.compute_overlap_density_ratio(uav_aligned, ugv_points)
        metrics_result["overlap_density_ratio"] = round(overlap_ratio, 4)

        # 模型完整性
        completeness = self.metrics.compute_model_completeness(
            len(uav_aligned), len(uav_aligned) + len(ugv_points))
        metrics_result["model_completeness"] = round(completeness, 4)

        # SSIM
        ssim = self.metrics.compute_ssim(uav_aligned, ugv_points)
        metrics_result["SSIM"] = round(ssim, 4)

        # 全流程耗时
        elapsed = time.time() - t0
        metrics_result["total_time_s"] = round(elapsed, 3)

        # 模拟成果交付时间（包含数据传输和处理）
        metrics_result["delivery_time_min"] = round(
            (self.scene.ugv_path_length_m / self.scene.ugv_speed_ms) / 60 +
            elapsed / 60, 1
        )

        results["metrics"] = metrics_result

        # ── 6. 保存结果 ────────────────────────────────────────────
        print("  6. 保存实验结果...")
        # 保存点云数据包
        np.savez_compressed(
            self.output_dir / "point_clouds.npz",
            uav_original=uav_points,
            ugv_original=ugv_points,
            uav_aligned=uav_aligned
        )

        # 保存评测指标
        with open(self.output_dir / "metrics.json", 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False, default=str)

        print(f"\n  ✓ 实验结果已保存至: {self.output_dir}")
        print(f"  ── 核心指标 ──")
        print(f"    ATE: {metrics_result.get('ATE_m', 'N/A')} m")
        print(f"    RPE: {metrics_result.get('RPE_m', 'N/A')} m")
        print(f"    点云 RMSE: {metrics_result.get('pointcloud_rmse_cm', 'N/A')} cm")
        print(f"    重叠密度比: {metrics_result.get('overlap_density_ratio', 'N/A')}")
        print(f"    模型完整性: {metrics_result.get('model_completeness', 'N/A')}")
        print(f"    SSIM: {metrics_result.get('SSIM', 'N/A')}")
        print(f"    全流程耗时: {metrics_result.get('total_time_s', 'N/A')} s")
        print(f"    成果交付时间: {metrics_result.get('delivery_time_min', 'N/A')} min")

        self.results = results
        return results


# ══════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════

def run_all_scenes(output_dir: str = "./experiment_results") -> Dict[str, Dict]:
    """运行全部四类场景"""
    all_results = {}

    for scene_id in ["E1", "E2", "E3", "E4"]:
        runner = ExperimentRunner(scene_id, output_dir)
        result = runner.run()
        all_results[scene_id] = result

    # 生成汇总报告
    summary = {
        "timestamp": time.time(),
        "scenes": {}
    }
    for sid, res in all_results.items():
        summary["scenes"][sid] = {
            "name": res["name"],
            "metrics": res.get("metrics", {})
        }

    summary_path = Path(output_dir) / "summary.json"
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False, default=str)

    print(f"\n{'='*60}")
    print(f"  全部场景实验完成！汇总报告: {summary_path}")
    print(f"{'='*60}")

    return all_results


def run_single_scene(scene_id: str, output_dir: str = "./experiment_results") -> Dict:
    """运行单个场景"""
    if scene_id not in SCENES:
        raise ValueError(f"未知场景 ID: {scene_id}，可选: {list(SCENES.keys())}")

    runner = ExperimentRunner(scene_id, output_dir)
    return runner.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='四类场景仿真实验')
    parser.add_argument('--scene', type=str, default=None,
                        help='场景ID (E1/E2/E3/E4)，不指定则运行全部')
    parser.add_argument('--output', type=str, default='./experiment_results',
                        help='输出目录')

    args = parser.parse_args()

    if args.scene:
        run_single_scene(args.scene, args.output)
    else:
        run_all_scenes(args.output)
