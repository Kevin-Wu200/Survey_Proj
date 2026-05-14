"""
粗配准模块 (Coarse Registration)

基于协同SLAM输出的位姿，将UAV SfM点云与UGV LiDAR点云对齐到同一坐标系。
使用手眼标定得到的 T_uav2ugv 变换矩阵实现粗配准。

目标精度：RMSE < 20cm
"""

import numpy as np
import time
from typing import List, Optional, Tuple, Dict

from .data_types import Pose3D
from .collaborative_optimizer import CollaborativeOptimizer, CollaborativeConfig


def _write_ply_header(fp, num_points: int):
    """写入 PLY 文件头（ASCII 格式）"""
    fp.write("ply\n")
    fp.write("format ascii 1.0\n")
    fp.write(f"element vertex {num_points}\n")
    fp.write("property float x\n")
    fp.write("property float y\n")
    fp.write("property float z\n")
    fp.write("end_header\n")


def export_ply(filepath: str, points: np.ndarray):
    """将点云导出为 PLY 文件

    Args:
        filepath: 输出文件路径
        points:  N×3 点云坐标数组
    """
    with open(filepath, 'w', encoding='utf-8') as f:
        _write_ply_header(f, points.shape[0])
        for pt in points:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")


class CoarseRegistrator:
    """粗配准器

    使用协同SLAM手眼标定结果将UAV SfM点云变换到UGV LiDAR坐标系下，
    实现空地点云的初步对齐。

    Usage:
        registrator = CoarseRegistrator()
        uav_transformed, ugv_points, T = registrator.align_point_clouds(
            uav_sfm_points, ugv_lidar_points, uav_poses, ugv_poses
        )
    """

    def __init__(self):
        """初始化粗配准器"""
        self._optimizer: Optional[CollaborativeOptimizer] = None
        self._T_uav2ugv: Optional[Pose3D] = None
        self._uav_points_before: Optional[np.ndarray] = None
        self._uav_points_after: Optional[np.ndarray] = None
        self._ugv_points: Optional[np.ndarray] = None
        self._rmse: Optional[float] = None
        self._report: Dict = {}

    def align_point_clouds(
        self,
        uav_sfm_points: np.ndarray,
        ugv_lidar_points: np.ndarray,
        uav_poses: List[Pose3D],
        ugv_poses: List[Pose3D],
        T_uav2ugv: Optional[Pose3D] = None,
    ) -> Tuple[np.ndarray, np.ndarray, Pose3D]:
        """将 UAV SfM 点云与 UGV LiDAR 点云对齐到同一坐标系

        Args:
            uav_sfm_points:    UAV SfM 点云，形状 (N, 3)
            ugv_lidar_points:  UGV LiDAR 点云，形状 (M, 3)
            uav_poses:         UAV 位姿序列
            ugv_poses:         UGV 位姿序列
            T_uav2ugv:         可选，已标定的 UAV→UGV 变换矩阵；
                               若为 None 则使用 CollaborativeOptimizer 求解

        Returns:
            (变换后的UAV点云, UGV点云(原样), 粗配准变换矩阵 Pose3D)
        """
        self._uav_points_before = uav_sfm_points.copy()
        self._ugv_points = ugv_lidar_points.copy()

        # --- 1. 获取或求解 T_uav2ugv ---
        if T_uav2ugv is not None:
            self._T_uav2ugv = T_uav2ugv
        else:
            # 使用 CollaborativeOptimizer 进行手眼标定
            config = CollaborativeConfig()
            self._optimizer = CollaborativeOptimizer(config)

            # 添加同步位姿对
            min_pairs = min(len(uav_poses), len(ugv_poses))
            for i in range(min_pairs):
                self._optimizer.add_sync_pair(uav_poses[i], ugv_poses[i])

            self._T_uav2ugv = self._optimizer.calibrate()

            if self._T_uav2ugv is None:
                # 标定失败，回退到单位变换
                self._T_uav2ugv = Pose3D.identity()

        # --- 2. 将 UAV 点云变换到 UGV 坐标系 ---
        R = self._T_uav2ugv.R
        t = self._T_uav2ugv.t
        transformed_uav = (R @ uav_sfm_points.T).T + t  # (N, 3)

        self._uav_points_after = transformed_uav

        # --- 3. 评估粗配准质量 ---
        self._rmse = self.compute_rmse(transformed_uav, ugv_lidar_points)

        # --- 4. 生成报告 ---
        self._report = self.generate_report()

        return transformed_uav, ugv_lidar_points, self._T_uav2ugv

    def compute_rmse(self, points_a: np.ndarray, points_b: np.ndarray) -> float:
        """计算两组点云之间的最近邻 RMSE

        对 points_a 中每个点，在 points_b 中找最近邻，计算欧氏距离的 RMSE。

        Args:
            points_a: 源点云，形状 (N, 3)
            points_b: 目标点云，形状 (M, 3)

        Returns:
            RMSE 值（米）
        """
        if len(points_a) == 0 or len(points_b) == 0:
            return float('inf')

        # 使用采样策略避免对大点云计算全部距离
        max_samples = 5000
        if len(points_a) > max_samples:
            indices = np.random.choice(len(points_a), max_samples, replace=False)
            pts_a = points_a[indices]
        else:
            pts_a = points_a

        # 向量化最近邻搜索：分块计算避免内存溢出
        chunk_size = 1000
        min_dists = []

        for i in range(0, len(pts_a), chunk_size):
            chunk = pts_a[i:i + chunk_size]
            # (chunk, 1, 3) - (1, M, 3) → (chunk, M, 3)
            diffs = chunk[:, np.newaxis, :] - points_b[np.newaxis, :, :]
            dists = np.linalg.norm(diffs, axis=2)  # (chunk, M)
            min_dists.append(np.min(dists, axis=1))

        all_min_dists = np.concatenate(min_dists)
        rmse = np.sqrt(np.mean(all_min_dists ** 2))
        return float(rmse)

    def generate_report(self) -> dict:
        """生成粗配准报告

        Returns:
            包含 RMSE、点云数量、变换矩阵等信息的字典
        """
        report = {
            "method": "Coarse Registration (Hand-Eye Calibration)",
            "timestamp": time.time(),
            "rmse_m": self._rmse,
            "rmse_cm": self._rmse * 100 if self._rmse is not None else None,
            "rmse_pass": self._rmse is not None and self._rmse < 0.20,
            "uav_points_count": len(self._uav_points_after) if self._uav_points_after is not None else 0,
            "ugv_points_count": len(self._ugv_points) if self._ugv_points is not None else 0,
            "transformation_matrix": None,
            "rotation_matrix": None,
            "translation_vector": None,
        }

        if self._T_uav2ugv is not None:
            report["transformation_matrix"] = self._T_uav2ugv.T.tolist()
            report["rotation_matrix"] = self._T_uav2ugv.R.tolist()
            report["translation_vector"] = self._T_uav2ugv.t.tolist()

        return report

    def export_aligned_ply(
        self,
        uav_output: str,
        ugv_output: str,
    ):
        """将对齐后的点云导出为 PLY 文件

        Args:
            uav_output: 变换后 UAV 点云输出路径
            ugv_output: UGV 点云输出路径
        """
        if self._uav_points_after is not None:
            export_ply(uav_output, self._uav_points_after)
        if self._ugv_points is not None:
            export_ply(ugv_output, self._ugv_points)

    @property
    def transformation(self) -> Optional[Pose3D]:
        """获取粗配准变换矩阵"""
        return self._T_uav2ugv

    @property
    def rmse(self) -> Optional[float]:
        """获取 RMSE"""
        return self._rmse


# ══════════════════════════════════════════════════════════════════════
# 测试入口
# ══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=== 粗配准模块测试 ===\n")

    # 生成模拟数据
    np.random.seed(42)
    n_uav = 500
    n_ugv = 800

    # 模拟 UAV SfM 点云（在某局部坐标系中）
    uav_points = np.random.randn(n_uav, 3) * 5.0

    # 模拟 UGV LiDAR 点云（在另一坐标系中，存在已知偏移）
    true_R = np.array([
        [0.999, -0.01,  0.02],
        [0.01,   0.999, 0.005],
        [-0.02, -0.005, 0.999],
    ])
    true_t = np.array([10.0, -2.0, 0.5])
    # UGV 点云 = 真值变换后的 UAV 点云 + 噪声
    ugv_points = (true_R @ uav_points.T).T + true_t + np.random.randn(n_uav, 3) * 0.05

    # 补充一些额外的 UGV 扫描点
    extra_ugv = np.random.randn(n_ugv - n_uav, 3) * 6.0 + true_t
    ugv_points = np.vstack([ugv_points, extra_ugv])

    # 生成同步位姿序列
    uav_poses = [Pose3D(R=np.eye(3), t=np.array([i * 0.5, 0.0, 0.0])) for i in range(20)]
    ugv_poses = [Pose3D(R=true_R, t=true_t + np.array([i * 0.5, 0.0, 0.0])) for i in range(20)]

    # 执行粗配准
    registrator = CoarseRegistrator()
    uav_aligned, ugv_out, T = registrator.align_point_clouds(
        uav_points, ugv_points, uav_poses, ugv_poses
    )

    # 输出结果
    report = registrator.generate_report()
    print(f"RMSE: {report['rmse_cm']:.2f} cm")
    print(f"RMSE 通过 (< 20cm): {report['rmse_pass']}")
    print(f"UAV 点云数量: {report['uav_points_count']}")
    print(f"UGV 点云数量: {report['ugv_points_count']}")
    print(f"变换平移向量: {np.round(T.t, 3)}")

    # 导出 PLY
    registrator.export_aligned_ply("/tmp/uav_coarse.ply", "/tmp/ugv_coarse.ply")
    print("\n已导出 PLY 文件: /tmp/uav_coarse.ply, /tmp/ugv_coarse.ply")
    print("\n粗配准测试完成！")
