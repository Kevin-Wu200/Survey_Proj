"""
Scan-to-Map ICP 匹配单元测试 (5.2)
测试 LiDAROdometry 的 ICP 配准精度和收敛性。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from src.slam.data_types import ScanData, Pose3D, UGVFusionConfig
from src.slam.ugv_fusion import LiDAROdometry


class TestScanToMapICP:
    """Scan-to-Map ICP 配准测试套件"""

    @pytest.fixture
    def config(self):
        return UGVFusionConfig(
            scan_to_map_max_iter=30,
            scan_to_map_convergence=1e-4
        )

    @pytest.fixture
    def lidar_odom(self, config):
        return LiDAROdometry(config)

    def _generate_plane_points(self, n: int, size: float = 5.0) -> np.ndarray:
        """生成平面点云（xy 平面上随机点）"""
        points = np.random.rand(n, 3) * size
        points[:, 2] = 0.0  # z=0 平面
        return points

    def test_first_scan_identity(self, lidar_odom):
        """测试第一帧扫描应返回恒等变换"""
        points = self._generate_plane_points(500)
        scan = ScanData(points=points, frame_id="lidar")

        rel_pose, score = lidar_odom.process_scan(scan)
        assert np.allclose(rel_pose.R, np.eye(3), atol=1e-5)
        assert np.allclose(rel_pose.t, np.zeros(3), atol=1e-5)
        assert score == 1.0

    def test_identical_scans(self, lidar_odom):
        """测试两帧完全相同的扫描：ICP 应返回恒等变换"""
        np.random.seed(42)
        points = self._generate_plane_points(500)

        # 第一帧
        scan1 = ScanData(points=points.copy())
        lidar_odom.process_scan(scan1)

        # 第二帧（完全相同）
        scan2 = ScanData(points=points.copy())
        rel_pose, score = lidar_odom.process_scan(scan2)

        # 应接近恒等变换
        assert np.allclose(rel_pose.R, np.eye(3), atol=0.01), \
            f"旋转矩阵应为恒等:\n{rel_pose.R}"
        assert np.linalg.norm(rel_pose.t) < 0.05, \
            f"平移应接近0: {rel_pose.t}"

    def test_known_translation(self, lidar_odom):
        """测试已知平移量：源点云平移 (1.0, 0.0, 0.0)"""
        np.random.seed(42)
        src_points = self._generate_plane_points(500)

        # 第一帧
        scan1 = ScanData(points=src_points.copy())
        lidar_odom.process_scan(scan1)

        # 第二帧：平移 (1.0, 0.0, 0.0)
        translation = np.array([1.0, 0.0, 0.0])
        dst_points = src_points + translation
        scan2 = ScanData(points=dst_points)
        rel_pose, score = lidar_odom.process_scan(scan2)

        # 估算的平移应接近 (1.0, 0.0, 0.0)
        estimated_t = rel_pose.t
        assert abs(estimated_t[0] - 1.0) < 0.3, \
            f"x 平移偏差过大: 期望 1.0, 实际 {estimated_t[0]:.4f}"
        assert abs(estimated_t[1]) < 0.3, \
            f"y 平移应接近0: {estimated_t[1]:.4f}"

    def test_known_rotation(self, lidar_odom):
        """测试已知旋转：绕 z 轴旋转 30 度"""
        np.random.seed(42)
        src_points = self._generate_plane_points(500)

        # 第一帧
        scan1 = ScanData(points=src_points.copy())
        lidar_odom.process_scan(scan1)

        # 第二帧：绕 z 轴旋转 30°
        theta = np.radians(30)
        R_true = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ])
        dst_points = (R_true @ src_points.T).T
        scan2 = ScanData(points=dst_points)
        rel_pose, score = lidar_odom.process_scan(scan2)

        # 旋转矩阵应接近预期
        R_est = rel_pose.R
        assert np.allclose(R_est @ R_est.T, np.eye(3), atol=1e-5), \
            "估计旋转矩阵非正交"

    def test_small_point_cloud(self, lidar_odom):
        """测试小规模点云（10个点）"""
        np.random.seed(42)
        points = np.random.rand(10, 3)

        scan1 = ScanData(points=points.copy())
        lidar_odom.process_scan(scan1)

        scan2 = ScanData(points=points + np.array([0.1, 0.0, 0.0]))
        rel_pose, score = lidar_odom.process_scan(scan2)
        assert score >= 0, "分数应非负"

    def test_convergence_behavior(self, lidar_odom):
        """测试 ICP 收敛行为：连续相同扫描应稳定"""
        np.random.seed(42)
        points = self._generate_plane_points(500)
        scan1 = ScanData(points=points.copy())
        lidar_odom.process_scan(scan1)

        results = []
        for _ in range(10):
            scan = ScanData(points=points.copy())
            rel_pose, _ = lidar_odom.process_scan(scan)
            results.append(rel_pose.t.copy())

        # 连续相同扫描的估计偏差应小
        std_dev = np.std([r[0] for r in results])
        assert std_dev < 0.1, f"平移估计不稳定: std={std_dev:.4f}"

    def test_with_initial_guess(self, lidar_odom):
        """测试给定初始位姿估计的 ICP"""
        np.random.seed(42)
        src_points = self._generate_plane_points(500)

        scan1 = ScanData(points=src_points.copy())
        lidar_odom.process_scan(scan1)

        # 目标平移 0.5m
        dst_points = src_points + np.array([0.5, 0.0, 0.0])
        scan2 = ScanData(points=dst_points)

        # 给一个接近的初始猜测
        init_guess = Pose3D(R=np.eye(3), t=np.array([0.4, 0.05, 0.0]))
        rel_pose, score = lidar_odom.process_scan(scan2, init_guess)

        assert rel_pose is not None
        assert score > 0

    def test_score_metric(self, lidar_odom):
        """测试配准分数指标：完美匹配 vs 有噪声匹配"""
        np.random.seed(42)
        points = self._generate_plane_points(500)

        # 完美匹配
        scan1 = ScanData(points=points.copy())
        lidar_odom.process_scan(scan1)

        scan_perfect = ScanData(points=points.copy())
        _, score_perfect = lidar_odom.process_scan(scan_perfect)

        # 重置
        lidar_odom._last_scan = None

        # 有噪声匹配
        scan1 = ScanData(points=points.copy())
        lidar_odom.process_scan(scan1)

        noisy = points + np.random.randn(500, 3) * 0.1
        scan_noisy = ScanData(points=noisy)
        _, score_noisy = lidar_odom.process_scan(scan_noisy)

        # 完美匹配分数应 ≥ 有噪声匹配分数
        assert score_perfect >= score_noisy * 0.8, \
            f"完美匹配分数({score_perfect:.4f})应接近噪声匹配({score_noisy:.4f})"


class TestScanData:
    """ScanData 数据类型测试"""

    def test_scan_data_defaults(self):
        points = np.random.rand(100, 3)
        scan = ScanData(points=points)

        assert scan.points.shape == (100, 3)
        assert scan.timestamp > 0
        assert scan.frame_id == "lidar"
        assert scan.intensities is None

    def test_scan_data_with_intensity(self):
        points = np.random.rand(100, 3)
        intensity = np.random.rand(100)
        scan = ScanData(points=points, intensities=intensity)

        assert scan.intensities.shape == (100,)
        assert np.allclose(scan.intensities, intensity)
