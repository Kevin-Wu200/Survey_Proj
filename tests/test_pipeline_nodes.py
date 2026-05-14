"""
数据管道节点单元测试 (5.2)
测试 pointcloud_filter, image_rectify, sfm, icp_registration, meshing 节点的独立逻辑。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest


# ══════════════════════════════════════════════════════════════════════
# 点云滤波测试
# ══════════════════════════════════════════════════════════════════════

class TestVoxelGrid:
    """体素降采样测试"""

    @pytest.fixture
    def voxel_filter(self):
        # 直接导入 VoxelGrid 类（避免 ROS2 依赖）
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.pointcloud_filter_node import VoxelGrid
        return VoxelGrid(leaf_size=0.05)

    def test_empty_input(self, voxel_filter):
        """测试空输入"""
        result = voxel_filter.filter(np.zeros((0, 3)))
        assert result.shape == (0, 3)

    def test_single_point(self, voxel_filter):
        """测试单点"""
        points = np.array([[1.0, 2.0, 3.0]])
        result = voxel_filter.filter(points)
        assert len(result) == 1
        np.testing.assert_array_almost_equal(result[0], points[0])

    def test_points_in_same_voxel(self, voxel_filter):
        """测试同一体素内的多个点应被合并为一个"""
        points = np.array([
            [0.01, 0.01, 0.01],
            [0.02, 0.02, 0.02],
            [0.03, 0.03, 0.03],
        ])
        result = voxel_filter.filter(points)
        assert len(result) == 1  # 所有点在同一体素内
        # 结果应为质心
        np.testing.assert_array_almost_equal(result[0], np.mean(points, axis=0))

    def test_points_in_different_voxels(self, voxel_filter):
        """测试不同体素内的点"""
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],  # 不同体素
            [0.0, 0.1, 0.0],  # 不同体素
            [0.1, 0.1, 0.0],  # 不同体素
        ])
        result = voxel_filter.filter(points)
        assert len(result) == 4  # 四个不同体素

    def test_reduction_ratio(self, voxel_filter):
        """测试降采样压缩比"""
        np.random.seed(42)
        # 生成密集点云（在 0.01m 范围内分布）
        points = np.random.rand(1000, 3) * 0.01
        result = voxel_filter.filter(points)
        # 体素 0.05m，所有点应在同一体素内
        assert len(result) < len(points), \
            f"降采样后点数 {len(result)} 应少于原始点数 {len(points)}"

    def test_large_point_cloud(self, voxel_filter):
        """测试大规模点云"""
        np.random.seed(42)
        points = np.random.rand(5000, 3) * 10.0
        result = voxel_filter.filter(points)
        assert len(result) > 0
        assert len(result) <= len(points)


class TestStatisticalOutlierRemoval:
    """统计离群点去除测试"""

    @pytest.fixture
    def sor_filter(self):
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.pointcloud_filter_node import StatisticalOutlierRemoval
        return StatisticalOutlierRemoval(mean_k=20, std_mul=1.0)

    def test_empty_input(self, sor_filter):
        result = sor_filter.filter(np.zeros((0, 3)))
        assert result.shape == (0, 3)

    def test_few_points(self, sor_filter):
        """测试点数不足时不报错"""
        points = np.random.rand(5, 3)
        result = sor_filter.filter(points)
        assert len(result) == 5  # 少于 mean_k 时返回原样

    def test_remove_outliers(self, sor_filter):
        """测试移除离群点"""
        np.random.seed(42)
        # 正常点云（聚集）
        cluster = np.random.randn(200, 3) * 0.5
        # 离群点
        outliers = np.random.randn(10, 3) * 10.0 + np.array([50, 50, 50])

        points = np.vstack([cluster, outliers])
        result = sor_filter.filter(points)
        # 离群点应被移除
        assert len(result) < len(points)
        assert np.max(np.linalg.norm(result, axis=1)) < 50

    def test_all_normal_points(self, sor_filter):
        """测试全部为正常点时不应过度移除"""
        np.random.seed(42)
        points = np.random.randn(200, 3) * 0.5
        result = sor_filter.filter(points)
        # 应保留大部分点
        assert len(result) >= len(points) * 0.8


class TestPassThroughFilter:
    """直通滤波器测试"""

    @pytest.fixture
    def pass_filter(self):
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.pointcloud_filter_node import PassThroughFilter
        return PassThroughFilter(axis='z', min_val=0.1, max_val=30.0)

    def test_filter_z_range(self, pass_filter):
        """测试 z 轴范围过滤"""
        points = np.array([
            [0.0, 0.0, 0.0],    # z=0 应被移除
            [0.0, 0.0, 10.0],   # z=10 应保留
            [1.0, 1.0, 50.0],   # z=50 应被移除
            [2.0, 2.0, 5.0],    # z=5 应保留
        ])
        result = pass_filter.filter(points)
        assert len(result) == 2
        assert np.all(result[:, 2] >= 0.1)
        assert np.all(result[:, 2] <= 30.0)

    def test_filter_x_axis(self):
        """测试 x 轴过滤"""
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.pointcloud_filter_node import PassThroughFilter
        pf = PassThroughFilter(axis='x', min_val=1.0, max_val=5.0)
        points = np.array([
            [0.5, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [6.0, 0.0, 0.0],
        ])
        result = pf.filter(points)
        assert len(result) == 1
        assert result[0, 0] == 3.0


# ══════════════════════════════════════════════════════════════════════
# ICP 配准核心算法测试
# ══════════════════════════════════════════════════════════════════════

class TestICPSVD:
    """SVD ICP 算法测试"""

    @pytest.fixture
    def solve_icp(self):
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.icp_registration_node import solve_icp_svd
        return solve_icp_svd

    def test_identity_transform(self, solve_icp):
        """测试恒等变换（完全相同点云）"""
        np.random.seed(42)
        source = np.random.rand(100, 3)
        target = source.copy()

        T, rmse = solve_icp(source, target)
        assert np.allclose(T, np.eye(4), atol=0.01)
        assert rmse < 0.01

    def test_known_translation(self, solve_icp):
        """测试已知平移"""
        np.random.seed(42)
        source = np.random.rand(200, 3)
        T_true = np.eye(4)
        T_true[:3, 3] = [1.0, 2.0, 0.0]

        target = (T_true[:3, :3] @ source.T).T + T_true[:3, 3]

        T_est, rmse = solve_icp(source, target)
        assert np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]) < 1.0, \
            f"平移估计偏差过大"

    def test_with_initial_transform(self, solve_icp):
        """测试给定初始变换的 ICP"""
        np.random.seed(42)
        source = np.random.rand(100, 3)

        # 目标点云平移 (0.5, 0.0, 0.0)
        target = source + np.array([0.5, 0.0, 0.0])

        # 初始变换接近真值
        init_T = np.eye(4)
        init_T[:3, 3] = [0.4, 0.1, 0.0]

        T_est, rmse = solve_icp(source, target, initial_transform=init_T)
        assert T_est.shape == (4, 4)
        assert rmse < 1.0

    def test_small_point_clouds(self, solve_icp):
        """测试小规模点云"""
        np.random.seed(42)
        source = np.random.rand(5, 3)
        target = source + np.array([0.1, 0.0, 0.0])

        T_est, rmse = solve_icp(source, target, max_iterations=10)
        assert T_est.shape == (4, 4)

    def test_rot_to_quat(self):
        """测试旋转矩阵转四元数"""
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.icp_registration_node import ICPRegistrationNode

        R = np.eye(3)
        q = ICPRegistrationNode._rot_to_quat(R)
        assert len(q) == 4
        # 恒等旋转→四元数 (0, 0, 0, 1)
        assert abs(q[3] - 1.0) < 0.01

    def test_transform_pointcloud(self):
        """测试点云变换"""
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.icp_registration_node import ICPRegistrationNode

        points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        T = np.eye(4)
        T[:3, 3] = [0.5, 0.5, 0.0]

        result = ICPRegistrationNode._transform_pointcloud(points, T)
        np.testing.assert_array_almost_equal(result[0], [1.5, 0.5, 0.0])
        np.testing.assert_array_almost_equal(result[1], [0.5, 1.5, 0.0])


# ══════════════════════════════════════════════════════════════════════
# SfM 简化版测试
# ══════════════════════════════════════════════════════════════════════

class TestSimpleSfM:
    """SfM 简化版算法测试"""

    @pytest.fixture
    def sfm(self):
        from src.ros2_ws.src.fusion_pipeline.fusion_pipeline.sfm_node import (
            SimpleSfM, SimpleKeyFrame, SimplePose3D
        )
        K = np.array([[800, 0, 960], [0, 800, 540], [0, 0, 1]], dtype=np.float64)
        self.SimplePose3D = SimplePose3D
        self.SimpleKeyFrame = SimpleKeyFrame
        return SimpleSfM(camera_matrix=K, min_inliers=10)

    def test_empty_triangulation(self, sfm):
        """测试无特征时的三角化"""
        kf1 = self.SimpleKeyFrame(
            id=0, pose=self.SimplePose3D.identity())
        kf2 = self.SimpleKeyFrame(
            id=1, pose=self.SimplePose3D(
                R=np.eye(3), t=np.array([1.0, 0.0, 0.0])))

        # 无图像特征时应返回空
        results = sfm.match_and_triangulate(kf1, kf2)
        assert len(results) == 0

    def test_simple_pose_identity(self):
        """测试 SimplePose3D 恒等"""
        pose = self.SimplePose3D.identity()
        assert np.allclose(pose.R, np.eye(3))
        assert np.allclose(pose.t, np.zeros(3))

    def test_simple_pose_difference(self):
        """测试位姿差计算"""
        p1 = self.SimplePose3D(R=np.eye(3), t=np.array([0.0, 0.0, 0.0]))
        p2 = self.SimplePose3D(R=np.eye(3), t=np.array([3.0, 4.0, 0.0]))

        delta = p1 - p2
        assert abs(delta.distance - 5.0) < 0.01  # 3-4-5 三角形


# ══════════════════════════════════════════════════════════════════════
# 协同一体化测试
# ══════════════════════════════════════════════════════════════════════

class TestHandEyeCalibrator:
    """手眼标定器测试"""

    @pytest.fixture
    def config(self):
        from src.slam.data_types import CollaborativeConfig
        return CollaborativeConfig(hand_eye_max_iter=50)

    @pytest.fixture
    def calibrator(self, config):
        from src.slam.collaborative_optimizer import HandEyeCalibrator
        return HandEyeCalibrator(config)

    def test_insufficient_pairs(self, calibrator):
        """测试位姿对不足时返回None"""
        from src.slam.data_types import Pose3D
        calibrator.add_measurement_pair(Pose3D.identity(), Pose3D.identity())
        result = calibrator.solve()
        assert result is None

    def test_solve_with_enough_pairs(self, calibrator):
        """测试足够位姿对的标定"""
        from src.slam.data_types import Pose3D

        np.random.seed(42)
        # 真值变换
        T_true = Pose3D(R=np.eye(3), t=np.array([5.0, -3.0, 1.0]))

        for i in range(10):
            uav_pose = Pose3D(R=np.eye(3), t=np.random.randn(3) * 2.0)
            ugv_pose = T_true.compose(uav_pose)
            # 加微小噪声
            ugv_pose = Pose3D(
                R=ugv_pose.R,
                t=ugv_pose.t + np.random.randn(3) * 0.02
            )
            calibrator.add_measurement_pair(uav_pose, ugv_pose)

        result = calibrator.solve()
        assert result is not None
        # 平移应接近真值
        assert np.linalg.norm(result.t - T_true.t) < 2.0


class TestCollaborativeOptimizer:
    """协同优化器集成测试"""

    @pytest.fixture
    def optimizer(self):
        from src.slam.collaborative_optimizer import CollaborativeOptimizer
        return CollaborativeOptimizer()

    def test_add_uav_keyframe(self, optimizer):
        """测试添加 UAV 关键帧"""
        pose = Pose3D.identity()
        nid = optimizer.add_uav_keyframe(0, pose)
        assert isinstance(nid, int)

    def test_add_ugv_pose(self, optimizer):
        """测试添加 UGV 位姿"""
        import numpy as np
        from src.slam.data_types import Pose3D as PD
        pose = PD.identity()
        nid = optimizer.add_ugv_pose(pose)
        assert nid == 0

    def test_initial_report(self, optimizer):
        """测试初始报告"""
        report = optimizer.generate_report()
        assert report['calibrated'] is False
        assert report['sync_pairs'] == 0


# 需要导入 Pose3D 用于协同优化器测试
from src.slam.data_types import Pose3D
