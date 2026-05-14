"""
因子图优化单元测试 (5.2)
测试因子图构建、优化和数据一致性。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from src.slam.data_types import (
    Pose3D, FactorGraph, FactorEdge, FactorType,
    UGVFusionConfig
)
from src.slam.ugv_fusion import UGVMultiSensorFusion
from src.slam.collaborative_optimizer import (
    CollaborativeOptimizer, CollaborativeConfig, SlidingWindowOptimizer
)
from src.slam.loop_closure import PoseGraphOptimizer, LoopClosureConfig


class TestFactorGraph:
    """因子图基本操作测试"""

    @pytest.fixture
    def empty_graph(self):
        return FactorGraph()

    def test_add_node(self, empty_graph):
        """测试添加节点"""
        pose = Pose3D.identity()
        empty_graph.add_node(0, pose, "ugv")
        assert 0 in empty_graph.nodes
        assert empty_graph.node_types[0] == "ugv"

    def test_add_factor(self, empty_graph):
        """测试添加因子边"""
        empty_graph.add_node(0, Pose3D.identity(), "ugv")
        empty_graph.add_node(1, Pose3D(R=np.eye(3), t=np.array([1.0, 0.0, 0.0])), "ugv")

        edge = FactorEdge(
            src_id=0, dst_id=1,
            factor_type=FactorType.LIDAR,
            measurement=np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            information=np.eye(6) * 10
        )
        empty_graph.add_factor(edge)
        assert len(empty_graph.edges) == 1
        assert empty_graph.edges[0].factor_type == FactorType.LIDAR

    def test_get_connected_nodes(self, empty_graph):
        """测试获取连接节点"""
        for i in range(3):
            empty_graph.add_node(i, Pose3D(R=np.eye(3), t=np.array([i * 1.0, 0.0, 0.0])))

        empty_graph.add_factor(FactorEdge(
            src_id=0, dst_id=1,
            factor_type=FactorType.LIDAR,
            measurement=np.zeros(6),
            information=np.eye(6)
        ))
        empty_graph.add_factor(FactorEdge(
            src_id=0, dst_id=2,
            factor_type=FactorType.LIDAR,
            measurement=np.zeros(6),
            information=np.eye(6)
        ))

        connected = empty_graph.get_connected_nodes(0)
        assert 1 in connected
        assert 2 in connected

    def test_multiple_factor_types(self, empty_graph):
        """测试多种因子类型"""
        empty_graph.add_node(0, Pose3D.identity(), "ugv")
        empty_graph.add_node(1, Pose3D(R=np.eye(3), t=np.array([0.5, 0.0, 0.0])), "ugv")

        # LiDAR 因子
        empty_graph.add_factor(FactorEdge(
            src_id=0, dst_id=1,
            factor_type=FactorType.LIDAR,
            measurement=np.zeros(6),
            information=np.eye(6) * 10
        ))

        # 视觉因子
        empty_graph.add_factor(FactorEdge(
            src_id=0, dst_id=1,
            factor_type=FactorType.VISUAL,
            measurement=np.zeros(6),
            information=np.eye(6) * 5
        ))

        # GPS 因子
        empty_graph.add_factor(FactorEdge(
            src_id=1, dst_id=1,
            factor_type=FactorType.GPS,
            measurement=np.array([0.5, 0.0, 0.0]),
            information=np.eye(3) * 100
        ))

        assert len(empty_graph.edges) == 3


class TestUGVMultiSensorFusion:
    """UGV 多传感器融合测试"""

    @pytest.fixture
    def config(self):
        return UGVFusionConfig(
            imu_frequency=100.0,
            lidar_frequency=10.0,
            output_frequency=10.0,
            imu_accel_noise=0.01,
            imu_gyro_noise=0.001
        )

    @pytest.fixture
    def fusion(self, config):
        return UGVMultiSensorFusion(config)

    def test_initial_pose(self, fusion):
        """测试初始位姿为恒等"""
        assert np.allclose(fusion._current_pose.R, np.eye(3))
        assert np.allclose(fusion._current_pose.t, np.zeros(3))

    def test_initial_factor_graph(self, fusion):
        """测试初始因子图包含先验节点"""
        assert len(fusion.factor_graph.nodes) == 1
        assert 0 in fusion.factor_graph.nodes
        assert len(fusion.factor_graph.edges) == 1
        assert fusion.factor_graph.edges[0].factor_type == FactorType.PRIOR

    def test_process_imu(self, fusion):
        """测试 IMU 数据处理"""
        from src.slam.data_types import IMUData
        imu = IMUData(accel=np.array([0.0, 0.0, 9.81]),
                       gyro=np.array([0.0, 0.0, 0.0]))
        fusion.process_imu(imu)
        assert len(fusion._imu_buffer) == 1

    def test_process_lidar_sequence(self, fusion):
        """测试 LiDAR 序列处理"""
        from src.slam.data_types import ScanData
        np.random.seed(42)

        # 处理多帧 LiDAR
        for i in range(5):
            points = np.random.rand(100, 3) + np.array([i * 0.1, 0.0, 0.0])
            scan = ScanData(points=points)
            pose = fusion.process_lidar(scan)

        assert len(fusion._poses) == 5
        assert pose is not None

    def test_optimize(self, fusion):
        """测试因子图优化"""
        from src.slam.data_types import ScanData, IMUData
        np.random.seed(42)

        # 积累足够的位姿
        for i in range(10):
            fusion.process_imu(IMUData(
                accel=np.array([0.0, 0.0, 9.81]),
                gyro=np.array([0.0, 0.0, 0.0])
            ))
            points = np.random.rand(100, 3) + np.array([i * 0.1, 0.0, 0.0])
            fusion.process_lidar(ScanData(points=points))

        result = fusion.optimize()
        assert result is not None
        assert result.t is not None

    def test_trajectory_output(self, fusion):
        """测试轨迹输出"""
        from src.slam.data_types import ScanData
        np.random.seed(42)

        for i in range(5):
            points = np.random.rand(100, 3) + np.array([i * 0.1, 0.0, 0.0])
            fusion.process_lidar(ScanData(points=points))

        traj = fusion.get_trajectory()
        assert traj.shape == (5, 3)
        # 轨迹应沿x轴递增
        assert np.all(np.diff(traj[:, 0]) >= -1e-6)

    def test_add_gnss_constraint(self, fusion):
        """测试添加 GNSS 约束"""
        gnss_pose = Pose3D(R=np.eye(3), t=np.array([0.5, 0.0, 0.0]))
        covariance = np.eye(3) * 0.1
        fusion.add_gnss_constraint(gnss_pose, covariance)

        gps_edges = [e for e in fusion.factor_graph.edges
                      if e.factor_type == FactorType.GPS]
        assert len(gps_edges) > 0

    def test_generate_report(self, fusion):
        """测试融合报告生成"""
        from src.slam.data_types import ScanData
        np.random.seed(42)

        for i in range(5):
            points = np.random.rand(100, 3) + np.array([i * 0.1, 0.0, 0.0])
            fusion.process_lidar(ScanData(points=points))

        report = fusion.generate_report()
        assert 'poses' in report
        assert report['poses'] == 5
        assert 'factor_nodes' in report
        assert 'trajectory_length_m' in report


class TestSlidingWindowOptimizer:
    """滑动窗口优化器测试"""

    @pytest.fixture
    def config(self):
        return CollaborativeConfig(sliding_window_size=10)

    @pytest.fixture
    def optimizer(self, config):
        return SlidingWindowOptimizer(config)

    def test_add_frame(self, optimizer):
        """测试添加帧到滑动窗口"""
        pose = Pose3D.identity()
        optimizer.add_frame(0, pose, "ugv")
        assert len(optimizer.window) == 1

    def test_slide_window_overflow(self, optimizer):
        """测试滑动窗口溢出（超出窗口大小的帧应被丢弃）"""
        for i in range(15):
            pose = Pose3D(R=np.eye(3), t=np.array([i * 0.5, 0.0, 0.0]))
            optimizer.add_frame(i, pose, "ugv")

        # 窗口最大为 10
        assert len(optimizer.window) == 10
        # 最早的帧应被丢弃
        window_ids = [f['id'] for f in optimizer.window]
        assert 0 not in window_ids

    def test_optimize_with_few_frames(self, optimizer):
        """测试帧数不足时不执行优化"""
        for i in range(2):
            pose = Pose3D(R=np.eye(3), t=np.array([i * 1.0, 0.0, 0.0]))
            optimizer.add_frame(i, pose, "ugv")

        result = optimizer.optimize()
        # 少于 3 帧时不执行优化
        assert result == optimizer._optimized_nodes

    def test_optimize_with_enough_frames(self, optimizer):
        """测试足够帧数的优化"""
        for i in range(5):
            pose = Pose3D(R=np.eye(3), t=np.array([i * 1.0, 0.0, 0.0]))
            optimizer.add_frame(i, pose, "ugv")

        result = optimizer.optimize()
        assert len(result) > 0
        for nid, pose in result.items():
            assert pose.R.shape == (3, 3)
            assert pose.t.shape == (3,)


class TestPoseGraphOptimizer:
    """位姿图优化器测试"""

    @pytest.fixture
    def pgo(self):
        return PoseGraphOptimizer(LoopClosureConfig(pgo_max_iter=50))

    def test_optimize_empty_graph(self, pgo):
        """测试空因子图优化"""
        graph = FactorGraph()
        result = pgo.optimize(graph, [])
        assert len(result.nodes) == 0

    def test_optimize_single_node(self, pgo):
        """测试单节点因子图优化"""
        graph = FactorGraph()
        graph.add_node(0, Pose3D.identity(), "ugv")
        result = pgo.optimize(graph, [])
        assert 0 in result.nodes

    def test_optimize_with_loop_closure(self, pgo):
        """测试带回环约束的优化"""
        graph = FactorGraph()

        # 构建一圈轨迹（应形成回环）
        for i in range(8):
            angle = i * np.pi / 4
            x = np.cos(angle) * 5.0
            y = np.sin(angle) * 5.0
            pose = Pose3D(R=np.eye(3), t=np.array([x, y, 0.0]))
            graph.add_node(i, pose, "ugv")

        # 添加帧间约束
        for i in range(7):
            graph.add_factor(FactorEdge(
                src_id=i, dst_id=i + 1,
                factor_type=FactorType.LIDAR,
                measurement=np.zeros(6),
                information=np.eye(6) * 10
            ))

        # 添加回环约束（首尾相连）
        loop_edge = FactorEdge(
            src_id=7, dst_id=0,
            factor_type=FactorType.LOOP,
            measurement=np.zeros(6),
            information=np.eye(6) * 5
        )

        result = pgo.optimize(graph, [loop_edge])
        assert len(result.nodes) == 8
        # 优化后的位姿应仍保持合理
        for nid, pose in result.nodes.items():
            assert np.allclose(pose.R @ pose.R.T, np.eye(3), atol=1e-4)


class TestFactorTypes:
    """因子类型枚举测试"""

    def test_all_factor_types(self):
        """测试所有因子类型值"""
        assert int(FactorType.PRIOR) == 0
        assert int(FactorType.IMU) == 1
        assert int(FactorType.LIDAR) == 2
        assert int(FactorType.VISUAL) == 3
        assert int(FactorType.LOOP) == 4
        assert int(FactorType.GPS) == 5

    def test_factor_type_equality(self):
        """测试因子类型比较"""
        assert FactorType.IMU != FactorType.LIDAR
        assert FactorType(2) == FactorType.LIDAR
