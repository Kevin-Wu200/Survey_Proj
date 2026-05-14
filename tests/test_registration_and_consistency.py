"""
协同 SLAM 数据配准与一致性维护单元测试 (5.2)
测试粗配准、精配准、一致性校验和回环检测模块。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from src.slam.data_types import (
    Pose3D, KeyFrame, TopoGraph, TopoNode,
    ScanData, IMUData
)


class TestCoarseRegistrator:
    """粗配准器测试"""

    @pytest.fixture
    def registrator(self):
        from src.slam.coarse_registration import CoarseRegistrator
        return CoarseRegistrator()

    def test_align_with_known_transform(self, registrator):
        """测试使用已知变换进行粗配准"""
        np.random.seed(42)

        # 生成模拟数据
        uav_points = np.random.randn(100, 3) * 5.0
        T_known = Pose3D(R=np.eye(3), t=np.array([10.0, -2.0, 0.5]))
        ugv_points = (T_known.R @ uav_points.T).T + T_known.t + np.random.randn(100, 3) * 0.05

        uav_aligned, ugv_out, T = registrator.align_point_clouds(
            uav_points, ugv_points, [], [], T_uav2ugv=T_known
        )

        assert uav_aligned.shape == uav_points.shape
        report = registrator.generate_report()
        assert 'rmse_m' in report
        assert 'method' in report

    def test_compute_rmse(self, registrator):
        """测试 RMSE 计算"""
        np.random.seed(42)
        pts_a = np.random.randn(100, 3)
        pts_b = pts_a + np.random.randn(100, 3) * 0.05

        rmse = registrator.compute_rmse(pts_a, pts_b)
        assert rmse > 0
        assert rmse < 1.0

    def test_empty_input_rmse(self, registrator):
        """测试空输入的 RMSE"""
        rmse = registrator.compute_rmse(np.empty((0, 3)), np.random.rand(10, 3))
        assert rmse == float('inf')

    def test_rmse_sampling(self, registrator):
        """测试大规模点云的采样 RMSE"""
        np.random.seed(42)
        pts_a = np.random.randn(10000, 3)  # 超过采样阈值
        pts_b = pts_a + np.random.randn(10000, 3) * 0.1

        rmse = registrator.compute_rmse(pts_a, pts_b)
        assert rmse > 0
        assert rmse < 1.0

    def test_ply_export(self, registrator, tmp_path):
        """测试 PLY 导出"""
        from src.slam.coarse_registration import export_ply
        points = np.random.rand(50, 3)
        filepath = str(tmp_path / "test.ply")

        export_ply(filepath, points)

        with open(filepath, 'r') as f:
            content = f.read()
            assert "ply" in content
            assert "format ascii 1.0" in content
            assert "element vertex 50" in content


class TestConsistencyChecker:
    """一致性校验器测试"""

    @pytest.fixture
    def checker(self):
        from src.slam.consistency import ConsistencyChecker
        return ConsistencyChecker(check_interval=0.0, drift_threshold=2.0)

    def test_overlap_detection(self, checker):
        """测试重叠区域检测"""
        graph = TopoGraph()
        for i in range(5):
            node = TopoNode(
                id=i,
                pose=Pose3D(R=np.eye(3), t=np.array([i * 5.0, 0.0, 0.0])),
                descriptor=np.random.randn(128),
                keyframe=KeyFrame(id=i, pose=Pose3D.identity())
            )
            graph.add_node(node)

        ugv_traj = np.array([[i * 5.0, 0.5, 0.0] for i in range(5)])

        overlaps = checker.overlap_detector.find_overlap(graph, ugv_traj)
        assert len(overlaps) > 0

    def test_overlap_ratio(self, checker):
        """测试重叠比例计算"""
        graph = TopoGraph()
        for i in range(10):
            node = TopoNode(
                id=i,
                pose=Pose3D(R=np.eye(3), t=np.array([i * 5.0, 0.0, 0.0])),
                descriptor=np.random.randn(128),
                keyframe=KeyFrame(id=i, pose=Pose3D.identity())
            )
            graph.add_node(node)

        # UGV 轨迹与 UAV 部分重叠
        ugv_traj = np.array([[i * 5.0, 0.5, 0.0] for i in range(6)])

        ratio = checker.overlap_detector.compute_overlap_ratio(graph, ugv_traj)
        assert 0 <= ratio <= 1.0

    def test_drift_detection(self, checker):
        """测试漂移检测"""
        for i in range(15):
            drift = 0.5 + i * 0.3  # 递增漂移
            checker.drift_detector.measure_drift(
                Pose3D.identity(),
                Pose3D(R=np.eye(3), t=np.array([drift, 0.0, 0.0]))
            )

        # 应检测到漂移
        assert checker.drift_detector.is_drifting()

    def test_no_drift_when_stable(self, checker):
        """测试稳定时不应报告漂移"""
        for i in range(15):
            checker.drift_detector.measure_drift(
                Pose3D.identity(),
                Pose3D(R=np.eye(3), t=np.array([0.1, 0.0, 0.0]))
            )

        assert not checker.drift_detector.is_drifting()

    def test_drift_trend(self, checker):
        """测试漂移趋势"""
        for i in range(30):
            drift = 1.0 + i * 0.1
            checker.drift_detector.measure_drift(
                Pose3D.identity(),
                Pose3D(R=np.eye(3), t=np.array([drift, 0.0, 0.0]))
            )

        trend = checker.drift_detector.get_drift_trend()
        assert trend > 0  # 趋势应为正（漂移增加）

    def test_map_alignment(self, checker):
        """测试地图对齐"""
        np.random.seed(42)
        uav_points = np.random.randn(50, 3)
        T_offset = Pose3D(R=np.eye(3), t=np.array([2.0, 1.0, 0.0]))
        ugv_points = (T_offset.R @ uav_points.T).T + T_offset.t

        correction = checker.map_aligner.align_maps(uav_points, ugv_points)
        assert correction is not None
        assert np.linalg.norm(correction.t - T_offset.t) < 1.0

    def test_check_consistency(self, checker):
        """测试一致性检查完整流程"""
        graph = TopoGraph()
        for i in range(8):
            node = TopoNode(
                id=i,
                pose=Pose3D(R=np.eye(3), t=np.array([i * 5.0, i * 0.1, 0.0])),
                descriptor=np.random.randn(128),
                keyframe=KeyFrame(id=i, pose=Pose3D.identity())
            )
            graph.add_node(node)

        ugv_traj = np.array([[i * 5.0, i * 0.1, 0.0] for i in range(8)])

        result = checker.check_consistency(graph, ugv_traj)
        assert 'needs_correction' in result
        assert 'drift_m' in result
        assert 'overlap_ratio' in result

    def test_generate_report(self, checker):
        """测试一致性报告生成"""
        report = checker.generate_report()
        assert 'checks_performed' in report
        assert 'corrections_applied' in report
        assert 'drift_trend' in report
        assert 'is_drifting' in report


class TestLoopClosure:
    """回环检测模块测试"""

    @pytest.fixture
    def detector(self):
        from src.slam.loop_closure import LoopClosureDetector
        return LoopClosureDetector()

    def test_set_uav_topo_graph(self, detector):
        """测试设置 UAV 拓扑图"""
        graph = TopoGraph()
        node = TopoNode(
            id=0,
            pose=Pose3D.identity(),
            descriptor=np.random.randn(128),
            keyframe=KeyFrame(id=0, pose=Pose3D.identity())
        )
        graph.add_node(node)
        detector.set_uav_topo_graph(graph)
        assert detector._uav_topo_graph is not None

    def test_add_ugv_keyframe(self, detector):
        """测试添加 UGV 关键帧"""
        kf = KeyFrame(id=0, pose=Pose3D.identity())
        detector.add_ugv_keyframe(kf)
        assert len(detector._ugv_keyframes) == 1

    def test_detect_loop_without_graph(self, detector):
        """测试无拓扑图时的回环检测"""
        kf = KeyFrame(id=0, pose=Pose3D.identity(),
                       descriptors=np.random.randn(10, 256))
        candidates = detector.detect_loop(kf)
        assert len(candidates) == 0

    def test_generate_report(self, detector):
        """测试回环检测报告"""
        report = detector.generate_report()
        assert 'total_candidates' in report
        assert 'valid_candidates' in report
        assert 'ugv_keyframes' in report

    def test_orb_fallback_toggle(self, detector):
        """测试 ORB 降级开关"""
        assert not detector._use_orb_fallback
        detector.enable_orb_fallback()
        assert detector._use_orb_fallback
        detector.disable_orb_fallback()
        assert not detector._use_orb_fallback


class TestRobustnessEnhancer:
    """鲁棒性增强器测试"""

    @pytest.fixture
    def enhancer(self):
        from src.slam.robustness import RobustnessEnhancer
        return RobustnessEnhancer()

    def test_initial_mode(self, enhancer):
        """测试初始模式为全融合"""
        assert enhancer.current_mode.name == 'FULL_FUSION'

    def test_assess_full_fusion(self, enhancer):
        """测试全传感器正常时应保持 FULL_FUSION"""
        sensor_data = {
            'feature_count': 200,
            'gnss_signal': 0.9,
            'image': np.zeros((480, 640, 3), dtype=np.uint8),
            'lidar_valid': True
        }
        mode = enhancer.assess_mode(sensor_data)
        assert mode.name == 'FULL_FUSION'

    def test_visual_degraded(self, enhancer):
        """测试视觉退化时应切换为 LIDAR_IMU_ONLY"""
        # 多次提交低特征数以触发退化
        for _ in range(10):
            enhancer.health_monitor.update_visual_status(20)  # < 30 → FAILED

        mode = enhancer.assess_mode({
            'feature_count': 20,
            'gnss_signal': 0.9,
            'image': np.zeros((480, 640, 3)),
            'lidar_valid': True
        })
        assert mode.name == 'LIDAR_IMU_ONLY'

    def test_gnss_lost_drift(self, enhancer):
        """测试 GNSS 丢失时的漂移估计"""
        current = np.array([
            [1, 0, 0, 3.0],
            [0, 1, 0, 1.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0]
        ])
        last_gnss = np.eye(4)

        drift = enhancer.estimate_drift_without_gnss(current, last_gnss, 10.0)
        # 漂移 ≈ sqrt(3² + 1²) = √10 ≈ 3.16
        assert abs(drift - 3.16) < 1.0

    def test_drift_acceptable(self, enhancer):
        """测试漂移可接受性判断"""
        enhancer._drift_since_gnss_lost = 3.0
        assert enhancer.is_drift_acceptable(max_drift_m=5.0)
        assert not enhancer.is_drift_acceptable(max_drift_m=2.0)

    def test_reset(self, enhancer):
        """测试重置鲁棒性增强器"""
        enhancer.health_monitor.update_visual_status(10)
        enhancer.reset()
        assert enhancer.current_mode.name == 'FULL_FUSION'
        assert enhancer.health_monitor.status['visual'].name == 'NORMAL'

    def test_low_texture_detection(self, enhancer):
        """测试弱纹理检测"""
        # 纯色图像（弱纹理）
        low_texture = np.ones((480, 640), dtype=np.float64) * 128
        score = enhancer.degeneracy_detector.detect_low_texture(low_texture)
        assert 0 <= score <= 1.0

        # 随机纹理图像（丰富纹理）
        rich_texture = np.random.randint(0, 256, (480, 640)).astype(np.float64)
        rich_score = enhancer.degeneracy_detector.detect_low_texture(rich_texture)
        assert rich_score >= score

    def test_generate_report(self, enhancer):
        """测试鲁棒性报告生成"""
        report = enhancer.generate_report()
        assert 'current_mode' in report
        assert 'sensor_status' in report
        assert 'visual_degraded' in report
        assert 'gnss_degraded' in report
        assert 'mode_switches' in report


class TestGeometricConsistencyChecker:
    """几何一致性校验器测试"""

    @pytest.fixture
    def checker(self):
        from src.slam.robustness import GeometricConsistencyChecker
        return GeometricConsistencyChecker()

    def test_perfect_match(self, checker):
        """测试完美匹配应通过校验"""
        np.random.seed(42)
        src = np.random.rand(20, 3)
        T = np.eye(4)
        dst = src.copy()

        is_valid, ratio = checker.check_loop_closure(src, dst, T)
        assert is_valid

    def test_poor_match(self, checker):
        """测试不匹配应不通过校验"""
        src = np.random.rand(20, 3)
        dst = np.random.rand(20, 3) * 10  # 完全不同
        T = np.eye(4)

        is_valid, ratio = checker.check_loop_closure(src, dst, T)
        assert not is_valid

    def test_ransac_alignment(self, checker):
        """测试 RANSAC 对齐"""
        np.random.seed(42)
        src = np.random.rand(50, 3)
        T_offset = np.eye(4)
        T_offset[:3, 3] = [2.0, 1.0, 0.0]
        dst = (T_offset[:3, :3] @ src.T).T + T_offset[:3, 3]

        result = checker.ransac_alignment(src, dst, max_iter=50)
        if result is not None:
            T_est, inliers = result
            assert T_est.shape == (4, 4)
            assert len(inliers) >= 3

    def test_ransac_insufficient_points(self, checker):
        """测试点数不足时 RANSAC 返回 None"""
        src = np.random.rand(2, 3)
        dst = np.random.rand(2, 3)
        result = checker.ransac_alignment(src, dst)
        assert result is None
