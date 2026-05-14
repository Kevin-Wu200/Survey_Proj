"""
空地地图一致性校验模块 (3B.3)

实现 UAV 拓扑图与 UGV 局部图的长期一致性维护：
  - 定期比对 UAV 拓扑图与 UGV 局部图的重叠区域
  - 自动检测地图漂移并触发重校正
  - 长时间运行时地图一致性保持
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from collections import deque
import time

from .data_types import Pose3D, TopoGraph


class OverlapDetector:
    """重叠区域检测器

    识别 UAV 拓扑图与 UGV 局部图之间的重叠区域。
    """

    def __init__(self, overlap_radius: float = 20.0):
        self.overlap_radius = overlap_radius

    def find_overlap(self, uav_graph: TopoGraph,
                      ugv_trajectory: np.ndarray) -> List[Tuple[int, int]]:
        """查找 UAV-UGV 重叠区域

        Returns:
            [(uav_node_id, ugv_frame_index)]
        """
        overlaps = []

        for uav_id, uav_node in uav_graph.nodes.items():
            uav_pos = uav_node.pose.t[:2]  # 仅水平位置

            for ugv_idx, ugv_pos in enumerate(ugv_trajectory):
                ugv_pos_2d = ugv_pos[:2]
                dist = np.linalg.norm(uav_pos - ugv_pos_2d)

                if dist < self.overlap_radius:
                    overlaps.append((uav_id, ugv_idx))

        return overlaps

    def compute_overlap_ratio(self, uav_graph: TopoGraph,
                               ugv_trajectory: np.ndarray) -> float:
        """计算重叠比例"""
        if uav_graph.num_nodes == 0 or len(ugv_trajectory) == 0:
            return 0.0

        overlaps = self.find_overlap(uav_graph, ugv_trajectory)
        uav_overlapped = len(set(o[0] for o in overlaps))
        return uav_overlapped / uav_graph.num_nodes


class DriftDetector:
    """漂移检测器

    检测 UGV 局部图相对于 UAV 拓扑图的累积漂移。
    """

    def __init__(self, drift_threshold: float = 2.0,
                 window_size: int = 50):
        self.drift_threshold = drift_threshold
        self.window_size = window_size
        self._drift_history: deque = deque(maxlen=window_size)
        self._cumulative_drift: float = 0.0

    def measure_drift(self, uav_pose: Pose3D, ugv_pose: Pose3D,
                       T_uav2ugv: Optional[Pose3D] = None) -> float:
        """测量 UAV-UGV 位姿偏差

        Args:
            uav_pose: UAV 位姿
            ugv_pose: UGV 位姿
            T_uav2ugv: UAV→UGV 坐标系变换

        Returns:
            漂移量 (m)
        """
        if T_uav2ugv is not None:
            uav_in_ugv = T_uav2ugv.compose(uav_pose)
            drift = np.linalg.norm(uav_in_ugv.t - ugv_pose.t)
        else:
            drift = np.linalg.norm(uav_pose.t - ugv_pose.t)

        self._drift_history.append(drift)
        self._cumulative_drift += drift
        return drift

    def is_drifting(self) -> bool:
        """判断是否正在漂移"""
        if len(self._drift_history) < 10:
            return False
        recent = list(self._drift_history)[-10:]
        avg_drift = sum(recent) / len(recent)
        return avg_drift > self.drift_threshold

    def get_drift_trend(self) -> float:
        """获取漂移趋势（正=增大，负=减小）"""
        if len(self._drift_history) < 20:
            return 0.0
        half = len(self._drift_history) // 2
        recent = list(self._drift_history)[-half:]
        older = list(self._drift_history)[:half]
        return (sum(recent) - sum(older)) / half

    def reset(self):
        """重置漂移检测器"""
        self._drift_history.clear()
        self._cumulative_drift = 0.0


class MapAligner:
    """地图对齐器

    当检测到漂移时，触发重校正。
    """

    def __init__(self, max_iter: int = 50, convergence: float = 1e-6):
        self.max_iter = max_iter
        self.convergence = convergence

    def align_maps(self, uav_points: np.ndarray,
                    ugv_points: np.ndarray) -> Optional[Pose3D]:
        """对齐两张地图

        Args:
            uav_points: UAV 地图中的点 (M×3)
            ugv_points: UGV 地图中的点 (N×3)

        Returns:
            对齐变换（UAV→UGV 修正）
        """
        if len(uav_points) < 3 or len(ugv_points) < 3:
            return None

        # 随机采样以加快计算
        n_samples = min(200, len(uav_points), len(ugv_points))
        uav_idx = np.random.choice(len(uav_points), n_samples, replace=False)
        ugv_idx = np.random.choice(len(ugv_points), n_samples, replace=False)
        src = uav_points[uav_idx]
        dst = ugv_points[ugv_idx]

        # 初始对齐
        centroid_src = np.mean(src, axis=0)
        centroid_dst = np.mean(dst, axis=0)
        H = (src - centroid_src).T @ (dst - centroid_dst)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = centroid_dst - R @ centroid_src

        best_transform = Pose3D(R=R, t=t)
        best_error = float('inf')

        for iteration in range(self.max_iter):
            # 变换源点
            transformed = (best_transform.R @ src.T).T + best_transform.t

            # 最近邻匹配
            diffs = transformed[:, None, :] - dst[None, :, :]
            min_dists = np.min(np.linalg.norm(diffs, axis=2), axis=1)
            correspondences = np.argmin(np.linalg.norm(diffs, axis=2), axis=1)

            current_error = np.mean(min_dists)
            if abs(current_error - best_error) < self.convergence:
                break
            best_error = current_error

            # 使用内点重新计算变换
            inlier_mask = min_dists < np.median(min_dists) * 2
            if np.sum(inlier_mask) < 3:
                break

            src_inliers = transformed[inlier_mask]
            dst_inliers = dst[correspondences][inlier_mask]

            c_src = np.mean(src_inliers, axis=0)
            c_dst = np.mean(dst_inliers, axis=0)
            H = (src_inliers - c_src).T @ (dst_inliers - c_dst)
            U, _, Vt = np.linalg.svd(H)
            R_new = Vt.T @ U.T
            if np.linalg.det(R_new) < 0:
                Vt[-1] *= -1
                R_new = Vt.T @ U.T
            t_new = c_dst - R_new @ c_src

            best_transform = Pose3D(R=R_new, t=t_new)

        return best_transform


class ConsistencyChecker:
    """空地地图一致性校验器

    定期比对并维护 UAV 和 UGV 地图的长期一致性。
    """

    def __init__(self, check_interval: float = 10.0,
                 drift_threshold: float = 2.0):
        self.check_interval = check_interval
        self.overlap_detector = OverlapDetector()
        self.drift_detector = DriftDetector(drift_threshold)
        self.map_aligner = MapAligner()
        self._last_check_time = 0.0
        self._check_count = 0
        self._correction_count = 0
        self._overlap_history: List[float] = []

    def check_consistency(self, uav_graph: TopoGraph,
                           ugv_trajectory: np.ndarray,
                           T_uav2ugv: Optional[Pose3D] = None,
                           uav_points: Optional[np.ndarray] = None,
                           ugv_points: Optional[np.ndarray] = None) -> dict:
        """执行一致性检查

        Returns:
            {
                'needs_correction': bool,
                'drift_m': float,
                'overlap_ratio': float,
                'correction_pose': Optional[Pose3D]
            }
        """
        result = {
            'needs_correction': False,
            'drift_m': 0.0,
            'overlap_ratio': 0.0,
            'correction_pose': None
        }

        # 只在间隔时间后检查
        now = time.time()
        if now - self._last_check_time < self.check_interval:
            return result
        self._last_check_time = now
        self._check_count += 1

        # 检测重叠区域
        overlap_ratio = self.overlap_detector.compute_overlap_ratio(
            uav_graph, ugv_trajectory)
        result['overlap_ratio'] = overlap_ratio
        self._overlap_history.append(overlap_ratio)

        # 在重叠区域中测量漂移
        overlaps = self.overlap_detector.find_overlap(uav_graph, ugv_trajectory)
        if overlaps:
            for uav_id, ugv_idx in overlaps[:min(10, len(overlaps))]:
                uav_pose = uav_graph.nodes[uav_id].pose
                ugv_pose = Pose3D(
                    R=np.eye(3),
                    t=ugv_trajectory[ugv_idx]
                )
                drift = self.drift_detector.measure_drift(
                    uav_pose, ugv_pose, T_uav2ugv)
                result['drift_m'] = max(result['drift_m'], drift)

        # 判断是否需要重校正
        if self.drift_detector.is_drifting():
            result['needs_correction'] = True

            # 执行重校正
            if uav_points is not None and ugv_points is not None:
                correction = self.map_aligner.align_maps(uav_points, ugv_points)
                if correction is not None:
                    result['correction_pose'] = correction
                    self._correction_count += 1

        return result

    def trigger_correction(self, uav_points: np.ndarray,
                            ugv_points: np.ndarray) -> Optional[Pose3D]:
        """手动触发重校正"""
        correction = self.map_aligner.align_maps(uav_points, ugv_points)
        if correction is not None:
            self._correction_count += 1
            self.drift_detector.reset()
        return correction

    def generate_report(self) -> dict:
        """生成一致性报告"""
        drift_trend = self.drift_detector.get_drift_trend()
        avg_overlap = np.mean(self._overlap_history) if self._overlap_history else 0.0

        return {
            "checks_performed": self._check_count,
            "corrections_applied": self._correction_count,
            "drift_trend": round(drift_trend, 4),
            "is_drifting": self.drift_detector.is_drifting(),
            "avg_overlap_ratio": round(avg_overlap, 4),
            "cumulative_drift_m": round(self.drift_detector._cumulative_drift, 3),
            "last_drift_m": round(
                self.drift_detector._drift_history[-1]
                if self.drift_detector._drift_history else 0.0, 3
            )
        }
