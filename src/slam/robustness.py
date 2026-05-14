"""
鲁棒性增强模块 (3B.2)

处理极端场景下的退化问题：
  - 弱纹理场景下 V-SLAM 失效 → 纯 LiDAR-IMU 降级
  - GNSS 信号丢失 → 纯视觉/激光定位保持
  - 回环误匹配剔除（几何一致性校验）
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from enum import IntEnum
from collections import deque
import time


class SensorStatus(IntEnum):
    """传感器状态"""
    NORMAL = 0           # 正常工作
    DEGRADED = 1         # 性能下降
    FAILED = 2           # 完全失效
    UNKNOWN = 3          # 未知状态


class SystemMode(IntEnum):
    """系统运行模式"""
    FULL_FUSION = 0      # 全传感器融合 (LiDAR+IMU+视觉+GNSS)
    LIDAR_IMU_ONLY = 1   # 纯 LiDAR-IMU（视觉失效）
    VISUAL_ONLY = 2      # 纯视觉定位（LiDAR 失效）
    IMU_ONLY = 3         # 纯 IMU 积分（极端退化）
    GNSS_AIDED = 4       # GNSS 辅助（信号恢复中）


class SensorHealthMonitor:
    """传感器健康状态监控"""

    def __init__(self):
        self.status = {
            'lidar': SensorStatus.NORMAL,
            'imu': SensorStatus.NORMAL,
            'visual': SensorStatus.NORMAL,
            'gnss': SensorStatus.NORMAL,
        }
        self._feature_counts: deque = deque(maxlen=50)
        self._gnss_signal_strength: deque = deque(maxlen=50)

    def reset(self):
        """重置所有状态"""
        self.status = {
            'lidar': SensorStatus.NORMAL,
            'imu': SensorStatus.NORMAL,
            'visual': SensorStatus.NORMAL,
            'gnss': SensorStatus.NORMAL,
        }
        self._feature_counts.clear()
        self._gnss_signal_strength.clear()

    def update_visual_status(self, feature_count: int):
        """更新视觉状态"""
        self._feature_counts.append(feature_count)
        if len(self._feature_counts) >= 3:  # 降低采样要求
            avg = sum(self._feature_counts) / len(self._feature_counts)
            if avg < 30:
                self.status['visual'] = SensorStatus.FAILED
            elif avg < 100:
                self.status['visual'] = SensorStatus.DEGRADED
            else:
                self.status['visual'] = SensorStatus.NORMAL

    def update_gnss_status(self, signal_strength: float):
        """更新 GNSS 状态"""
        self._gnss_signal_strength.append(signal_strength)
        if len(self._gnss_signal_strength) >= 3:  # 降低采样要求
            avg = sum(self._gnss_signal_strength) / len(self._gnss_signal_strength)
            if avg < 0.1:
                self.status['gnss'] = SensorStatus.FAILED
            elif avg < 0.5:
                self.status['gnss'] = SensorStatus.DEGRADED
            else:
                self.status['gnss'] = SensorStatus.NORMAL

    def get_effective_sensors(self) -> List[str]:
        """获取当前有效的传感器列表"""
        return [name for name, s in self.status.items()
                if s != SensorStatus.FAILED]


class DegradationDetector:
    """退化检测器

    检测 SLAM 系统的性能退化：
      - 弱纹理场景检测
      - 特征贫瘠区域检测
      - 运动退化检测
    """

    def __init__(self):
        self._texture_history: deque = deque(maxlen=100)
        self._degeneracy_threshold = 0.3

    def detect_low_texture(self, image: np.ndarray) -> float:
        """检测弱纹理场景

        Returns:
            纹理丰富度分数 [0, 1]，越低越弱纹理
        """
        if len(image.shape) == 3:
            gray = np.mean(image, axis=2)
        else:
            gray = image.astype(np.float64)

        # 计算梯度幅值
        gy, gx = np.gradient(gray)
        gradient_mag = np.sqrt(gx ** 2 + gy ** 2)
        texture_score = np.mean(gradient_mag) / 255.0
        texture_score = min(1.0, texture_score * 10)

        self._texture_history.append(texture_score)
        return texture_score

    def is_low_texture(self, image: np.ndarray) -> bool:
        """判断当前是否为弱纹理场景"""
        score = self.detect_low_texture(image)
        return score < self._degeneracy_threshold

    def detect_motion_degeneracy(self, poses: List[np.ndarray]) -> float:
        """检测运动退化（如纯旋转导致三角化失败）"""
        if len(poses) < 3:
            return 0.0

        translations = np.array([p[:3, 3] if p.shape == (4, 4) else p[:3]
                                  for p in poses[-10:]])
        if len(translations) < 2:
            return 0.0

        # 计算平移变化
        diffs = np.diff(translations, axis=0)
        avg_translation = np.mean(np.linalg.norm(diffs, axis=1))

        # 平移量小 = 高退化风险
        if avg_translation < 0.01:
            return 1.0
        elif avg_translation < 0.1:
            return 0.5
        return 0.0

    def get_degeneracy_direction(self, H: np.ndarray) -> Optional[np.ndarray]:
        """检测优化问题的退化方向"""
        eigenvalues, eigenvectors = np.linalg.eigh(H)
        min_eigenvalue = eigenvalues[0]
        if min_eigenvalue < 1e-6:
            return eigenvectors[:, 0]
        return None


class GeometricConsistencyChecker:
    """几何一致性校验器

    用于回环误匹配剔除。
    """

    def __init__(self, inlier_threshold: float = 0.05,
                 min_inlier_ratio: float = 0.3):
        self.inlier_threshold = inlier_threshold
        self.min_inlier_ratio = min_inlier_ratio

    def check_loop_closure(self, src_pts: np.ndarray, dst_pts: np.ndarray,
                            transform: np.ndarray) -> Tuple[bool, float]:
        """校验回环匹配的几何一致性

        Args:
            src_pts: 源点云 (N×3)
            dst_pts: 目标点云 (N×3)
            transform: 4x4 变换矩阵

        Returns:
            (是否通过校验, 内点比例)
        """
        if len(src_pts) < 5 or len(dst_pts) < 5:
            return False, 0.0

        # 变换源点云
        R = transform[:3, :3]
        t = transform[:3, 3]
        transformed = (R @ src_pts.T).T + t

        # 计算匹配距离
        diffs = transformed[:, None, :] - dst_pts[None, :, :]
        min_dists = np.min(np.linalg.norm(diffs, axis=2), axis=1)

        inliers = min_dists < self.inlier_threshold
        inlier_ratio = np.mean(inliers)

        return inlier_ratio >= self.min_inlier_ratio, float(inlier_ratio)

    def ransac_alignment(self, src: np.ndarray, dst: np.ndarray,
                          max_iter: int = 100,
                          sample_size: int = 3) -> Optional[Tuple[np.ndarray, List[int]]]:
        """RANSAC 点云对齐（用于误匹配剔除）

        Returns:
            (最优变换矩阵, 内点索引列表)
        """
        if len(src) < sample_size or len(dst) < sample_size:
            return None

        best_inliers = []
        best_transform = None

        for _ in range(max_iter):
            # 随机采样
            indices = np.random.choice(len(src), sample_size, replace=False)
            src_sample = src[indices]
            dst_sample = dst[indices]

            # 计算变换
            centroid_src = np.mean(src_sample, axis=0)
            centroid_dst = np.mean(dst_sample, axis=0)
            H = (src_sample - centroid_src).T @ (dst_sample - centroid_dst)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            t = centroid_dst - R @ centroid_src

            transform = np.eye(4)
            transform[:3, :3] = R
            transform[:3, 3] = t

            # 计算内点
            transformed = (R @ src.T).T + t
            diffs = np.linalg.norm(transformed - dst, axis=1)
            inliers = np.where(diffs < self.inlier_threshold)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers.tolist()
                best_transform = transform

        if best_transform is None or len(best_inliers) < 3:
            return None

        return best_transform, best_inliers


class RobustnessEnhancer:
    """鲁棒性增强器

    统一管理系统的降级与恢复策略。
    """

    def __init__(self):
        self.health_monitor = SensorHealthMonitor()
        self.degeneracy_detector = DegradationDetector()
        self.geometric_checker = GeometricConsistencyChecker()
        self.current_mode = SystemMode.FULL_FUSION
        self._mode_history: List[SystemMode] = []
        self._gnss_lost_start: Optional[float] = None
        self._drift_since_gnss_lost: float = 0.0

    def reset(self):
        """重置所有内部状态"""
        self.health_monitor.reset()
        self.current_mode = SystemMode.FULL_FUSION
        self._mode_history.clear()
        self._gnss_lost_start = None
        self._drift_since_gnss_lost = 0.0

    def assess_mode(self, sensor_data: dict) -> SystemMode:
        """评估当前应使用的运行模式

        Args:
            sensor_data: {'feature_count': int, 'gnss_signal': float,
                          'image': np.ndarray, 'lidar_valid': bool}
        """
        self.health_monitor.update_visual_status(
            sensor_data.get('feature_count', 0))
        self.health_monitor.update_gnss_status(
            sensor_data.get('gnss_signal', 0.0))

        effective = self.health_monitor.get_effective_sensors()

        # 判断模式
        lidar_ok = 'lidar' in effective
        imu_ok = 'imu' in effective
        visual_ok = 'visual' in effective
        gnss_ok = 'gnss' in effective

        if lidar_ok and imu_ok and visual_ok and gnss_ok:
            mode = SystemMode.FULL_FUSION
        elif lidar_ok and imu_ok:
            mode = SystemMode.LIDAR_IMU_ONLY
        elif visual_ok:
            mode = SystemMode.VISUAL_ONLY
        elif gnss_ok:
            mode = SystemMode.GNSS_AIDED
        else:
            mode = SystemMode.IMU_ONLY

        if mode != self.current_mode:
            self._mode_history.append(self.current_mode)
            self.current_mode = mode

        return mode

    def estimate_drift_without_gnss(self, current_pose: np.ndarray,
                                     last_gnss_pose: np.ndarray,
                                     elapsed_seconds: float) -> float:
        """估计 GNSS 失锁期间的漂移

        Returns:
            漂移距离 (m)
        """
        if current_pose.shape == (4, 4):
            current_t = current_pose[:3, 3]
            last_t = last_gnss_pose[:3, 3]
        else:
            current_t = current_pose[:3]
            last_t = last_gnss_pose[:3]

        drift = np.linalg.norm(current_t - last_t)
        self._drift_since_gnss_lost = drift
        return drift

    def is_drift_acceptable(self, max_drift_m: float = 5.0) -> bool:
        """判断漂移是否在可接受范围内"""
        return self._drift_since_gnss_lost < max_drift_m

    def filter_loop_candidates(self, candidates: List,
                                src_pts: np.ndarray,
                                dst_pts: np.ndarray) -> List:
        """过滤回环候选（剔除误匹配）"""
        filtered = []
        for candidate in candidates:
            if candidate.relative_pose is None:
                continue
            transform = candidate.relative_pose.T
            is_valid, ratio = self.geometric_checker.check_loop_closure(
                src_pts, dst_pts, transform)
            if is_valid:
                filtered.append(candidate)
        return filtered

    def generate_report(self) -> dict:
        """生成鲁棒性报告"""
        return {
            "current_mode": self.current_mode.name,
            "sensor_status": {k: v.name for k, v in self.health_monitor.status.items()},
            "visual_degraded": self.health_monitor.status['visual'] != SensorStatus.NORMAL,
            "gnss_degraded": self.health_monitor.status['gnss'] != SensorStatus.NORMAL,
            "drift_since_gnss_lost_m": round(self._drift_since_gnss_lost, 3),
            "mode_switches": len(self._mode_history)
        }
