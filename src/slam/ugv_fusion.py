"""
UGV 多传感器融合定位模块 (3A.2)

实现基于 FAST-LIO2 风格的 LiDAR-IMU 前端 + 双目视觉特征跟踪：
  - LiDAR-IMU 紧耦合前端
  - 双目视觉里程计
  - 因子图融合：e_imu + e_lidar + e_vis
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from collections import deque
import time

from .data_types import (
    Pose3D, KeyFrame, ScanData, IMUData, StereoImage,
    FactorGraph, FactorEdge, FactorType, UGVFusionConfig
)


class IMUPreintegrator:
    """IMU 预积分器

    在两帧之间对 IMU 测量值进行预积分，计算相对位姿变化。
    """

    def __init__(self, config: UGVFusionConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.delta_R = np.eye(3)
        self.delta_v = np.zeros(3)
        self.delta_p = np.zeros(3)
        self.dt = 0.0
        self._last_gyro = None

    def integrate(self, imu: IMUData) -> bool:
        """积分单帧 IMU 数据"""
        if self._last_gyro is None:
            self._last_gyro = imu.gyro
            return False

        dt = 0.01  # 假设 IMU 频率100Hz
        self.dt += dt

        # 中值积分
        gyro_mid = (self._last_gyro + imu.gyro) / 2
        accel_mid = imu.accel  # 简化：只用当前加速度

        # 旋转增量
        dR = self._so3_exp(gyro_mid * dt)
        self.delta_R = self.delta_R @ dR

        # 速度增量
        self.delta_v += self.delta_R @ accel_mid * dt

        # 位置增量
        self.delta_p += self.delta_v * dt + 0.5 * self.delta_R @ accel_mid * dt * dt

        self._last_gyro = imu.gyro
        return True

    def get_relative_pose(self) -> Pose3D:
        """获取预积分的相对位姿"""
        return Pose3D(R=self.delta_R.copy(), t=self.delta_p.copy())

    def get_information_matrix(self) -> np.ndarray:
        """获取预积分信息矩阵 (6x6)"""
        acc_noise = self.config.imu_accel_noise
        gyro_noise = self.config.imu_gyro_noise
        info = np.eye(6)
        info[:3, :3] /= (acc_noise ** 2 * self.dt + 1e-8)
        info[3:, 3:] /= (gyro_noise ** 2 * self.dt + 1e-8)
        return info

    @staticmethod
    def _so3_exp(omega: np.ndarray) -> np.ndarray:
        """SO(3) 指数映射"""
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3)
        axis = omega / theta
        K = np.array([[0, -axis[2], axis[1]],
                       [axis[2], 0, -axis[0]],
                       [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K


class LiDAROdometry:
    """FAST-LIO2 风格的 LiDAR 里程计

    实现 scan-to-map 的 ICP 配准，估计帧间位姿变化。
    """

    def __init__(self, config: UGVFusionConfig):
        self.config = config
        self._local_map: Optional[np.ndarray] = None
        self._last_scan: Optional[np.ndarray] = None

    def process_scan(self, scan: ScanData, initial_guess: Optional[Pose3D] = None) \
            -> Tuple[Pose3D, float]:
        """处理单帧 LiDAR 扫描

        Args:
            scan: LiDAR 扫描数据
            initial_guess: 初始位姿估计（来自IMU预积分）

        Returns:
            (相对位姿变化, 配准分数)
        """
        if self._last_scan is None:
            self._last_scan = scan.points.copy()
            return Pose3D.identity(), 1.0

        if initial_guess is None:
            initial_guess = Pose3D.identity()

        # 简化的 ICP 配准
        src = scan.points.copy()
        dst = self._last_scan.copy()

        # 随机采样减少计算量
        n_samples = min(500, len(src), len(dst))
        src_idx = np.random.choice(len(src), n_samples, replace=False)
        dst_idx = np.random.choice(len(dst), n_samples, replace=False)
        src_pts = src[src_idx]
        dst_pts = dst[dst_idx]

        # 应用初始位姿
        transform = initial_guess
        best_score = float('inf')
        best_transform = transform

        for _ in range(self.config.scan_to_map_max_iter):
            # 变换源点云
            transformed = (transform.R @ src_pts.T).T + transform.t

            # 最近邻匹配
            diffs = transformed[:, None, :] - dst_pts[None, :, :]
            dists = np.linalg.norm(diffs, axis=2)
            min_dists = np.min(dists, axis=1)
            correspondences = np.argmin(dists, axis=1)

            score = np.mean(min_dists)

            if abs(score - best_score) < self.config.scan_to_map_convergence:
                break
            best_score = score

            # 最小二乘求解最优变换
            matched_dst = dst_pts[correspondences]
            centroid_src = np.mean(transformed, axis=0)
            centroid_dst = np.mean(matched_dst, axis=0)

            H = (transformed - centroid_src).T @ (matched_dst - centroid_dst)
            U, _, Vt = np.linalg.svd(H)
            R_new = Vt.T @ U.T
            if np.linalg.det(R_new) < 0:
                Vt[-1] *= -1
                R_new = Vt.T @ U.T
            t_new = centroid_dst - R_new @ centroid_src

            transform = Pose3D(R=R_new, t=t_new)

        self._last_scan = scan.points.copy()
        return transform, 1.0 / (1.0 + best_score)


class StereoVisualOdometry:
    """双目视觉里程计

    基于双目特征匹配估计帧间运动。
    """

    def __init__(self, config: UGVFusionConfig):
        self.config = config
        self._last_features: Optional[np.ndarray] = None
        self._last_3d_points: Optional[np.ndarray] = None
        self._baseline = 0.12  # 双目基线 (m)
        self._focal_length = 800.0  # 焦距 (pixels)

    def process_stereo(self, stereo: StereoImage) -> Tuple[Optional[Pose3D], int]:
        """处理双目图像对

        Returns:
            (相对位姿变化, 内点数量)
        """
        if stereo.left_features is None or stereo.right_features is None:
            return None, 0

        # 计算视差 → 三维点
        points_3d = []
        for fl, fr in zip(stereo.left_features, stereo.right_features):
            disparity = abs(fl[0] - fr[0])
            if disparity < 1.0:
                continue
            z = self._focal_length * self._baseline / disparity
            x = (fl[0] - 640) * z / self._focal_length
            y = (fl[1] - 360) * z / self._focal_length
            points_3d.append([x, y, z])

        if len(points_3d) < self.config.visual_feature_thresh:
            return None, len(points_3d)

        pts = np.array(points_3d)

        if self._last_3d_points is None:
            self._last_3d_points = pts
            return Pose3D.identity(), len(points_3d)

        # PnP 求解帧间运动（简化：3D-3D 对应）
        n = min(len(pts), len(self._last_3d_points))
        centroid_new = np.mean(pts[:n], axis=0)
        centroid_old = np.mean(self._last_3d_points[:n], axis=0)

        H = (pts[:n] - centroid_new).T @ (self._last_3d_points[:n] - centroid_old)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = centroid_old - R @ centroid_new

        self._last_3d_points = pts
        return Pose3D(R=R, t=t), n


class UGVMultiSensorFusion:
    """UGV 多传感器融合定位系统

    融合 LiDAR、IMU、双目视觉，通过因子图优化输出稳定位姿。
    """

    def __init__(self, config: Optional[UGVFusionConfig] = None):
        self.config = config or UGVFusionConfig()
        self.imu_integrator = IMUPreintegrator(self.config)
        self.lidar_odom = LiDAROdometry(self.config)
        self.visual_odom = StereoVisualOdometry(self.config)
        self.factor_graph = FactorGraph()

        # 状态
        self._current_pose = Pose3D.identity()
        self._poses: List[Pose3D] = []
        self._keyframes: List[KeyFrame] = []
        self._imu_buffer: deque = deque(maxlen=200)
        self._frame_count = 0
        self._node_id_counter = 0

        # 初始化因子图（先验节点）
        self.factor_graph.add_node(0, Pose3D.identity(), "ugv")
        self.factor_graph.add_factor(FactorEdge(
            src_id=0, dst_id=0, factor_type=FactorType.PRIOR,
            measurement=np.zeros(6),
            information=np.eye(6) * 1000
        ))
        self._node_id_counter = 1

    def process_imu(self, imu: IMUData) -> None:
        """处理 IMU 数据（高频）"""
        self._imu_buffer.append(imu)
        self.imu_integrator.integrate(imu)

    def process_lidar(self, scan: ScanData) -> Optional[Pose3D]:
        """处理 LiDAR 扫描，估计里程计"""
        imu_guess = self.imu_integrator.get_relative_pose()
        rel_pose, score = self.lidar_odom.process_scan(scan, imu_guess)
        self.imu_integrator.reset()

        # 更新当前位姿
        self._current_pose = self._current_pose.compose(rel_pose)
        self._poses.append(self._current_pose)

        # 添加 LiDAR 因子
        self.factor_graph.add_node(self._node_id_counter, self._current_pose, "ugv")
        self.factor_graph.add_factor(FactorEdge(
            src_id=self._node_id_counter - 1,
            dst_id=self._node_id_counter,
            factor_type=FactorType.LIDAR,
            measurement=rel_pose.t.tolist() + [0, 0, 0],  # 6D
            information=np.eye(6) * score * 10
        ))
        self._node_id_counter += 1

        self._frame_count += 1
        return self._current_pose

    def process_stereo(self, stereo: StereoImage) -> Optional[Pose3D]:
        """处理双目图像，添加视觉约束"""
        rel_pose, inliers = self.visual_odom.process_stereo(stereo)
        if rel_pose is None or inliers < self.config.visual_feature_thresh:
            return None

        # 添加视觉因子
        self.factor_graph.add_factor(FactorEdge(
            src_id=self._node_id_counter - 1,
            dst_id=self._node_id_counter,
            factor_type=FactorType.VISUAL,
            measurement=rel_pose.t.tolist() + [0, 0, 0],
            information=np.eye(6) * inliers / 100.0
        ))

        return self._current_pose

    def add_gnss_constraint(self, gnss_pose: Pose3D, covariance: np.ndarray) -> None:
        """添加 GNSS/RTK 绝对约束"""
        info = np.linalg.inv(covariance + np.eye(3) * 1e-6)
        self.factor_graph.add_factor(FactorEdge(
            src_id=self._node_id_counter - 1,
            dst_id=self._node_id_counter - 1,
            factor_type=FactorType.GPS,
            measurement=gnss_pose.t,
            information=info
        ))

    def optimize(self) -> Pose3D:
        """执行因子图优化（简化版高斯-牛顿）"""
        # 对于原型，使用简单的滑动窗口平滑
        if len(self._poses) >= 5:
            window = self._poses[-5:]
            avg_t = np.mean([p.t for p in window], axis=0)
            # 平均旋转（使用四元数平均）
            quats = [self._rotation_to_quat(p.R) for p in window]
            avg_q = np.mean(quats, axis=0)
            avg_q /= np.linalg.norm(avg_q)
            avg_R = self._quat_to_rotation(avg_q)
            self._current_pose = Pose3D(R=avg_R, t=avg_t)

        return self._current_pose

    @staticmethod
    def _rotation_to_quat(R: np.ndarray) -> np.ndarray:
        """旋转矩阵 → 四元数"""
        w = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        if w > 1e-6:
            return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w),
                             (R[0, 2] - R[2, 0]) / (4 * w),
                             (R[1, 0] - R[0, 1]) / (4 * w)])
        return np.array([0, 0, 0, 1])

    @staticmethod
    def _quat_to_rotation(q: np.ndarray) -> np.ndarray:
        """四元数 → 旋转矩阵"""
        w, x, y, z = q
        return np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
            [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
            [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y]
        ])

    def get_trajectory(self) -> np.ndarray:
        """获取位姿轨迹"""
        if not self._poses:
            return np.empty((0, 3))
        return np.array([p.t for p in self._poses])

    def generate_report(self) -> dict:
        """生成融合定位报告"""
        traj = self.get_trajectory()
        if len(traj) > 1:
            total_dist = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
        else:
            total_dist = 0.0
        return {
            "poses": len(self._poses),
            "keyframes": len(self._keyframes),
            "factor_nodes": len(self.factor_graph.nodes),
            "factor_edges": len(self.factor_graph.edges),
            "trajectory_length_m": float(total_dist),
            "output_frequency_hz": self.config.output_frequency
        }
