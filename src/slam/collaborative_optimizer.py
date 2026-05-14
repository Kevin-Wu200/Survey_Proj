"""
空地位姿协同优化模块 (3A.3)

实现 UAV 与 UGV 坐标系对齐与联合优化：
  - 手眼标定求解器（UAV → UGV 坐标系变换 T_uav2ugv）
  - 基于共视匹配点集的协同优化
  - 滑动时间窗口内联合优化
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from collections import deque

from .data_types import (
    Pose3D, KeyFrame, TopoGraph,
    FactorGraph, FactorEdge, FactorType,
    CollaborativeConfig
)
from .uav_topology import UAVTopologyMapper
from .ugv_fusion import UGVMultiSensorFusion


class HandEyeCalibrator:
    """手眼标定求解器

    求解 UAV 坐标系到 UGV 坐标系的变换 T_uav2ugv。
    基于 AX = XB 经典手眼标定问题。
    """

    def __init__(self, config: CollaborativeConfig):
        self.config = config
        self._uav_poses: List[Pose3D] = []
        self._ugv_poses: List[Pose3D] = []
        self.T_uav2ugv: Optional[Pose3D] = None

    def add_measurement_pair(self, uav_pose: Pose3D, ugv_pose: Pose3D):
        """添加一对同步测量"""
        self._uav_poses.append(uav_pose)
        self._ugv_poses.append(ugv_pose)

    def solve(self) -> Optional[Pose3D]:
        """求解手眼标定（最小二乘）

        T_ugv = T_uav2ugv * T_uav
        即最小化 Σ ||T_ugv_i - T * T_uav_i||²
        """
        if len(self._uav_poses) < 3:
            return None

        # 提取平移部分求解
        uav_t = np.array([p.t for p in self._uav_poses])
        ugv_t = np.array([p.t for p in self._ugv_poses])

        # Umeyama 方法：求解旋转和平移
        centroid_uav = np.mean(uav_t, axis=0)
        centroid_ugv = np.mean(ugv_t, axis=0)

        H = (uav_t - centroid_uav).T @ (ugv_t - centroid_ugv)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = centroid_ugv - R @ centroid_uav

        # 迭代优化旋转和平移
        best_R, best_t = R, t
        best_error = float('inf')

        for _ in range(self.config.hand_eye_max_iter):
            # 计算当前误差
            total_error = 0.0
            for i in range(len(uav_t)):
                predicted = best_R @ uav_t[i] + best_t
                error = np.linalg.norm(predicted - ugv_t[i])
                total_error += error

            if abs(total_error - best_error) < self.config.hand_eye_convergence:
                break
            best_error = total_error

            # 梯度下降步
            dR = np.zeros((3, 3))
            dt = np.zeros(3)
            for i in range(len(uav_t)):
                predicted = best_R @ uav_t[i] + best_t
                residual = predicted - ugv_t[i]
                dt -= 2 * residual / len(uav_t)
                dR -= 2 * np.outer(residual, uav_t[i]) / len(uav_t)

            alpha = 0.01
            best_R = best_R - alpha * dR
            best_t = best_t - alpha * dt

            # 保持旋转矩阵正交性
            U, _, Vt = np.linalg.svd(best_R)
            best_R = U @ Vt

        self.T_uav2ugv = Pose3D(R=best_R, t=best_t)
        return self.T_uav2ugv

    def get_transformation_error(self) -> float:
        """计算变换误差"""
        if self.T_uav2ugv is None or len(self._uav_poses) == 0:
            return float('inf')
        errors = []
        for uav_p, ugv_p in zip(self._uav_poses, self._ugv_poses):
            predicted = self.T_uav2ugv.R @ uav_p.t + self.T_uav2ugv.t
            errors.append(np.linalg.norm(predicted - ugv_p.t))
        return float(np.mean(errors))


class SlidingWindowOptimizer:
    """滑动窗口联合优化器

    在时间窗口内联合优化 UAV 和 UGV 的位姿。
    """

    def __init__(self, config: CollaborativeConfig):
        self.config = config
        self.window: deque = deque(maxlen=config.sliding_window_size)
        self._optimized_nodes: Dict[int, Pose3D] = {}

    def add_frame(self, node_id: int, pose: Pose3D,
                   node_type: str, constraints: List[Tuple[int, Pose3D, float]] = None):
        """添加一帧到滑动窗口"""
        self.window.append({
            'id': node_id,
            'pose': pose,
            'type': node_type,
            'constraints': constraints or []
        })

    def optimize(self) -> Dict[int, Pose3D]:
        """执行滑动窗口内联合优化（简化版位姿图优化）"""
        if len(self.window) < 3:
            return self._optimized_nodes

        # 构建局部因子图
        fg = FactorGraph()
        for item in self.window:
            fg.add_node(item['id'], item['pose'], item['type'])

        # 添加帧间约束（UAV/UGV 各自序列约束）
        items = list(self.window)
        for i in range(len(items) - 1):
            if items[i]['type'] == items[i + 1]['type']:
                rel_pose = items[i]['pose'].inverse().compose(items[i + 1]['pose'])
                fg.add_factor(FactorEdge(
                    src_id=items[i]['id'],
                    dst_id=items[i + 1]['id'],
                    factor_type=FactorType.LIDAR,
                    measurement=np.array(rel_pose.t.tolist() + [0, 0, 0]),
                    information=np.eye(6) * 10
                ))

        # 添加跨模态约束
        for item in items:
            for target_id, rel_pose, weight in item['constraints']:
                if target_id in fg.nodes:
                    fg.add_factor(FactorEdge(
                        src_id=item['id'],
                        dst_id=target_id,
                        factor_type=FactorType.LOOP,
                        measurement=np.array(rel_pose.t.tolist() + [0, 0, 0]),
                        information=np.eye(6) * weight
                    ))

        # 简化优化：加权平均
        for nid in fg.nodes:
            connected = fg.get_connected_nodes(nid)
            if not connected:
                self._optimized_nodes[nid] = fg.nodes[nid]
                continue

            # 收集所有连接的位姿估计
            t_estimates = [fg.nodes[nid].t]
            R_estimates = [fg.nodes[nid].R]
            weights = [1.0]

            for cid in connected:
                t_estimates.append(fg.nodes[cid].t)
                R_estimates.append(fg.nodes[cid].R)
                weights.append(0.5)

            weights = np.array(weights) / sum(weights)
            avg_t = np.average(t_estimates, axis=0, weights=weights)
            avg_R = np.average(R_estimates, axis=0, weights=weights)
            U, _, Vt = np.linalg.svd(avg_R)
            avg_R = U @ Vt

            self._optimized_nodes[nid] = Pose3D(R=avg_R, t=avg_t)

        return self._optimized_nodes


class CollaborativeOptimizer:
    """空地协同优化器

    集成手眼标定与滑动窗口优化，实现 UAV-UGV 协同定位。
    """

    def __init__(self, config: Optional[CollaborativeConfig] = None):
        self.config = config or CollaborativeConfig()
        self.calibrator = HandEyeCalibrator(self.config)
        self.sliding_optimizer = SlidingWindowOptimizer(self.config)
        self.T_uav2ugv: Optional[Pose3D] = None
        self._uav_node_id_counter = 1000
        self._ugv_node_id_counter = 0

    def add_uav_keyframe(self, node_id: int, pose: Pose3D,
                          descriptors: Optional[np.ndarray] = None) -> int:
        """添加 UAV 关键帧"""
        nid = self._uav_node_id_counter + node_id
        self.sliding_optimizer.add_frame(nid, pose, 'uav')
        return nid

    def add_ugv_pose(self, pose: Pose3D):
        """添加 UGV 位姿"""
        nid = self._ugv_node_id_counter
        self.sliding_optimizer.add_frame(nid, pose, 'ugv')
        self._ugv_node_id_counter += 1
        return nid

    def add_sync_pair(self, uav_pose: Pose3D, ugv_pose: Pose3D):
        """添加同步位姿对（用于手眼标定）"""
        self.calibrator.add_measurement_pair(uav_pose, ugv_pose)

    def calibrate(self) -> Optional[Pose3D]:
        """执行手眼标定"""
        self.T_uav2ugv = self.calibrator.solve()
        return self.T_uav2ugv

    def transform_uav_to_ugv(self, uav_pose: Pose3D) -> Optional[Pose3D]:
        """将 UAV 位姿转换到 UGV 坐标系"""
        if self.T_uav2ugv is None:
            return None
        return self.T_uav2ugv.compose(uav_pose)

    def add_cross_modal_constraint(self, uav_node_id: int, ugv_node_id: int,
                                    rel_pose: Pose3D, weight: float):
        """添加跨模态位姿约束"""
        uav_nid = self._uav_node_id_counter + uav_node_id
        self.sliding_optimizer.window[-1]['constraints'].append(
            (uav_nid, rel_pose, weight))

    def optimize(self) -> Dict[int, Pose3D]:
        """执行联合优化"""
        return self.sliding_optimizer.optimize()

    def generate_report(self) -> dict:
        """生成协同优化报告"""
        return {
            "calibrated": self.T_uav2ugv is not None,
            "calibration_error_m": self.calibrator.get_transformation_error()
            if self.T_uav2ugv else None,
            "sync_pairs": len(self.calibrator._uav_poses),
            "window_frames": len(self.sliding_optimizer.window),
            "optimized_nodes": len(self.sliding_optimizer._optimized_nodes)
        }
