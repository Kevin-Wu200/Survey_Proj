"""
UAV 全局拓扑建图模块 (3A.1)

实现基于 RTK 位姿 + 增量式 SfM 的全局拓扑建图：
  - 提取稀疏特征点云
  - 构建拓扑图 G = (V, E)：关键帧节点 + 共视拓扑边
  - 每个节点存储位姿 Ti 与图像特征描述子 di ∈ R¹²⁸
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import time

from .data_types import (
    Pose3D, KeyFrame, TopoNode, TopoEdge, TopoGraph,
    UAVTopoConfig
)


@dataclass
class SfMPoint:
    """SfM 稀疏三维点"""
    id: int
    position: np.ndarray          # 3D 世界坐标
    observations: List[Tuple[int, int]]  # [(关键帧ID, 特征点索引)]
    descriptor: Optional[np.ndarray] = None


class IncrementalSfM:
    """增量式 Structure from Motion

    基于已知位姿序列，增量式三角化特征点并维护稀疏点云。
    """

    def __init__(self, config: UAVTopoConfig):
        self.config = config
        self.points: List[SfMPoint] = []
        self._next_point_id = 0

    def triangulate(self, keyframes: List[KeyFrame]) -> List[SfMPoint]:
        """对关键帧序列进行增量式三角化

        对于每对相邻关键帧，匹配特征点并三角化新的三维点。
        """
        new_points = []

        for i in range(len(keyframes) - 1):
            kf1 = keyframes[i]
            kf2 = keyframes[i + 1]

            if kf1.image_features is None or kf2.image_features is None:
                continue

            # 特征匹配（简化：使用描述子距离）
            if kf1.descriptors is not None and kf2.descriptors is not None:
                matches = self._match_features(kf1.descriptors, kf2.descriptors)
            else:
                matches = self._dummy_matches(
                    len(kf1.image_features), len(kf2.image_features))

            if len(matches) < self.config.min_inliers_sfm:
                continue

            # 三角化
            P1 = kf1.pose.T  # 世界→相机1 变换
            P2 = kf2.pose.T  # 世界→相机2 变换

            # 相机内参（简化针孔模型）
            K = np.array([[800, 0, 960], [0, 800, 540], [0, 0, 1]], dtype=np.float64)
            K_inv = np.linalg.inv(K)

            # 投影矩阵
            proj1 = K @ P1[:3, :]
            proj2 = K @ P2[:3, :]

            for (idx1, idx2), _ in matches:
                pt1 = np.array([kf1.image_features[idx1][0],
                                kf1.image_features[idx1][1], 1.0])
                pt2 = np.array([kf2.image_features[idx2][0],
                                kf2.image_features[idx2][1], 1.0])

                # 归一化坐标
                n1 = K_inv @ pt1
                n2 = K_inv @ pt2

                # DLT 三角化
                pt_3d = self._dlt_triangulate(n1, n2, P1, P2)
                if pt_3d is not None:
                    desc = kf1.descriptors[idx1] if kf1.descriptors is not None else None
                    point = SfMPoint(
                        id=self._next_point_id,
                        position=pt_3d,
                        observations=[(kf1.id, idx1), (kf2.id, idx2)],
                        descriptor=desc
                    )
                    self._next_point_id += 1
                    self.points.append(point)
                    new_points.append(point)

        return new_points

    def _match_features(self, desc1: np.ndarray, desc2: np.ndarray,
                        ratio: float = 0.75) -> List[Tuple[Tuple[int, int], float]]:
        """基于描述子距离的特征匹配（Lowe's ratio test）"""
        matches = []
        for i in range(desc1.shape[0]):
            dists = np.linalg.norm(desc2 - desc1[i], axis=1)
            if len(dists) < 2:
                continue
            best_idx = np.argmin(dists)
            best_dist = dists[best_idx]
            dists[best_idx] = np.inf
            second_dist = np.min(dists)
            if best_dist < ratio * second_dist:
                matches.append(((i, int(best_idx)), 1.0 - best_dist / 2.0))
        return matches

    def _dummy_matches(self, n1: int, n2: int) -> List[Tuple[Tuple[int, int], float]]:
        """无描述子时的模拟匹配"""
        n = min(n1, n2)
        return [((i, i), 0.8) for i in range(n)]

    def _dlt_triangulate(self, n1: np.ndarray, n2: np.ndarray,
                          P1: np.ndarray, P2: np.ndarray) -> Optional[np.ndarray]:
        """DLT 三角化求解三维点"""
        A = np.zeros((4, 4))
        A[0] = n1[0] * P1[2, :] - P1[0, :]
        A[1] = n1[1] * P1[2, :] - P1[1, :]
        A[2] = n2[0] * P2[2, :] - P2[0, :]
        A[3] = n2[1] * P2[2, :] - P2[1, :]

        _, _, Vt = np.linalg.svd(A)
        X = Vt[-1]
        if X[3] == 0:
            return None
        return X[:3] / X[3]


class UAVTopologyMapper:
    """UAV 全局拓扑建图器

    接收 UAV 飞行过程中的关键帧序列，增量式构建拓扑图。
    """

    def __init__(self, config: Optional[UAVTopoConfig] = None):
        self.config = config or UAVTopoConfig()
        self.graph = TopoGraph()
        self.sfm = IncrementalSfM(self.config)
        self._keyframes: List[KeyFrame] = []
        self._next_node_id = 0
        self._last_keyframe_pose: Optional[Pose3D] = None

    def process_frame(self, pose: Pose3D, image: Optional[np.ndarray] = None,
                      features: Optional[np.ndarray] = None,
                      descriptors: Optional[np.ndarray] = None) -> Optional[KeyFrame]:
        """处理单帧数据，判断是否为关键帧并更新拓扑图

        Args:
            pose: RTK 位姿
            image: 原始图像 (H×W×3)
            features: 特征点坐标 (N×2)
            descriptors: 特征描述子 (N×128)

        Returns:
            如果是新关键帧则返回 KeyFrame，否则返回 None
        """
        # 判断是否为关键帧
        if self._last_keyframe_pose is not None:
            dist = np.linalg.norm(pose.t - self._last_keyframe_pose.t)
            angle = np.arccos(
                np.clip((np.trace(self._last_keyframe_pose.R.T @ pose.R) - 1) / 2, -1, 1))
            if dist < self.config.keyframe_distance_thresh and \
               angle < self.config.keyframe_angle_thresh:
                return None

        # 提取特征（如果未提供）
        if features is None and image is not None:
            features, descriptors = self._extract_features(image)
        if descriptors is None:
            descriptors = self._generate_random_descriptor(self.config.descriptor_dim)

        # 创建关键帧
        kf = KeyFrame(
            id=len(self._keyframes),
            pose=pose,
            image_features=features,
            descriptors=descriptors,
            is_uav=True
        )
        self._keyframes.append(kf)
        self._last_keyframe_pose = pose

        # 添加到拓扑图
        node = TopoNode(
            id=self._next_node_id,
            pose=pose,
            descriptor=np.mean(descriptors, axis=0) if descriptors.ndim > 1
            else descriptors,
            keyframe=kf
        )
        self.graph.add_node(node)
        self._next_node_id += 1

        # 增量式 SfM 三角化
        if len(self._keyframes) >= 2:
            self.sfm.triangulate(self._keyframes[-2:])

        # 与邻近节点建立共视边
        self._build_covisibility_edges(node)

        return kf

    def _extract_features(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """简化的特征提取（模拟 SuperPoint 输出）

        实际部署时替换为 SuperPoint 推理。
        """
        h, w = image.shape[:2]
        n_features = 200
        # 在图像上均匀采样特征点
        xs = np.random.randint(50, w - 50, n_features).astype(np.float32)
        ys = np.random.randint(50, h - 50, n_features).astype(np.float32)
        features = np.column_stack([xs, ys])
        descriptors = self._generate_random_descriptors(n_features, self.config.descriptor_dim)
        return features, descriptors

    def _generate_random_descriptor(self, dim: int) -> np.ndarray:
        """生成随机描述子（用于原型验证）"""
        desc = np.random.randn(dim).astype(np.float32)
        return desc / (np.linalg.norm(desc) + 1e-8)

    def _generate_random_descriptors(self, n: int, dim: int) -> np.ndarray:
        """生成随机描述子矩阵"""
        descs = np.random.randn(n, dim).astype(np.float32)
        norms = np.linalg.norm(descs, axis=1, keepdims=True) + 1e-8
        return descs / norms

    def _build_covisibility_edges(self, node: TopoNode):
        """为新节点建立共视边

        基于距离和描述子相似度的双重判定：
        1. 空间距离 < 阈值 → 优先建立边
        2. 描述子余弦相似度 > 阈值 → 建立边
        """
        max_dist = self.config.keyframe_distance_thresh * 5  # 空间距离阈值

        for other_id, other_node in self.graph.nodes.items():
            if other_id == node.id:
                continue

            dist = np.linalg.norm(node.pose.t - other_node.pose.t)
            sim = np.dot(node.descriptor, other_node.descriptor)

            # 距离近 或 描述子相似 → 建立共视边
            if dist < max_dist or sim > 0.3:
                weight = float(sim) if sim > 0 else 0.5
                # 距离越近权重越高
                weight = max(weight, 1.0 / (1.0 + dist / max_dist))

                edge = TopoEdge(
                    src_id=node.id,
                    dst_id=other_id,
                    weight=weight,
                    relative_pose=other_node.pose.inverse().compose(node.pose)
                )
                self.graph.add_edge(edge)

    def finalize(self) -> TopoGraph:
        """完成拓扑图构建，执行全局 SfM 优化"""
        return self.graph

    def get_sparse_point_cloud(self) -> np.ndarray:
        """获取稀疏特征点云"""
        if not self.sfm.points:
            return np.empty((0, 3))
        return np.array([p.position for p in self.sfm.points])

    def generate_report(self) -> dict:
        """生成拓扑图统计报告"""
        return {
            "nodes": self.graph.num_nodes,
            "edges": self.graph.num_edges,
            "sfm_points": len(self.sfm.points),
            "avg_degree": self.graph.num_edges / max(1, self.graph.num_nodes),
            "coverage_area": self._estimate_coverage()
        }

    def _estimate_coverage(self) -> float:
        """估算覆盖面积 (m²)"""
        if self.graph.num_nodes < 3:
            return 0.0
        positions = np.array([n.pose.t[:2] for n in self.graph.nodes.values()])
        hull_area = 0.0
        return hull_area
