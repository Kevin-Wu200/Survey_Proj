"""
UAV 辅助回环检测模块 (3A.4)

实现基于 UAV 拓扑图的跨视角回环检测：
  - 在 UAV 拓扑图中搜索 UGV 邻近关键帧
  - 跨视角特征匹配（SuperPoint + SuperGlue 风格）
  - 回环约束边 → 加入因子图 → PGO 全局优化
"""

import numpy as np
from typing import List, Optional, Dict, Tuple
from collections import deque

from .data_types import (
    Pose3D, KeyFrame, TopoGraph, TopoNode,
    FactorGraph, FactorEdge, FactorType, LoopCandidate,
    LoopClosureConfig
)


class SuperPointSimulator:
    """SuperPoint 特征检测模拟器

    在原型阶段模拟 SuperPoint 的输出。
    实际部署时替换为 ONNX/TensorRT 推理。
    """

    def __init__(self, config: LoopClosureConfig):
        self.config = config

    def detect_and_compute(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """检测关键点并计算描述子

        Returns:
            keypoints: N×2 关键点坐标
            descriptors: N×256 描述子（SuperPoint 标准 256 维）
        """
        h, w = image.shape[:2]
        n_keypoints = min(500, (w * h) // 1000)

        # 均匀采样 + 随机扰动
        grid_x = np.linspace(50, w - 50, int(np.sqrt(n_keypoints)))
        grid_y = np.linspace(50, h - 50, int(np.sqrt(n_keypoints)))
        xx, yy = np.meshgrid(grid_x, grid_y)
        keypoints = np.column_stack([
            xx.ravel() + np.random.randn(len(xx.ravel())) * 10,
            yy.ravel() + np.random.randn(len(yy.ravel())) * 10
        ])

        # 生成描述子
        descriptors = np.random.randn(len(keypoints), 256).astype(np.float32)
        norms = np.linalg.norm(descriptors, axis=1, keepdims=True) + 1e-8
        descriptors /= norms

        return keypoints, descriptors


class SuperGlueSimulator:
    """SuperGlue 特征匹配模拟器

    在原型阶段模拟跨视角特征匹配。
    """

    def __init__(self, config: LoopClosureConfig):
        self.config = config

    def match(self, desc1: np.ndarray, desc2: np.ndarray,
              kp1: Optional[np.ndarray] = None,
              kp2: Optional[np.ndarray] = None) -> List[Tuple[int, int, float]]:
        """跨视角特征匹配

        Returns:
            [(idx1, idx2, confidence)]
        """
        if len(desc1) == 0 or len(desc2) == 0:
            return []

        # 计算相似度矩阵
        sim = desc1 @ desc2.T

        matches = []
        for i in range(len(desc1)):
            scores = sim[i]
            if len(scores) < 2:
                continue

            best_idx = np.argmax(scores)
            best_score = scores[best_idx]
            scores[best_idx] = -np.inf
            second_score = np.max(scores)

            if best_score > self.config.feature_match_thresh and \
               best_score > second_score * 0.8:
                matches.append((i, int(best_idx), float(best_score)))

        return matches


class ORBMatcher:
    """ORB 特征匹配降级方案

    当 SuperPoint+SuperGlue 推理帧率不足时使用。
    """

    def __init__(self, config: LoopClosureConfig):
        self.config = config
        self._orb = None  # cv2.ORB_create() - 延迟导入

    def detect_and_compute(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """ORB 特征检测与描述"""
        try:
            import cv2
            if self._orb is None:
                self._orb = cv2.ORB_create(
                    nfeatures=self.config.orb_n_features,
                    scaleFactor=self.config.orb_scale_factor
                )
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            kp, desc = self._orb.detectAndCompute(gray, None)
            if desc is None or len(kp) == 0:
                return np.empty((0, 2)), np.empty((0, 32))
            pts = np.array([p.pt for p in kp])
            return pts, desc.astype(np.float32)
        except ImportError:
            # 无 cv2 时的降级
            n = self.config.orb_n_features
            pts = np.random.rand(n, 2) * 1000
            desc = np.random.randn(n, 32).astype(np.float32)
            return pts, desc

    def match(self, desc1: np.ndarray, desc2: np.ndarray) -> List[Tuple[int, int, float]]:
        """ORB 描述子匹配"""
        try:
            import cv2
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            raw_matches = bf.knnMatch(
                desc1.astype(np.uint8), desc2.astype(np.uint8), k=2)
            matches = []
            for m, n in raw_matches:
                if m.distance < 0.75 * n.distance:
                    matches.append((m.queryIdx, m.trainIdx,
                                    1.0 - m.distance / 256.0))
            return matches
        except (ImportError, cv2.error):
            return []


class PoseGraphOptimizer:
    """位姿图优化器 (PGO)

    基于回环约束执行全局位姿图优化。
    """

    def __init__(self, config: LoopClosureConfig):
        self.config = config

    def optimize(self, graph: FactorGraph,
                  loop_edges: List[FactorEdge]) -> FactorGraph:
        """执行全局 PGO 优化（简化版高斯-牛顿）

        Args:
            graph: 当前因子图
            loop_edges: 回环约束边

        Returns:
            优化后的因子图
        """
        all_edges = graph.edges + loop_edges
        if len(all_edges) == 0:
            return graph

        # 简化的全局优化：对每个节点，加权平均其邻居的位姿
        optimized = FactorGraph()
        optimized.nodes = {k: Pose3D(R=v.R.copy(), t=v.t.copy())
                           for k, v in graph.nodes.items()}
        optimized.node_types = dict(graph.node_types)

        for iteration in range(self.config.pgo_max_iter):
            max_change = 0.0

            for nid in list(optimized.nodes.keys()):
                t_sum = np.zeros(3)
                R_sum = np.zeros((3, 3))
                total_weight = 0.0

                # 收集所有连接的邻居位姿
                for edge in all_edges:
                    if edge.src_id == nid and edge.dst_id in optimized.nodes:
                        neighbor = optimized.nodes[edge.dst_id]
                        weight = np.trace(edge.information) / 6.0
                        t_sum += weight * neighbor.t
                        R_sum += weight * neighbor.R
                        total_weight += weight
                    elif edge.dst_id == nid and edge.src_id in optimized.nodes:
                        neighbor = optimized.nodes[edge.src_id]
                        weight = np.trace(edge.information) / 6.0
                        t_sum += weight * neighbor.t
                        R_sum += weight * neighbor.R
                        total_weight += weight

                if total_weight > 0:
                    new_t = t_sum / total_weight
                    new_R = R_sum / total_weight
                    U, _, Vt = np.linalg.svd(new_R)
                    new_R = U @ Vt

                    change = np.linalg.norm(new_t - optimized.nodes[nid].t)
                    max_change = max(max_change, change)

                    optimized.nodes[nid] = Pose3D(R=new_R, t=new_t)

            if max_change < 1e-6:
                break

        optimized.edges = all_edges
        return optimized


class LoopClosureDetector:
    """UAV 辅助回环检测器

    利用 UAV 拓扑图提供的全局视角，辅助 UGV 回环检测。
    """

    def __init__(self, config: Optional[LoopClosureConfig] = None):
        self.config = config or LoopClosureConfig()
        self.superpoint = SuperPointSimulator(self.config)
        self.superglue = SuperGlueSimulator(self.config)
        self.orb_matcher = ORBMatcher(self.config)
        self.pgo = PoseGraphOptimizer(self.config)

        self._uav_topo_graph: Optional[TopoGraph] = None
        self._ugv_keyframes: List[KeyFrame] = []
        self._loop_candidates: List[LoopCandidate] = []
        self._factor_graph = FactorGraph()
        self._use_orb_fallback = False

    def set_uav_topo_graph(self, graph: TopoGraph):
        """设置 UAV 拓扑图"""
        self._uav_topo_graph = graph

    def add_ugv_keyframe(self, kf: KeyFrame):
        """添加 UGV 关键帧"""
        self._ugv_keyframes.append(kf)
        self._factor_graph.add_node(kf.id, kf.pose, "ugv")

    def detect_loop(self, query_kf: KeyFrame) -> List[LoopCandidate]:
        """执行回环检测

        1. 在 UAV 拓扑图中搜索邻近关键帧
        2. 跨视角特征匹配
        3. 几何一致性校验
        """
        candidates = []

        if self._uav_topo_graph is None:
            return candidates

        # 步骤 1：在 UAV 拓扑图中搜索距离 < d_th 的邻近关键帧
        nearby_uav_nodes = []
        for nid, node in self._uav_topo_graph.nodes.items():
            dist = np.linalg.norm(node.pose.t - query_kf.pose.t)
            if dist < self.config.distance_thresh:
                nearby_uav_nodes.append((nid, node, dist))

        if not nearby_uav_nodes:
            return candidates

        # 步骤 2：跨视角特征匹配
        if query_kf.descriptors is None:
            return candidates

        for uav_nid, uav_node, dist in nearby_uav_nodes:
            uav_desc = uav_node.keyframe.descriptors
            if uav_desc is None:
                continue

            # SuperPoint+SuperGlue 匹配
            if self._use_orb_fallback:
                matches = self.superglue.match(
                    query_kf.descriptors, uav_desc)
            else:
                matches = self.superglue.match(
                    query_kf.descriptors, uav_desc)

            if len(matches) < self.config.min_inliers:
                continue

            # 步骤 3：几何一致性校验
            rel_pose = self._estimate_relative_pose(
                query_kf, uav_node, matches)

            if rel_pose is None:
                continue

            geometric_score = self._geometric_check(
                query_kf, uav_node, rel_pose, matches)

            if geometric_score < self.config.geometric_check_thresh:
                continue

            candidate = LoopCandidate(
                query_id=query_kf.id,
                match_id=uav_nid,
                score=float(np.mean([m[2] for m in matches])),
                relative_pose=rel_pose,
                inliers=len(matches),
                is_valid=True
            )
            candidates.append(candidate)

        # 排序，取最佳候选
        candidates.sort(key=lambda c: c.score, reverse=True)
        self._loop_candidates.extend(candidates)

        return candidates

    def _estimate_relative_pose(self, query_kf: KeyFrame, uav_node: TopoNode,
                                 matches: List[Tuple[int, int, float]]) -> Optional[Pose3D]:
        """根据特征匹配估计相对位姿"""
        # 使用 UAV 已知位姿作为基础估计
        base_rel_pose = uav_node.pose.inverse().compose(query_kf.pose)

        if query_kf.image_features is None or \
           uav_node.keyframe.image_features is None:
            return base_rel_pose

        # 至少需要3对匹配
        if len(matches) < 3:
            return None

        src_pts = query_kf.image_features[[m[0] for m in matches]]
        dst_pts = uav_node.keyframe.image_features[[m[1] for m in matches]]

        # 简化的本质矩阵估计
        centroid_src = np.mean(src_pts, axis=0)
        centroid_dst = np.mean(dst_pts, axis=0)
        H = (src_pts - centroid_src).T @ (dst_pts - centroid_dst)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = centroid_dst - R @ centroid_src

        # 尺度因子（使用已知距离）
        known_dist = np.linalg.norm(uav_node.pose.t - query_kf.pose.t)
        current_dist = np.linalg.norm(t)
        if current_dist > 0:
            scale = known_dist / current_dist
            t *= scale

        return Pose3D(R=R, t=t)

    def _geometric_check(self, query_kf: KeyFrame, uav_node: TopoNode,
                          rel_pose: Pose3D,
                          matches: List[Tuple[int, int, float]]) -> float:
        """几何一致性校验：验证匹配的内点比例"""
        if not matches or query_kf.image_features is None or \
           uav_node.keyframe.image_features is None:
            return 0.0

        inlier_count = 0
        for idx1, idx2, _ in matches:
            pt1 = np.array([query_kf.image_features[idx1][0],
                            query_kf.image_features[idx1][1], 1.0])
            pt2 = np.array([uav_node.keyframe.image_features[idx2][0],
                            uav_node.keyframe.image_features[idx2][1], 1.0])
            # 简化的对极几何检查
            predicted = rel_pose.R @ pt1[:2] + rel_pose.t[:2]
            error = np.linalg.norm(predicted - pt2[:2])
            if error < 50:  # 像素阈值
                inlier_count += 1

        return inlier_count / len(matches) if matches else 0.0

    def add_loop_to_factor_graph(self, candidate: LoopCandidate) -> FactorEdge:
        """将回环候选添加到因子图"""
        edge = FactorEdge(
            src_id=candidate.query_id,
            dst_id=candidate.match_id + 10000,  # UAV 节点 ID 偏移
            factor_type=FactorType.LOOP,
            measurement=np.array(candidate.relative_pose.t.tolist() + [0, 0, 0]),
            information=np.eye(6) * candidate.score * candidate.inliers / 100.0,
            loss_scale=1.0
        )
        self._factor_graph.add_factor(edge)
        return edge

    def run_pgo(self) -> FactorGraph:
        """执行全局位姿图优化"""
        loop_edges = [e for e in self._factor_graph.edges
                       if e.factor_type == FactorType.LOOP]
        self._factor_graph = self.pgo.optimize(self._factor_graph, loop_edges)
        return self._factor_graph

    def enable_orb_fallback(self):
        """启用 ORB 降级方案"""
        self._use_orb_fallback = True

    def disable_orb_fallback(self):
        """禁用 ORB 降级"""
        self._use_orb_fallback = False

    def generate_report(self) -> dict:
        """生成回环检测报告"""
        valid_candidates = [c for c in self._loop_candidates if c.is_valid]
        return {
            "total_candidates": len(self._loop_candidates),
            "valid_candidates": len(valid_candidates),
            "best_score": max([c.score for c in valid_candidates]) if valid_candidates else 0,
            "ugv_keyframes": len(self._ugv_keyframes),
            "uav_topo_nodes": self._uav_topo_graph.num_nodes if self._uav_topo_graph else 0,
            "factor_edges": len(self._factor_graph.edges),
            "using_orb_fallback": self._use_orb_fallback
        }
