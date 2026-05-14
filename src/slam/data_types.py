"""
协同 SLAM 数据类型定义

定义空地协同 SLAM 所需的核心数据结构：
  位姿、关键帧、拓扑图、传感器数据、因子图、回环候选等。
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import IntEnum
import time


# ══════════════════════════════════════════════════════════════════════
# 基础数据类型
# ══════════════════════════════════════════════════════════════════════

@dataclass
class Pose3D:
    """三维位姿：旋转矩阵 R ∈ SO(3) + 平移向量 t ∈ R³"""
    R: np.ndarray  # 3x3 旋转矩阵
    t: np.ndarray  # 3x1 平移向量
    timestamp: float = 0.0

    @classmethod
    def identity(cls) -> "Pose3D":
        return cls(R=np.eye(3), t=np.zeros(3))

    @classmethod
    def from_rtk(cls, lat: float, lon: float, alt: float,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                 home_lat: float = 30.0, home_lon: float = 120.0) -> "Pose3D":
        """从 RTK 经纬度+姿态角构建位姿（局部ENU坐标系）"""
        cos_lat = np.cos(np.radians(home_lat))
        x = (lon - home_lon) * 111320.0 * cos_lat
        y = (lat - home_lat) * 111320.0
        z = alt

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])
        return cls(R=R, t=np.array([x, y, z]))

    @property
    def T(self) -> np.ndarray:
        """4x4 齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t
        return T

    def inverse(self) -> "Pose3D":
        R_inv = self.R.T
        return Pose3D(R=R_inv, t=-R_inv @ self.t, timestamp=self.timestamp)

    def compose(self, other: "Pose3D") -> "Pose3D":
        """组合变换: self ∘ other = T_self * T_other"""
        return Pose3D(
            R=self.R @ other.R,
            t=self.R @ other.t + self.t,
            timestamp=max(self.timestamp, other.timestamp)
        )

    def transform_point(self, pt: np.ndarray) -> np.ndarray:
        """变换三维点坐标"""
        return self.R @ pt + self.t


@dataclass
class KeyFrame:
    """SLAM 关键帧"""
    id: int
    pose: Pose3D
    image_features: Optional[np.ndarray] = None   # N×2 图像特征点
    descriptors: Optional[np.ndarray] = None       # N×128 特征描述子
    point_cloud: Optional[np.ndarray] = None       # N×3 关联三维点云
    timestamp: float = field(default_factory=time.time)
    is_uav: bool = False  # 是否来自无人机


# ══════════════════════════════════════════════════════════════════════
# 拓扑图数据类型
# ══════════════════════════════════════════════════════════════════════

@dataclass
class TopoNode:
    """拓扑图节点（对应一个关键帧）"""
    id: int
    pose: Pose3D                          # 位姿 Ti
    descriptor: np.ndarray                # 图像全局描述子 di ∈ R¹²⁸
    keyframe: KeyFrame                    # 关联关键帧
    is_active: bool = True

    def __hash__(self):
        return self.id

    def __eq__(self, other):
        return self.id == other.id


@dataclass
class TopoEdge:
    """拓扑图边（共视关系）"""
    src_id: int
    dst_id: int
    weight: float                  # 边权重（共视特征点数）
    relative_pose: Optional[Pose3D] = None  # 相对位姿约束
    covariance: Optional[np.ndarray] = None  # 6x6 信息矩阵


@dataclass
class TopoGraph:
    """拓扑图 G = (V, E)"""
    nodes: Dict[int, TopoNode] = field(default_factory=dict)
    edges: List[TopoEdge] = field(default_factory=list)
    adjacency: Dict[int, List[int]] = field(default_factory=dict)

    def add_node(self, node: TopoNode):
        self.nodes[node.id] = node
        if node.id not in self.adjacency:
            self.adjacency[node.id] = []

    def add_edge(self, edge: TopoEdge):
        self.edges.append(edge)
        if edge.src_id not in self.adjacency:
            self.adjacency[edge.src_id] = []
        if edge.dst_id not in self.adjacency:
            self.adjacency[edge.dst_id] = []
        self.adjacency[edge.src_id].append(edge.dst_id)
        self.adjacency[edge.dst_id].append(edge.src_id)

    def get_neighbors(self, node_id: int, max_dist: float = float('inf')) -> List[int]:
        """获取距离 < max_dist 的邻居节点"""
        if node_id not in self.nodes:
            return []
        ref_pose = self.nodes[node_id].pose
        neighbors = []
        for nid in self.adjacency.get(node_id, []):
            if nid in self.nodes:
                dist = np.linalg.norm(self.nodes[nid].pose.t - ref_pose.t)
                if dist < max_dist:
                    neighbors.append(nid)
        return neighbors

    @property
    def num_nodes(self) -> int:
        return len(self.nodes)

    @property
    def num_edges(self) -> int:
        return len(self.edges)


# ══════════════════════════════════════════════════════════════════════
# 传感器数据类型
# ══════════════════════════════════════════════════════════════════════

@dataclass
class ScanData:
    """LiDAR 扫描数据"""
    points: np.ndarray              # N×3 点云 (x, y, z)
    intensities: Optional[np.ndarray] = None  # N×1 强度
    timestamp: float = field(default_factory=time.time)
    frame_id: str = "lidar"


@dataclass
class IMUData:
    """IMU 惯性测量数据"""
    accel: np.ndarray               # 3×1 线加速度
    gyro: np.ndarray                # 3×1 角速度
    timestamp: float = field(default_factory=time.time)
    accel_cov: Optional[np.ndarray] = None  # 3×3
    gyro_cov: Optional[np.ndarray] = None   # 3×3


@dataclass
class StereoImage:
    """双目相机图像"""
    left: np.ndarray                # H×W×3 左目图像
    right: np.ndarray               # H×W×3 右目图像
    timestamp: float = field(default_factory=time.time)
    left_features: Optional[np.ndarray] = None   # N×2
    right_features: Optional[np.ndarray] = None  # N×2


# ══════════════════════════════════════════════════════════════════════
# 因子图数据类型
# ══════════════════════════════════════════════════════════════════════

class FactorType(IntEnum):
    """因子类型"""
    PRIOR = 0       # 先验因子
    IMU = 1         # IMU 预积分因子
    LIDAR = 2       # LiDAR 里程计因子
    VISUAL = 3      # 视觉里程计因子
    LOOP = 4        # 回环检测因子
    GPS = 5         # GNSS/RTK 约束因子


@dataclass
class FactorEdge:
    """因子图边"""
    src_id: int                     # 源节点
    dst_id: int                     # 目标节点
    factor_type: FactorType
    measurement: np.ndarray         # 测量值
    information: np.ndarray         # 信息矩阵
    loss_scale: float = 1.0         # 鲁棒核尺度


@dataclass
class FactorGraph:
    """因子图 G = (X, F)"""
    nodes: Dict[int, Pose3D] = field(default_factory=dict)  # 变量节点
    edges: List[FactorEdge] = field(default_factory=list)
    node_types: Dict[int, str] = field(default_factory=dict)

    def add_node(self, node_id: int, pose: Pose3D, node_type: str = ""):
        self.nodes[node_id] = pose
        if node_type:
            self.node_types[node_id] = node_type

    def add_factor(self, edge: FactorEdge):
        self.edges.append(edge)

    def get_connected_nodes(self, node_id: int) -> List[int]:
        connected = []
        for e in self.edges:
            if e.src_id == node_id:
                connected.append(e.dst_id)
            elif e.dst_id == node_id:
                connected.append(e.src_id)
        return connected


# ══════════════════════════════════════════════════════════════════════
# 回环检测数据类型
# ══════════════════════════════════════════════════════════════════════

@dataclass
class LoopCandidate:
    """回环候选"""
    query_id: int                   # 查询关键帧ID
    match_id: int                   # 匹配关键帧ID
    score: float                    # 匹配置信度
    relative_pose: Pose3D           # 估计的相对位姿
    inliers: int = 0
    is_valid: bool = True


# ══════════════════════════════════════════════════════════════════════
# 配置数据类型
# ══════════════════════════════════════════════════════════════════════

@dataclass
class UAVTopoConfig:
    """UAV 拓扑建图配置"""
    keyframe_distance_thresh: float = 5.0     # 关键帧最小距离 (m)
    keyframe_angle_thresh: float = 0.2         # 关键帧最小角度变化 (rad)
    covisibility_thresh: int = 30              # 共视特征点阈值
    descriptor_dim: int = 128                  # 描述子维度
    min_inliers_sfm: int = 20                  # SfM 最少内点数


@dataclass
class UGVFusionConfig:
    """UGV 多传感器融合配置"""
    imu_frequency: float = 100.0               # IMU 频率 (Hz)
    lidar_frequency: float = 10.0              # LiDAR 频率 (Hz)
    camera_frequency: float = 30.0             # 相机频率 (Hz)
    output_frequency: float = 10.0             # 输出位姿频率 (Hz)
    scan_to_map_max_iter: int = 30             # Scan-to-Map 最大迭代
    scan_to_map_convergence: float = 1e-4      # 收敛阈值
    imu_accel_noise: float = 0.01              # 加速度计噪声
    imu_gyro_noise: float = 0.001              # 陀螺仪噪声
    visual_feature_thresh: int = 100           # 视觉最少特征点
    keyframe_interval: int = 10                # 关键帧间隔（帧数）


@dataclass
class CollaborativeConfig:
    """协同优化配置"""
    sliding_window_size: int = 20              # 滑动窗口关键帧数
    hand_eye_max_iter: int = 100               # 手眼标定最大迭代
    hand_eye_convergence: float = 1e-6         # 收敛阈值
    rtk_marker_radius: float = 0.5             # RTK标记点半径 (m)
    min_cross_view_matches: int = 10           # 跨视角最少匹配点


@dataclass
class LoopClosureConfig:
    """回环检测配置"""
    distance_thresh: float = 10.0              # 邻近关键帧距离阈值 (m)
    feature_match_thresh: float = 0.7          # 特征匹配阈值
    min_inliers: int = 30                      # 最少内点数
    geometric_check_thresh: float = 0.05       # 几何一致性检查阈值
    pgo_max_iter: int = 50                     # PGO最大迭代
    orb_scale_factor: float = 1.2              # ORB降级参数
    orb_n_features: int = 1000                 # ORB特征数
