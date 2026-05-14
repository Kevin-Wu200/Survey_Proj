"""
空地协同无人化智能测绘系统 - 协同 SLAM 模块

三阶段实现：
  3A: 可行性原型（UAV拓扑建图、UGV融合定位、协同优化、回环检测）
  3B: 迭代优化（实时性、鲁棒性、一致性校验）
"""

__version__ = "0.1.0"

from .data_types import (
    Pose3D, KeyFrame, TopoNode, TopoEdge, TopoGraph,
    ScanData, IMUData, StereoImage, LoopCandidate,
    FactorType, FactorEdge, FactorGraph,
    UAVTopoConfig, UGVFusionConfig, CollaborativeConfig, LoopClosureConfig,
)

from .uav_topology import UAVTopologyMapper
from .ugv_fusion import UGVMultiSensorFusion
from .collaborative_optimizer import CollaborativeOptimizer
from .loop_closure import LoopClosureDetector
from .realtime_opt import RealtimeOptimizer
from .robustness import RobustnessEnhancer
from .consistency import ConsistencyChecker

__all__ = [
    "Pose3D", "KeyFrame", "TopoNode", "TopoEdge", "TopoGraph",
    "ScanData", "IMUData", "StereoImage", "LoopCandidate",
    "FactorType", "FactorEdge", "FactorGraph",
    "UAVTopoConfig", "UGVFusionConfig", "CollaborativeConfig", "LoopClosureConfig",
    "UAVTopologyMapper", "UGVMultiSensorFusion",
    "CollaborativeOptimizer", "LoopClosureDetector",
    "RealtimeOptimizer", "RobustnessEnhancer", "ConsistencyChecker",
]
