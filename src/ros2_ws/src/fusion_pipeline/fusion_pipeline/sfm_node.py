"""
增量式 SfM 节点 (Structure from Motion Node)

订阅 UAV 校正图像与 RTK 位姿，增量式三角化稀疏特征点，
发布稀疏点云和关键帧位姿。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped

try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    PC2_AVAILABLE = False

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import time

# 尝试导入 cv_bridge
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# 尝试导入 OpenCV（用于特征提取）
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ── 尝试导入 src/slam 中的数据类型 ─────────────────────────────────
try:
    import sys
    import os
    # 将项目根目录加入 sys.path 以导入 slam 模块
    _proj_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
    _proj_root = os.path.abspath(_proj_root)
    if _proj_root not in sys.path:
        sys.path.insert(0, _proj_root)

    from src.slam.data_types import Pose3D, KeyFrame, UAVTopoConfig
    from src.slam.uav_topology import IncrementalSfM
    SLAM_AVAILABLE = True
except ImportError:
    SLAM_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════
# 内置简化版 SfM（当 src/slam 模块不可用时使用）
# ══════════════════════════════════════════════════════════════════════

@dataclass
class SimplePose3D:
    """简化三维位姿"""
    R: np.ndarray              # 3x3 旋转矩阵
    t: np.ndarray              # 3x1 平移向量
    timestamp: float = 0.0

    @classmethod
    def identity(cls) -> "SimplePose3D":
        return cls(R=np.eye(3), t=np.zeros(3))

    @classmethod
    def from_pose_msg(cls, msg: PoseStamped) -> "SimplePose3D":
        """从 ROS PoseStamped 消息构造位姿"""
        # 四元数 → 旋转矩阵
        q = msg.pose.orientation
        R = cls._quat_to_rot(q.x, q.y, q.z, q.w)
        t = np.array([msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z])
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return cls(R=R, t=t, timestamp=ts)

    @staticmethod
    def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        """四元数 → 旋转矩阵"""
        R = np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
        ])
        return R

    @property
    def T(self) -> np.ndarray:
        """4x4 齐次变换矩阵"""
        T_mat = np.eye(4)
        T_mat[:3, :3] = self.R
        T_mat[:3, 3] = self.t
        return T_mat

    def __sub__(self, other: "SimplePose3D") -> "SimplePoseDelta":
        """计算相对位姿差"""
        return SimplePoseDelta(
            distance=np.linalg.norm(self.t - other.t),
            angle=np.arccos(
                np.clip((np.trace(other.R.T @ self.R) - 1) / 2, -1.0, 1.0))
        )


@dataclass
class SimplePoseDelta:
    """位姿差异"""
    distance: float  # 平移距离 (m)
    angle: float     # 旋转角度 (rad)


@dataclass
class SimpleKeyFrame:
    """简化关键帧"""
    id: int
    pose: SimplePose3D
    image: Optional[np.ndarray] = None
    keypoints: Optional[List[cv2.KeyPoint]] = None
    descriptors: Optional[np.ndarray] = None
    timestamp: float = field(default_factory=time.time)


class SimpleSfM:
    """简化版增量式 Structure from Motion

    基于已知位姿序列，对相邻关键帧进行特征匹配和三角化。
    """

    def __init__(self, camera_matrix: np.ndarray,
                 min_inliers: int = 20):
        """
        Args:
            camera_matrix: 3x3 相机内参矩阵 K
            min_inliers: 最小内点数阈值
        """
        self.K = camera_matrix
        self.min_inliers = min_inliers
        self.orb = None
        if CV2_AVAILABLE:
            self.orb = cv2.ORB_create(nfeatures=500)
        self.points_3d: List[np.ndarray] = []
        self._next_point_id = 0

    def match_and_triangulate(self, kf1: SimpleKeyFrame,
                               kf2: SimpleKeyFrame) -> List[np.ndarray]:
        """对两个关键帧进行特征匹配和三角化

        Args:
            kf1: 前一关键帧
            kf2: 当前关键帧

        Returns:
            新三角化的三维点列表
        """
        new_pts = []

        # 提取特征（如果未预提取）
        if kf1.keypoints is None and kf1.image is not None and self.orb is not None:
            kf1.keypoints, kf1.descriptors = self.orb.detectAndCompute(kf1.image, None)
        if kf2.keypoints is None and kf2.image is not None and self.orb is not None:
            kf2.keypoints, kf2.descriptors = self.orb.detectAndCompute(kf2.image, None)

        # 特征匹配
        if kf1.descriptors is None or kf2.descriptors is None:
            return new_pts

        matches = self._match_descriptors(kf1.descriptors, kf2.descriptors)
        if len(matches) < self.min_inliers:
            return new_pts

        # 计算投影矩阵
        P1 = self.K @ kf1.pose.T[:3, :]   # 3x4 投影矩阵（世界→像素）
        P2 = self.K @ kf2.pose.T[:3, :]

        # 三角化匹配点对
        for m in matches[:200]:  # 限制匹配点数
            pt1 = kf1.keypoints[m.queryIdx].pt
            pt2 = kf2.keypoints[m.trainIdx].pt

            pt_3d = self._dlt_triangulate(
                np.array(pt1), np.array(pt2), P1, P2)

            if pt_3d is not None:
                # 校验深度为正（在相机前方）
                if pt_3d[2] > 0:
                    new_pts.append(pt_3d)

        self.points_3d.extend(new_pts)
        return new_pts

    @staticmethod
    def _match_descriptors(desc1: np.ndarray, desc2: np.ndarray) -> list:
        """基于描述子距离的暴力匹配"""
        if desc1 is None or desc2 is None or len(desc1) == 0 or len(desc2) == 0:
            return []

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(desc1, desc2)
        # 按距离排序，保留前80%
        matches = sorted(matches, key=lambda x: x.distance)
        return matches[:int(len(matches) * 0.8)]

    @staticmethod
    def _dlt_triangulate(pt1: np.ndarray, pt2: np.ndarray,
                         P1: np.ndarray, P2: np.ndarray) -> Optional[np.ndarray]:
        """DLT 三角化求解三维点"""
        A = np.zeros((4, 4))
        A[0] = pt1[0] * P1[2, :] - P1[0, :]
        A[1] = pt1[1] * P1[2, :] - P1[1, :]
        A[2] = pt2[0] * P2[2, :] - P2[0, :]
        A[3] = pt2[1] * P2[2, :] - P2[1, :]

        try:
            _, _, Vt = np.linalg.svd(A)
            X = Vt[-1]
            if abs(X[3]) < 1e-10:
                return None
            return X[:3] / X[3]
        except np.linalg.LinAlgError:
            return None


# ══════════════════════════════════════════════════════════════════════
# ROS2 节点
# ══════════════════════════════════════════════════════════════════════

class SfMNode(Node):
    """增量式 SfM 节点

    订阅话题:
        /fusion/image_rectified (sensor_msgs/Image)
        /uav/pose (geometry_msgs/PoseStamped)

    发布话题:
        /fusion/sfm_points (sensor_msgs/PointCloud2)
        /fusion/sfm_keyframe_pose (geometry_msgs/PoseStamped)
    """

    def __init__(self):
        super().__init__('sfm_node')

        # ── 声明参数 ──────────────────────────────────────────────
        self.declare_parameter('keyframe_distance_thresh', 5.0)
        self.declare_parameter('keyframe_angle_thresh', 0.2)
        self.declare_parameter('min_inliers', 20)
        # 相机内参（与 image_rectify_node 保持一致）
        self.declare_parameter('camera_matrix',
                               [800.0, 0.0, 960.0,
                                0.0, 800.0, 540.0,
                                0.0, 0.0, 1.0])
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)

        # ── 读取参数 ──────────────────────────────────────────────
        self.kf_dist_thresh = self.get_parameter('keyframe_distance_thresh').get_parameter_value().double_value
        self.kf_angle_thresh = self.get_parameter('keyframe_angle_thresh').get_parameter_value().double_value
        self.min_inliers = self.get_parameter('min_inliers').get_parameter_value().integer_value

        K_flat = self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        self.K = np.array(K_flat, dtype=np.float64).reshape(3, 3)

        # ── 状态变量 ──────────────────────────────────────────────
        self.keyframes: List[SimpleKeyFrame] = []
        self._next_kf_id = 0
        self._latest_pose: Optional[SimplePose3D] = None
        self._latest_image: Optional[np.ndarray] = None
        self._last_kf_pose: Optional[SimplePose3D] = None

        # 增量式 SfM（优先使用 slam 模块，否则使用内置简化版）
        if SLAM_AVAILABLE:
            self.get_logger().info('使用 src/slam 模块中的 IncrementalSfM')
            config = UAVTopoConfig(
                keyframe_distance_thresh=self.kf_dist_thresh,
                keyframe_angle_thresh=self.kf_angle_thresh,
                min_inliers_sfm=self.min_inliers
            )
            self.sfm = IncrementalSfM(config)
            self._use_slam_module = True
        else:
            self.get_logger().warn('src/slam 模块不可用，使用内置简化版 SfM')
            self.sfm = SimpleSfM(camera_matrix=self.K,
                                 min_inliers=self.min_inliers)
            self._use_slam_module = False

        # cv_bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn('cv_bridge 不可用，将无法处理图像消息')

        # ── 订阅者 ────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, '/fusion/image_rectified',
            self.image_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose',
            self.pose_callback, 10)

        # ── 发布者 ────────────────────────────────────────────────
        self.points_pub = self.create_publisher(
            PointCloud2, '/fusion/sfm_points', 10)
        self.kf_pose_pub = self.create_publisher(
            PoseStamped, '/fusion/sfm_keyframe_pose', 10)

        self.get_logger().info(
            f'SfM 节点已启动: kf_dist={self.kf_dist_thresh}m, '
            f'kf_angle={self.kf_angle_thresh}rad, '
            f'min_inliers={self.min_inliers}')

    def pose_callback(self, msg: PoseStamped):
        """接收 UAV 位姿"""
        self._latest_pose = SimplePose3D.from_pose_msg(msg)

    def image_callback(self, msg: Image):
        """接收校正图像"""
        if self.bridge is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        self._latest_image = cv_image

        # 需要同时有位姿和图像
        if self._latest_pose is None:
            return

        current_pose = self._latest_pose

        # 判断是否为关键帧
        if not self._is_keyframe(current_pose):
            return

        # 创建关键帧
        kf = SimpleKeyFrame(
            id=self._next_kf_id,
            pose=current_pose,
            image=cv_image if not self._use_slam_module else None,
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        self.keyframes.append(kf)
        self._next_kf_id += 1
        self._last_kf_pose = current_pose

        self.get_logger().info(
            f'新关键帧 #{kf.id}: 位置=({current_pose.t[0]:.2f}, '
            f'{current_pose.t[1]:.2f}, {current_pose.t[2]:.2f}), '
            f'总计 {len(self.keyframes)} 个关键帧')

        # 发布关键帧位姿
        self._publish_keyframe_pose(kf, msg.header)

        # 至少2个关键帧才做三角化
        if len(self.keyframes) >= 2:
            self._triangulate_and_publish(msg.header)

    def _is_keyframe(self, pose: SimplePose3D) -> bool:
        """判断当前帧是否为关键帧"""
        if self._last_kf_pose is None:
            return True

        delta = pose - self._last_kf_pose
        return delta.distance >= self.kf_dist_thresh or \
            delta.angle >= self.kf_angle_thresh

    def _triangulate_and_publish(self, header):
        """对最新两个关键帧进行三角化并发布稀疏点云"""
        kf1 = self.keyframes[-2]
        kf2 = self.keyframes[-1]

        if self._use_slam_module:
            # 使用 slam 模块的 IncrementalSfM
            # 构造 KeyFrame 对象
            from src.slam.data_types import KeyFrame as SlamKF
            skf1 = SlamKF(id=kf1.id, pose=None)  # 简化适配
            skf2 = SlamKF(id=kf2.id, pose=None)
            try:
                self.sfm.triangulate([skf1, skf2])
            except Exception as e:
                self.get_logger().warn(f'slam 模块三角化失败: {e}')
                return
        else:
            new_pts = self.sfm.match_and_triangulate(kf1, kf2)

        # 收集所有三维点
        all_pts = self.sfm.points_3d if hasattr(self.sfm, 'points_3d') else []
        if not all_pts:
            return

        # 发布 PointCloud2
        self._publish_points(all_pts, header)

    def _publish_points(self, points: List[np.ndarray], header):
        """发布稀疏点云"""
        pts_array = np.array(points, dtype=np.float32)
        if pts_array.size == 0:
            return

        try:
            pc_msg = pc2.create_cloud_xyz32(header, pts_array)
            self.points_pub.publish(pc_msg)
            self.get_logger().debug(
                f'发布稀疏点云: {len(pts_array)} 个点')
        except Exception as e:
            self.get_logger().error(f'点云发布失败: {e}')

    def _publish_keyframe_pose(self, kf: SimpleKeyFrame, header):
        """发布关键帧位姿"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(kf.pose.t[0])
        pose_msg.pose.position.y = float(kf.pose.t[1])
        pose_msg.pose.position.z = float(kf.pose.t[2])

        # 旋转矩阵 → 四元数
        R = kf.pose.R
        q = self._rot_to_quat(R)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.kf_pose_pub.publish(pose_msg)

    @staticmethod
    def _rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
        """旋转矩阵 → 四元数 [x, y, z, w]"""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)

    def destroy_node(self):
        self.get_logger().info('SfM 节点正在关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SfMNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
