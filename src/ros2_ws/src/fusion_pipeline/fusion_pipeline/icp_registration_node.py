"""
ICP 精配准节点 (ICP Registration Node)

订阅 UGV LiDAR 滤波点云和 UAV SfM 稀疏点云，使用 ICP 算法进行精配准，
将 UAV SfM 点云对齐到 UGV LiDAR 坐标系下。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PoseStamped

try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    PC2_AVAILABLE = False

import numpy as np
from typing import Optional, Tuple

# 尝试导入 Open3D
try:
    import open3d as o3d
    O3D_AVAILABLE = True
except ImportError:
    O3D_AVAILABLE = False

# 尝试导入 tf2
try:
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════
# 基于 numpy SVD 的 ICP 实现（当 Open3D 不可用时）
# ══════════════════════════════════════════════════════════════════════

def solve_icp_svd(source: np.ndarray, target: np.ndarray,
                  max_iterations: int = 50,
                  max_correspondence_distance: float = 0.5,
                  transformation_epsilon: float = 1e-6,
                  initial_transform: Optional[np.ndarray] = None
                  ) -> Tuple[np.ndarray, float]:
    """基于 SVD 的迭代最近点 (ICP) 配准

    Args:
        source: N×3 源点云
        target: M×3 目标点云
        max_iterations: 最大迭代次数
        max_correspondence_distance: 最大对应点距离阈值
        transformation_epsilon: 变换收敛阈值
        initial_transform: 4×4 初始变换矩阵

    Returns:
        (4×4 变换矩阵, RMSE)
    """
    if initial_transform is None:
        T = np.eye(4)
    else:
        T = initial_transform.copy()

    prev_rmse = float('inf')

    for iteration in range(max_iterations):
        # 变换源点云
        src_h = np.hstack([np.ones((len(source), 1))])
        src_h[:, :3] = source
        src_trans = (T @ src_h.T).T[:, :3]

        # 最近邻对应（暴力搜索，适用于小规模点云）
        correspondences = []
        for i, pt in enumerate(src_trans):
            dists = np.linalg.norm(target - pt, axis=1)
            min_idx = np.argmin(dists)
            min_dist = dists[min_idx]
            if min_dist <= max_correspondence_distance:
                correspondences.append((i, min_idx))

        if len(correspondences) < 3:
            break

        # 构建对应点集
        src_pts = np.array([src_trans[c[0]] for c in correspondences])
        tgt_pts = np.array([target[c[1]] for c in correspondences])

        # 计算质心
        src_centroid = np.mean(src_pts, axis=0)
        tgt_centroid = np.mean(tgt_pts, axis=0)

        # 去质心
        src_centered = src_pts - src_centroid
        tgt_centered = tgt_pts - tgt_centroid

        # SVD 求解旋转
        H = src_centered.T @ tgt_centered
        try:
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            # 保证 det(R) = 1
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
        except np.linalg.LinAlgError:
            break

        t = tgt_centroid - R @ src_centroid

        # 构建增量变换
        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T

        # 计算 RMSE
        residuals = src_centered @ R + src_centroid + t - tgt_pts
        rmse = np.sqrt(np.mean(np.sum(residuals * residuals, axis=1)))

        # 收敛判断
        if abs(prev_rmse - rmse) < transformation_epsilon:
            break
        prev_rmse = rmse

    return T, prev_rmse


# ══════════════════════════════════════════════════════════════════════
# ROS2 节点
# ══════════════════════════════════════════════════════════════════════

class ICPRegistrationNode(Node):
    """ICP 精配准节点

    订阅话题:
        /fusion/pointcloud_filtered (sensor_msgs/PointCloud2) - UGV LiDAR 滤波点云（目标）
        /fusion/sfm_points (sensor_msgs/PointCloud2) - UAV SfM 稀疏点云（源）

    发布话题:
        /fusion/registered_cloud (sensor_msgs/PointCloud2)

    广播 TF:
        ugv_lidar_frame → uav_sfm_frame
    """

    def __init__(self):
        super().__init__('icp_registration_node')

        # ── 声明参数 ──────────────────────────────────────────────
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('transformation_epsilon', 1e-6)
        # 初始粗配准变换（4x4 行优先，16 个元素）
        self.declare_parameter('initial_transform',
                               [1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('source_frame', 'uav_sfm_frame')
        self.declare_parameter('target_frame', 'ugv_lidar_frame')
        self.declare_parameter('publish_tf', True)

        # ── 读取参数 ──────────────────────────────────────────────
        self.max_iterations = self.get_parameter('max_iterations').get_parameter_value().integer_value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').get_parameter_value().double_value
        self.trans_epsilon = self.get_parameter('transformation_epsilon').get_parameter_value().double_value

        T_flat = self.get_parameter('initial_transform').get_parameter_value().double_array_value
        self.initial_T = np.array(T_flat, dtype=np.float64).reshape(4, 4)

        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # ── 状态变量 ──────────────────────────────────────────────
        self.latest_target: Optional[np.ndarray] = None  # UGV LiDAR 点云（目标坐标系）
        self.latest_source: Optional[np.ndarray] = None  # UAV SfM 点云（源坐标系）
        self.current_T = self.initial_T.copy()  # 当前累积变换
        self._target_header = None
        self._source_header = None

        # ── 订阅者 ────────────────────────────────────────────────
        self.target_sub = self.create_subscription(
            PointCloud2, '/fusion/pointcloud_filtered',
            self.target_callback, 10)
        self.source_sub = self.create_subscription(
            PointCloud2, '/fusion/sfm_points',
            self.source_callback, 10)

        # ── 发布者 ────────────────────────────────────────────────
        self.registered_pub = self.create_publisher(
            PointCloud2, '/fusion/registered_cloud', 10)

        # ── TF 广播器 ─────────────────────────────────────────────
        if TF2_AVAILABLE and self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            if not TF2_AVAILABLE:
                self.get_logger().warn('tf2_ros 不可用，TF 广播已禁用')

        if O3D_AVAILABLE:
            self.get_logger().info('ICP 配准节点已启动 (使用 Open3D)')
        else:
            self.get_logger().info('ICP 配准节点已启动 (使用 numpy SVD)')

        self.get_logger().info(
            f'参数: max_iter={self.max_iterations}, '
            f'max_dist={self.max_corr_dist}m, '
            f'epsilon={self.trans_epsilon}')

    def target_callback(self, msg: PointCloud2):
        """接收 UGV LiDAR 滤波点云（目标）"""
        self._target_header = msg.header
        try:
            pc_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
            if pc_list:
                self.latest_target = np.array(pc_list, dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f'目标点云读取失败: {e}')

    def source_callback(self, msg: PointCloud2):
        """接收 UAV SfM 稀疏点云（源）"""
        self._source_header = msg.header
        try:
            pc_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
            if pc_list:
                self.latest_source = np.array(pc_list, dtype=np.float64)
                # 触发 ICP 配准
                self._run_registration(msg.header)
        except Exception as e:
            self.get_logger().error(f'源点云读取失败: {e}')

    def _run_registration(self, header):
        """执行 ICP 配准"""
        if self.latest_source is None or self.latest_target is None:
            return
        if len(self.latest_source) < 3 or len(self.latest_target) < 3:
            self.get_logger().warn('点云点数不足，跳过 ICP 配准')
            return

        source = self.latest_source
        target = self.latest_target

        try:
            if O3D_AVAILABLE:
                T_matrix, rmse = self._icp_open3d(source, target)
            else:
                T_matrix, rmse = solve_icp_svd(
                    source, target,
                    max_iterations=self.max_iterations,
                    max_correspondence_distance=self.max_corr_dist,
                    transformation_epsilon=self.trans_epsilon,
                    initial_transform=self.current_T
                )
        except Exception as e:
            self.get_logger().error(f'ICP 配准失败: {e}')
            return

        # 更新累积变换
        self.current_T = T_matrix

        # 发布 RMS 误差
        self.get_logger().info(f'ICP 配准完成: RMSE = {rmse:.4f}m')

        # 变换源点云并发布
        try:
            registered = self._transform_pointcloud(source, T_matrix)
            # 注意：注册后的点云应使用目标坐标系
            if self._target_header is not None:
                header.frame_id = self._target_header.frame_id

            registered_msg = pc2.create_cloud_xyz32(header, registered.astype(np.float32))
            self.registered_pub.publish(registered_msg)
        except Exception as e:
            self.get_logger().error(f'注册点云发布失败: {e}')

        # 广播 TF
        if self.tf_broadcaster is not None:
            self._broadcast_tf(T_matrix, header)

    def _icp_open3d(self, source: np.ndarray, target: np.ndarray
                     ) -> Tuple[np.ndarray, float]:
        """使用 Open3D 进行 point-to-plane ICP"""
        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source)

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target)

        # 估计法线（point-to-plane ICP 需要）
        tgt_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

        # 使用初始变换
        init = np.eye(4)  # source 已在 SfM 坐标系下，初始为恒等

        result = o3d.pipelines.registration.registration_icp(
            src_pcd, tgt_pcd, self.max_corr_dist, init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=self.trans_epsilon,
                relative_rmse=self.trans_epsilon,
                max_iteration=self.max_iterations)
        )

        T = result.transformation
        rmse = result.inlier_rmse
        return T, rmse

    @staticmethod
    def _transform_pointcloud(points: np.ndarray, T: np.ndarray) -> np.ndarray:
        """使用 4×4 齐次变换矩阵变换点云

        Args:
            points: N×3 点云
            T: 4×4 变换矩阵

        Returns:
            N×3 变换后的点云
        """
        pts_h = np.hstack([points, np.ones((len(points), 1))])
        transformed = (T @ pts_h.T).T
        return transformed[:, :3]

    def _broadcast_tf(self, T: np.ndarray, header):
        """广播 ICP 变换到 tf2"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target_frame
        t.child_frame_id = self.source_frame

        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])

        # 旋转矩阵 → 四元数
        R = T[:3, :3]
        q = self._rot_to_quat(R)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

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
        self.get_logger().info('ICP 配准节点正在关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ICPRegistrationNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
