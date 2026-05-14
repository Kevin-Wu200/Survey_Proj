"""
点云滤波节点 (PointCloud Filter Node)

订阅 UGV LiDAR 原始点云，实现体素降采样、统计离群点去除和可选直通滤波，
发布滤波后的点云。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    PC2_AVAILABLE = False

import numpy as np


class VoxelGrid:
    """体素栅格降采样

    将点云空间划分为固定大小的体素，每个体素内用质心替代所有点。
    """

    def __init__(self, leaf_size: float = 0.05):
        self.leaf_size = leaf_size

    def filter(self, points: np.ndarray) -> np.ndarray:
        """对 N×3 点云进行体素降采样

        Args:
            points: N×3 输入点云

        Returns:
            M×3 降采样后的点云
        """
        if points.size == 0:
            return np.zeros((0, 3))

        # 量化到体素索引
        voxel_indices = np.floor(points / self.leaf_size).astype(np.int64)

        # 使用字典聚合同一体素的点
        voxel_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])

        # 每个体素用质心表示
        filtered = np.zeros((len(voxel_dict), 3))
        for i, pts in enumerate(voxel_dict.values()):
            filtered[i] = np.mean(pts, axis=0)

        return filtered


class StatisticalOutlierRemoval:
    """统计离群点去除

    计算每个点到其 K 近邻的平均距离，移除超过阈值（均值 + std_mul * 标准差）的点。
    """

    def __init__(self, mean_k: int = 50, std_mul: float = 1.0):
        self.mean_k = mean_k
        self.std_mul = std_mul

    def filter(self, points: np.ndarray) -> np.ndarray:
        """去除统计离群点

        Args:
            points: N×3 输入点云

        Returns:
            M×3 滤波后的点云
        """
        if len(points) < self.mean_k:
            return points

        n = len(points)
        # 计算所有点对距离（仅对最近 K 个）
        # 使用分块计算避免内存爆炸
        mean_distances = np.zeros(n)

        for i in range(n):
            diff = points - points[i]
            dists = np.sqrt(np.sum(diff * diff, axis=1))
            # 排除自身（距离为0）
            dists_sorted = np.sort(dists)
            # 取第 1 到 mean_k+1 个（跳过自身的0）
            mean_distances[i] = np.mean(dists_sorted[1:self.mean_k + 1])

        threshold = np.mean(mean_distances) + self.std_mul * np.std(mean_distances)
        mask = mean_distances <= threshold
        return points[mask]


class PassThroughFilter:
    """直通滤波器

    保留指定坐标轴在给定范围内的点。
    """

    def __init__(self, axis: str = 'z', min_val: float = 0.1, max_val: float = 30.0):
        self.axis = axis
        self.min_val = min_val
        self.max_val = max_val

    def filter(self, points: np.ndarray) -> np.ndarray:
        """按坐标范围过滤

        Args:
            points: N×3 输入点云

        Returns:
            M×3 滤波后的点云
        """
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[self.axis]
        mask = (points[:, axis_idx] >= self.min_val) & \
               (points[:, axis_idx] <= self.max_val)
        return points[mask]


class PointCloudFilterNode(Node):
    """点云滤波节点

    订阅话题:
        /ugv/lidar/points (sensor_msgs/PointCloud2)

    发布话题:
        /fusion/pointcloud_filtered (sensor_msgs/PointCloud2)
    """

    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # ── 声明参数 ──────────────────────────────────────────────
        # VoxelGrid 体素大小
        self.declare_parameter('voxel_leaf_size', 0.05)
        # StatisticalOutlierRemoval 参数
        self.declare_parameter('sor_mean_k', 50)
        self.declare_parameter('sor_std_mul', 1.0)
        # 直通滤波
        self.declare_parameter('passthrough_enabled', True)
        self.declare_parameter('passthrough_axis', 'z')
        self.declare_parameter('passthrough_min', 0.1)
        self.declare_parameter('passthrough_max', 30.0)
        # 是否启用 SOR
        self.declare_parameter('sor_enabled', True)

        # ── 读取参数 ──────────────────────────────────────────────
        voxel_leaf_size = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
        sor_mean_k = self.get_parameter('sor_mean_k').get_parameter_value().integer_value
        sor_std_mul = self.get_parameter('sor_std_mul').get_parameter_value().double_value
        self.sor_enabled = self.get_parameter('sor_enabled').get_parameter_value().bool_value
        self.passthrough_enabled = self.get_parameter('passthrough_enabled').get_parameter_value().bool_value
        pt_axis = self.get_parameter('passthrough_axis').get_parameter_value().string_value
        pt_min = self.get_parameter('passthrough_min').get_parameter_value().double_value
        pt_max = self.get_parameter('passthrough_max').get_parameter_value().double_value

        # ── 初始化滤波器 ──────────────────────────────────────────
        self.voxel_filter = VoxelGrid(leaf_size=voxel_leaf_size)
        self.sor_filter = StatisticalOutlierRemoval(mean_k=sor_mean_k,
                                                     std_mul=sor_std_mul)
        self.passthrough_filter = PassThroughFilter(axis=pt_axis,
                                                     min_val=pt_min,
                                                     max_val=pt_max)

        # ── 订阅者 ────────────────────────────────────────────────
        self.pc_sub = self.create_subscription(
            PointCloud2, '/ugv/lidar/points',
            self.pc_callback, 10)

        # ── 发布者 ────────────────────────────────────────────────
        self.filtered_pub = self.create_publisher(
            PointCloud2, '/fusion/pointcloud_filtered', 10)

        self.get_logger().info(
            f'点云滤波节点已启动: voxel={voxel_leaf_size}m, '
            f'sor_k={sor_mean_k}, sor_mul={sor_std_mul}, '
            f'passthrough={"启用" if self.passthrough_enabled else "禁用"}'
            f'({pt_axis}∈[{pt_min},{pt_max}])')

    def pc_callback(self, msg: PointCloud2):
        """接收 PointCloud2 消息并处理"""
        try:
            # 转换 PointCloud2 → numpy 数组 (N×3 xyz)
            pc_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
            if not pc_list:
                return
            points = np.array(pc_list, dtype=np.float32)
            original_count = len(points)
        except Exception as e:
            self.get_logger().error(f'点云读取失败: {e}')
            return

        try:
            # 直通滤波（可选）
            if self.passthrough_enabled:
                points = self.passthrough_filter.filter(points)

            # 体素降采样
            points = self.voxel_filter.filter(points)

            # 统计离群点去除（可选）
            if self.sor_enabled and len(points) > 0:
                points = self.sor_filter.filter(points)

            filtered_count = len(points)
            self.get_logger().debug(
                f'点云滤波: {original_count} → {filtered_count} 点 '
                f'(减少 {100 * (1 - filtered_count / original_count):.1f}%)')
        except Exception as e:
            self.get_logger().error(f'点云滤波失败: {e}')
            return

        if filtered_count == 0:
            self.get_logger().warn('滤波后点云为空')
            return

        # 构建输出 PointCloud2 消息
        try:
            filtered_msg = pc2.create_cloud_xyz32(msg.header, points)
            self.filtered_pub.publish(filtered_msg)
        except Exception as e:
            self.get_logger().error(f'点云消息创建失败: {e}')

    def destroy_node(self):
        self.get_logger().info('点云滤波节点正在关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PointCloudFilterNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
