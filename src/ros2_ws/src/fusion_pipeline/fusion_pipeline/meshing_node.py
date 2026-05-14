"""
三角网格生成节点 (Meshing Node)

订阅 ICP 精配准后的融合点云，使用法线估计 + 泊松重建生成三角网格，
支持 Open3D 和 numpy 简化实现两种模式。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    PC2_AVAILABLE = False

import numpy as np
from typing import Optional, List, Tuple
import os
import time

# 尝试导入 Open3D
try:
    import open3d as o3d
    O3D_AVAILABLE = True
except ImportError:
    O3D_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════
# 基于 numpy 的简化 2.5D Delaunay 网格生成（当 Open3D 不可用时）
# ══════════════════════════════════════════════════════════════════════

def generate_mesh_numpy(points: np.ndarray, grid_resolution: float = 0.1
                         ) -> Tuple[np.ndarray, np.ndarray]:
    """基于 numpy 的简化 2.5D 网格生成

    将点云投影到 X-Y 平面，构建规则网格，取每个网格单元的最高点
    作为顶点，连接相邻顶点形成三角面片。

    Args:
        points: N×3 输入点云
        grid_resolution: 网格分辨率

    Returns:
        (vertices: V×3, triangles: T×3)
    """
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.int32)

    # 计算边界
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()

    # 创建规则网格
    nx = max(int((x_max - x_min) / grid_resolution) + 1, 2)
    ny = max(int((y_max - y_min) / grid_resolution) + 1, 2)

    # 将点分配到网格单元
    grid_z = np.full((nx, ny), -np.inf)
    grid_count = np.zeros((nx, ny), dtype=np.int32)

    for pt in points:
        ix = int((pt[0] - x_min) / grid_resolution)
        iy = int((pt[1] - y_min) / grid_resolution)
        ix = np.clip(ix, 0, nx - 1)
        iy = np.clip(iy, 0, ny - 1)
        grid_z[ix, iy] = max(grid_z[ix, iy], pt[2])
        grid_count[ix, iy] += 1

    # 收集有效顶点
    vertices = []
    vertex_index = -np.ones((nx, ny), dtype=np.int32)

    for ix in range(nx):
        for iy in range(ny):
            if grid_count[ix, iy] > 0:
                vx = x_min + ix * grid_resolution
                vy = y_min + iy * grid_resolution
                vertices.append([vx, vy, grid_z[ix, iy]])
                vertex_index[ix, iy] = len(vertices) - 1

    vertices = np.array(vertices, dtype=np.float32)

    # 构建三角面片
    triangles = []
    for ix in range(nx - 1):
        for iy in range(ny - 1):
            v00 = vertex_index[ix, iy]
            v10 = vertex_index[ix + 1, iy]
            v01 = vertex_index[ix, iy + 1]
            v11 = vertex_index[ix + 1, iy + 1]

            # 两个三角形组成一个矩形单元
            if all(v >= 0 for v in [v00, v10, v01]):
                triangles.append([v00, v10, v01])
            if all(v >= 0 for v in [v10, v11, v01]):
                triangles.append([v10, v11, v01])

    triangles = np.array(triangles, dtype=np.int32)
    return vertices, triangles


def save_mesh_ply(vertices: np.ndarray, triangles: np.ndarray,
                   filepath: str):
    """将网格保存为 PLY 格式

    Args:
        vertices: V×3 顶点数组
        triangles: T×3 面片索引数组
        filepath: 输出文件路径
    """
    os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.',
                exist_ok=True)

    with open(filepath, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(triangles)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")

        for v in vertices:
            f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        for t in triangles:
            f.write(f"3 {t[0]} {t[1]} {t[2]}\n")


def save_mesh_obj(vertices: np.ndarray, triangles: np.ndarray,
                   filepath: str):
    """将网格保存为 OBJ 格式

    Args:
        vertices: V×3 顶点数组
        triangles: T×3 面片索引数组
        filepath: 输出文件路径
    """
    os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.',
                exist_ok=True)

    with open(filepath, 'w') as f:
        f.write("# 空地协同测绘网格数据\n")

        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        # OBJ 面片索引从 1 开始
        for t in triangles:
            f.write(f"f {t[0] + 1} {t[1] + 1} {t[2] + 1}\n")


# ══════════════════════════════════════════════════════════════════════
# ROS2 节点
# ══════════════════════════════════════════════════════════════════════

class MeshingNode(Node):
    """三角网格生成节点

    订阅话题:
        /fusion/registered_cloud (sensor_msgs/PointCloud2)

    发布话题:
        /fusion/meshing_status (std_msgs/String)
    """

    def __init__(self):
        super().__init__('meshing_node')

        # ── 声明参数 ──────────────────────────────────────────────
        self.declare_parameter('mesh_depth', 8)
        self.declare_parameter('mesh_scale', 1.1)
        self.declare_parameter('output_dir', './output/meshes')
        self.declare_parameter('output_format', 'ply')  # 'ply' 或 'obj' 或 'both'
        self.declare_parameter('grid_resolution', 0.1)   # numpy 2.5D 网格分辨率
        self.declare_parameter('normal_radius', 0.1)     # 法线估计半径
        self.declare_parameter('auto_save', True)        # 是否自动保存网格

        # ── 读取参数 ──────────────────────────────────────────────
        self.mesh_depth = self.get_parameter('mesh_depth').get_parameter_value().integer_value
        self.mesh_scale = self.get_parameter('mesh_scale').get_parameter_value().double_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_format = self.get_parameter('output_format').get_parameter_value().string_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.normal_radius = self.get_parameter('normal_radius').get_parameter_value().double_value
        self.auto_save = self.get_parameter('auto_save').get_parameter_value().bool_value

        # ── 状态变量 ──────────────────────────────────────────────
        self.latest_points: Optional[np.ndarray] = None
        self._mesh_counter = 0
        self._latest_header = None

        # ── 订阅者 ────────────────────────────────────────────────
        self.pc_sub = self.create_subscription(
            PointCloud2, '/fusion/registered_cloud',
            self.pc_callback, 10)

        # ── 发布者 ────────────────────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/fusion/meshing_status', 10)

        if O3D_AVAILABLE:
            self.get_logger().info('网格生成节点已启动 (使用 Open3D 泊松重建)')
        else:
            self.get_logger().info('网格生成节点已启动 (使用 numpy 2.5D 网格)')

        self.get_logger().info(
            f'参数: depth={self.mesh_depth}, scale={self.mesh_scale}, '
            f'output={self.output_dir}')

    def pc_callback(self, msg: PointCloud2):
        """接收注册后的融合点云"""
        self._latest_header = msg.header
        try:
            pc_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
            if not pc_list:
                self.get_logger().warn('收到空点云')
                return
            self.latest_points = np.array(pc_list, dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f'点云读取失败: {e}')
            return

        # 自动生成网格
        if self.auto_save:
            self._generate_and_save()

    def _generate_and_save(self):
        """生成网格并保存到文件"""
        if self.latest_points is None or len(self.latest_points) < 3:
            self._publish_status('error', '点云点数不足，无法生成网格')
            return

        self._publish_status('processing', f'开始生成网格 ({len(self.latest_points)} 个点)...')
        t_start = time.time()

        try:
            if O3D_AVAILABLE:
                vertices, triangles = self._mesh_open3d(self.latest_points)
                method = 'Open3D Poisson'
            else:
                vertices, triangles = generate_mesh_numpy(
                    self.latest_points, self.grid_resolution)
                method = 'numpy 2.5D'

            elapsed = time.time() - t_start
        except Exception as e:
            self.get_logger().error(f'网格生成失败: {e}')
            self._publish_status('error', f'网格生成失败: {e}')
            return

        if len(vertices) == 0 or len(triangles) == 0:
            self._publish_status('error', '生成的网格为空')
            return

        # 生成文件名
        timestamp = int(time.time())
        self._mesh_counter += 1
        base_name = f'mesh_{self._mesh_counter:04d}_{timestamp}'

        # 保存文件
        saved_files = []
        try:
            if self.output_format in ('ply', 'both'):
                ply_path = os.path.join(self.output_dir, f'{base_name}.ply')
                save_mesh_ply(vertices, triangles, ply_path)
                saved_files.append(ply_path)

            if self.output_format in ('obj', 'both'):
                obj_path = os.path.join(self.output_dir, f'{base_name}.obj')
                save_mesh_obj(vertices, triangles, obj_path)
                saved_files.append(obj_path)
        except Exception as e:
            self.get_logger().error(f'网格文件保存失败: {e}')
            self._publish_status('error', f'文件保存失败: {e}')
            return

        status_msg = (
            f'网格生成完成 ({method}): '
            f'{len(vertices)} 顶点, {len(triangles)} 面片, '
            f'耗时 {elapsed:.2f}s, '
            f'文件: {", ".join(saved_files)}'
        )
        self.get_logger().info(status_msg)
        self._publish_status('done', status_msg)

    def _mesh_open3d(self, points: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray]:
        """使用 Open3D 进行法线估计 + 泊松重建

        Args:
            points: N×3 点云

        Returns:
            (vertices: V×3, triangles: T×3)
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 法线估计
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(k=30)

        # 泊松重建
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=self.mesh_depth, scale=self.mesh_scale)

        # 移除低密度区域的三角形
        density_threshold = np.quantile(densities, 0.1)
        vertices_to_remove = densities < density_threshold
        mesh.remove_vertices_by_mask(vertices_to_remove)

        # 提取顶点和面片
        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        triangles = np.asarray(mesh.triangles, dtype=np.int32)

        return vertices, triangles

    def _publish_status(self, state: str, message: str):
        """发布网格生成状态"""
        msg = String()
        msg.data = f'[{state.upper()}] {message}'
        self.status_pub.publish(msg)

    def generate_on_demand(self, points: Optional[np.ndarray] = None):
        """手动触发网格生成（可通过服务或参数调用）

        Args:
            points: 可选，直接传入 N×3 点云；None 则使用最近收到的点云
        """
        if points is not None:
            self.latest_points = points
        self._generate_and_save()

    def destroy_node(self):
        self.get_logger().info('网格生成节点正在关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MeshingNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
