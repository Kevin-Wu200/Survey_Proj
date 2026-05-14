"""
精配准模块 (Fine Registration)

实现 ICP point-to-plane 精配准和 NDT 备选算法。
优先使用 Open3D，若不支持则回退到纯 numpy 实现。

目标精度：RMSE < 5cm
"""

import numpy as np
import time
from typing import Optional, Tuple, Dict


# ══════════════════════════════════════════════════════════════════════
# Open3D 可用性检测
# ══════════════════════════════════════════════════════════════════════

try:
    import open3d as o3d
    _HAS_OPEN3D = True
except ImportError:
    _HAS_OPEN3D = False


# ══════════════════════════════════════════════════════════════════════
# 默认配置
# ══════════════════════════════════════════════════════════════════════

DEFAULT_CONFIG = {
    "max_iterations": 100,
    "max_correspondence_distance": 0.3,
    "transformation_epsilon": 1e-8,
    "use_point_to_plane": True,
    "use_open3d": True,
    "rmse_threshold": 0.05,
    "ndt_voxel_size": 0.5,
    "ndt_max_iterations": 50,
    "ndt_step_size": 0.1,
    "ndt_epsilon": 1e-5,
}


# ══════════════════════════════════════════════════════════════════════
# KD-Tree 实现（纯 numpy）
# ══════════════════════════════════════════════════════════════════════

class _SimpleKDTree:
    """简易 KD-Tree，用于纯 numpy ICP 中的最近邻搜索"""

    def __init__(self, points: np.ndarray, leaf_size: int = 32):
        self._points = points
        self._leaf_size = leaf_size
        self._dim = points.shape[1]
        self._tree = self._build(points, list(range(len(points))), 0)

    def _build(self, pts: np.ndarray, indices: list, depth: int) -> dict:
        """递归构建 KD-Tree"""
        if len(indices) <= self._leaf_size:
            return {"indices": indices, "split_dim": None, "split_val": None,
                    "left": None, "right": None}

        dim = depth % self._dim
        vals = pts[indices, dim]
        median_idx = len(indices) // 2
        sorted_order = np.argsort(vals)
        sorted_indices = [indices[i] for i in sorted_order]

        node = {"indices": None, "split_dim": dim,
                "split_val": vals[sorted_order[median_idx]]}

        node["left"] = self._build(pts, sorted_indices[:median_idx], depth + 1)
        node["right"] = self._build(pts, sorted_indices[median_idx:], depth + 1)
        return node

    def query(self, point: np.ndarray, k: int = 1) -> Tuple[np.ndarray, np.ndarray]:
        """查询 k 个最近邻

        Returns:
            (distances, indices)
        """
        best_dists = np.full(k, np.inf)
        best_idxs = np.full(k, -1, dtype=int)
        self._search(self._tree, point, best_dists, best_idxs, 0)
        # 只返回最近的
        return best_dists[:k], best_idxs[:k]

    def _search(self, node: dict, point: np.ndarray,
                best_dists: np.ndarray, best_idxs: np.ndarray, depth: int):
        """递归搜索最近邻"""
        if node["indices"] is not None:
            # 叶子节点：暴力搜索
            leaf_pts = self._points[node["indices"]]
            dists = np.linalg.norm(leaf_pts - point, axis=1)
            for i, d in enumerate(dists):
                if d < best_dists[0]:
                    best_dists[0] = d
                    best_idxs[0] = node["indices"][i]
            return

        dim = node["split_dim"]
        if point[dim] < node["split_val"]:
            nearer, farther = node["left"], node["right"]
        else:
            nearer, farther = node["right"], node["left"]

        self._search(nearer, point, best_dists, best_idxs, depth + 1)

        # 检查是否需要搜索远端分支
        if abs(point[dim] - node["split_val"]) < best_dists[0]:
            self._search(farther, point, best_dists, best_idxs, depth + 1)


# ══════════════════════════════════════════════════════════════════════
# 法向量估计（纯 numpy）
# ══════════════════════════════════════════════════════════════════════

def _estimate_normals(points: np.ndarray, k: int = 20) -> np.ndarray:
    """使用 PCA 估计点云法向量

    Args:
        points:  (N, 3) 点云
        k:       近邻点数

    Returns:
        (N, 3) 法向量数组
    """
    tree = _SimpleKDTree(points)
    normals = np.zeros_like(points)

    for i, pt in enumerate(points):
        _, idxs = tree.query(pt, k=k)
        neighbors = points[idxs]

        # PCA：协方差矩阵最小特征值对应的特征向量即为法向量
        centered = neighbors - neighbors.mean(axis=0)
        cov = centered.T @ centered / (len(neighbors) - 1)

        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, 0]  # 最小特征值对应特征向量
        normals[i] = normal / (np.linalg.norm(normal) + 1e-10)

    return normals


# ══════════════════════════════════════════════════════════════════════
# numpy 版 ICP point-to-plane
# ══════════════════════════════════════════════════════════════════════

def _icp_point_to_plane_numpy(
    source: np.ndarray,
    target: np.ndarray,
    initial_transform: np.ndarray,
    max_iterations: int,
    max_correspondence_distance: float,
    transformation_epsilon: float,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """纯 numpy 实现的 point-to-plane ICP

    使用 KD-Tree 最近邻搜索建立对应关系，
    通过 SVD 求解增量变换，迭代至收敛。

    Args:
        source:                     源点云 (N, 3)
        target:                     目标点云 (M, 3)
        initial_transform:          初始 4x4 变换矩阵
        max_iterations:             最大迭代次数
        max_correspondence_distance: 最大对应点距离阈值
        transformation_epsilon:     变换收敛阈值

    Returns:
        (变换后的源点云, RMSE, 4x4 变换矩阵)
    """
    # 预计算目标点云法向量和 KD-Tree
    target_normals = _estimate_normals(target)
    target_tree = _SimpleKDTree(target)

    # 初始化累积变换
    T = initial_transform.copy()
    T_accum = np.eye(4)

    # 将源点云应用初始变换
    transformed = (T[:3, :3] @ source.T).T + T[:3, 3]

    prev_error = float('inf')

    for iteration in range(max_iterations):
        # --- 1. 对应点搜索 ---
        correspondences_source = []
        correspondences_target = []
        correspondences_normals = []

        for i, pt in enumerate(transformed):
            dists, idxs = target_tree.query(pt, k=1)
            if dists[0] < max_correspondence_distance:
                correspondences_source.append(pt)
                correspondences_target.append(target[idxs[0]])
                correspondences_normals.append(target_normals[idxs[0]])

        if len(correspondences_source) < 3:
            break

        src_corr = np.array(correspondences_source)        # (C, 3)
        tgt_corr = np.array(correspondences_target)        # (C, 3)
        nrm_corr = np.array(correspondences_normals)       # (C, 3)

        # --- 2. 构建 point-to-plane 线性系统 ---
        # 残差: r_i = n_i^T (R * p_i + t - q_i)
        # 线性化: r_i ≈ n_i^T (p_i + α×p_i + t_inc - q_i)
        # 其中 α 为微小旋转向量，t_inc 为微小平移
        # 构建 A x = b，其中 x = [α; t_inc] ∈ R⁶

        A = np.zeros((len(src_corr), 6))
        b = np.zeros(len(src_corr))

        for i in range(len(src_corr)):
            p = src_corr[i]
            q = tgt_corr[i]
            n = nrm_corr[i]

            # 旋转雅可比: ∂(n^T R p)/∂α ≈ (p × n)^T
            #   n^T (α × p) = α^T (p × n)，故系数为 (p × n)
            cross_pn = np.cross(p, n)
            A[i, :3] = cross_pn
            A[i, 3:] = n

            # 残差
            b[i] = np.dot(n, q - p)

        # --- 3. 求解增量变换 ---
        try:
            x, residuals, rank, singulars = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            break

        # 提取旋转和平移增量
        alpha = x[:3]       # 旋转向量 (angle-axis)
        t_inc = x[3:]

        # 旋转向量 → 旋转矩阵 (Rodrigues公式)
        theta = np.linalg.norm(alpha)
        if theta < 1e-10:
            R_inc = np.eye(3)
        else:
            axis = alpha / theta
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R_inc = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

        T_inc = np.eye(4)
        T_inc[:3, :3] = R_inc
        T_inc[:3, 3] = t_inc

        # --- 4. 更新变换 ---
        T_accum = T_inc @ T_accum
        T = T_accum @ initial_transform

        # 应用当前累积变换
        transformed = (T_accum[:3, :3] @ (initial_transform[:3, :3] @ source.T + initial_transform[:3, 3:4])).T
        transformed += T_accum[:3, 3]

        # --- 5. 收敛检查 ---
        current_error = np.sqrt(np.mean(b ** 2))
        if abs(prev_error - current_error) < transformation_epsilon:
            break
        prev_error = current_error

    # 计算最终 RMSE
    final_tree = _SimpleKDTree(target)
    final_dists = []
    for pt in transformed:
        d, _ = final_tree.query(pt, k=1)
        final_dists.append(d[0])
    rmse = float(np.sqrt(np.mean(np.array(final_dists) ** 2)))

    return transformed, rmse, T


# ══════════════════════════════════════════════════════════════════════
# Open3D 版 ICP
# ══════════════════════════════════════════════════════════════════════

def _icp_open3d(
    source: np.ndarray,
    target: np.ndarray,
    initial_transform: np.ndarray,
    max_correspondence_distance: float,
    transformation_epsilon: float,
    max_iterations: int,
    use_point_to_plane: bool,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """使用 Open3D 进行 ICP 配准

    Args:
        source:                     源点云 (N, 3)
        target:                     目标点云 (M, 3)
        initial_transform:          初始 4x4 变换矩阵
        max_correspondence_distance: 最大对应距离
        transformation_epsilon:     变换收敛阈值
        max_iterations:             最大迭代次数
        use_point_to_plane:         是否使用 point-to-plane

    Returns:
        (变换后的源点云, RMSE, 4x4 变换矩阵)
    """
    # 构建 Open3D 点云对象
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source)

    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target)

    # 如使用 point-to-plane，估计法向量
    if use_point_to_plane:
        tgt_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30)
        )
        src_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30)
        )

    # 选择 ICP 方法
    if use_point_to_plane:
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        loss = o3d.pipelines.registration.TukeyLoss(k=max_correspondence_distance)
    else:
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        loss = o3d.pipelines.registration.HuberLoss(k=max_correspondence_distance)

    # 执行 ICP
    result = o3d.pipelines.registration.registration_icp(
        src_pcd, tgt_pcd,
        max_correspondence_distance,
        initial_transform,
        estimation_method=estimation_method,
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=transformation_epsilon,
            relative_rmse=transformation_epsilon,
            max_iteration=max_iterations,
        ),
    )

    # 变换源点云
    transformed = (result.transformation[:3, :3] @ source.T).T + result.transformation[:3, 3]

    return transformed, result.inlier_rmse, result.transformation


# ══════════════════════════════════════════════════════════════════════
# numpy 版 NDT
# ══════════════════════════════════════════════════════════════════════

def _compute_ndt_cells(points: np.ndarray, voxel_size: float
                       ) -> Dict[Tuple, Dict]:
    """将目标点云栅格化，计算每个栅格的均值和协方差

    Args:
        points:     目标点云 (M, 3)
        voxel_size: 体素大小

    Returns:
        字典 {栅格索引: {"mean": μ, "cov": Σ, "count": n}}
    """
    cells: Dict[Tuple, Dict] = {}

    for pt in points:
        idx = tuple(np.floor(pt / voxel_size).astype(int))
        if idx not in cells:
            cells[idx] = {"points": []}
        cells[idx]["points"].append(pt)

    # 计算每个栅格的均值与协方差
    for idx in list(cells.keys()):
        pts = np.array(cells[idx]["points"])
        n = len(pts)
        if n < 3:  # 最少3点才能估计协方差
            del cells[idx]
            continue

        mean = pts.mean(axis=0)
        cov = np.cov(pts.T, bias=True)  # 使用总体协方差

        # 正则化：防止奇异矩阵
        cov += np.eye(3) * 1e-4

        cells[idx] = {
            "mean": mean,
            "cov": cov,
            "inv_cov": np.linalg.inv(cov),
            "count": n,
        }

    return cells


def _ndt_score_and_gradient(
    points: np.ndarray,
    cells: Dict[Tuple, Dict],
    voxel_size: float,
    T: np.ndarray,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """计算 NDT 评分函数值及其雅可比和黑塞矩阵

    评分函数: s(p) = -Σ exp(-½ (p_i' - μ_k)^T Σ_k^{-1} (p_i' - μ_k))
    其中 p_i' = R p_i + t

    Args:
        points:     源点云 (N, 3)
        cells:      目标 NDT 栅格
        voxel_size: 体素大小
        T:          当前 4x4 变换矩阵

    Returns:
        (score, gradient[6], hessian[6,6])
    """
    R = T[:3, :3]
    t = T[:3, 3]

    score = 0.0
    gradient = np.zeros(6)
    hessian = np.zeros((6, 6))

    for pt in points:
        # 变换源点
        pt_t = R @ pt + t

        # 找到对应栅格
        idx = tuple(np.floor(pt_t / voxel_size).astype(int))
        if idx not in cells:
            continue

        cell = cells[idx]
        mu = cell["mean"]
        inv_cov = cell["inv_cov"]

        # 残差
        q = pt_t - mu  # (3,)

        # NDT 概率: exp(-0.5 * q^T Σ^{-1} q)
        exponent = -0.5 * q @ inv_cov @ q
        if exponent < -50:  # 数值稳定
            continue

        prob = np.exp(np.clip(exponent, -100, 100))

        # 对变换参数的雅可比: J = ∂p'/∂x = [I, -[p']_×]
        # 分数对变换的导数
        d_score_d_p = -prob * (inv_cov @ q)  # (3,)

        # 雅可比 (3, 6)：[∂p'/∂t, ∂p'/∂α]
        J = np.zeros((3, 6))
        J[:, 3:] = np.eye(3)  # ∂p'/∂t = I
        # ∂p'/∂α = -[p']_× (旋转雅可比)
        J[:, :3] = np.array([
            [0, pt_t[2], -pt_t[1]],
            [-pt_t[2], 0, pt_t[0]],
            [pt_t[1], -pt_t[0], 0],
        ])

        # 黑塞近似: J^T Σ^{-1} J * prob
        H_approx = J.T @ inv_cov @ J * prob

        score += prob
        gradient += J.T @ d_score_d_p

        # 添加黑塞贡献
        hessian += H_approx

        # 外部积贡献
        Jg = J.T @ d_score_d_p
        hessian += np.outer(Jg, Jg) / (prob + 1e-10)

    return -score, -gradient, -hessian


def _ndt_register_numpy(
    source: np.ndarray,
    target: np.ndarray,
    initial_transform: np.ndarray,
    voxel_size: float,
    max_iterations: int,
    step_size: float,
    epsilon: float,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """纯 numpy 实现的 NDT 精配准

    使用高斯-牛顿优化最小化 NDT 评分函数。

    Args:
        source:            源点云 (N, 3)
        target:            目标点云 (M, 3)
        initial_transform: 初始 4x4 变换矩阵
        voxel_size:        体素大小
        max_iterations:    最大迭代次数
        step_size:         高斯-牛顿步长
        epsilon:           收敛阈值

    Returns:
        (变换后的源点云, RMSE, 4x4 变换矩阵)
    """
    # 构建目标 NDT 栅格
    cells = _compute_ndt_cells(target, voxel_size)

    if len(cells) == 0:
        return source.copy(), float('inf'), initial_transform.copy()

    T = initial_transform.copy()
    prev_score = float('inf')

    for iteration in range(max_iterations):
        score, grad, hess = _ndt_score_and_gradient(
            source, cells, voxel_size, T
        )

        # 高斯-牛顿更新
        try:
            # 正则化黑塞矩阵
            hess_reg = hess + np.eye(6) * 1e-2
            delta = np.linalg.solve(hess_reg, -grad)
        except np.linalg.LinAlgError:
            delta = np.zeros(6)

        # 回溯线搜索
        alpha = step_size
        best_alpha = 0.0
        best_new_score = score
        for _ in range(6):  # 最多6次回溯
            # 构建候选变换
            delta_scaled = delta * alpha
            delta_alpha = delta_scaled[:3]
            delta_t = delta_scaled[3:]

            theta = np.linalg.norm(delta_alpha)
            if theta < 1e-10:
                R_delta = np.eye(3)
            else:
                axis_scaled = delta_alpha / theta
                K = np.array([
                    [0, -axis_scaled[2], axis_scaled[1]],
                    [axis_scaled[2], 0, -axis_scaled[0]],
                    [-axis_scaled[1], axis_scaled[0], 0],
                ])
                R_delta = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

            T_delta_candidate = np.eye(4)
            T_delta_candidate[:3, :3] = R_delta
            T_delta_candidate[:3, 3] = delta_t

            T_candidate = T_delta_candidate @ T

            new_score, _, _ = _ndt_score_and_gradient(
                source, cells, voxel_size, T_candidate
            )

            if new_score < best_new_score:
                best_new_score = new_score
                best_alpha = alpha
            alpha *= 0.5

        # 应用最佳步长
        if best_alpha > 0:
            delta_best = delta * best_alpha
        else:
            delta_best = delta * step_size * 0.1

        delta_alpha = delta_best[:3]
        delta_t = delta_best[3:]

        theta = np.linalg.norm(delta_alpha)
        if theta < 1e-10:
            R_delta = np.eye(3)
        else:
            axis = delta_alpha / theta
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0],
            ])
            R_delta = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

        T_delta = np.eye(4)
        T_delta[:3, :3] = R_delta
        T_delta[:3, 3] = delta_t

        T = T_delta @ T

        # 收敛检查
        if abs(prev_score - best_new_score) < epsilon:
            break
        prev_score = best_new_score

    # 变换源点云并计算 RMSE
    transformed = (T[:3, :3] @ source.T).T + T[:3, 3]
    tree = _SimpleKDTree(target)
    dists = [tree.query(pt, k=1)[0][0] for pt in transformed]
    rmse = float(np.sqrt(np.mean(np.array(dists) ** 2)))

    return transformed, rmse, T


# ══════════════════════════════════════════════════════════════════════
# FineRegistrator 主类
# ══════════════════════════════════════════════════════════════════════

class FineRegistrator:
    """精配准器

    使用 ICP point-to-plane 作为主要算法，NDT 作为备选。
    支持 Open3D 和纯 numpy 两种后端。

    Usage:
        registrator = FineRegistrator()
        aligned, rmse, T = registrator.register_icp(source, target, init_transform)
        report = registrator.generate_report()
    """

    def __init__(self, config: dict = None):
        """初始化精配准器

        Args:
            config: 配置字典，未指定的键使用默认值
                    默认配置: max_iterations=100, max_correspondence_distance=0.3,
                    transformation_epsilon=1e-8, use_point_to_plane=True,
                    use_open3d=True, rmse_threshold=0.05
        """
        self.config = DEFAULT_CONFIG.copy()
        if config:
            self.config.update(config)

        self._source: Optional[np.ndarray] = None
        self._target: Optional[np.ndarray] = None
        self._aligned: Optional[np.ndarray] = None
        self._transform: Optional[np.ndarray] = None
        self._rmse: Optional[float] = None
        self._method: str = ""
        self._evaluation: Dict = {}
        self._report: Dict = {}

    def register_icp(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_transform: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, float, np.ndarray]:
        """ICP point-to-plane 精配准

        优先使用 Open3D，否则回退到纯 numpy SVD 实现。

        Args:
            source:            源点云 (N, 3)
            target:            目标点云 (M, 3)
            initial_transform: 初始 4x4 变换矩阵，默认为单位矩阵

        Returns:
            (变换后的源点云, RMSE, 4x4 变换矩阵)
        """
        self._source = source.copy()
        self._target = target.copy()
        self._method = "ICP"

        if initial_transform is None:
            initial_transform = np.eye(4)

        use_o3d = self.config["use_open3d"] and _HAS_OPEN3D

        if use_o3d:
            self._method = "ICP (Open3D Point-to-Plane)"
            self._aligned, self._rmse, self._transform = _icp_open3d(
                source, target, initial_transform,
                self.config["max_correspondence_distance"],
                self.config["transformation_epsilon"],
                self.config["max_iterations"],
                self.config["use_point_to_plane"],
            )
        else:
            self._method = "ICP (NumPy Point-to-Plane)"
            self._aligned, self._rmse, self._transform = _icp_point_to_plane_numpy(
                source, target, initial_transform,
                self.config["max_iterations"],
                self.config["max_correspondence_distance"],
                self.config["transformation_epsilon"],
            )

        # 评估配准质量
        self._evaluation = self.evaluate_registration(source, target, self._transform)
        self._report = self.generate_report()

        return self._aligned, self._rmse, self._transform

    def register_ndt(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_transform: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, float, np.ndarray]:
        """NDT (Normal Distributions Transform) 精配准

        3D 栅格化目标点云，使用高斯-牛顿优化最小化 NDT 评分函数。

        Args:
            source:            源点云 (N, 3)
            target:            目标点云 (M, 3)
            initial_transform: 初始 4x4 变换矩阵，默认为单位矩阵

        Returns:
            (变换后的源点云, RMSE, 4x4 变换矩阵)
        """
        self._source = source.copy()
        self._target = target.copy()
        self._method = "NDT (NumPy)"

        if initial_transform is None:
            initial_transform = np.eye(4)

        self._aligned, self._rmse, self._transform = _ndt_register_numpy(
            source, target, initial_transform,
            self.config["ndt_voxel_size"],
            self.config["ndt_max_iterations"],
            self.config["ndt_step_size"],
            self.config["ndt_epsilon"],
        )

        # 评估配准质量
        self._evaluation = self.evaluate_registration(source, target, self._transform)
        self._report = self.generate_report()

        return self._aligned, self._rmse, self._transform

    def evaluate_registration(
        self,
        source: np.ndarray,
        target: np.ndarray,
        transform: np.ndarray,
    ) -> dict:
        """全面评估配准质量

        评估指标包括：
        - RMSE：最近邻均方根误差
        - 重叠率：落在目标点云附近的源点比例
        - 对应点统计：有效对应点数量

        Args:
            source:    源点云 (N, 3)
            target:    目标点云 (M, 3)
            transform: 4x4 变换矩阵

        Returns:
            评估结果字典
        """
        # 变换源点云
        transformed = (transform[:3, :3] @ source.T).T + transform[:3, 3]

        # 构建目标 KD-Tree
        tree = _SimpleKDTree(target)

        # 计算最近邻距离
        all_dists = []
        overlap_count = 0
        dist_threshold = self.config["max_correspondence_distance"]

        for pt in transformed:
            d, _ = tree.query(pt, k=1)
            all_dists.append(d[0])
            if d[0] < dist_threshold:
                overlap_count += 1

        all_dists = np.array(all_dists)
        rmse = float(np.sqrt(np.mean(all_dists ** 2)))
        overlap_rate = overlap_count / len(source) if len(source) > 0 else 0.0
        mean_dist = float(np.mean(all_dists))
        median_dist = float(np.median(all_dists))
        max_dist = float(np.max(all_dists))
        std_dist = float(np.std(all_dists))

        return {
            "rmse_m": rmse,
            "rmse_cm": rmse * 100,
            "overlap_rate": overlap_rate,
            "correspondence_threshold": dist_threshold,
            "valid_correspondences": overlap_count,
            "total_source_points": len(source),
            "total_target_points": len(target),
            "mean_distance_m": mean_dist,
            "median_distance_m": median_dist,
            "max_distance_m": max_dist,
            "std_distance_m": std_dist,
            "rmse_pass": rmse < self.config["rmse_threshold"],
        }

    def generate_report(self) -> dict:
        """生成精配准报告

        Returns:
            包含配准方法、RMSE、变换矩阵、评估指标等信息的字典
        """
        report = {
            "method": self._method,
            "timestamp": time.time(),
            "rmse_m": self._rmse,
            "rmse_cm": self._rmse * 100 if self._rmse is not None else None,
            "rmse_threshold_cm": self.config["rmse_threshold"] * 100,
            "rmse_pass": self._rmse is not None and self._rmse < self.config["rmse_threshold"],
            "config": {
                "max_iterations": self.config["max_iterations"],
                "max_correspondence_distance": self.config["max_correspondence_distance"],
                "transformation_epsilon": self.config["transformation_epsilon"],
                "use_point_to_plane": self.config["use_point_to_plane"],
                "use_open3d": self.config["use_open3d"] and _HAS_OPEN3D,
            },
            "transformation_matrix": None,
            "evaluation": self._evaluation,
        }

        if self._transform is not None:
            report["transformation_matrix"] = self._transform.tolist()

        return report

    @property
    def aligned_points(self) -> Optional[np.ndarray]:
        """获取配准后的源点云"""
        return self._aligned

    @property
    def transformation(self) -> Optional[np.ndarray]:
        """获取 4x4 变换矩阵"""
        return self._transform

    @property
    def rmse(self) -> Optional[float]:
        """获取 RMSE"""
        return self._rmse


# ══════════════════════════════════════════════════════════════════════
# 测试入口
# ══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=== 精配准模块测试 ===\n")

    np.random.seed(42)

    # 生成模拟点云（3D 体素点云，确保法向量方向多样）
    n_target = 600

    # 目标点云：随机立方体表面 + 内部点
    target_parts = []
    # 立方体6个面（范围 [-2, 2]）
    for _ in range(300):
        face = np.random.randint(0, 6)
        if face == 0:
            pt = np.array([np.random.uniform(-2, 2), np.random.uniform(-2, 2), -2])
        elif face == 1:
            pt = np.array([np.random.uniform(-2, 2), np.random.uniform(-2, 2), 2])
        elif face == 2:
            pt = np.array([np.random.uniform(-2, 2), -2, np.random.uniform(-2, 2)])
        elif face == 3:
            pt = np.array([np.random.uniform(-2, 2), 2, np.random.uniform(-2, 2)])
        elif face == 4:
            pt = np.array([-2, np.random.uniform(-2, 2), np.random.uniform(-2, 2)])
        else:
            pt = np.array([2, np.random.uniform(-2, 2), np.random.uniform(-2, 2)])
        target_parts.append(pt + np.random.randn(3) * 0.02)
    # 添加内部随机点
    for _ in range(300):
        pt = np.random.uniform(-2, 2, 3)
        target_parts.append(pt)
    target_points = np.array(target_parts)

    # 真值变换（小角度旋转 + 平移）
    theta = np.radians(5.0)  # 5度旋转
    axis = np.array([0.3, 0.7, 0.5])
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    true_R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    true_t = np.array([0.15, -0.08, 0.03])

    source_points = (true_R @ target_points.T).T + true_t + np.random.randn(n_target, 3) * 0.01

    # 初始变换（故意偏离真值约3度/3cm）
    theta_init = np.radians(3.0)
    init_R = np.eye(3) + np.sin(theta_init) * K + (1 - np.cos(theta_init)) * (K @ K)
    init_t = np.array([0.10, -0.05, 0.05])
    init_transform = np.eye(4)
    init_transform[:3, :3] = init_R
    init_transform[:3, 3] = init_t

    # --- 测试 ICP ---
    print("--- ICP Point-to-Plane 测试 ---")
    registrator = FineRegistrator({
        "max_iterations": 50,
        "max_correspondence_distance": 0.5,
        "use_open3d": _HAS_OPEN3D,
    })

    aligned, rmse, T_icp = registrator.register_icp(
        source_points, target_points, init_transform
    )

    eval_result = registrator.evaluate_registration(
        source_points, target_points, T_icp
    )
    print(f"后端: {'Open3D' if _HAS_OPEN3D else 'NumPy'}")
    print(f"RMSE: {eval_result['rmse_cm']:.3f} cm")
    print(f"重叠率: {eval_result['overlap_rate']:.2%}")
    print(f"RMSE 通过 (< 5cm): {eval_result['rmse_pass']}")
    print(f"有效对应点: {eval_result['valid_correspondences']}/{eval_result['total_source_points']}")

    report = registrator.generate_report()
    print(f"\n配准报告:")
    print(f"  方法: {report['method']}")
    print(f"  RMSE: {report['rmse_cm']:.3f} cm")
    print(f"  通过阈值: {report['rmse_pass']}")

    # --- 测试 NDT ---
    # NDT 需要高密度点云才能有效工作。使用小场景密集点云
    print("\n--- NDT 测试 ---")
    np.random.seed(123)
    n_ndt = 3000
    # 目标：小范围立方体表面 + 内部点 (场景 [-1, 1]^3)
    ndt_target = []
    for _ in range(n_ndt // 2):
        face = np.random.randint(0, 6)
        if face == 0:
            pt = np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1), -1])
        elif face == 1:
            pt = np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1), 1])
        elif face == 2:
            pt = np.array([np.random.uniform(-1, 1), -1, np.random.uniform(-1, 1)])
        elif face == 3:
            pt = np.array([np.random.uniform(-1, 1), 1, np.random.uniform(-1, 1)])
        elif face == 4:
            pt = np.array([-1, np.random.uniform(-1, 1), np.random.uniform(-1, 1)])
        else:
            pt = np.array([1, np.random.uniform(-1, 1), np.random.uniform(-1, 1)])
        ndt_target.append(pt + np.random.randn(3) * 0.01)
    for _ in range(n_ndt // 2):
        ndt_target.append(np.random.uniform(-1, 1, 3))
    ndt_target = np.array(ndt_target)

    ndt_source = (true_R @ ndt_target.T).T + true_t + np.random.randn(n_ndt, 3) * 0.01

    ndt_reg = FineRegistrator({
        "ndt_voxel_size": 0.25,
        "ndt_max_iterations": 50,
        "ndt_step_size": 0.5,
        "use_open3d": False,
    })

    aligned_ndt, rmse_ndt, T_ndt = ndt_reg.register_ndt(
        ndt_source, ndt_target, init_transform
    )

    print(f"NDT RMSE: {rmse_ndt * 100:.3f} cm")
    print(f"NDT 变换平移: {np.round(T_ndt[:3, 3], 4)} (真值: {true_t})")
    print(f"NDT 通过 (< 5cm): {rmse_ndt < 0.05}")

    print("\n精配准测试完成！")
