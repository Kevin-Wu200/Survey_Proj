"""
实时性优化模块 (3B.1)

优化各模块时延以达标：
  - Scan-to-Map 时延目标：50~100ms
  - 回环检测时延目标：100~200ms
  - 因子图优化移至后台线程异步执行
  - ORB 特征匹配降级方案
"""

import time
import threading
import numpy as np
from typing import Optional, Callable, Any
from collections import deque
from dataclasses import dataclass


@dataclass
class TimingStats:
    """时延统计"""
    name: str
    target_ms: float
    measurements: deque = None

    def __post_init__(self):
        if self.measurements is None:
            self.measurements = deque(maxlen=100)

    def record(self, elapsed_ms: float):
        self.measurements.append(elapsed_ms)

    @property
    def avg_ms(self) -> float:
        if not self.measurements:
            return 0.0
        return sum(self.measurements) / len(self.measurements)

    @property
    def p95_ms(self) -> float:
        if not self.measurements:
            return 0.0
        sorted_times = sorted(self.measurements)
        idx = int(len(sorted_times) * 0.95)
        return sorted_times[min(idx, len(sorted_times) - 1)]

    @property
    def is_meeting_target(self) -> bool:
        return self.avg_ms <= self.target_ms


class TimingProfiler:
    """时延分析器"""

    def __init__(self):
        self.stats = {
            'scan_to_map': TimingStats('Scan-to-Map', 100.0),
            'feature_extraction': TimingStats('特征提取', 50.0),
            'loop_detection': TimingStats('回环检测', 200.0),
            'pgo': TimingStats('位姿图优化', 500.0),
            'imu_integration': TimingStats('IMU预积分', 5.0),
        }

    def measure(self, name: str):
        """上下文管理器：测量代码块耗时"""

        class TimerContext:
            def __init__(self, stats_dict, stat_name):
                self.stats_dict = stats_dict
                self.stat_name = stat_name
                self.start = None

            def __enter__(self):
                self.start = time.perf_counter()
                return self

            def __exit__(self, *args):
                elapsed = (time.perf_counter() - self.start) * 1000
                if self.stat_name in self.stats_dict:
                    self.stats_dict[self.stat_name].record(elapsed)

        return TimerContext(self.stats, name)

    def report(self) -> dict:
        """生成时延报告"""
        return {
            name: {
                "avg_ms": round(stat.avg_ms, 2),
                "p95_ms": round(stat.p95_ms, 2),
                "target_ms": stat.target_ms,
                "meeting_target": stat.is_meeting_target
            }
            for name, stat in self.stats.items()
        }


class AsyncFactorGraphOptimizer:
    """异步因子图优化器

    将因子图优化移至后台线程，避免阻塞主线程。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._pending_edges = []
        self._optimized_result = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._optimization_count = 0

    def submit(self, factor_graph, loop_edges=None):
        """提交优化任务"""
        with self._lock:
            self._pending_edges = loop_edges or []

    def start(self, optimize_fn: Callable, interval: float = 1.0):
        """启动后台优化线程"""
        if self._running:
            return
        self._running = True

        def _worker():
            while self._running:
                with self._lock:
                    edges = list(self._pending_edges)
                    if edges:
                        self._pending_edges.clear()

                if edges:
                    try:
                        result = optimize_fn(edges)
                        with self._lock:
                            self._optimized_result = result
                            self._optimization_count += 1
                    except Exception:
                        pass

                time.sleep(interval)

        self._thread = threading.Thread(target=_worker, daemon=True)
        self._thread.start()

    def stop(self):
        """停止后台优化"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_result(self):
        """获取最新优化结果"""
        with self._lock:
            return self._optimized_result


class ScanToMapOptimizer:
    """Scan-to-Map 优化器

    通过下采样和增量更新优化 scan-to-map 配准时延。
    """

    def __init__(self, voxel_size: float = 0.1):
        self.voxel_size = voxel_size
        self._local_map: Optional[np.ndarray] = None
        self._voxel_grid: dict = {}

    def downsample(self, points: np.ndarray) -> np.ndarray:
        """体素下采样"""
        if len(points) == 0:
            return points

        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        unique_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in unique_dict:
                unique_dict[key] = points[i]
        return np.array(list(unique_dict.values()))

    def incremental_scan_to_map(self, scan: np.ndarray,
                                 local_map: np.ndarray,
                                 initial_guess: np.ndarray) -> np.ndarray:
        """增量式 scan-to-map 配准

        仅使用新扫描点与局部地图的子集进行配准。
        """
        # 下采样
        scan_down = self.downsample(scan)
        map_down = self.downsample(local_map)

        if len(scan_down) < 10 or len(map_down) < 10:
            return initial_guess

        # 仅使用最邻近的地图点子集
        center = initial_guess[:3, 3] if initial_guess.shape == (4, 4) else \
                 initial_guess[:3]
        dists = np.linalg.norm(map_down - center, axis=1)
        nearby_idx = np.argsort(dists)[:500]
        map_subset = map_down[nearby_idx]

        # 快速 ICP（固定迭代次数）
        transform = initial_guess.copy() if initial_guess.shape == (4, 4) \
                    else np.eye(4)
        if transform.shape != (4, 4):
            transform = np.eye(4)
            transform[:3, 3] = initial_guess[:3]

        for _ in range(10):  # 减少迭代次数
            transformed = (transform[:3, :3] @ scan_down.T).T + transform[:3, 3]
            diffs = transformed[:, None, :] - map_subset[None, :, :]
            dists = np.linalg.norm(diffs, axis=2)
            matches = np.argmin(dists, axis=1)
            matched = map_subset[matches]

            centroid_src = np.mean(transformed, axis=0)
            centroid_dst = np.mean(matched, axis=0)
            H = (transformed - centroid_src).T @ (matched - centroid_dst)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1] *= -1
                R = Vt.T @ U.T
            t = centroid_dst - R @ centroid_src

            transform[:3, :3] = R @ transform[:3, :3]
            transform[:3, 3] = R @ transform[:3, 3] + t

        return transform


class RealtimeOptimizer:
    """实时性优化器

    统一管理各模块的时延优化。
    """

    def __init__(self, enable_orb_fallback: bool = False):
        self.profiler = TimingProfiler()
        self.async_pgo = AsyncFactorGraphOptimizer()
        self.scan_to_map_opt = ScanToMapOptimizer()
        self.enable_orb_fallback = enable_orb_fallback

    def optimize_scan_to_map(self, scan: np.ndarray, local_map: np.ndarray,
                              initial_guess: np.ndarray) -> np.ndarray:
        """优化的 scan-to-map"""
        with self.profiler.measure('scan_to_map'):
            result = self.scan_to_map_opt.incremental_scan_to_map(
                scan, local_map, initial_guess)
        return result

    def start_async_optimization(self, optimize_fn: Callable, interval: float = 1.0):
        """启动异步优化"""
        self.async_pgo.start(optimize_fn, interval)

    def stop_async_optimization(self):
        """停止异步优化"""
        self.async_pgo.stop()

    def should_use_orb(self, superpoint_fps: float) -> bool:
        """判断是否应降级到 ORB"""
        return superpoint_fps < 10.0 or self.enable_orb_fallback

    def get_timing_report(self) -> dict:
        """获取时延报告"""
        return self.profiler.report()

    def generate_report(self) -> dict:
        """生成实时性优化报告"""
        timing = self.profiler.report()
        return {
            "timing": timing,
            "orb_fallback_enabled": self.enable_orb_fallback,
            "async_pgo_count": self.async_pgo._optimization_count,
            "all_targets_met": all(
                s.is_meeting_target for s in self.profiler.stats.values()
            )
        }
