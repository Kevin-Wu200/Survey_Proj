#!/usr/bin/env python3
"""
地面站缓存管理节点
功能：管理本地 SSD 缓存目录，定期清理过期数据
- 缓存目录结构:
  ~/airunway_cache/
    ├── bags/          # ros2 bag 文件存储
    ├── images/        # 图像缓存
    │   ├── uav/       # UAV 拍摄图像
    │   └── ugv/       # UGV 相机图像
    ├── lidar/         # LiDAR 点云缓存
    ├── telemetry/     # 遥测数据
    │   ├── uav/
    │   └── ugv/
    └── logs/          # 运行日志
"""

import os
import time
import shutil
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CacheManager(Node):
    """缓存管理 - 目录维护和过期清理"""

    def __init__(self):
        super().__init__('cache_manager')

        # --- 参数 ---
        self.declare_parameter('cache_root', os.path.expanduser('~/airunway_cache'))
        self.declare_parameter('max_cache_size_gb', 50.0)           # 最大缓存 (GB)
        self.declare_parameter('retention_days', 7)                 # 数据保留天数
        self.declare_parameter('cleanup_interval_hours', 6)         # 清理间隔 (小时)

        self.cache_root = self.get_parameter('cache_root').value
        self.max_cache_size_gb = self.get_parameter('max_cache_size_gb').value
        self.retention_days = self.get_parameter('retention_days').value
        self.cleanup_interval_hours = self.get_parameter('cleanup_interval_hours').value

        # --- 创建缓存目录结构 ---
        self.cache_dirs = {
            'bags': os.path.join(self.cache_root, 'bags'),
            'images_uav': os.path.join(self.cache_root, 'images', 'uav'),
            'images_ugv': os.path.join(self.cache_root, 'images', 'ugv'),
            'lidar': os.path.join(self.cache_root, 'lidar'),
            'telemetry_uav': os.path.join(self.cache_root, 'telemetry', 'uav'),
            'telemetry_ugv': os.path.join(self.cache_root, 'telemetry', 'ugv'),
            'logs': os.path.join(self.cache_root, 'logs'),
        }
        self._create_cache_dirs()

        # --- 订阅者 ---
        self.cmd_sub = self.create_subscription(
            String, '/ground_station/cache_cmd', self.cmd_callback, 10)

        # --- 发布者 ---
        self.status_pub = self.create_publisher(
            String, '/ground_station/cache_status', 10)

        # --- 清理定时器 ---
        cleanup_sec = self.cleanup_interval_hours * 3600
        self.cleanup_timer = self.create_timer(cleanup_sec, self.cleanup_callback)

        self.get_logger().info(f'缓存管理节点已启动，缓存目录: {self.cache_root}')
        self.report_cache_info()

    def _create_cache_dirs(self):
        """创建所有缓存子目录"""
        for name, path in self.cache_dirs.items():
            os.makedirs(path, exist_ok=True)
            self.get_logger().debug(f'缓存目录已就绪: {path}')

    def cmd_callback(self, msg: String):
        """处理管理指令: cleanup / status / clear"""
        cmd = msg.data.strip().lower()
        if cmd == 'cleanup':
            self.cleanup_callback()
        elif cmd == 'status':
            self.report_cache_info()
        elif cmd == 'clear':
            self.clear_all_cache()

    def report_cache_info(self):
        """报告缓存使用情况"""
        total_size = 0
        dir_sizes = {}
        for name, path in self.cache_dirs.items():
            size = self._get_dir_size(path)
            dir_sizes[name] = size
            total_size += size

        total_gb = total_size / (1024 ** 3)
        info = {
            'cache_root': self.cache_root,
            'total_size_gb': round(total_gb, 2),
            'max_size_gb': self.max_cache_size_gb,
            'retention_days': self.retention_days,
            'dirs': {k: f'{v / 1024 / 1024:.1f} MB' for k, v in dir_sizes.items()},
        }

        self.get_logger().info(f'缓存总大小: {total_gb:.2f} GB / {self.max_cache_size_gb} GB')
        self.get_logger().info(f'缓存目录: {json.dumps(info["dirs"], ensure_ascii=False)}')

        msg = String()
        msg.data = json.dumps(info, ensure_ascii=False)
        self.status_pub.publish(msg)

    def cleanup_callback(self):
        """清理过期和超限数据"""
        self.get_logger().info('开始缓存清理...')

        now = time.time()
        retention_sec = self.retention_days * 86400
        deleted_count = 0
        deleted_size = 0

        # 1. 清理过期文件
        for name, path in self.cache_dirs.items():
            if not os.path.exists(path):
                continue
            for entry in os.listdir(path):
                entry_path = os.path.join(path, entry)
                try:
                    mtime = os.path.getmtime(entry_path)
                    if now - mtime > retention_sec:
                        size = self._get_dir_size(entry_path) if os.path.isdir(entry_path) else os.path.getsize(entry_path)
                        if os.path.isdir(entry_path):
                            shutil.rmtree(entry_path)
                        else:
                            os.remove(entry_path)
                        deleted_count += 1
                        deleted_size += size
                except OSError as e:
                    self.get_logger().warn(f'无法删除 {entry_path}: {e}')

        # 2. 检查总大小是否超限
        total_size = sum(self._get_dir_size(p) for p in self.cache_dirs.values())
        total_gb = total_size / (1024 ** 3)

        if total_gb > self.max_cache_size_gb:
            self.get_logger().warn(
                f'缓存超限: {total_gb:.2f}GB > {self.max_cache_size_gb}GB，清理最旧数据...')
            # 按修改时间排序，删除最旧的文件直到低于阈值
            all_files = []
            for path in self.cache_dirs.values():
                if os.path.exists(path):
                    for root, _, files in os.walk(path):
                        for f in files:
                            fp = os.path.join(root, f)
                            try:
                                all_files.append((os.path.getmtime(fp), os.path.getsize(fp), fp))
                            except OSError:
                                pass

            all_files.sort(key=lambda x: x[0])  # 按时间升序

            for _, size, fp in all_files:
                if total_size / (1024 ** 3) <= self.max_cache_size_gb * 0.8:
                    break
                try:
                    os.remove(fp)
                    total_size -= size
                    deleted_count += 1
                    deleted_size += size
                except OSError:
                    pass

        self.get_logger().info(
            f'清理完成: 删除 {deleted_count} 个文件/目录, '
            f'释放 {deleted_size / 1024 / 1024:.1f} MB')

    def clear_all_cache(self):
        """清空所有缓存 (危险操作)"""
        self.get_logger().warn('清空所有缓存!')
        for name, path in self.cache_dirs.items():
            if os.path.exists(path):
                for entry in os.listdir(path):
                    entry_path = os.path.join(path, entry)
                    try:
                        if os.path.isdir(entry_path):
                            shutil.rmtree(entry_path)
                        else:
                            os.remove(entry_path)
                    except OSError as e:
                        self.get_logger().error(f'无法删除 {entry_path}: {e}')
        self.get_logger().info('所有缓存已清空')

    @staticmethod
    def _get_dir_size(path):
        """计算目录总大小"""
        total = 0
        if os.path.isfile(path):
            return os.path.getsize(path)
        if os.path.isdir(path):
            for dirpath, _, filenames in os.walk(path):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    try:
                        total += os.path.getsize(fp)
                    except OSError:
                        pass
        return total


def main(args=None):
    rclpy.init(args=args)
    node = CacheManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('缓存管理节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
