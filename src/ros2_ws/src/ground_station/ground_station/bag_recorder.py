#!/usr/bin/env python3
"""
地面站 Bag 录制节点
功能：基于 ros2 bag record 录制数据包
- 录制指定 Topic 到本地 SSD
- 支持分段录制和自动命名
- 提供录制状态反馈
"""

import os
import subprocess
import time
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_interfaces.msg import SystemAlert


class BagRecorder(Node):
    """Bag 录制管理节点"""

    # 默认录制 Topic 列表
    DEFAULT_TOPICS = [
        '/uav/pose',
        '/uav/status',
        '/uav/heartbeat',
        '/uav/image_raw/compressed',
        '/ugv/pose',
        '/ugv/status',
        '/ugv/heartbeat',
        '/ugv/lidar/points',
        '/ugv/imu',
        '/ugv/camera/left/image_raw/compressed',
        '/ugv/camera/right/image_raw/compressed',
        '/ugv/odom',
        '/ground_station/alert',
    ]

    def __init__(self):
        super().__init__('bag_recorder')

        # --- 参数 ---
        self.declare_parameter('bag_dir', os.path.expanduser('~/airunway_bags'))
        self.declare_parameter('max_bag_size', 1024 * 1024 * 1024)   # 1GB 分片
        self.declare_parameter('max_bag_duration', 600)              # 10分钟分片
        self.declare_parameter('compression_mode', 'zstd')
        self.declare_parameter('topics', BagRecorder.DEFAULT_TOPICS)

        self.bag_dir = self.get_parameter('bag_dir').value
        self.max_bag_size = self.get_parameter('max_bag_size').value
        self.max_bag_duration = self.get_parameter('max_bag_duration').value
        self.compression_mode = self.get_parameter('compression_mode').value
        self.topics = self.get_parameter('topics').value

        # 确保目录存在
        os.makedirs(self.bag_dir, exist_ok=True)

        # --- 状态 ---
        self.recording = False
        self.recording_process = None
        self.current_bag_name = ''

        # --- 订阅者 ---
        self.cmd_sub = self.create_subscription(
            String, '/ground_station/bag_cmd', self.cmd_callback, 10)

        # --- 发布者 ---
        self.status_pub = self.create_publisher(
            String, '/ground_station/bag_status', 10)
        self.alert_pub = self.create_publisher(
            SystemAlert, '/ground_station/alert', 10)

        self.get_logger().info(f'Bag 录制节点已启动，存储目录: {self.bag_dir}')

    def cmd_callback(self, msg: String):
        """处理录制指令: start / stop / status"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到 Bag 指令: {cmd}')

        if cmd == 'start':
            self.start_recording()
        elif cmd == 'stop':
            self.stop_recording()
        elif cmd == 'status':
            self.report_status()

    def start_recording(self):
        """开始录制"""
        if self.recording:
            self.get_logger().warn('已在录制中')
            self._publish_status('already_recording')
            return

        # 生成文件名: airunway_YYYYMMDD_HHMMSS
        self.current_bag_name = f'airunway_{time.strftime("%Y%m%d_%H%M%S")}'
        bag_path = os.path.join(self.bag_dir, self.current_bag_name)

        # 构建 ros2 bag record 命令
        cmd = ['ros2', 'bag', 'record', '-o', bag_path,
               '--max-bag-size', str(self.max_bag_size),
               '--max-bag-duration', str(self.max_bag_duration),
               '--compression-mode', self.compression_mode,
               '--storage', 'sqlite3']
        cmd.extend(self.topics)

        self.get_logger().info(f'开始录制: {self.current_bag_name}')
        self.get_logger().info(f'录制 {len(self.topics)} 个 Topic')

        try:
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            self.recording = True
            self._publish_status('recording_started')

            # 发布告警 (info 级别)
            alert = SystemAlert()
            alert.header.stamp = self.get_clock().now().to_msg()
            alert.source = 'bag_recorder'
            alert.level = 0
            alert.message = f'开始录制: {self.current_bag_name}'
            self.alert_pub.publish(alert)

        except Exception as e:
            self.get_logger().error(f'启动录制失败: {e}')
            self._publish_status('error')

    def stop_recording(self):
        """停止录制"""
        if not self.recording or self.recording_process is None:
            self.get_logger().warn('未在录制中')
            self._publish_status('not_recording')
            return

        self.get_logger().info(f'停止录制: {self.current_bag_name}')

        try:
            # 发送 SIGINT 优雅停止 ros2 bag
            self.recording_process.send_signal(signal.SIGINT)
            self.recording_process.wait(timeout=15)
            self.recording = False
            self.recording_process = None

            bag_path = os.path.join(self.bag_dir, self.current_bag_name)
            if os.path.exists(bag_path):
                # 获取 bag 大小
                total_size = 0
                for dirpath, _, filenames in os.walk(bag_path):
                    for f in filenames:
                        total_size += os.path.getsize(os.path.join(dirpath, f))
                self.get_logger().info(
                    f'录制完成: {self.current_bag_name} ({total_size / 1024 / 1024:.1f} MB)')

            self._publish_status('recording_stopped')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('停止超时，强制终止')
            self.recording_process.kill()
            self.recording = False
            self.recording_process = None
            self._publish_status('recording_stopped_force')

    def report_status(self):
        """报告当前状态"""
        if self.recording:
            self._publish_status(f'recording:{self.current_bag_name}')
        else:
            self._publish_status('idle')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        """节点销毁时自动停止录制"""
        if self.recording:
            self.get_logger().info('节点关闭，自动停止录制...')
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bag 录制节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
