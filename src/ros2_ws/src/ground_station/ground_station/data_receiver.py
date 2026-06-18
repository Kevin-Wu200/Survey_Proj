#!/usr/bin/env python3
"""
地面站数据接收节点
功能：订阅 UAV/UGV 所有 Topic，汇总并转发数据
- 订阅所有传感器数据
- 汇总后通过内部 Topic 发布
- 提供数据统计
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, PointCloud2, Imu
from common_interfaces.msg import UAVStatus, UGVStatus, Heartbeat, SystemAlert


class DataReceiver(Node):
    """地面站数据接收 - 汇总所有 UAV/UGV 数据"""

    def __init__(self):
        super().__init__('data_receiver')

        # --- 统计 ---
        self.uav_pose_count = 0
        self.ugv_pose_count = 0
        self.uav_image_count = 0
        self.ugv_lidar_count = 0
        self.last_uav_hb = None
        self.last_ugv_hb = None
        self.start_time = time.time()

        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ========== UAV 订阅 ==========
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose', self.uav_pose_cb, reliable_qos)
        self.uav_status_sub = self.create_subscription(
            UAVStatus, '/uav/status', self.uav_status_cb, reliable_qos)
        self.uav_heartbeat_sub = self.create_subscription(
            Heartbeat, '/uav/heartbeat', self.uav_heartbeat_cb, reliable_qos)
        self.uav_image_sub = self.create_subscription(
            CompressedImage, '/uav/image_raw/compressed', self.uav_image_cb, reliable_qos)

        # ========== UGV 订阅 ==========
        self.ugv_pose_sub = self.create_subscription(
            PoseStamped, '/ugv/pose', self.ugv_pose_cb, reliable_qos)
        self.ugv_status_sub = self.create_subscription(
            UGVStatus, '/ugv/status', self.ugv_status_cb, reliable_qos)
        self.ugv_heartbeat_sub = self.create_subscription(
            Heartbeat, '/ugv/heartbeat', self.ugv_heartbeat_cb, reliable_qos)
        self.ugv_lidar_sub = self.create_subscription(
            PointCloud2, '/ugv/lidar/points', self.ugv_lidar_cb, best_effort_qos)
        self.ugv_imu_sub = self.create_subscription(
            Imu, '/ugv/imu', self.ugv_imu_cb, best_effort_qos)
        self.ugv_cam_left_sub = self.create_subscription(
            CompressedImage, '/ugv/camera/left/image_raw/compressed', self.ugv_cam_left_cb, reliable_qos)
        self.ugv_cam_right_sub = self.create_subscription(
            CompressedImage, '/ugv/camera/right/image_raw/compressed', self.ugv_cam_right_cb, reliable_qos)

        # ========== 告警发布 ==========
        self.alert_pub = self.create_publisher(
            SystemAlert, '/ground_station/alert', reliable_qos)

        # ========== 统计定时器 ==========
        self.stats_timer = self.create_timer(10.0, self.stats_callback)

        # ========== 存活检测 ==========
        self.health_timer = self.create_timer(5.0, self.health_check_callback)

        self.get_logger().info('地面站数据接收节点已启动')
        self.get_logger().info('已订阅所有 UAV/UGV Topic')

    # --- UAV 回调 ---
    def uav_pose_cb(self, msg: PoseStamped):
        self.uav_pose_count += 1

    def uav_status_cb(self, msg: UAVStatus):
        pass  # 状态由 WebSocket bridge 直接转发

    def uav_heartbeat_cb(self, msg: Heartbeat):
        self.last_uav_hb = time.time()

    def uav_image_cb(self, msg: CompressedImage):
        self.uav_image_count += 1

    # --- UGV 回调 ---
    def ugv_pose_cb(self, msg: PoseStamped):
        self.ugv_pose_count += 1

    def ugv_status_cb(self, msg: UGVStatus):
        pass

    def ugv_heartbeat_cb(self, msg: Heartbeat):
        self.last_ugv_hb = time.time()

    def ugv_lidar_cb(self, msg: PointCloud2):
        self.ugv_lidar_count += 1

    def ugv_imu_cb(self, msg: Imu):
        pass

    def ugv_cam_left_cb(self, msg: CompressedImage):
        pass

    def ugv_cam_right_cb(self, msg: CompressedImage):
        pass

    # --- 统计 ---
    def stats_callback(self):
        """每10秒输出统计信息"""
        uptime = time.time() - self.start_time
        self.get_logger().info(
            f'数据统计 [{uptime:.0f}s]: '
            f'UAV位姿={self.uav_pose_count} '
            f'UGV位姿={self.ugv_pose_count} '
            f'UAV图像={self.uav_image_count} '
            f'UGV点云={self.ugv_lidar_count}'
        )

    # --- 存活检测 ---
    def health_check_callback(self):
        """检测 UAV/UGV 心跳是否超时"""
        now = time.time()
        alert_msg = SystemAlert()
        alert_msg.header.stamp = self.get_clock().now().to_msg()
        alert_msg.source = 'ground_station'

        if self.last_uav_hb is not None:
            uav_timeout = now - self.last_uav_hb
            if uav_timeout > 10.0:
                alert_msg.level = 2  # error
                alert_msg.message = f'UAV 心跳超时 ({uav_timeout:.1f}s)'
                self.alert_pub.publish(alert_msg)
                self.get_logger().warn(alert_msg.message)

        if self.last_ugv_hb is not None:
            ugv_timeout = now - self.last_ugv_hb
            if ugv_timeout > 10.0:
                alert_msg.level = 2
                alert_msg.message = f'UGV 心跳超时 ({ugv_timeout:.1f}s)'
                self.alert_pub.publish(alert_msg)
                self.get_logger().warn(alert_msg.message)


def main(args=None):
    rclpy.init(args=args)
    node = DataReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('地面站数据接收节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
