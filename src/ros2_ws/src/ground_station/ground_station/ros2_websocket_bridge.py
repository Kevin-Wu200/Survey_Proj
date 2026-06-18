#!/usr/bin/env python3
"""
ROS2 - WebSocket 数据桥接节点
订阅 ROS2 UAV/UGV Topic，将数据推送到 FastAPI WebSocket 后端

启动前需先启动 FastAPI 后端: uvicorn main:app --host 0.0.0.0 --port 8000
"""

import json
import threading
import time
import math

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import UAVStatus, UGVStatus, Heartbeat


class Ros2WebsocketBridge(Node):
    """ROS2 → HTTP REST 数据桥接节点"""

    def __init__(self):
        super().__init__('ros2_websocket_bridge')

        # --- 参数 ---
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('update_interval', 0.5)  # 推送间隔 (秒)

        self.backend_url = self.get_parameter('backend_url').value
        self.update_interval = self.get_parameter('update_interval').value

        # --- 最新数据缓存 ---
        self.lock = threading.Lock()
        self._uav_lat = 30.0
        self._uav_lon = 120.0
        self._uav_alt = 0.0
        self._uav_heading = 0.0
        self._uav_speed = 0.0
        self._uav_flight_mode = 0
        self._uav_armed = False
        self._uav_battery = 100.0
        self._uav_battery_v = 22.8
        self._uav_status_text = '待机'

        self._ugv_lat = 30.0
        self._ugv_lon = 120.0
        self._ugv_alt = 0.0
        self._ugv_heading = 0.0
        self._ugv_speed = 0.0
        self._ugv_battery = 100.0
        self._ugv_battery_v = 24.0
        self._ugv_status_text = '待机'
        self._ugv_remote_control = False

        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 订阅 UAV Topic ---
        self.uav_status_sub = self.create_subscription(
            UAVStatus, '/uav/status', self.uav_status_cb, reliable_qos)
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose', self.uav_pose_cb, reliable_qos)

        # --- 订阅 UGV Topic ---
        self.ugv_status_sub = self.create_subscription(
            UGVStatus, '/ugv/status', self.ugv_status_cb, reliable_qos)
        self.ugv_pose_sub = self.create_subscription(
            PoseStamped, '/ugv/pose', self.ugv_pose_cb, reliable_qos)

        # --- 推送定时器 ---
        self.push_timer = self.create_timer(
            self.update_interval, self.push_callback)

        # --- HTTP Session ---
        self.session = requests.Session()
        self.session.headers.update({'Content-Type': 'application/json'})

        self.get_logger().info(f'ROS2-WebSocket 桥接节点已启动 → {self.backend_url}')

    # --- UAV 回调 ---
    def uav_status_cb(self, msg: UAVStatus):
        with self.lock:
            self._uav_lat = msg.latitude
            self._uav_lon = msg.longitude
            self._uav_alt = msg.altitude
            self._uav_heading = math.degrees(msg.yaw)
            self._uav_speed = msg.ground_speed
            self._uav_flight_mode = msg.flight_mode
            self._uav_armed = msg.armed
            self._uav_battery = msg.battery_percentage
            self._uav_battery_v = msg.battery_voltage
            self._uav_status_text = msg.status_text

    def uav_pose_cb(self, msg: PoseStamped):
        # 从位姿中提取偏航角 (四元数 → 欧拉角简化)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self.lock:
            self._uav_heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # --- UGV 回调 ---
    def ugv_status_cb(self, msg: UGVStatus):
        with self.lock:
            self._ugv_lat = msg.latitude
            self._ugv_lon = msg.longitude
            self._ugv_alt = msg.altitude
            self._ugv_heading = math.degrees(msg.heading)
            self._ugv_speed = msg.linear_velocity
            self._ugv_battery = msg.battery_percentage
            self._ugv_battery_v = msg.battery_voltage
            self._ugv_status_text = msg.status_text
            self._ugv_remote_control = msg.remote_control

    def ugv_pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self.lock:
            self._ugv_heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # --- 推送 ---
    def push_callback(self):
        """定期将最新数据推送到后端 HTTP API"""
        with self.lock:
            uav_data = {
                'lat': self._uav_lat,
                'lon': self._uav_lon,
                'alt': self._uav_alt,
                'heading': self._uav_heading,
                'speed': self._uav_speed,
                'flight_mode': self._uav_flight_mode,
                'armed': self._uav_armed,
                'battery': self._uav_battery,
                'battery_v': self._uav_battery_v,
                'status_text': self._uav_status_text,
            }
            ugv_data = {
                'lat': self._ugv_lat,
                'lon': self._ugv_lon,
                'alt': self._ugv_alt,
                'heading': self._ugv_heading,
                'speed': self._ugv_speed,
                'battery': self._ugv_battery,
                'battery_v': self._ugv_battery_v,
                'status_text': self._ugv_status_text,
                'remote_control': self._ugv_remote_control,
            }

        # 推送到后端 (非阻塞，忽略失败)
        try:
            self.session.post(
                f'{self.backend_url}/api/uav/update',
                json=uav_data, timeout=0.5)
            self.session.post(
                f'{self.backend_url}/api/ugv/update',
                json=ugv_data, timeout=0.5)
        except Exception:
            pass  # 后端可能未启动或网络问题


def main(args=None):
    rclpy.init(args=args)
    node = Ros2WebsocketBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ROS2-WebSocket 桥接节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
