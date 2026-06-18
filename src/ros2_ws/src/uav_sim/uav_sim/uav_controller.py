#!/usr/bin/env python3
"""
UAV 控制器节点
功能：模拟 DJI M300 RTK 四旋翼飞行控制
- 起飞、悬停、降落三个基础动作
- 发布 /uav/pose (位姿)、/uav/status (状态)、/uav/heartbeat (心跳)
- 订阅 /uav/cmd (控制指令)
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from common_interfaces.msg import UAVStatus, Heartbeat


class UAVController(Node):
    """UAV 飞行控制器 - 模拟四旋翼基本飞行"""

    def __init__(self):
        super().__init__('uav_controller')

        # --- 参数 ---
        self.declare_parameter('home_lat', 30.0)          # 起飞点纬度
        self.declare_parameter('home_lon', 120.0)         # 起飞点经度
        self.declare_parameter('home_alt', 0.0)           # 起飞点海拔
        self.declare_parameter('takeoff_height', 10.0)    # 起飞高度 (m)
        self.declare_parameter('hover_duration', 30.0)    # 悬停持续时间 (秒)
        self.declare_parameter('update_rate', 20.0)       # 更新频率 (Hz)

        # 读取参数
        self.home_lat = self.get_parameter('home_lat').value
        self.home_lon = self.get_parameter('home_lon').value
        self.home_alt = self.get_parameter('home_alt').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.hover_duration = self.get_parameter('hover_duration').value
        self.update_rate = self.get_parameter('update_rate').value

        # --- 状态变量 ---
        self.flight_mode = 0          # 0=待机 1=起飞 2=悬停 3=航线 4=降落 5=返航
        self.armed = False
        self.current_lat = self.home_lat
        self.current_lon = self.home_lon
        self.current_alt = self.home_alt
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ground_speed = 0.0
        self.battery_voltage = 22.8   # 6S 电池标称电压
        self.battery_pct = 100.0
        self.start_time = self.get_clock().now()

        # --- QoS 配置 ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 发布者 ---
        self.pose_pub = self.create_publisher(
            PoseStamped, '/uav/pose', reliable_qos)
        self.status_pub = self.create_publisher(
            UAVStatus, '/uav/status', reliable_qos)
        self.heartbeat_pub = self.create_publisher(
            Heartbeat, '/uav/heartbeat', reliable_qos)

        # --- 订阅者 ---
        self.cmd_sub = self.create_subscription(
            String, '/uav/cmd', self.cmd_callback, 10)

        # --- 定时器 ---
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        self.battery_timer = self.create_timer(
            1.0, self.battery_callback)  # 每秒更新电池

        self.get_logger().info('UAV 控制器节点已启动')
        self.get_logger().info(
            f'起飞点: ({self.home_lat:.6f}, {self.home_lon:.6f})')

    def cmd_callback(self, msg: String):
        """处理控制指令"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到指令: {cmd}')

        if cmd == 'takeoff':
            self._execute_takeoff()
        elif cmd == 'land':
            self._execute_land()
        elif cmd == 'hover':
            self._set_hover()
        elif cmd == 'arm':
            self.armed = True
            self.flight_mode = 0
            self.get_logger().info('无人机已解锁')
        elif cmd == 'disarm':
            self.armed = False
            self.get_logger().info('无人机已锁定')

    def _execute_takeoff(self):
        """执行起飞动作 - 在独立线程中模拟飞行过程"""
        if not self.armed:
            self.get_logger().warn('无人机未解锁，请先发送 arm 指令')
            return
        if self.flight_mode != 0:
            self.get_logger().warn('无人机不在待机状态')
            return

        self.flight_mode = 1  # 起飞中
        self.get_logger().info('开始起飞...')

        def takeoff_thread():
            # 加速上升
            target_alt = self.home_alt + self.takeoff_height
            steps = int(3.0 * self.update_rate)  # 3秒起飞
            for i in range(steps):
                progress = (i + 1) / steps
                self.current_alt = self.home_alt + self.takeoff_height * progress
                self.ground_speed = 2.0 * (1.0 - abs(progress - 0.5) * 2.0)
                time.sleep(1.0 / self.update_rate)
            self.current_alt = target_alt
            self.ground_speed = 0.0
            # 进入悬停
            self.flight_mode = 2
            self.get_logger().info(
                f'到达悬停高度 {self.takeoff_height:.1f}m，进入悬停')

            # 悬停指定时间后自动降落
            time.sleep(self.hover_duration)
            if self.flight_mode == 2:
                self._execute_land()

        thread = threading.Thread(target=takeoff_thread, daemon=True)
        thread.start()

    def _execute_land(self):
        """执行降落动作"""
        self.flight_mode = 4
        self.get_logger().info('开始降落...')

        def land_thread():
            start_alt = self.current_alt
            steps = int(3.0 * self.update_rate)  # 3秒降落
            for i in range(steps):
                progress = (i + 1) / steps
                self.current_alt = start_alt * (1.0 - progress) + self.home_alt * progress
                self.ground_speed = 1.5
                time.sleep(1.0 / self.update_rate)
            self.current_alt = self.home_alt
            self.ground_speed = 0.0
            self.flight_mode = 0  # 回到待机
            self.get_logger().info('降落完成')

        thread = threading.Thread(target=land_thread, daemon=True)
        thread.start()

    def _set_hover(self):
        """进入悬停状态"""
        if self.flight_mode in [1, 2]:
            self.flight_mode = 2
            self.ground_speed = 0.0
            self.get_logger().info('进入悬停')

    def update_callback(self):
        """定时更新 - 发布位姿和状态"""
        now = self.get_clock().now()

        # --- 发布位姿 ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'map'
        # 使用简化的经纬度→局部坐标转换 (小范围近似)
        # 纬度1度 ≈ 111320m, 经度1度 ≈ 111320m * cos(lat)
        cos_lat = math.cos(math.radians(self.home_lat))
        pose_msg.pose.position.x = (self.current_lon - self.home_lon) * 111320.0 * cos_lat
        pose_msg.pose.position.y = (self.current_lat - self.home_lat) * 111320.0
        pose_msg.pose.position.z = self.current_alt
        # 四元数 (简化：仅偏航)
        half_yaw = self.yaw * 0.5
        pose_msg.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_yaw)
        self.pose_pub.publish(pose_msg)

        # --- 发布状态 ---
        status_msg = UAVStatus()
        status_msg.header.stamp = now.to_msg()
        status_msg.header.frame_id = 'map'
        status_msg.latitude = self.current_lat
        status_msg.longitude = self.current_lon
        status_msg.altitude = self.current_alt
        status_msg.roll = self.roll
        status_msg.pitch = self.pitch
        status_msg.yaw = self.yaw
        status_msg.ground_speed = self.ground_speed
        status_msg.battery_voltage = self.battery_voltage
        status_msg.battery_percentage = self.battery_pct
        status_msg.flight_mode = self.flight_mode
        status_msg.armed = self.armed
        status_msg.camera_trigger = (self.flight_mode == 2)  # 悬停时触发拍照
        mode_names = ['待机', '起飞', '悬停', '航线', '降落', '返航']
        status_msg.status_text = mode_names[self.flight_mode]
        self.status_pub.publish(status_msg)

    def battery_callback(self):
        """电池模拟 - 每秒消耗"""
        self.battery_pct = max(0.0, self.battery_pct - 0.02)  # 每秒消耗0.02%
        self.battery_voltage = 18.0 + (self.battery_pct / 100.0) * 7.4  # 18V~25.4V

    def heartbeat_callback(self):
        """心跳发布"""
        now = self.get_clock().now()
        uptime = (now - self.start_time).nanoseconds / 1e9

        msg = Heartbeat()
        msg.header.stamp = now.to_msg()
        msg.node_name = 'uav_controller'
        msg.node_type = 'uav'
        msg.uptime_seconds = uptime
        msg.emergency_stop = False
        self.heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UAVController()

    # 额外启动心跳定时器
    heartbeat_timer = node.create_timer(2.0, node.heartbeat_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UAV 控制器节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
