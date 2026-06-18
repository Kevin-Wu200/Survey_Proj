#!/usr/bin/env python3
"""
UGV 控制器节点
功能：模拟四轮差速底盘无人车
- 接收键盘遥控指令 (teleop_twist_keyboard 风格)
- 发布 /ugv/pose (位姿)、/ugv/status (状态)、/ugv/heartbeat (心跳)
- 模拟差速运动学模型
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from common_interfaces.msg import UGVStatus, Heartbeat


class UGVController(Node):
    """UGV 控制器 - 四轮差速底盘运动学模拟"""

    def __init__(self):
        super().__init__('ugv_controller')

        # --- 参数 ---
        self.declare_parameter('home_lat', 30.0)
        self.declare_parameter('home_lon', 120.0)
        self.declare_parameter('home_alt', 0.0)
        self.declare_parameter('wheel_base', 0.5)         # 轴距 (m)
        self.declare_parameter('max_linear_speed', 3.0)   # 最大线速度 (m/s)
        self.declare_parameter('max_angular_speed', 2.0)  # 最大角速度 (rad/s)
        self.declare_parameter('update_rate', 20.0)       # 更新频率 (Hz)

        self.home_lat = self.get_parameter('home_lat').value
        self.home_lon = self.get_parameter('home_lon').value
        self.home_alt = self.get_parameter('home_alt').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.update_rate = self.get_parameter('update_rate').value

        # --- 状态变量 ---
        self.current_lat = self.home_lat
        self.current_lon = self.home_lon
        self.current_alt = self.home_alt
        self.x = 0.0              # 局部坐标系 X (m)
        self.y = 0.0              # 局部坐标系 Y (m)
        self.heading = 0.0        # 航向角 (弧度)
        self.linear_vel = 0.0     # 当前线速度
        self.angular_vel = 0.0    # 当前角速度
        self.battery_voltage = 24.0  # 6S 电池
        self.battery_pct = 100.0
        self.remote_control = True
        self.start_time = self.get_clock().now()

        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 发布者 ---
        self.pose_pub = self.create_publisher(
            PoseStamped, '/ugv/pose', reliable_qos)
        self.status_pub = self.create_publisher(
            UGVStatus, '/ugv/status', reliable_qos)
        self.heartbeat_pub = self.create_publisher(
            Heartbeat, '/ugv/heartbeat', reliable_qos)
        self.odom_pub = self.create_publisher(
            Odometry, '/ugv/odom', reliable_qos)

        # --- 订阅者 ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/ugv/cmd_vel', self.cmd_vel_callback, reliable_qos)

        # --- 定时器 ---
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        self.battery_timer = self.create_timer(1.0, self.battery_callback)

        self.get_logger().info('UGV 控制器节点已启动')
        self.get_logger().info(
            f'起点: ({self.home_lat:.6f}, {self.home_lon:.6f})')
        self.get_logger().info(
            f'最大速度: {self.max_linear_speed}m/s, '
            f'最大角速度: {self.max_angular_speed}rad/s')

    def cmd_vel_callback(self, msg: Twist):
        """接收键盘遥控速度指令"""
        # 限幅
        self.linear_vel = max(-self.max_linear_speed,
                              min(self.max_linear_speed, msg.linear.x))
        self.angular_vel = max(-self.max_angular_speed,
                               min(self.max_angular_speed, msg.angular.z))
        self.remote_control = True

    def update_callback(self):
        """运动学更新 - 差速模型"""
        dt = 1.0 / self.update_rate

        # 更新航向
        self.heading += self.angular_vel * dt
        # 归一化到 [-pi, pi]
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # 更新局部位置
        self.x += self.linear_vel * math.cos(self.heading) * dt
        self.y += self.linear_vel * math.sin(self.heading) * dt

        # 局部坐标 → 经纬度 (小范围平面近似)
        cos_lat = math.cos(math.radians(self.home_lat))
        self.current_lon = self.home_lon + self.x / (111320.0 * cos_lat)
        self.current_lat = self.home_lat + self.y / 111320.0

        now = self.get_clock().now()

        # --- 发布位姿 ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.current_alt
        # 四元数
        half_h = self.heading * 0.5
        pose_msg.pose.orientation.z = math.sin(half_h)
        pose_msg.pose.orientation.w = math.cos(half_h)
        self.pose_pub.publish(pose_msg)

        # --- 发布里程计 ---
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = 'ugv_base_link'
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel
        self.odom_pub.publish(odom_msg)

        # --- 发布状态 ---
        status_msg = UGVStatus()
        status_msg.header.stamp = now.to_msg()
        status_msg.header.frame_id = 'map'
        status_msg.latitude = self.current_lat
        status_msg.longitude = self.current_lon
        status_msg.altitude = self.current_alt
        status_msg.heading = self.heading
        status_msg.linear_velocity = self.linear_vel
        status_msg.angular_velocity = self.angular_vel
        status_msg.battery_voltage = self.battery_voltage
        status_msg.battery_percentage = self.battery_pct
        status_msg.remote_control = self.remote_control
        status_msg.status_text = '遥控中' if self.remote_control else '待机'
        self.status_pub.publish(status_msg)

    def battery_callback(self):
        """电池模拟"""
        self.battery_pct = max(0.0, self.battery_pct - 0.01)
        self.battery_voltage = 18.0 + (self.battery_pct / 100.0) * 7.4

    def heartbeat_callback(self):
        """心跳发布"""
        now = self.get_clock().now()
        uptime = (now - self.start_time).nanoseconds / 1e9

        msg = Heartbeat()
        msg.header.stamp = now.to_msg()
        msg.node_name = 'ugv_controller'
        msg.node_type = 'ugv'
        msg.uptime_seconds = uptime
        msg.emergency_stop = False
        self.heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UGVController()

    heartbeat_timer = node.create_timer(2.0, node.heartbeat_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UGV 控制器节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
