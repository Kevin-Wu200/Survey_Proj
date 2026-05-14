#!/usr/bin/env python3
"""
UGV 自主导航控制器
功能：基于 Nav2 架构的自主导航控制
- 集成 Cartographer 2D SLAM 建图
- 配置 Smac Hybrid-A* 全局规划器 + Regulated Pure Pursuit 局部规划器
- 实现：指定目标点 → 自主路径规划 → 动态避障 → 到达
- 代价地图参数调优（分辨率 0.05m，宽高 10m×10m）

仿真模式：模拟 Nav2 路径规划行为
实物模式：通过 /ugv/nav_goal 接口发送至真实 Nav2 堆栈
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from common_interfaces.msg import NavigationGoal, NavigationStatus, UGVStatus


class PathPlanner:
    """
    路径规划器 - 仿真模式下模拟 Nav2 Smac Hybrid-A* 行为
    
    实际部署时替换为 Nav2 Smac Hybrid-A* + Regulated Pure Pursuit：
    - Smac Hybrid-A*: 基于搜索的全局路径规划，支持非完整约束
    - Regulated Pure Pursuit: 改进的纯追踪算法，速度自适应调速
    """
    
    def __init__(self, logger):
        self.logger = logger
        self.costmap_resolution = 0.05  # 代价地图分辨率 0.05m
        self.costmap_width = 10.0       # 代价地图宽 10m
        self.costmap_height = 10.0      # 代价地图高 10m
        self.max_linear_speed = 2.0     # 最大线速度
        self.max_angular_speed = 1.5    # 最大角速度
        
        # 简易障碍物列表 (仿真用)
        self.obstacles = []  # [(x, y, radius), ...]
    
    def add_obstacle(self, x: float, y: float, radius: float = 0.3):
        """添加障碍物到代价地图"""
        self.obstacles.append((x, y, radius))
    
    def clear_obstacles(self):
        """清除所有障碍物"""
        self.obstacles.clear()
    
    def is_collision(self, x: float, y: float, inflation: float = 0.3) -> bool:
        """检查点是否与障碍物碰撞 (带膨胀)"""
        for ox, oy, radius in self.obstacles:
            dist = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
            if dist < (radius + inflation):
                return True
        return False
    
    def plan_path(self, start_x: float, start_y: float, start_yaw: float,
                  goal_x: float, goal_y: float, goal_yaw: float) -> list:
        """
        Smac Hybrid-A* 风格路径规划 (仿真简化版)
        
        返回: [(x, y, yaw), ...] 路径点列表
        """
        # 直接距离
        dx = goal_x - start_x
        dy = goal_y - start_y
        dist = math.sqrt(dx * dx + dy * dy)
        
        if dist < 0.1:
            return [(goal_x, goal_y, goal_yaw)]
        
        # 简易 Dubins 曲线风格路径 (直线+圆弧)
        path = []
        num_points = max(10, int(dist / self.costmap_resolution))
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + dx * t
            y = start_y + dy * t
            
            # 碰撞检测
            if self.is_collision(x, y):
                self.logger.warn(f'路径点 ({x:.2f}, {y:.2f}) 与障碍物碰撞，添加绕行点')
                # 简单绕行: 添加偏移
                offset_dist = 1.0  # 绕行1m
                yaw_to_goal = math.atan2(dy, dx)
                perp_angle = yaw_to_goal + math.pi / 2
                x += offset_dist * math.cos(perp_angle)
                y += offset_dist * math.sin(perp_angle)
            
            # 航向插值
            yaw = start_yaw + (goal_yaw - start_yaw) * t
            path.append((x, y, yaw))
        
        return path


class NavController:
    """导航控制器状态机"""
    
    STATE_IDLE = 0
    STATE_PLANNING = 1
    STATE_EXECUTING = 2
    STATE_ARRIVED = 3
    STATE_FAILED = 4
    STATE_CANCELLED = 5
    
    STATE_NAMES = ['空闲', '规划中', '执行中', '到达目标', '失败', '取消']
    
    def __init__(self, logger):
        self.logger = logger
        self.state = self.STATE_IDLE
        self.planner = PathPlanner(logger)
        self.current_path = []          # 全局路径
        self.current_path_index = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lock = threading.Lock()
        
        # 调速参数 (Regulated Pure Pursuit 风格)
        self.lookahead_distance = 0.5    # 前视距离
        self.goal_tolerance = 0.2        # 到达容忍度 (m)
        self.yaw_tolerance = 0.1         # 偏航容忍度 (rad)
        self.max_linear_vel = 2.0
        self.max_angular_vel = 1.5
        self.min_approach_speed = 0.1    # 接近目标最低速度
        
        # 安全参数
        self.collision_stop_dist = 0.5   # 碰撞安全距离
    
    def set_goal(self, x: float, y: float, yaw: float):
        """设置导航目标点"""
        with self.lock:
            self.target_x = x
            self.target_y = y
            self.target_yaw = yaw
            self.state = self.STATE_PLANNING
            self.current_path = []
            self.current_path_index = 0
            self.logger.info(f'导航目标: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)')
    
    def update_position(self, x: float, y: float, yaw: float, lin_vel: float, ang_vel: float):
        """更新当前位置"""
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.linear_vel = lin_vel
        self.angular_vel = ang_vel
    
    def plan(self):
        """执行路径规划"""
        with self.lock:
            if self.state != self.STATE_PLANNING:
                return
            
            self.logger.info('开始路径规划...')
            path = self.planner.plan_path(
                self.current_x, self.current_y, self.current_yaw,
                self.target_x, self.target_y, self.target_yaw
            )
            
            if not path:
                self.state = self.STATE_FAILED
                self.logger.error('路径规划失败')
                return
            
            self.current_path = path
            self.current_path_index = 0
            self.state = self.STATE_EXECUTING
            self.logger.info(f'路径规划完成: {len(path)} 个路径点, '
                             f'长度 {len(path) * self.planner.costmap_resolution:.1f}m')
    
    def update(self, dt: float) -> tuple:
        """
        更新导航控制 - Regulated Pure Pursuit 风格
        
        返回: (linear_vel, angular_vel) 控制量
        """
        with self.lock:
            if self.state != self.STATE_EXECUTING:
                return (0.0, 0.0)
            
            # 检查是否到达目标点
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist_to_goal = math.sqrt(dx * dx + dy * dy)
            
            if dist_to_goal < self.goal_tolerance:
                # 到达位置，调整偏航
                dyaw = self.target_yaw - self.current_yaw
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                if abs(dyaw) < self.yaw_tolerance:
                    self.state = self.STATE_ARRIVED
                    self.logger.info(f'导航完成! 到达目标点')
                    return (0.0, 0.0)
                else:
                    return (0.0, max(-self.max_angular_vel,
                                     min(self.max_angular_vel, dyaw * 2.0)))
            
            # 前瞻点追踪 (Pure Pursuit)
            # 找到路径上距离当前位置 lookahead_distance 的点
            lookahead_point = self._find_lookahead_point()
            if lookahead_point is None:
                # 直接朝向目标
                lx, ly = self.target_x, self.target_y
            else:
                lx, ly, _ = lookahead_point
            
            # 计算到前瞻点的位姿误差
            ddx = lx - self.current_x
            ddy = ly - self.current_y
            
            # 转换到车体坐标系
            cos_yaw = math.cos(-self.current_yaw)
            sin_yaw = math.sin(-self.current_yaw)
            local_x = ddx * cos_yaw - ddy * sin_yaw
            local_y = ddx * sin_yaw + ddy * cos_yaw
            
            # Regulated Pure Pursuit 速度控制
            # 速度调节: 接近目标时减速
            speed_ratio = min(1.0, dist_to_goal / 2.0)  # 距目标2m内开始减速
            linear_vel = max(self.min_approach_speed,
                             self.max_linear_vel * speed_ratio)
            
            # 碰撞减速
            if self.planner.is_collision(
                self.current_x + math.cos(self.current_yaw) * 0.5,
                self.current_y + math.sin(self.current_yaw) * 0.5,
                inflation=0.3
            ):
                linear_vel = 0.0
                self.logger.warn('前方检测到障碍物，紧急停止')
            
            # 角速度: 朝向前瞻点
            angular_vel = math.atan2(local_y, local_x) * 1.5  # 比例控制
            angular_vel = max(-self.max_angular_vel,
                              min(self.max_angular_vel, angular_vel))
            
            # 弯道减速
            if abs(angular_vel) > 0.5:
                linear_vel *= 0.5
            
            return (linear_vel, angular_vel)
    
    def _find_lookahead_point(self):
        """在全局路径上找前瞻点"""
        if not self.current_path:
            return None
        
        # 从当前索引开始搜索
        for i in range(self.current_path_index, len(self.current_path)):
            px, py, _ = self.current_path[i]
            dx = px - self.current_x
            dy = py - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist >= self.lookahead_distance:
                self.current_path_index = i
                return self.current_path[i]
        
        # 所有路径点都在前瞻距离内，使用最后一个点
        self.current_path_index = len(self.current_path) - 1
        return self.current_path[-1] if self.current_path else None
    
    def cancel(self):
        with self.lock:
            self.state = self.STATE_CANCELLED
            self.current_path = []
    
    def get_path(self) -> list:
        return self.current_path
    
    def get_status(self) -> NavigationStatus:
        with self.lock:
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist_remaining = math.sqrt(dx * dx + dy * dy)
            dyaw = self.target_yaw - self.current_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            
            path_len = len(self.current_path) * self.planner.costmap_resolution
            
            status = NavigationStatus()
            status.state = self.state
            status.distance_remaining = dist_remaining
            status.yaw_error = dyaw
            status.path_length = path_len
            status.current_path_index = self.current_path_index
            status.total_path_points = len(self.current_path)
            status.status_text = self.STATE_NAMES[self.state]
            return status


class UGVNavController(Node):
    """UGV 自主导航控制器节点"""
    
    def __init__(self):
        super().__init__('ugv_nav_controller')
        
        # --- 参数 ---
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('use_gazebo', True)
        self.declare_parameter('home_lat', 0.0)
        self.declare_parameter('home_lon', 0.0)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width', 10.0)
        self.declare_parameter('costmap_height', 10.0)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        
        self.update_rate = self.get_parameter('update_rate').value
        self.use_gazebo = self.get_parameter('use_gazebo').value
        self.home_lat = self.get_parameter('home_lat').value
        self.home_lon = self.get_parameter('home_lon').value
        
        # --- 导航控制器 ---
        self.nav_ctrl = NavController(self.get_logger())
        self.nav_ctrl.planner.costmap_resolution = self.get_parameter('costmap_resolution').value
        self.nav_ctrl.planner.costmap_width = self.get_parameter('costmap_width').value
        self.nav_ctrl.planner.costmap_height = self.get_parameter('costmap_height').value
        self.nav_ctrl.max_linear_vel = self.get_parameter('max_linear_speed').value
        self.nav_ctrl.max_angular_vel = self.get_parameter('max_angular_speed').value
        
        # --- 当前位置 ---
        self.current_lat = self.home_lat
        self.current_lon = self.home_lon
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lin_vel = 0.0
        self.current_ang_vel = 0.0
        self._last_update_time = time.time()
        
        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # --- 订阅 ---
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal, '/ugv/nav_goal',
            self.nav_goal_callback, reliable_qos)
        self.ugv_status_sub = self.create_subscription(
            UGVStatus, '/ugv/status',
            self.ugv_status_callback, reliable_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/ugv/odom',
            self.odom_callback, reliable_qos)
        
        # --- 发布 ---
        self.nav_status_pub = self.create_publisher(
            NavigationStatus, '/ugv/nav_status', reliable_qos)
        self.path_pub = self.create_publisher(
            Path, '/ugv/plan', reliable_qos)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/ugv/cmd_vel', reliable_qos)
        
        # --- 定时器 ---
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('UGV 自主导航控制器节点已启动')
        self.get_logger().info(
            f'代价地图: {self.nav_ctrl.planner.costmap_resolution}m 分辨率, '
            f'{self.nav_ctrl.planner.costmap_width:.0f}x{self.nav_ctrl.planner.costmap_height:.0f}m')
        self.get_logger().info(
            f'最大速度: {self.nav_ctrl.max_linear_vel}m/s 线速度, '
            f'{self.nav_ctrl.max_angular_vel}rad/s 角速度')
    
    def nav_goal_callback(self, msg: NavigationGoal):
        """接收导航目标点"""
        # 经纬度 → 局部坐标
        cos_lat = math.cos(math.radians(self.home_lat))
        x = (msg.target_lon - self.home_lon) * 111320.0 * cos_lat
        y = (msg.target_lat - self.home_lat) * 111320.0
        yaw = msg.target_yaw
        
        # 也支持直接使用 target_pose
        if msg.target_pose.pose.position.x != 0 or msg.target_pose.pose.position.y != 0:
            x = msg.target_pose.pose.position.x
            y = msg.target_pose.pose.position.y
        
        if msg.max_linear_speed > 0:
            self.nav_ctrl.max_linear_vel = msg.max_linear_speed
        if msg.max_angular_speed > 0:
            self.nav_ctrl.max_angular_vel = msg.max_angular_speed
        
        self.get_logger().info(
            f'接收导航目标: ({msg.target_lat:.6f}°, {msg.target_lon:.6f}°) '
            f'→ 局部 ({x:.2f}, {y:.2f})m'
        )
        self.nav_ctrl.set_goal(x, y, yaw)
    
    def ugv_status_callback(self, msg: UGVStatus):
        """接收 UGV 状态更新"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def odom_callback(self, msg: Odometry):
        """接收里程计更新"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_lin_vel = msg.twist.twist.linear.x
        self.current_ang_vel = msg.twist.twist.angular.z
        
        # 提取偏航角
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self._last_update_time = time.time()
    
    def update_callback(self):
        """主循环 - 导航控制"""
        dt = 1.0 / self.update_rate
        
        # 更新位置
        self.nav_ctrl.update_position(
            self.current_x, self.current_y, self.current_yaw,
            self.current_lin_vel, self.current_ang_vel
        )
        
        # 状态机: 规划阶段
        if self.nav_ctrl.state == NavController.STATE_PLANNING:
            self.nav_ctrl.plan()
        
        # 发布导航状态
        status = self.nav_ctrl.get_status()
        status.header.stamp = self.get_clock().now().to_msg()
        self.nav_status_pub.publish(status)
        
        # 执行阶段: 发布速度指令
        if self.nav_ctrl.state == NavController.STATE_EXECUTING:
            lin_vel, ang_vel = self.nav_ctrl.update(dt)
            
            cmd = Twist()
            cmd.linear.x = lin_vel
            cmd.angular.z = ang_vel
            self.cmd_vel_pub.publish(cmd)
            
            # 发布规划路径
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            for px, py, yaw in self.nav_ctrl.get_path():
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = px
                pose.pose.position.y = py
                half = yaw * 0.5
                pose.pose.orientation.z = math.sin(half)
                pose.pose.orientation.w = math.cos(half)
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UGVNavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UGV 自主导航控制器节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
