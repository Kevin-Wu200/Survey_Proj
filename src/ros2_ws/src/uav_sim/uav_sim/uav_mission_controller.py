#!/usr/bin/env python3
"""
UAV 航点任务控制器节点
功能：基于 DJI OSDK Waypoint V2 接口封装
- 经纬度航点上传 → 航线上传 → 任务启动 → 进度反馈
- 支持等距拍照触发（相机快门与航点绑定）
- 支持多边形航线、蛇形航线自动生成
- 在仿真中验证航线自动执行
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from common_interfaces.msg import (
    WaypointMission,
    WaypointMissionStatus,
    UAVStatus,
)


class WaypointMissionManager:
    """
    航点任务管理器 — 模拟 DJI OSDK Waypoint V2 接口
    
    DJI Waypoint V2 核心概念:
    - Waypoint: 单个航点 (经纬度 + 高度 + 速度 + 航向 + 动作)
    - WaypointMission: 完整航线 (航点列表 + 全局参数)
    - Mission State: 空闲/上传中/就绪/执行中/暂停/完成/失败
    """
    
    # 任务状态常量
    STATE_IDLE = 0        # 空闲
    STATE_UPLOADING = 1   # 上传中
    STATE_READY = 2       # 就绪
    STATE_RUNNING = 3     # 执行中
    STATE_PAUSED = 4      # 暂停
    STATE_COMPLETED = 5   # 完成
    STATE_FAILED = 6      # 失败
    STATE_CANCELLED = 7   # 取消
    
    STATE_NAMES = ['空闲', '上传中', '就绪', '执行中', '暂停', '完成', '失败', '取消']
    
    def __init__(self, logger):
        self.logger = logger
        self.state = self.STATE_IDLE
        self.mission_id = ''
        self.waypoints = []           # 航点列表
        self.current_index = 0
        self.photos_taken = 0
        self.camera_trigger_mode = 0  # 0=等距 1=等时
        self.camera_trigger_interval = 50.0  # 默认50m拍照
        self.route_type = 0
        self.lock = threading.Lock()
        
        # 当前位置 (由外部更新)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        # 拍照距离累计
        self._photo_distance_accum = 0.0
        self._last_photo_lat = 0.0
        self._last_photo_lon = 0.0
        
    def _parse_waypoints(self, waypoints_json: str) -> list:
        """解析航点 JSON"""
        try:
            data = json.loads(waypoints_json)
            if not isinstance(data, list):
                raise ValueError('航点数据必须是数组')
            result = []
            for wp in data:
                result.append({
                    'lat': float(wp.get('lat', 0)),
                    'lon': float(wp.get('lon', 0)),
                    'alt': float(wp.get('alt', 50)),
                    'speed': float(wp.get('speed', 8)),
                    'heading': float(wp.get('heading', 0)),
                    'action': wp.get('action', 'photo'),
                    'dwell_time': float(wp.get('dwell_time', 0)),  # 悬停时间
                })
            return result
        except Exception as e:
            self.logger.error(f'航点解析失败: {e}')
            return []
    
    def generate_polygon_waypoints(self, params_json: str) -> list:
        """
        生成多边形航线航点
        
        参数: {
            "center_lat": 0.0, "center_lon": 0.0,
            "radius": 200.0,      # 多边形半径 (米)
            "sides": 6,           # 边数
            "altitude": 50.0,     # 飞行高度 (米)
            "speed": 8.0,         # 飞行速度 (m/s)
            "laps": 1             # 圈数
        }
        """
        try:
            params = json.loads(params_json)
        except:
            params = {}
        
        center_lat = params.get('center_lat', 0.0)
        center_lon = params.get('center_lon', 0.0)
        radius = params.get('radius', 200.0)
        sides = max(3, params.get('sides', 6))
        altitude = params.get('altitude', 50.0)
        speed = params.get('speed', 8.0)
        laps = max(1, params.get('laps', 1))
        
        waypoints = []
        # 每个顶点作为一个航点
        for lap in range(laps):
            for i in range(sides):
                angle = (2 * math.pi * i / sides) - math.pi / 2  # 从正北开始
                lat = center_lat + (radius * math.cos(angle) / 111320.0)
                lon = center_lon + (radius * math.sin(angle) / (111320.0 * math.cos(math.radians(center_lat))))
                waypoints.append({
                    'lat': lat,
                    'lon': lon,
                    'alt': altitude,
                    'speed': speed,
                    'heading': math.degrees(angle),
                    'action': 'photo',
                    'dwell_time': 1.0,  # 顶点悬停1秒拍照
                })
            # 闭合航线: 每圈完成后连接回起点
            if lap < laps - 1:
                wp0 = waypoints[0]
                waypoints.append({
                    'lat': wp0['lat'],
                    'lon': wp0['lon'],
                    'alt': altitude,
                    'speed': speed,
                    'heading': 0,
                    'action': 'photo',
                    'dwell_time': 1.0,
                })
        
        return waypoints
    
    def generate_snake_waypoints(self, params_json: str) -> list:
        """
        生成蛇形/弓字形航线航点
        
        参数: {
            "start_lat": 0.0, "start_lon": 0.0,
            "end_lat": 30.002, "end_lon": 120.002,
            "line_spacing": 50.0,   # 扫描线间距 (米)
            "altitude": 50.0,       # 飞行高度 (米)
            "speed": 8.0,           # 飞行速度 (m/s)
            "heading": 0            # 主方向角度 (度)
        }
        """
        try:
            params = json.loads(params_json)
        except:
            params = {}
        
        start_lat = params.get('start_lat', 0.0)
        start_lon = params.get('start_lon', 0.0)
        end_lat = params.get('end_lat', 30.002)
        end_lon = params.get('end_lon', 120.002)
        line_spacing = params.get('line_spacing', 50.0)
        altitude = params.get('altitude', 50.0)
        speed = params.get('speed', 8.0)
        heading = math.radians(params.get('heading', 0))
        
        # 计算扫描区域
        cos_lat = math.cos(math.radians((start_lat + end_lat) / 2))
        dx = (end_lon - start_lon) * 111320.0 * cos_lat  # 东向距离
        dy = (end_lat - start_lat) * 111320.0             # 北向距离
        
        if abs(dx) < 1.0 and abs(dy) < 1.0:
            self.logger.warn('蛇形航线区域太小')
            return []
        
        # 旋转到主方向
        cos_h = math.cos(-heading)
        sin_h = math.sin(-heading)
        rx = dx * cos_h - dy * sin_h
        ry = dx * sin_h + dy * cos_h
        
        num_lines = max(2, int(abs(ry) / line_spacing) + 1)
        step = ry / (num_lines - 1) if num_lines > 1 else 0
        
        waypoints = []
        for i in range(num_lines):
            y = i * step
            if i % 2 == 0:  # 从左到右
                x_start, x_end = 0, rx
            else:            # 从右到左
                x_start, x_end = rx, 0
            
            # 旋转回原始方向
            x1 = x_start * cos_h + y * sin_h
            y1 = -x_start * sin_h + y * cos_h
            x2 = x_end * cos_h + y * sin_h
            y2 = -x_end * sin_h + y * cos_h
            
            lat1 = start_lat + y1 / 111320.0
            lon1 = start_lon + x1 / (111320.0 * cos_lat)
            lat2 = start_lat + y2 / 111320.0
            lon2 = start_lon + x2 / (111320.0 * cos_lat)
            
            wp_heading = math.degrees(math.atan2(x2 - x1, y2 - y1))
            if wp_heading < 0:
                wp_heading += 360
            
            waypoints.append({
                'lat': lat1, 'lon': lon1, 'alt': altitude,
                'speed': speed, 'heading': wp_heading,
                'action': 'photo', 'dwell_time': 0,
            })
            waypoints.append({
                'lat': lat2, 'lon': lon2, 'alt': altitude,
                'speed': speed, 'heading': wp_heading,
                'action': 'photo', 'dwell_time': 0,
            })
        
        return waypoints
    
    def upload_mission(self, msg: WaypointMission):
        """上传任务"""
        with self.lock:
            if self.state not in [self.STATE_IDLE, self.STATE_COMPLETED,
                                   self.STATE_FAILED, self.STATE_CANCELLED]:
                self.logger.warn(f'无法上传: 当前状态 {self.STATE_NAMES[self.state]}')
                return False
            
            self.state = self.STATE_UPLOADING
            self.mission_id = msg.mission_id or f'mission_{int(time.time())}'
            self.route_type = msg.route_type
            self.camera_trigger_mode = msg.camera_trigger_mode
            self.camera_trigger_interval = msg.camera_trigger_interval
            
            # 解析或生成航点
            if msg.route_type == 1:  # 多边形
                self.waypoints = self.generate_polygon_waypoints(msg.mission_params_json)
            elif msg.route_type == 2:  # 蛇形
                self.waypoints = self.generate_snake_waypoints(msg.mission_params_json)
            else:
                self.waypoints = self._parse_waypoints(msg.waypoints_json)
            
            if not self.waypoints:
                self.state = self.STATE_FAILED
                self.logger.error('航点列表为空')
                return False
            
            self.logger.info(
                f'任务上传成功: {self.mission_id}, '
                f'{len(self.waypoints)} 个航点, '
                f'航线类型: {["自定义","多边形","蛇形"][self.route_type]}'
            )
            self.state = self.STATE_READY
            return True
    
    def start_mission(self):
        """启动任务"""
        with self.lock:
            if self.state != self.STATE_READY:
                self.logger.warn(f'无法启动: 当前状态 {self.STATE_NAMES[self.state]}')
                return False
            self.state = self.STATE_RUNNING
            self.current_index = 0
            self.photos_taken = 0
            self._photo_distance_accum = 0.0
            self._last_photo_lat = self.current_lat
            self._last_photo_lon = self.current_lon
            self.logger.info(f'任务启动: {self.mission_id}')
            return True
    
    def pause_mission(self):
        """暂停任务"""
        with self.lock:
            if self.state != self.STATE_RUNNING:
                return False
            self.state = self.STATE_PAUSED
            self.logger.info('任务已暂停')
            return True
    
    def resume_mission(self):
        """恢复任务"""
        with self.lock:
            if self.state != self.STATE_PAUSED:
                return False
            self.state = self.STATE_RUNNING
            self.logger.info('任务已恢复')
            return True
    
    def stop_mission(self):
        """停止任务"""
        with self.lock:
            if self.state not in [self.STATE_RUNNING, self.STATE_PAUSED]:
                return False
            self.state = self.STATE_CANCELLED
            self.logger.info('任务已取消')
            return True
    
    def update_position(self, lat: float, lon: float, alt: float):
        """更新当前 UAV 位置 (由外部控制器调用)"""
        self.current_lat = lat
        self.current_lon = lon
        self.current_alt = alt
    
    def should_trigger_photo(self) -> bool:
        """判断是否应该触发拍照 (等距模式)"""
        if self.camera_trigger_mode != 0:  # 非等距模式，由航点动作控制
            return False
        
        if self._last_photo_lat == 0:
            self._last_photo_lat = self.current_lat
            self._last_photo_lon = self.current_lon
            return False
        
        # 计算距离
        cos_lat = math.cos(math.radians(self.current_lat))
        dl = (self.current_lat - self._last_photo_lat) * 111320.0
        dlo = (self.current_lon - self._last_photo_lon) * 111320.0 * cos_lat
        dist = math.sqrt(dl * dl + dlo * dlo)
        
        if dist >= self.camera_trigger_interval:
            self._last_photo_lat = self.current_lat
            self._last_photo_lon = self.current_lon
            self.photos_taken += 1
            return True
        return False
    
    def advance_waypoint(self) -> bool:
        """
        推进到下一航点
        返回 True 表示任务继续，False 表示任务完成
        """
        with self.lock:
            if self.state != self.STATE_RUNNING:
                return True
            
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.state = self.STATE_COMPLETED
                self.logger.info(
                    f'任务完成: {self.mission_id}, '
                    f'拍照 {self.photos_taken} 张'
                )
                return False
            
            return True
    
    def get_current_waypoint(self) -> dict:
        """获取当前目标航点"""
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return {}
    
    def get_status(self) -> WaypointMissionStatus:
        """获取任务状态"""
        with self.lock:
            total = len(self.waypoints)
            progress = (self.current_index / total * 100.0) if total > 0 else 0.0
            
            status = WaypointMissionStatus()
            status.state = self.state
            status.current_waypoint_index = self.current_index
            status.total_waypoints = total
            status.progress = progress
            status.photos_taken = self.photos_taken
            status.mission_id = self.mission_id
            status.status_text = self.STATE_NAMES[self.state]
            
            wp = self.get_current_waypoint()
            status.current_waypoint_lat = wp.get('lat', 0.0)
            status.current_waypoint_lon = wp.get('lon', 0.0)
            status.current_waypoint_alt = wp.get('alt', 0.0)
            
            return status


class UAVMissionController(Node):
    """UAV 航点任务控制器节点"""
    
    def __init__(self):
        super().__init__('uav_mission_controller')
        
        # --- 参数 ---
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('waypoint_arrival_threshold', 3.0)  # 到达航点阈值 (米)
        self.declare_parameter('max_airspeed', 12.0)  # 最大空速
        
        self.update_rate = self.get_parameter('update_rate').value
        self.arrival_threshold = self.get_parameter('waypoint_arrival_threshold').value
        self.max_airspeed = self.get_parameter('max_airspeed').value
        
        # --- 任务管理器 ---
        self.mission_mgr = WaypointMissionManager(self.get_logger())
        
        # --- QoS ---
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # --- 订阅 ---
        self.mission_cmd_sub = self.create_subscription(
            WaypointMission, '/uav/mission/cmd',
            self.mission_cmd_callback, reliable_qos)
        self.uav_status_sub = self.create_subscription(
            UAVStatus, '/uav/status',
            self.uav_status_callback, reliable_qos)
        
        # --- 发布 ---
        self.mission_status_pub = self.create_publisher(
            WaypointMissionStatus, '/uav/mission/status', reliable_qos)
        
        # 发布模拟控制指令 (与现有 uav_controller 配合)
        self.cmd_pub = self.create_publisher(
            String, '/uav/cmd', reliable_qos)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/uav/mission/target_pose', reliable_qos)
        
        # --- 定时器 ---
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('UAV 航点任务控制器节点已启动')
    
    def mission_cmd_callback(self, msg: WaypointMission):
        """接收任务指令"""
        action_names = ['上传', '启动', '暂停', '恢复', '停止', '查询']
        action_name = action_names[msg.action] if msg.action < len(action_names) else '未知'
        self.get_logger().info(f'收到任务指令: {action_name}')
        
        if msg.action == 0:  # 上传航点
            self.mission_mgr.upload_mission(msg)
        elif msg.action == 1:  # 启动任务
            self.mission_mgr.start_mission()
        elif msg.action == 2:  # 暂停
            self.mission_mgr.pause_mission()
        elif msg.action == 3:  # 恢复
            self.mission_mgr.resume_mission()
        elif msg.action == 4:  # 停止
            self.mission_mgr.stop_mission()
        # action == 5: 查询 (不做任何操作，状态通过定时器推送)
    
    def uav_status_callback(self, msg: UAVStatus):
        """接收 UAV 当前位置更新"""
        self.mission_mgr.update_position(
            msg.latitude, msg.longitude, msg.altitude)
    
    def update_callback(self):
        """主循环 - 航线执行控制"""
        mgr = self.mission_mgr
        
        # 推送状态
        status = mgr.get_status()
        status.header.stamp = self.get_clock().now().to_msg()
        self.mission_status_pub.publish(status)
        
        # 如果在执行中，进行航线导航
        if mgr.state != mgr.STATE_RUNNING:
            return
        
        wp = mgr.get_current_waypoint()
        if not wp:
            return
        
        # 计算到目标航点的距离
        cos_lat = math.cos(math.radians(mgr.current_lat))
        dl = (wp['lat'] - mgr.current_lat) * 111320.0
        dlo = (wp['lon'] - mgr.current_lon) * 111320.0 * cos_lat
        dist = math.sqrt(dl * dl + dlo * dlo)
        
        # 到达阈值 → 下一航点
        if dist < self.arrival_threshold:
            # 触发拍照 (航点动作)
            if wp.get('action') == 'photo':
                mgr.photos_taken += 1
                self.get_logger().info(
                    f'拍照! 航点 {mgr.current_index + 1}/{len(mgr.waypoints)}, '
                    f'已拍 {mgr.photos_taken} 张'
                )
            
            # 悬停时间
            dwell = wp.get('dwell_time', 0)
            if dwell > 0:
                time.sleep(dwell)
            
            if not mgr.advance_waypoint():
                self.get_logger().info(
                    f'任务执行完成: 共 {mgr.photos_taken} 张照片'
                )
            return
        
        # 等距拍照检测
        if mgr.should_trigger_photo():
            self.get_logger().info(
                f'等距拍照触发! 距离 {mgr.camera_trigger_interval}m, '
                f'已拍 {mgr.photos_taken} 张'
            )
        
        # 发布目标位姿
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = (wp['lon'] - mgr.current_lon) * 111320.0 * cos_lat
        target_pose.pose.position.y = (wp['lat'] - mgr.current_lat) * 111320.0
        target_pose.pose.position.z = wp['alt']
        target_pose.pose.orientation.w = 1.0
        self.pose_pub.publish(target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = UAVMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UAV 航点任务控制器节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
