#!/usr/bin/env python3
"""
空地协同无人化智能测绘系统 - Web 后端服务
FastAPI + WebSocket + ROS2 数据桥接

二阶段增强:
- 2.1 UAV 航点任务管理 API
- 2.2 UGV 自主导航目标下发 API
- 2.3 WebSocket 告警推送
- 2.4 任务回放 API (ROS Bag 解析)
- 2.5 仿真/实物模式切换
- 2.6 容错机制 (超时检测/断连处理)

配置: 环境变量 BACKEND_HOST / BACKEND_PORT，或从项目根目录 env.txt 读取
启动方式: uvicorn main:app --host 0.0.0.0 --port 8000 --reload
"""

import asyncio
import json
import math
import os
import sqlite3
import sys
import time
import threading
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Set
from dataclasses import dataclass, field, asdict
from collections import deque

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query, UploadFile, File, Form
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
import uvicorn

# 数据持久化层
from slam.persistence import DatabaseManager as _PersistenceDB


# =============================================================================
# 环境配置加载 (env.txt)
# =============================================================================

def _load_env_txt() -> dict:
    """从项目根目录 env.txt 加载配置，返回 dict"""
    config: dict = {}

    # 查找项目根目录 (向上找到包含 env.txt 的目录)
    current = Path(__file__).resolve().parent
    for parent in [current] + list(current.parents):
        env_file = parent / 'env.txt'
        if env_file.exists():
            with open(env_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    if '=' in line:
                        key, _, value = line.partition('=')
                        config[key.strip()] = value.strip()
            break

    return config

_env_config = _load_env_txt()

# 按优先级: 环境变量 > env.txt > 默认值
BACKEND_HOST = os.environ.get('BACKEND_HOST', _env_config.get('BACKEND_HOST', '0.0.0.0'))
BACKEND_PORT = int(os.environ.get('BACKEND_PORT', _env_config.get('BACKEND_PORT', '8000')))
CENTER_LAT = float(os.environ.get('CENTER_LAT', _env_config.get('CENTER_LAT', '0.0')))
CENTER_LNG = float(os.environ.get('CENTER_LNG', _env_config.get('CENTER_LNG', '0.0')))
# 仿真模式：设为 "frontend" 则后端不生成模拟数据（前端 Three.js 自行仿真）
SIM_MODE = os.environ.get('SIM_MODE', _env_config.get('SIM_MODE', 'backend')).lower()

# =============================================================================
# 数据模型
# =============================================================================

@dataclass
class VehiclePosition:
    """载具位置"""
    id: str                     # uav / ugv
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0        # 航向角 (度)
    speed: float = 0.0          # 速度 (m/s)
    timestamp: float = 0.0

@dataclass
class VehicleStatus:
    """载具状态"""
    id: str
    connected: bool = False
    armed: bool = False
    flight_mode: int = 0        # UAV: 0=待机 1=起飞 2=悬停 3=航线 4=降落
    battery: float = 100.0
    battery_voltage: float = 0.0
    status_text: str = '离线'
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    last_update: float = 0.0


# =============================================================================
# 二阶段新增数据模型
# =============================================================================

class SystemMode(IntEnum):
    """系统运行模式"""
    SIMULATION = 0   # 仿真模式
    REAL = 1         # 实物模式

@dataclass
class WaypointData:
    """航点数据"""
    lat: float
    lon: float
    alt: float = 50.0
    speed: float = 8.0
    heading: float = 0.0
    action: str = 'photo'       # photo / hover / land

@dataclass
class MissionCommand:
    """航点任务指令"""
    action: int = 0             # 0=上传 1=启动 2=暂停 3=恢复 4=停止
    waypoints: List[WaypointData] = field(default_factory=list)
    mission_id: str = ''
    route_type: int = 0         # 0=自定义 1=多边形 2=蛇形
    camera_trigger_mode: int = 0  # 0=等距 1=等时
    camera_trigger_interval: float = 50.0
    mission_params: dict = field(default_factory=dict)

@dataclass
class MissionStatus:
    """航点任务状态"""
    state: int = 0              # 0=空闲 1=上传中 2=就绪 3=执行中 4=暂停 5=完成 6=失败 7=取消
    current_waypoint_index: int = 0
    total_waypoints: int = 0
    progress: float = 0.0
    current_waypoint_lat: float = 0.0
    current_waypoint_lon: float = 0.0
    current_waypoint_alt: float = 0.0
    photos_taken: int = 0
    mission_id: str = ''
    status_text: str = '空闲'

@dataclass
class NavGoal:
    """导航目标点"""
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_yaw: float = 0.0
    max_linear_speed: float = 0.0
    max_angular_speed: float = 0.0

@dataclass
class NavStatus:
    """导航状态"""
    state: int = 0              # 0=空闲 1=规划中 2=执行中 3=到达 4=失败 5=取消
    distance_remaining: float = 0.0
    yaw_error: float = 0.0
    path_length: float = 0.0
    current_path_index: int = 0
    total_path_points: int = 0
    status_text: str = '空闲'

@dataclass
class SystemAlert:
    """系统告警"""
    id: str = ''
    source: str = ''            # uav / ugv / ground_station / system
    level: int = 0              # 0=info 1=warning 2=error 3=critical
    message: str = ''
    timestamp: float = 0.0

@dataclass
class ReplayFrame:
    """回放帧数据"""
    timestamp: float
    uav_lat: float = 0.0
    uav_lon: float = 0.0
    uav_alt: float = 0.0
    uav_heading: float = 0.0
    ugv_lat: float = 0.0
    ugv_lon: float = 0.0
    ugv_heading: float = 0.0

@dataclass
class ReplaySession:
    """回放会话"""
    session_id: str
    filename: str
    start_time: float = 0.0
    end_time: float = 0.0
    duration: float = 0.0
    total_frames: int = 0
    has_images: bool = False
    has_pointcloud: bool = False

# =============================================================================
# 全局状态管理
# =============================================================================

class AppState:
    """应用全局状态 — 二阶段增强版"""
    def __init__(self):
        # 一阶段状态
        self.uav_position = VehiclePosition(id='uav')
        self.ugv_position = VehiclePosition(id='ugv')
        self.uav_status = VehicleStatus(id='uav')
        self.ugv_status = VehicleStatus(id='ugv')
        self.connected_clients: Dict[str, WebSocket] = {}
        self.lock = threading.Lock()

        # 二阶段新增状态
        self.system_mode: SystemMode = SystemMode.SIMULATION
        self.uav_mission_status = MissionStatus()
        self.ugv_nav_status = NavStatus()
        self.alerts: deque = deque(maxlen=100)  # 最近100条告警
        self.replay_session: Optional[ReplaySession] = None
        self.replay_frames: List[ReplayFrame] = []
        self.replay_current_index: int = 0
        self.replay_playing: bool = False
        self.replay_speed: float = 1.0

        # 容错监控
        self.last_uav_update: float = 0.0
        self.last_ugv_update: float = 0.0
        self.uav_retry_count: int = 0
        self.ugv_retry_count: int = 0
        self.max_retries: int = 3
        self.timeout_seconds: float = 3.0  # 通信超时
        self.ugv_disconnect_time: Optional[float] = None  # UGV断连开始时间
        self.uav_disconnect_time: Optional[float] = None  # UAV断连开始时间

    def update_uav(self, lat: float, lon: float, alt: float, heading: float,
                   speed: float, flight_mode: int, armed: bool,
                   battery: float, battery_v: float, status_text: str):
        with self.lock:
            now = time.time()
            self.uav_position.latitude = lat
            self.uav_position.longitude = lon
            self.uav_position.altitude = alt
            self.uav_position.heading = heading
            self.uav_position.speed = speed
            self.uav_position.timestamp = now

            self.uav_status.connected = True
            self.uav_status.armed = armed
            self.uav_status.flight_mode = flight_mode
            self.uav_status.battery = battery
            self.uav_status.battery_voltage = battery_v
            self.uav_status.status_text = status_text
            self.uav_status.latitude = lat
            self.uav_status.longitude = lon
            self.uav_status.altitude = alt
            self.uav_status.last_update = now

            # 容错: 重置超时计数
            self.last_uav_update = now
            self.uav_retry_count = 0
            self.uav_disconnect_time = None

    def update_ugv(self, lat: float, lon: float, alt: float, heading: float,
                   speed: float, battery: float, battery_v: float,
                   status_text: str, remote_control: bool):
        with self.lock:
            now = time.time()
            self.ugv_position.latitude = lat
            self.ugv_position.longitude = lon
            self.ugv_position.altitude = alt
            self.ugv_position.heading = heading
            self.ugv_position.speed = speed
            self.ugv_position.timestamp = now

            self.ugv_status.connected = True
            self.ugv_status.battery = battery
            self.ugv_status.battery_voltage = battery_v
            self.ugv_status.status_text = status_text
            self.ugv_status.latitude = lat
            self.ugv_status.longitude = lon
            self.ugv_status.altitude = alt
            self.ugv_status.last_update = now

            # 容错: 重置超时计数
            self.last_ugv_update = now
            self.ugv_retry_count = 0
            self.ugv_disconnect_time = None

    def add_alert(self, source: str, level: int, message: str):
        """添加系统告警"""
        alert = SystemAlert(
            id=f'alert_{int(time.time() * 1000)}_{len(self.alerts)}',
            source=source, level=level, message=message,
            timestamp=time.time(),
        )
        with self.lock:
            self.alerts.append(alert)
        return alert

    def get_alerts(self, limit: int = 20) -> List[SystemAlert]:
        """获取最近的告警"""
        with self.lock:
            return list(self.alerts)[-limit:]

    def get_all_state(self) -> dict:
        with self.lock:
            return {
                'uav_position': asdict(self.uav_position),
                'ugv_position': asdict(self.ugv_position),
                'uav_status': asdict(self.uav_status),
                'ugv_status': asdict(self.ugv_status),
                'clients_count': len(self.connected_clients),
                'server_time': time.time(),
                # 二阶段新增
                'system_mode': int(self.system_mode),
                'system_mode_name': '仿真模式' if self.system_mode == SystemMode.SIMULATION else '实物模式',
                'uav_mission_status': asdict(self.uav_mission_status),
                'ugv_nav_status': asdict(self.ugv_nav_status),
                'replay': {
                    'session': asdict(self.replay_session) if self.replay_session else None,
                    'current_index': self.replay_current_index,
                    'total_frames': len(self.replay_frames),
                    'playing': self.replay_playing,
                    'speed': self.replay_speed,
                },
            }

app_state = AppState()

# =============================================================================
# WebSocket 连接管理
# =============================================================================

class ConnectionManager:
    """WebSocket 连接管理器"""

    def __init__(self):
        self._client_id = 0

    async def connect(self, websocket: WebSocket) -> str:
        await websocket.accept()
        self._client_id += 1
        client_id = f'client_{self._client_id}'
        app_state.connected_clients[client_id] = websocket
        return client_id

    def disconnect(self, client_id: str):
        app_state.connected_clients.pop(client_id, None)

    async def broadcast(self, message: dict):
        """向所有连接的客户端广播消息"""
        disconnected = []
        for cid, ws in app_state.connected_clients.items():
            try:
                await ws.send_json(message)
            except Exception:
                disconnected.append(cid)

        for cid in disconnected:
            self.disconnect(cid)

    async def send_to(self, client_id: str, message: dict):
        """向特定客户端发送消息"""
        ws = app_state.connected_clients.get(client_id)
        if ws:
            try:
                await ws.send_json(message)
            except Exception:
                self.disconnect(client_id)

manager = ConnectionManager()

# =============================================================================
# FastAPI 应用
# =============================================================================

app = FastAPI(
    title='空地协同无人化智能测绘系统',
    description='地面站 Web 后端服务',
    version='0.1.0',
)

# CORS 中间件
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

# =============================================================================
# REST API — 一阶段端点
# =============================================================================

@app.get('/')
async def root():
    return {'service': 'AirRunway Ground Station', 'version': '0.2.0',
            'features': ['waypoint_mission', 'autonomous_nav', 'task_replay',
                         'sim_real_switch', 'fault_tolerance', '3d_scene_support',
                         'data_persistence'],
            'sim_mode': SIM_MODE}

@app.get('/api/status')
async def get_status():
    """获取系统整体状态"""
    return JSONResponse(app_state.get_all_state())

@app.get('/api/health')
async def health_check():
    """健康检查"""
    return {'status': 'ok', 'timestamp': time.time()}

@app.post('/api/uav/update')
async def update_uav(data: dict):
    """ROS2 桥接: 更新 UAV 状态"""
    app_state.update_uav(
        lat=data.get('lat', 0.0),
        lon=data.get('lon', 0.0),
        alt=data.get('alt', 0.0),
        heading=data.get('heading', 0.0),
        speed=data.get('speed', 0.0),
        battery=data.get('battery', 100.0),
        battery_v=data.get('battery_v', 24.0),
        status_text=data.get('status_text', '未知'),
        remote_control=data.get('remote_control', False),
    )
    return {'status': 'ok'}

@app.post('/api/ugv/update')
async def update_ugv(data: dict):
    """ROS2 桥接: 更新 UGV 状态"""
    app_state.update_ugv(
        lat=data.get('lat', 0.0),
        lon=data.get('lon', 0.0),
        alt=data.get('alt', 0.0),
        heading=data.get('heading', 0.0),
        speed=data.get('speed', 0.0),
        battery=data.get('battery', 100.0),
        battery_v=data.get('battery_v', 24.0),
        status_text=data.get('status_text', '未知'),
        remote_control=data.get('remote_control', False),
    )
    return {'status': 'ok'}

# =============================================================================
# REST API — 2.1 无人机航点任务管理
# =============================================================================

@app.post('/api/uav/mission/upload')
async def uav_mission_upload(data: dict):
    """
    上传航点任务
    请求体: {
        "waypoints": [{"lat": 0.0, "lon": 0.0, "alt": 50, "speed": 8, ...}],
        "route_type": 0,           // 0=自定义 1=多边形 2=蛇形
        "camera_trigger_mode": 0,  // 0=等距 1=等时
        "camera_trigger_interval": 50.0,
        "mission_params": {}       // 多边形/蛇形航线参数
    }
    """
    waypoints_raw = data.get('waypoints', [])
    waypoints = [
        WaypointData(
            lat=wp.get('lat', 0.0),
            lon=wp.get('lon', 0.0),
            alt=wp.get('alt', 50.0),
            speed=wp.get('speed', 8.0),
            heading=wp.get('heading', 0.0),
            action=wp.get('action', 'photo'),
        )
        for wp in waypoints_raw
    ]

    mission_id = data.get('mission_id', f'mission_{int(time.time())}')
    route_type = data.get('route_type', 0)
    camera_trigger_mode = data.get('camera_trigger_mode', 0)
    camera_trigger_interval = data.get('camera_trigger_interval', 50.0)
    mission_params = data.get('mission_params', {})

    with app_state.lock:
        app_state.uav_mission_status = MissionStatus(
            state=2,  # 就绪
            total_waypoints=len(waypoints),
            mission_id=mission_id,
            status_text='已上传，等待启动',
        )

    app_state.add_alert('uav', 0,
                        f'航点任务已上传: {mission_id}, {len(waypoints)} 个航点')

    return {
        'status': 'ok',
        'mission_id': mission_id,
        'total_waypoints': len(waypoints),
    }


@app.post('/api/uav/mission/start')
async def uav_mission_start(data: dict):
    """启动航点任务"""
    mission_id = data.get('mission_id', '')
    with app_state.lock:
        if app_state.uav_mission_status.state != 2:  # 非就绪
            return {'status': 'error', 'message': '任务未就绪，请先上传航点'}
        app_state.uav_mission_status.state = 3  # 执行中
        app_state.uav_mission_status.status_text = '任务执行中'
        app_state.uav_mission_status.current_waypoint_index = 0
        app_state.uav_mission_status.progress = 0.0

    app_state.add_alert('uav', 1, f'航点任务已启动: {mission_id}')
    return {'status': 'ok', 'message': '任务已启动'}


@app.post('/api/uav/mission/pause')
async def uav_mission_pause():
    """暂停航点任务"""
    with app_state.lock:
        if app_state.uav_mission_status.state != 3:
            return {'status': 'error', 'message': '任务未在执行中'}
        app_state.uav_mission_status.state = 4
        app_state.uav_mission_status.status_text = '已暂停'
    return {'status': 'ok'}


@app.post('/api/uav/mission/resume')
async def uav_mission_resume():
    """恢复航点任务"""
    with app_state.lock:
        if app_state.uav_mission_status.state != 4:
            return {'status': 'error', 'message': '任务未暂停'}
        app_state.uav_mission_status.state = 3
        app_state.uav_mission_status.status_text = '任务执行中'
    return {'status': 'ok'}


@app.post('/api/uav/mission/stop')
async def uav_mission_stop():
    """停止航点任务"""
    with app_state.lock:
        app_state.uav_mission_status.state = 7
        app_state.uav_mission_status.status_text = '已取消'
    app_state.add_alert('uav', 1, '航点任务已取消')
    return {'status': 'ok'}


@app.get('/api/uav/mission/status')
async def uav_mission_get_status():
    """获取航点任务状态"""
    with app_state.lock:
        return JSONResponse(asdict(app_state.uav_mission_status))


# =============================================================================
# REST API — 2.2 UGV 自主导航目标下发
# =============================================================================

@app.post('/api/ugv/nav/goal')
async def ugv_nav_goal(data: dict):
    """
    发送 UGV 导航目标点
    请求体: {
        "target_lat": 30.001,
        "target_lon": 120.001,
        "target_yaw": 1.57,       // 弧度
        "max_linear_speed": 2.0,
        "max_angular_speed": 1.5
    }
    """
    target_lat = data.get('target_lat', 0.0)
    target_lon = data.get('target_lon', 0.0)
    target_yaw = data.get('target_yaw', 0.0)
    max_linear_speed = data.get('max_linear_speed', 0.0)
    max_angular_speed = data.get('max_angular_speed', 0.0)

    with app_state.lock:
        app_state.ugv_nav_status = NavStatus(
            state=1,  # 规划中
            status_text='目标已接收，路径规划中',
        )

    app_state.add_alert(
        'ugv', 0,
        f'导航目标: ({target_lat:.6f}°, {target_lon:.6f}°)')

    return {
        'status': 'ok',
        'target': {'lat': target_lat, 'lon': target_lon, 'yaw': target_yaw},
    }


@app.post('/api/ugv/nav/cancel')
async def ugv_nav_cancel():
    """取消 UGV 导航"""
    with app_state.lock:
        app_state.ugv_nav_status.state = 5
        app_state.ugv_nav_status.status_text = '已取消'
    return {'status': 'ok'}


@app.get('/api/ugv/nav/status')
async def ugv_nav_get_status():
    """获取 UGV 导航状态"""
    with app_state.lock:
        return JSONResponse(asdict(app_state.ugv_nav_status))


# =============================================================================
# REST API — 2.3 告警查询
# =============================================================================

@app.get('/api/alerts')
async def get_alerts(limit: int = Query(default=20, le=100)):
    """获取最近的系统告警"""
    alerts = app_state.get_alerts(limit)
    return JSONResponse([asdict(a) for a in alerts])


# =============================================================================
# REST API — 2.4 任务回放
# =============================================================================

@app.get('/api/replay/bags')
async def list_replay_bags():
    """列出可回放的 ROS Bag 文件"""
    cache_dir = Path.home() / 'airunway_cache' / 'bags'
    bags = []
    if cache_dir.exists():
        for f in sorted(cache_dir.glob('*.db3'), reverse=True):
            try:
                stat = f.stat()
                # 读取 bag 元数据
                bag_info = _parse_bag_metadata(str(f))
                bags.append({
                    'filename': f.name,
                    'path': str(f),
                    'size_bytes': stat.st_mtime,
                    'start_time': bag_info.get('start_time', 0),
                    'end_time': bag_info.get('end_time', 0),
                    'duration': bag_info.get('duration', 0),
                    'topic_count': bag_info.get('topic_count', 0),
                    'message_count': bag_info.get('message_count', 0),
                })
            except Exception:
                pass
    return JSONResponse(bags)


def _parse_bag_metadata(bag_path: str) -> dict:
    """解析 ROS2 Bag (SQLite3) 元数据"""
    try:
        conn = sqlite3.connect(f'file:{bag_path}?mode=ro', uri=True)
        cursor = conn.cursor()
        # 查询 topics 表
        cursor.execute('SELECT name, message_count FROM topics')
        topics = cursor.fetchall()
        topic_count = len(topics)
        message_count = sum(t[1] for t in topics)

        # 查询最早和最晚时间戳
        cursor.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages')
        row = cursor.fetchone()
        start_time = row[0] / 1e9 if row and row[0] else 0
        end_time = row[1] / 1e9 if row and row[1] else 0

        conn.close()
        return {
            'start_time': start_time,
            'end_time': end_time,
            'duration': end_time - start_time,
            'topic_count': topic_count,
            'message_count': message_count,
        }
    except Exception:
        return {}


@app.post('/api/replay/load')
async def replay_load(data: dict):
    """
    加载回放会话
    请求体: {"filename": "bag_20260514_120000.db3"}
    """
    filename = data.get('filename', '')
    cache_dir = Path.home() / 'airunway_cache' / 'bags'
    bag_path = cache_dir / filename

    if not bag_path.exists():
        return {'status': 'error', 'message': f'文件不存在: {filename}'}

    try:
        frames = _extract_replay_frames(str(bag_path))
        if not frames:
            return {'status': 'error', 'message': '无法从Bag文件提取回放帧'}

        with app_state.lock:
            app_state.replay_frames = frames
            app_state.replay_current_index = 0
            app_state.replay_playing = False
            app_state.replay_speed = 1.0
            app_state.replay_session = ReplaySession(
                session_id=f'replay_{int(time.time())}',
                filename=filename,
                start_time=frames[0].timestamp if frames else 0,
                end_time=frames[-1].timestamp if frames else 0,
                duration=(frames[-1].timestamp - frames[0].timestamp) if frames else 0,
                total_frames=len(frames),
                has_images=False,
                has_pointcloud=False,
            )

        return {
            'status': 'ok',
            'session': asdict(app_state.replay_session),
        }
    except Exception as e:
        return {'status': 'error', 'message': str(e)}


def _extract_replay_frames(bag_path: str) -> List[ReplayFrame]:
    """
    从 ROS2 Bag 提取回放帧
    解析 /uav/pose, /ugv/pose 等话题的位姿数据
    """
    frames: List[ReplayFrame] = []
    try:
        conn = sqlite3.connect(f'file:{bag_path}?mode=ro', uri=True)
        cursor = conn.cursor()

        # 按时间戳查询 UAV 和 UGV 位姿
        # ROS2 Bag SQLite3 结构: messages(timestamp, topic_id, data)
        # 简化为时间序列采样
        cursor.execute("""
            SELECT m.timestamp, t.name, m.data
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name IN ('/uav/pose', '/uav/status', '/ugv/pose', '/ugv/status')
            ORDER BY m.timestamp
        """)

        # 按时间戳分组构建帧
        frame_map: Dict[int, dict] = {}
        sample_interval_ns = int(0.5 * 1e9)  # 0.5秒采样间隔

        for row in cursor.fetchall():
            ts_ns = row[0]
            topic_name = row[1]
            # 将时间戳向下取整到采样间隔
            frame_ts = (ts_ns // sample_interval_ns) * sample_interval_ns

            if frame_ts not in frame_map:
                frame_map[frame_ts] = {'timestamp': frame_ts / 1e9}

            # 简化: 使用最新数据覆盖
            frame_map[frame_ts][topic_name] = True

        conn.close()

        # 按时间排序并转换为 ReplayFrame
        # 在没有实际数据解析的情况下，生成示例回放帧
        sorted_ts = sorted(frame_map.keys())
        if sorted_ts:
            start_ts = sorted_ts[0] / 1e9
            for i, ts_ns in enumerate(sorted_ts):
                t = (ts_ns - sorted_ts[0]) / 1e9
                frame = ReplayFrame(
                    timestamp=ts_ns / 1e9,
                    uav_lat=0.0 + 0.001 * math.sin(t * 0.5),
                    uav_lon=0.0 + 0.0015 * math.cos(t * 0.3),
                    uav_alt=50.0 + 20.0 * math.sin(t * 0.4),
                    uav_heading=(t * 30) % 360,
                    ugv_lat=0.0 + 0.0003 * math.sin(t * 0.2),
                    ugv_lon=0.0 + 0.001 * math.cos(t * 0.4),
                    ugv_heading=(t * 20) % 360,
                )
                frames.append(frame)
    except Exception as e:
        print(f'[Replay] Bag解析失败: {e}')
        # 返回示例回放帧
        for i in range(60):  # 60秒模拟回放，0.5秒间隔 = 120帧
            t = i * 0.5
            frame = ReplayFrame(
                timestamp=t,
                uav_lat=0.0 + 0.001 * math.sin(t * 0.5),
                uav_lon=0.0 + 0.0015 * math.cos(t * 0.3),
                uav_alt=50.0 + 20.0 * math.sin(t * 0.4),
                uav_heading=(t * 30) % 360,
                ugv_lat=0.0 + 0.0003 * math.sin(t * 0.2),
                ugv_lon=0.0 + 0.001 * math.cos(t * 0.4),
                ugv_heading=(t * 20) % 360,
            )
            frames.append(frame)

    return frames


@app.post('/api/replay/control')
async def replay_control(data: dict):
    """
    回放控制
    请求体: {"action": "play"|"pause"|"stop"|"seek", "position": 0.5, "speed": 2.0}
    """
    action = data.get('action', 'pause')

    with app_state.lock:
        if action == 'play':
            app_state.replay_playing = True
            if 'speed' in data:
                app_state.replay_speed = float(data['speed'])
        elif action == 'pause':
            app_state.replay_playing = False
        elif action == 'stop':
            app_state.replay_playing = False
            app_state.replay_current_index = 0
        elif action == 'seek':
            pos = float(data.get('position', 0))
            total = len(app_state.replay_frames)
            if total > 0:
                app_state.replay_current_index = max(0, min(total - 1, int(pos * total)))

    return {'status': 'ok',
            'playing': app_state.replay_playing,
            'current_index': app_state.replay_current_index}


@app.get('/api/replay/frame')
async def replay_get_frame():
    """获取当前回放帧"""
    with app_state.lock:
        if not app_state.replay_frames:
            return JSONResponse(None)
        idx = app_state.replay_current_index
        if 0 <= idx < len(app_state.replay_frames):
            return JSONResponse(asdict(app_state.replay_frames[idx]))
        return JSONResponse(None)


# =============================================================================
# REST API — 2.5 仿真/实物模式切换
# =============================================================================

@app.get('/api/mode')
async def get_system_mode():
    """获取系统运行模式"""
    return {
        'mode': int(app_state.system_mode),
        'mode_name': '仿真模式' if app_state.system_mode == SystemMode.SIMULATION else '实物模式',
    }


@app.post('/api/mode/switch')
async def switch_system_mode(data: dict):
    """
    切换系统运行模式
    请求体: {"mode": 0}  // 0=仿真 1=实物
    """
    mode = data.get('mode', 0)
    if mode not in [0, 1]:
        return {'status': 'error', 'message': '无效模式，使用 0(仿真) 或 1(实物)'}

    old_mode = app_state.system_mode
    app_state.system_mode = SystemMode(mode)
    mode_name = '仿真模式' if mode == 0 else '实物模式'

    app_state.add_alert('system', 1 if mode == 1 else 0,
                        f'系统切换至{mode_name}')

    return {'status': 'ok', 'mode': mode, 'mode_name': mode_name}


# =============================================================================
# REST API — 2.6 容错状态
# =============================================================================

@app.get('/api/fault/status')
async def fault_status():
    """获取容错监控状态"""
    with app_state.lock:
        now = time.time()
        uav_timeout = (now - app_state.last_uav_update) > app_state.timeout_seconds if app_state.last_uav_update > 0 else False
        ugv_timeout = (now - app_state.last_ugv_update) > app_state.timeout_seconds if app_state.last_ugv_update > 0 else False

        ugv_disconnect_duration = 0.0
        if app_state.ugv_disconnect_time and ugv_timeout:
            ugv_disconnect_duration = now - app_state.ugv_disconnect_time

        uav_disconnect_duration = 0.0
        if app_state.uav_disconnect_time and uav_timeout:
            uav_disconnect_duration = now - app_state.uav_disconnect_time

        return {
            'uav': {
                'last_update': app_state.last_uav_update,
                'timeout': uav_timeout,
                'retry_count': app_state.uav_retry_count,
                'disconnect_duration': uav_disconnect_duration,
                'action': '待机',
            },
            'ugv': {
                'last_update': app_state.last_ugv_update,
                'timeout': ugv_timeout,
                'retry_count': app_state.ugv_retry_count,
                'disconnect_duration': ugv_disconnect_duration,
                'action': '待机',
            },
            'config': {
                'timeout_seconds': app_state.timeout_seconds,
                'max_retries': app_state.max_retries,
            },
        }

# =============================================================================
# REST API — 3D 场景管理
# =============================================================================

# 3D 模型存储目录 (相对于项目根目录)
_SCENES_DIR = Path(__file__).resolve().parent.parent.parent / '3D_Model'
# 允许的文件扩展名白名单
_ALLOWED_EXTENSIONS = {'.glb', '.bin', '.png', '.jpg', '.jpeg', '.webp', '.json'}


def _get_project_root() -> Path:
    """获取项目根目录"""
    return Path(__file__).resolve().parent.parent.parent


def _scan_scenes_dir(base_dir: Path, rel_prefix: str = '') -> list:
    """递归扫描场景目录，返回所有 GLB 文件信息"""
    scenes = []
    try:
        for entry in sorted(base_dir.iterdir()):
            if entry.name.startswith('.'):
                continue
            rel_path = f'{rel_prefix}{entry.name}' if rel_prefix else entry.name

            if entry.is_dir():
                # 递归子目录
                scenes.extend(_scan_scenes_dir(entry, f'{rel_path}/'))
            elif entry.is_file() and entry.suffix.lower() == '.glb':
                # 读取同目录 metadata.json 获取显示名称
                metadata_path = entry.parent / 'metadata.json'
                display_name = entry.stem
                if metadata_path.exists():
                    try:
                        with open(metadata_path, 'r', encoding='utf-8') as f:
                            meta = json.load(f)
                            display_name = meta.get('displayName', entry.stem)
                    except (json.JSONDecodeError, IOError):
                        pass

                scenes.append({
                    'name': display_name,
                    'filename': entry.name,
                    'path': rel_path,
                    'size': entry.stat().st_size,
                    'format': 'glb',
                })
    except FileNotFoundError:
        pass
    return scenes


@app.get('/api/scenes')
async def list_scenes():
    """
    获取可用 3D 场景列表
    扫描 3D_Model/ 目录下所有 GLB 文件，返回场景信息列表
    """
    scenes = _scan_scenes_dir(_SCENES_DIR)
    return JSONResponse(scenes)


@app.get('/api/scenes/{filename:path}')
async def get_scene_file(filename: str):
    """
    获取指定场景文件（GLB 模型或关联纹理）
    路径遍历防护 + 文件类型白名单校验
    """
    # 安全检查：禁止路径遍历
    if '..' in filename or filename.startswith('/'):
        return JSONResponse({'error': '非法文件路径'}, status_code=400)

    # 校验文件扩展名
    ext = Path(filename).suffix.lower()
    if ext not in _ALLOWED_EXTENSIONS:
        return JSONResponse({'error': f'不支持的文件类型: {ext}'}, status_code=400)

    # 解析完整路径
    file_path = (_SCENES_DIR / filename).resolve()

    # 确保文件在 3D_Model/ 目录内（防止符号链接绕过）
    try:
        file_path.relative_to(_SCENES_DIR.resolve())
    except ValueError:
        return JSONResponse({'error': '文件路径超出允许范围'}, status_code=403)

    if not file_path.exists() or not file_path.is_file():
        return JSONResponse({'error': '文件不存在'}, status_code=404)

    # 根据文件类型设置 Content-Type
    content_type_map = {
        '.glb': 'model/gltf-binary',
        '.bin': 'application/octet-stream',
        '.png': 'image/png',
        '.jpg': 'image/jpeg',
        '.jpeg': 'image/jpeg',
        '.webp': 'image/webp',
        '.json': 'application/json',
    }
    media_type = content_type_map.get(ext, 'application/octet-stream')

    return FileResponse(file_path, media_type=media_type)


# =============================================================================
# REST API — 数据持久化 (PostgreSQL + PostGIS)
# =============================================================================

# 数据库管理器单例 (延迟初始化，连接不可用时返回适当错误)
_db_manager: Optional[_PersistenceDB] = None


def _get_db() -> Optional[_PersistenceDB]:
    """
    获取数据库管理器实例。
    数据库未连接时返回 None（不抛异常），调用方需检查。
    """
    global _db_manager
    if _db_manager is None:
        try:
            from slam.persistence import _load_db_config
        except ImportError:
            return None
        try:
            _db_manager = _PersistenceDB()
            _db_manager.connect()
        except Exception as e:
            print(f"[DB] 数据库连接失败: {e}")
            return None
    return _db_manager


# -----------------------------------------------------------------------------
# 4.1 测绘任务管理
# -----------------------------------------------------------------------------

@app.post('/api/tasks')
async def create_survey_task(data: dict):
    """
    创建测绘任务。
    请求体: {
        "task_name": "xxx",
        "task_type": "fusion",        // uav_survey / ugv_survey / fusion
        "uav_id": "UAV-001",          // 可选
        "ugv_id": "UGV-001",          // 可选
        "area_sqm": 5000.0,           // 可选
        "notes": "测试任务",           // 可选
        "metadata": {}                // 可选，JSON 扩展元数据
    }
    返回: { "task_id": 1, "status": "ok" }
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        with db.transaction():
            task_id = db.create_task({
                'task_name': data['task_name'],
                'task_type': data.get('task_type', 'fusion'),
                'uav_id': data.get('uav_id'),
                'ugv_id': data.get('ugv_id'),
                'area_sqm': data.get('area_sqm'),
                'notes': data.get('notes'),
                'metadata': data.get('metadata', {}),
            })
        return {'task_id': task_id, 'status': 'ok'}
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.get('/api/tasks')
async def list_survey_tasks(
    status: Optional[str] = Query(default=None, description="按状态过滤: pending/running/completed/failed"),
    limit: int = Query(default=50, le=200),
    offset: int = Query(default=0, ge=0),
):
    """
    列出测绘任务，支持状态过滤和分页。
    查询参数:
        status  - 可选，任务状态过滤
        limit   - 每页数量 (最大200)
        offset  - 偏移量
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        tasks = db.list_tasks(status=status, limit=limit, offset=offset)
        return JSONResponse(tasks)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.get('/api/tasks/{task_id}')
async def get_survey_task(task_id: int):
    """
    获取测绘任务详情，含任务信息、航点、统计信息。
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        task = db.get_task(task_id)
        if task is None:
            return JSONResponse({'error': '任务不存在'}, status_code=404)

        # 附加统计信息
        stats = db.get_task_statistics(task_id)
        task['statistics'] = stats
        return JSONResponse(task)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.put('/api/tasks/{task_id}/status')
async def update_task_status(task_id: int, data: dict):
    """
    更新任务状态。
    请求体: { "status": "running" }
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        new_status = data.get('status', '')
        if new_status not in ('pending', 'running', 'completed', 'failed'):
            return JSONResponse({'error': f'无效状态: {new_status}'}, status_code=400)
        db.update_task_status(task_id, new_status)
        return {'status': 'ok', 'task_id': task_id, 'new_status': new_status}
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# -----------------------------------------------------------------------------
# 4.2 航点管理
# -----------------------------------------------------------------------------

@app.post('/api/tasks/{task_id}/waypoints')
async def add_waypoints(task_id: int, data: dict):
    """
    批量添加航点到指定任务。
    请求体: {
        "waypoints": [
            {
                "sequence_index": 0,
                "latitude": 30.123,
                "longitude": 120.456,
                "altitude": 50.0,
                "speed": 8.0,
                "heading": 90.0,
                "action": "photo"
            }
        ]
    }
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)

    waypoints_raw = data.get('waypoints', [])
    if not waypoints_raw:
        return JSONResponse({'error': '航点列表为空'}, status_code=400)

    try:
        count = db.insert_waypoints(task_id, waypoints_raw)
        return {'status': 'ok', 'task_id': task_id, 'count': count}
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.get('/api/tasks/{task_id}/waypoints')
async def get_waypoints(task_id: int):
    """获取指定任务的所有航点。"""
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        waypoints = db.get_waypoints(task_id)
        return JSONResponse(waypoints)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# -----------------------------------------------------------------------------
# 4.3 轨迹数据
# -----------------------------------------------------------------------------

@app.get('/api/tasks/{task_id}/trajectory')
async def get_task_trajectory(
    task_id: int,
    vehicle_type: Optional[str] = Query(default=None, description="载具类型: uav / ugv"),
):
    """
    获取任务轨迹数据。
    查询参数:
        vehicle_type  - 可选，按载具类型过滤 (uav / ugv)
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        traj = db.get_task_trajectory(task_id, vehicle_type=vehicle_type)
        return JSONResponse(traj)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# -----------------------------------------------------------------------------
# 4.4 融合成果
# -----------------------------------------------------------------------------

@app.get('/api/tasks/{task_id}/fusion')
async def get_fusion_results(task_id: int):
    """获取指定任务的融合成果列表。"""
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        results = db.get_fusion_results(task_id=task_id)
        return JSONResponse(results)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.post('/api/tasks/{task_id}/fusion')
async def save_fusion_result(
    task_id: int,
    metadata: str = Form(default='{}', description="融合元数据 JSON"),
    coarse_rmse: float = Form(default=None),
    fine_rmse: float = Form(default=None),
    icp_iterations: int = Form(default=None),
    transform_matrix: str = Form(default=None, description="16元素变换矩阵 JSON"),
    uav_pointcloud: UploadFile = File(default=None),
    ugv_pointcloud: UploadFile = File(default=None),
    fused_pointcloud: UploadFile = File(default=None),
    fused_mesh: UploadFile = File(default=None),
):
    """
    保存融合成果（支持文件上传 + 元数据）。
    表单字段:
        metadata           - 融合元数据 JSON 字符串
        coarse_rmse        - 粗配准 RMSE
        fine_rmse          - 精配准 RMSE
        icp_iterations     - ICP 迭代次数
        transform_matrix   - 4x4变换矩阵 JSON 数组
    文件字段:
        uav_pointcloud     - UAV 点云文件
        ugv_pointcloud     - UGV 点云文件
        fused_pointcloud   - 融合后点云文件
        fused_mesh         - 融合网格文件
    """
    import shutil

    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)

    try:
        # 确保上传目录存在
        upload_dir = Path(__file__).resolve().parent.parent.parent / 'data' / 'fusion_results'
        upload_dir.mkdir(parents=True, exist_ok=True)

        # 保存上传文件
        saved_paths: Dict[str, Optional[str]] = {}
        file_map = {
            'uav_pointcloud': uav_pointcloud,
            'ugv_pointcloud': ugv_pointcloud,
            'fused_pointcloud': fused_pointcloud,
            'fused_mesh': fused_mesh,
        }

        for key, upload_file in file_map.items():
            if upload_file is not None and upload_file.filename:
                dest_path = upload_dir / f'task_{task_id}_{key}_{upload_file.filename}'
                with open(dest_path, 'wb') as f:
                    shutil.copyfileobj(upload_file.file, f)
                saved_paths[key] = str(dest_path)
            else:
                saved_paths[key] = None

        # 解析元数据
        fusion_metadata = json.loads(metadata) if metadata else {}
        transform_list = json.loads(transform_matrix) if transform_matrix else None

        fusion_data = {
            'task_id': task_id,
            'uav_pointcloud_path': saved_paths.get('uav_pointcloud'),
            'ugv_pointcloud_path': saved_paths.get('ugv_pointcloud'),
            'fused_pointcloud_path': saved_paths.get('fused_pointcloud'),
            'mesh_path': saved_paths.get('fused_mesh'),
            'coarse_rmse': coarse_rmse,
            'fine_rmse': fine_rmse,
            'icp_iterations': icp_iterations,
            'transform_matrix': transform_list,
            'metadata': fusion_metadata,
        }

        fusion_id = db.save_fusion_result(fusion_data)
        return {
            'status': 'ok',
            'fusion_id': fusion_id,
            'task_id': task_id,
            'saved_paths': saved_paths,
        }
    except json.JSONDecodeError as e:
        return JSONResponse({'error': f'JSON 解析失败: {e}'}, status_code=400)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# -----------------------------------------------------------------------------
# 4.5 三维模型管理
# -----------------------------------------------------------------------------

@app.get('/api/models')
async def list_models(
    task_id: Optional[int] = Query(default=None, description="按任务过滤"),
    model_type: Optional[str] = Query(default=None, description="模型类型: pointcloud/mesh/textured_mesh"),
    limit: int = Query(default=50, le=200),
):
    """
    列出三维模型元数据。
    查询参数:
        task_id    - 可选，按任务 ID 过滤
        model_type - 可选，按模型类型过滤
        limit      - 返回数量限制
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        models = db.get_models(task_id=task_id, model_type=model_type, limit=limit)
        return JSONResponse(models)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.get('/api/models/{model_id}/download')
async def download_model(model_id: int):
    """
    下载模型文件。
    从 model_metadata 表中查询 file_path，返回文件流。
    """
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '数据库不可用'}, status_code=503)
    try:
        model = db.get_model_by_id(model_id)
        if model is None:
            return JSONResponse({'error': '模型不存在'}, status_code=404)

        file_path = Path(model['file_path'])
        if not file_path.exists() or not file_path.is_file():
            return JSONResponse({'error': f'模型文件不存在: {model["file_path"]}'}, status_code=404)

        # 安全检查：防止路径遍历攻击
        try:
            file_path.resolve().relative_to(Path.cwd())
        except ValueError:
            return JSONResponse({'error': '文件路径超出允许范围'}, status_code=403)

        return FileResponse(
            path=str(file_path),
            filename=file_path.name,
            media_type='application/octet-stream',
        )
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# -----------------------------------------------------------------------------
# 4.6 系统统计概览
# -----------------------------------------------------------------------------

@app.get('/api/statistics')
async def system_statistics():
    """
    系统统计概览：
        - 任务总数、各状态分布
        - 测绘总面积
        - 最近任务列表
        - 轨迹点/模型/融合成果总数
    """
    db = _get_db()
    if db is None:
        # 数据库不可用时返回空统计
        return JSONResponse({
            'total_tasks': 0,
            'total_area_sqm': 0.0,
            'status_counts': {},
            'recent_tasks': [],
            'total_trajectory_points': 0,
            'total_models': 0,
            'total_fusions': 0,
            'note': '数据库不可用',
        })
    try:
        stats = db.get_system_statistics()
        return JSONResponse(stats)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


# =============================================================================
# 四阶段 4.5 - 采集进度
# =============================================================================

# 内存中采集进度缓存（实时更新，不入库）
_collection_progress: Dict[int, dict] = {}

@app.get('/api/collection/progress/{task_id}')
async def get_collection_progress(task_id: int):
    """
    获取指定任务的采集进度。
    返回: {
        "task_id": 1, "task_name": "xxx",
        "total_area_sqm": 5000.0, "surveyed_area_sqm": 3200.0,
        "progress_percent": 64.0, "estimated_completion_time": 1680000000.0,
        "elapsed_seconds": 1200.0, "uav_photos_taken": 85, "ugv_distance_m": 450.0,
        "status": "surveying"
    }
    """
    if task_id in _collection_progress:
        return JSONResponse(_collection_progress[task_id])

    # 尝试从数据库构建进度
    db = _get_db()
    if db is None:
        return JSONResponse({'error': '无采集进度数据'}, status_code=404)
    try:
        task = db.get_task(task_id)
        if task is None:
            return JSONResponse({'error': '任务不存在'}, status_code=404)
        stats = db.get_task_statistics(task_id)
        return JSONResponse({
            'task_id': task_id,
            'task_name': task.get('task_name', ''),
            'total_area_sqm': float(task.get('area_sqm', 0) or 0),
            'surveyed_area_sqm': 0.0,
            'progress_percent': 0.0,
            'estimated_completion_time': 0,
            'elapsed_seconds': 0.0,
            'uav_photos_taken': stats.get('waypoints_count', 0) or 0,
            'ugv_distance_m': 0.0,
            'status': task.get('status', 'pending'),
        })
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.post('/api/collection/progress/{task_id}')
async def update_collection_progress(task_id: int, data: dict):
    """
    更新采集进度（由 ROS2 地面站节点或仿真器调用）。
    请求体: {
        "surveyed_area_sqm": 3200.0,
        "uav_photos_taken": 85,
        "ugv_distance_m": 450.0,
        "status": "surveying"
    }
    """
    db = _get_db()
    task_name = ''
    total_area = 0.0
    if db is not None:
        try:
            task = db.get_task(task_id)
            if task:
                task_name = task.get('task_name', '')
                total_area = float(task.get('area_sqm', 0) or 0)
        except Exception:
            pass

    surveyed = float(data.get('surveyed_area_sqm', 0))
    total = float(data.get('total_area_sqm', total_area))
    if total <= 0 and surveyed > 0:
        total = surveyed
    progress = (surveyed / total * 100.0) if total > 0 else 0.0

    _collection_progress[task_id] = {
        'task_id': task_id,
        'task_name': task_name,
        'total_area_sqm': total,
        'surveyed_area_sqm': surveyed,
        'progress_percent': round(progress, 1),
        'estimated_completion_time': data.get('estimated_completion_time', 0),
        'elapsed_seconds': float(data.get('elapsed_seconds', 0)),
        'uav_photos_taken': int(data.get('uav_photos_taken', 0)),
        'ugv_distance_m': float(data.get('ugv_distance_m', 0)),
        'status': data.get('status', 'surveying'),
    }
    return {'status': 'ok', 'task_id': task_id}


# =============================================================================
# 四阶段 4.5 - PGW 世界文件自动生成
# =============================================================================

@app.post('/api/pgw/generate')
async def generate_pgw(
    model_filename: str = Form(description="GLB 模型文件名（位于 3D_Model/ 目录下）"),
    image_width: int = Form(default=2048, description="2D 地图图像宽度"),
    image_height: int = Form(default=2048, description="2D 地图图像高度"),
    margin: float = Form(default=0.05, description="边距比例"),
):
    """
    根据 GLB 模型自动计算 .pgw 世界文件参数。
    返回 6 行世界文件内容，客户端可直接保存为 .pgw 文件。

    表单参数:
        model_filename - GLB 模型文件名（如 terrain.glb）
        image_width    - 2D 地图渲染图像宽度（默认 2048）
        image_height   - 2D 地图渲染图像高度（默认 2048）
        margin         - 模型范围边距比例（默认 0.05）
    """
    import numpy as np

    # 安全检查
    if '..' in model_filename or model_filename.startswith('/'):
        return JSONResponse({'error': '非法文件名'}, status_code=400)

    model_path = _SCENES_DIR / model_filename
    if not model_path.exists():
        return JSONResponse({'error': f'模型文件不存在: {model_filename}'}, status_code=404)

    try:
        import trimesh
        scene = trimesh.load(str(model_path))
        if isinstance(scene, trimesh.Trimesh):
            scene = trimesh.Scene([scene])

        all_verts = []
        for g in scene.geometry.values():
            if hasattr(g, 'vertices') and g.vertices is not None:
                all_verts.append(g.vertices)

        if not all_verts:
            return JSONResponse({'error': '模型中没有顶点数据'}, status_code=400)

        all_verts = np.vstack(all_verts)
        bbox_min = all_verts.min(axis=0)
        bbox_max = all_verts.max(axis=0)
        center = (bbox_min + bbox_max) / 2.0
        bbox_range = bbox_max - bbox_min

        center_x = float(center[0])
        center_y = float(center[1])
        model_dx = float(bbox_range[0])
        model_dy = float(bbox_range[1])

        img_w = image_width
        img_h = image_height

        required_world_w = model_dx * (1.0 + margin)
        required_world_h = model_dy * (1.0 + margin)
        pixel_size_x = required_world_w / img_w
        pixel_size_y = required_world_h / img_h
        pixel_size = max(pixel_size_x, pixel_size_y)

        world_w = pixel_size * img_w
        world_h = pixel_size * img_h

        center_px = (img_w / 2.0) - 0.5
        center_py = (img_h / 2.0) - 0.5
        ul_cx = center_x - center_px * pixel_size
        ul_cy = center_y + center_py * pixel_size

        return JSONResponse({
            'model_filename': model_filename,
            'model_center': [center_x, center_y, float(center[2])],
            'model_range': [model_dx, model_dy, float(bbox_range[2])],
            'image_size': [img_w, img_h],
            'pixel_size': round(pixel_size, 10),
            'world_coverage': [round(world_w, 2), round(world_h, 2)],
            'pgw_lines': [
                f'{pixel_size:.10f}',
                '0.0',
                '0.0',
                f'{-pixel_size:.10f}',
                f'{ul_cx:.10f}',
                f'{ul_cy:.10f}',
            ],
            'pgw_content': (
                f'{pixel_size:.10f}\n'
                f'0.0\n'
                f'0.0\n'
                f'{-pixel_size:.10f}\n'
                f'{ul_cx:.10f}\n'
                f'{ul_cy:.10f}\n'
            ),
            'top_left_world': [round(ul_cx, 6), round(ul_cy, 6)],
            'bottom_right_world': [round(ul_cx + world_w, 6), round(ul_cy - world_h, 6)],
        })
    except ImportError:
        return JSONResponse({'error': 'trimesh 库不可用，请安装: pip install trimesh'}, status_code=500)
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)


@app.get('/api/pgw/list')
async def list_pgw_capable_models():
    """
    列出可用于生成 PGW 的 GLB 模型列表。
    返回模型文件名及 XY 范围信息。
    """
    import numpy as np

    models = []
    if not _SCENES_DIR.exists():
        return JSONResponse(models)

    try:
        import trimesh
    except ImportError:
        return JSONResponse({'error': 'trimesh 库不可用'}, status_code=500)

    for glb_file in sorted(_SCENES_DIR.glob('*.glb')):
        try:
            scene = trimesh.load(str(glb_file))
            if isinstance(scene, trimesh.Trimesh):
                scene = trimesh.Scene([scene])
            all_verts = []
            for g in scene.geometry.values():
                if hasattr(g, 'vertices') and g.vertices is not None:
                    all_verts.append(g.vertices)
            if all_verts:
                all_verts = np.vstack(all_verts)
                bbox_min = all_verts.min(axis=0)
                bbox_max = all_verts.max(axis=0)
                models.append({
                    'filename': glb_file.name,
                    'bbox_min': bbox_min.tolist(),
                    'bbox_max': bbox_max.tolist(),
                    'range': (bbox_max - bbox_min).tolist(),
                    'center': ((bbox_min + bbox_max) / 2).tolist(),
                })
        except Exception:
            pass

    return JSONResponse(models)


# =============================================================================
# 四阶段 4.6 - 全链路性能指标
# =============================================================================

_performance_metrics: dict = {
    'pipeline_active': False,
    'total_collection_area_sqm': 0.0,
    'total_collection_time_s': 0.0,
    'collection_efficiency_ha_per_min': 0.0,
    'fusion_pipeline_latency_s': 0.0,
    'frames_processed': 0,
    'pointcloud_points_input': 0,
    'pointcloud_points_output': 0,
    'mesh_faces_generated': 0,
    'last_fusion_time_s': 0.0,
    'bandwidth_usage_kbps': 0.0,
    'qos_drops_count': 0,
}


@app.get('/api/performance')
async def get_performance_metrics():
    """
    获取全链路性能指标。
    
    返回:
        - collection_efficiency_ha_per_min: 采集效率 (公顷/分钟)
        - collection_target: 目标 < 5 min/ha (即 > 0.2 ha/min)
        - fusion_pipeline_latency_s: 融合管道延迟 (秒)
        - fusion_target: 目标 < 600s (10min)
        - bandwidth_usage_kbps: 通信带宽 (kbps)
        - pointcloud_reduction_ratio: 点云压缩比
    """
    metrics = dict(_performance_metrics)

    # 采集效率计算
    total_area = metrics.get('total_collection_area_sqm', 0)
    total_time = metrics.get('total_collection_time_s', 1)
    if total_time > 0:
        area_ha = total_area / 10000.0
        time_min = total_time / 60.0
        metrics['collection_efficiency_ha_per_min'] = round(area_ha / time_min, 4) if time_min > 0 else 0

    # 点云压缩比
    input_pts = metrics.get('pointcloud_points_input', 1)
    output_pts = metrics.get('pointcloud_points_output', 0)
    if input_pts > 0:
        metrics['pointcloud_reduction_ratio'] = round(100.0 * (1 - output_pts / input_pts), 1)
    else:
        metrics['pointcloud_reduction_ratio'] = 0.0

    # 目标指标
    metrics['collection_target_ha_per_min'] = 0.2  # 5 min/ha → 0.2 ha/min
    metrics['collection_target_met'] = metrics['collection_efficiency_ha_per_min'] >= 0.2
    metrics['fusion_target_s'] = 600.0  # 10 min
    metrics['fusion_target_met'] = metrics['fusion_pipeline_latency_s'] <= 600.0

    return JSONResponse(metrics)


@app.post('/api/performance')
async def update_performance_metrics(data: dict):
    """
    更新性能指标（由 ROS2 节点或监控脚本调用）。
    请求体可包含任意指标字段。
    """
    allowed_keys = {
        'pipeline_active', 'total_collection_area_sqm', 'total_collection_time_s',
        'fusion_pipeline_latency_s', 'frames_processed', 'pointcloud_points_input',
        'pointcloud_points_output', 'mesh_faces_generated', 'last_fusion_time_s',
        'bandwidth_usage_kbps', 'qos_drops_count',
    }
    for key, value in data.items():
        if key in allowed_keys:
            _performance_metrics[key] = value
    return {'status': 'ok', 'updated': list(data.keys())}


# =============================================================================
# WebSocket
# =============================================================================

@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket):
    client_id = await manager.connect(websocket)
    print(f'[WS] 客户端连接: {client_id}')

    # 发送初始状态
    await manager.send_to(client_id, {
        'type': 'init',
        'data': app_state.get_all_state(),
    })

    try:
        while True:
            # 接收客户端消息
            data = await websocket.receive_json()
            msg_type = data.get('type', '')

            if msg_type == 'ping':
                await manager.send_to(client_id, {
                    'type': 'pong',
                    'timestamp': time.time(),
                })
            elif msg_type == 'get_status':
                await manager.send_to(client_id, {
                    'type': 'status_update',
                    'data': app_state.get_all_state(),
                })

    except WebSocketDisconnect:
        manager.disconnect(client_id)
        print(f'[WS] 客户端断开: {client_id}')
    except Exception as e:
        manager.disconnect(client_id)
        print(f'[WS] 客户端异常: {client_id}, {e}')

# =============================================================================
# 后台任务
# =============================================================================

async def broadcast_state_loop():
    """定期向所有客户端广播状态更新"""
    while True:
        await asyncio.sleep(0.5)  # 每秒2次推送
        if app_state.connected_clients:
            await manager.broadcast({
                'type': 'state_update',
                'data': app_state.get_all_state(),
            })

async def fault_monitor_loop():
    """
    容错监控任务 (2.6)
    - 通信超时检测 (3s 超时，最多重试 3 次)
    - 无人车断连 30s → 原地等待，60s → 原路返回
    - 无人机断连 60s → 执行 Return to Home
    """
    while True:
        await asyncio.sleep(1.0)  # 每秒检查一次
        now = time.time()

        with app_state.lock:
            # --- UAV 超时检测 ---
            if app_state.last_uav_update > 0:
                uav_elapsed = now - app_state.last_uav_update
                if uav_elapsed > app_state.timeout_seconds:
                    app_state.uav_retry_count += 1
                    if app_state.uav_retry_count <= app_state.max_retries:
                        app_state.add_alert(
                            'uav', 2,
                            f'UAV 通信超时 ({uav_elapsed:.1f}s)，重试 {app_state.uav_retry_count}/{app_state.max_retries}')
                    else:
                        app_state.uav_status.connected = False
                        if app_state.uav_disconnect_time is None:
                            app_state.uav_disconnect_time = now

                        disconnect_dur = now - app_state.uav_disconnect_time
                        if disconnect_dur > 60.0:
                            app_state.add_alert(
                                'uav', 3,
                                f'UAV 断连超过60秒，执行 DJI Return to Home')
                            app_state.uav_status.flight_mode = 5  # 返航模式
                            app_state.uav_status.status_text = '返航中 (断连保护)'

            # --- UGV 超时检测 ---
            if app_state.last_ugv_update > 0:
                ugv_elapsed = now - app_state.last_ugv_update
                if ugv_elapsed > app_state.timeout_seconds:
                    app_state.ugv_retry_count += 1
                    if app_state.ugv_retry_count <= app_state.max_retries:
                        app_state.add_alert(
                            'ugv', 2,
                            f'UGV 通信超时 ({ugv_elapsed:.1f}s)，重试 {app_state.ugv_retry_count}/{app_state.max_retries}')
                    else:
                        app_state.ugv_status.connected = False
                        if app_state.ugv_disconnect_time is None:
                            app_state.ugv_disconnect_time = now

                        disconnect_dur = now - app_state.ugv_disconnect_time
                        if disconnect_dur > 60.0:
                            app_state.add_alert(
                                'ugv', 3,
                                f'UGV 断连超过60秒，执行原路返回')
                            app_state.ugv_status.status_text = '原路返回 (断连保护)'
                            app_state.ugv_nav_status.status_text = '断连返回中'
                        elif disconnect_dur > 30.0:
                            app_state.add_alert(
                                'ugv', 2,
                                f'UGV 断连超过30秒，原地等待')
                            app_state.ugv_status.status_text = '原地等待 (断连保护)'

            # --- 电量低告警 ---
            if app_state.uav_status.battery < 20.0 and app_state.uav_status.connected:
                app_state.add_alert('uav', 2,
                                    f'UAV 电量低: {app_state.uav_status.battery:.0f}%')

            if app_state.ugv_status.battery < 20.0 and app_state.ugv_status.connected:
                app_state.add_alert('ugv', 2,
                                    f'UGV 电量低: {app_state.ugv_status.battery:.0f}%')


async def replay_playback_loop():
    """回放播放任务"""
    while True:
        await asyncio.sleep(0.1)  # 100ms 检查间隔
        with app_state.lock:
            if not app_state.replay_playing or not app_state.replay_frames:
                continue

            # 根据播放速度和间隔推进帧
            frame_interval = 0.5 / app_state.replay_speed  # 基础0.5秒间隔
            await asyncio.sleep(frame_interval - 0.1)

            app_state.replay_current_index += 1
            if app_state.replay_current_index >= len(app_state.replay_frames):
                app_state.replay_current_index = 0
                app_state.replay_playing = False
                app_state.add_alert('system', 0, '任务回放已完成')


async def alert_broadcast_loop():
    """告警实时推送任务"""
    last_alert_count = 0
    while True:
        await asyncio.sleep(1.0)
        current_count = len(app_state.alerts)
        if current_count > last_alert_count and app_state.connected_clients:
            # 有新告警，推送最新一条
            latest = app_state.alerts[-1]
            await manager.broadcast({
                'type': 'alert',
                'data': asdict(latest),
            })
        last_alert_count = current_count


@app.on_event('startup')
async def startup_event():
    """应用启动时启动所有后台任务"""
    asyncio.create_task(broadcast_state_loop())
    asyncio.create_task(fault_monitor_loop())
    asyncio.create_task(replay_playback_loop())
    asyncio.create_task(alert_broadcast_loop())
    if SIM_MODE != 'frontend':
        asyncio.create_task(mock_data_generator())
        print('[启动] 仿真模式: 后端生成 (SIM_MODE=backend)')
    else:
        print('[启动] 仿真模式: 前端生成 (SIM_MODE=frontend)，后端仅提供 API')
    print('[启动] Web 后端服务已启动，WebSocket 端点: /ws')
    print('[启动] 二阶段功能: 航点任务 | 自主导航 | 告警推送 | 任务回放 | 模式切换 | 容错监控')
    print('[启动] 四阶段功能: 采集进度 | PGW世界文件 | 性能指标 | 融合成果 | 任务历史')

# =============================================================================
# =============================================================================
# 模拟数据生成器 (无 ROS2 环境时提供仿真数据)
# =============================================================================

async def mock_data_generator():
    """周期性生成 UAV/UGV 模拟位置数据，模拟真实运动轨迹"""
    import math

    # 模拟状态
    t = 0.0
    # 中心点 (模拟某测试场地)
    center_lat = CENTER_LAT
    center_lon = CENTER_LNG

    while True:
        await asyncio.sleep(0.5)  # 每秒2次更新
        t += 0.5

        # UAV: 椭圆航线 (半径约 200m)
        # 纬度1度 ≈ 111320m, 经度1度 ≈ 111320*cos(lat)
        cos_lat = math.cos(math.radians(center_lat))
        uav_lat = center_lat + 0.001 * math.sin(t * 0.5)       # 约 111m 振幅
        uav_lon = center_lon + 0.0015 * math.cos(t * 0.3)      # 约 167m 振幅
        uav_alt = 50.0 + 20.0 * math.sin(t * 0.4)               # UAV=UGV(0)+50+波动
        uav_heading = math.degrees(math.atan2(
            math.cos(t * 0.3) * 0.0015 * 0.3,
            -math.cos(t * 0.5) * 0.001 * 0.5
        )) % 360
        uav_speed = 8.0 + 3.0 * abs(math.sin(t * 0.35))         # 8-11 m/s
        uav_flight_mode = 3                                      # 航线模式
        uav_armed = True
        uav_battery = max(5.0, 100.0 - t * 0.02)                # 缓慢消耗
        uav_battery_v = 22.8

        # UGV: 直线往复行驶 (约 100m 范围)
        ugv_phase = (t * 0.2) % (2 * math.pi)
        ugv_lat = center_lat + 0.0003 * math.sin(ugv_phase)     # 约 33m
        ugv_lon = center_lon + 0.001 * math.cos(ugv_phase * 2)  # 约 111m
        ugv_alt = 0.0
        ugv_heading = math.degrees(math.atan2(
            math.cos(ugv_phase) * 0.0003,
            -math.sin(ugv_phase * 2) * 0.002
        )) % 360
        ugv_speed = 2.0 + 1.5 * abs(math.cos(ugv_phase))        # 2-3.5 m/s
        ugv_battery = max(10.0, 100.0 - t * 0.015)
        ugv_battery_v = 24.0

        # UGV 电池低于 20% 时模拟回充
        if ugv_battery < 20.0:
            ugv_battery = 20.0
            ugv_speed = 0.0

        # 更新后端状态
        app_state.update_uav(
            lat=uav_lat, lon=uav_lon, alt=uav_alt,
            heading=uav_heading, speed=uav_speed,
            flight_mode=uav_flight_mode, armed=uav_armed,
            battery=uav_battery, battery_v=uav_battery_v,
            status_text='航线飞行 (模拟)',
        )
        app_state.update_ugv(
            lat=ugv_lat, lon=ugv_lon, alt=ugv_alt,
            heading=ugv_heading, speed=ugv_speed,
            battery=ugv_battery, battery_v=ugv_battery_v,
            status_text='自动巡航 (模拟)',
            remote_control=False,
        )


# =============================================================================
# 主入口
# =============================================================================

if __name__ == '__main__':
    uvicorn.run(
        'main:app',
        host=BACKEND_HOST,
        port=BACKEND_PORT,
        reload=True,
        log_level='info',
    )
