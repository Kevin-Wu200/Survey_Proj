#!/usr/bin/env python3
"""
空地协同无人化智能测绘系统 - Web 后端服务
FastAPI + WebSocket + ROS2 数据桥接

启动方式: uvicorn main:app --host 0.0.0.0 --port 8000 --reload
"""

import asyncio
import json
import time
import threading
from typing import Dict, Set, Optional
from dataclasses import dataclass, field, asdict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import uvicorn

# =============================================================================
# 数据模型
# =============================================================================

@dataclass
class VehiclePosition:
    """载具位置"""
    id: str                     # uav / ugv
    latitude: float = 30.0
    longitude: float = 120.0
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
    latitude: float = 30.0
    longitude: float = 120.0
    altitude: float = 0.0
    last_update: float = 0.0

# =============================================================================
# 全局状态管理
# =============================================================================

class AppState:
    """应用全局状态"""
    def __init__(self):
        self.uav_position = VehiclePosition(id='uav')
        self.ugv_position = VehiclePosition(id='ugv')
        self.uav_status = VehicleStatus(id='uav')
        self.ugv_status = VehicleStatus(id='ugv')
        self.connected_clients: Dict[str, WebSocket] = {}
        self.lock = threading.Lock()

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

    def get_all_state(self) -> dict:
        with self.lock:
            return {
                'uav_position': asdict(self.uav_position),
                'ugv_position': asdict(self.ugv_position),
                'uav_status': asdict(self.uav_status),
                'ugv_status': asdict(self.ugv_status),
                'clients_count': len(self.connected_clients),
                'server_time': time.time(),
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
# REST API
# =============================================================================

@app.get('/')
async def root():
    return {'service': 'AirRunway Ground Station', 'version': '0.1.0'}

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
        lat=data.get('lat', 30.0),
        lon=data.get('lon', 120.0),
        alt=data.get('alt', 0.0),
        heading=data.get('heading', 0.0),
        speed=data.get('speed', 0.0),
        flight_mode=data.get('flight_mode', 0),
        armed=data.get('armed', False),
        battery=data.get('battery', 100.0),
        battery_v=data.get('battery_v', 22.8),
        status_text=data.get('status_text', '未知'),
    )
    return {'status': 'ok'}

@app.post('/api/ugv/update')
async def update_ugv(data: dict):
    """ROS2 桥接: 更新 UGV 状态"""
    app_state.update_ugv(
        lat=data.get('lat', 30.0),
        lon=data.get('lon', 120.0),
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
# 数据推送后台任务
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

@app.on_event('startup')
async def startup_event():
    """应用启动时启动广播任务和模拟数据生成器"""
    asyncio.create_task(broadcast_state_loop())
    asyncio.create_task(mock_data_generator())
    print('[启动] Web 后端服务已启动，WebSocket 端点: /ws')
    print('[启动] 模拟数据生成器已启动 (UAV椭圆航线 + UGV往复行驶)')

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
    center_lat = 30.0
    center_lon = 120.0

    while True:
        await asyncio.sleep(0.5)  # 每秒2次更新
        t += 0.5

        # UAV: 椭圆航线 (半径约 200m)
        # 纬度1度 ≈ 111320m, 经度1度 ≈ 111320*cos(lat)
        cos_lat = math.cos(math.radians(center_lat))
        uav_lat = center_lat + 0.001 * math.sin(t * 0.5)       # 约 111m 振幅
        uav_lon = center_lon + 0.0015 * math.cos(t * 0.3)      # 约 167m 振幅
        uav_alt = 50.0 + 20.0 * math.sin(t * 0.4)               # 50-70m 高度
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
        host='0.0.0.0',
        port=8000,
        reload=True,
        log_level='info',
    )
