"""
空地协同无人化智能测绘系统 - 数据持久化层
DatabaseManager: PostgreSQL + PostGIS 数据库读写操作封装

功能:
  - 测绘任务 CRUD
  - 航点批量导入
  - 轨迹点实时写入与批量导入
  - 三维模型元数据管理
  - 融合成果存储
  - 任务统计信息查询

配置来源: 项目根目录 env.txt (DB_HOST, DB_PORT, DB_NAME, DB_USER, DB_PASSWORD)
依赖: psycopg2 (PostgreSQL 15 + PostGIS 3)
"""

import logging
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional

import psycopg2
import psycopg2.extras
from psycopg2 import OperationalError, sql

# =============================================================================
# 日志配置
# =============================================================================
logger = logging.getLogger(__name__)


# =============================================================================
# 配置文件加载 (env.txt)
# =============================================================================

def _load_db_config() -> Dict[str, Any]:
    """
    从项目根目录 env.txt 加载数据库配置。
    优先级: 环境变量 > env.txt > 默认值
    """
    import os

    config: Dict[str, Any] = {
        'host': '127.0.0.1',
        'port': 5432,
        'dbname': 'air_survey_db',
        'user': 'air_survey',
        'password': 'air_survey_2024',
    }

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
                        key = key.strip()
                        value = value.strip()
                        if key == 'DB_HOST':
                            config['host'] = value
                        elif key == 'DB_PORT':
                            config['port'] = int(value)
                        elif key == 'DB_NAME':
                            config['dbname'] = value
                        elif key == 'DB_USER':
                            config['user'] = value
                        elif key == 'DB_PASSWORD':
                            config['password'] = value
            break

    # 环境变量覆盖
    config['host'] = os.environ.get('DB_HOST', config['host'])
    config['port'] = int(os.environ.get('DB_PORT', config['port']))
    config['dbname'] = os.environ.get('DB_NAME', config['dbname'])
    config['user'] = os.environ.get('DB_USER', config['user'])
    config['password'] = os.environ.get('DB_PASSWORD', config['password'])

    return config


# =============================================================================
# DatabaseManager 类
# =============================================================================

class DatabaseManager:
    """
    PostgreSQL + PostGIS 数据库管理器。

    提供测绘任务、航点、轨迹、模型元数据、融合成果的完整 CRUD 操作。
    支持上下文管理器 (with 语句)，自动处理连接和事务。

    使用示例:
        db = DatabaseManager()
        with db:
            task_id = db.create_task({'task_name': '测试任务', 'task_type': 'fusion'})

        # 或手动管理:
        db.connect()
        try:
            tasks = db.list_tasks()
        finally:
            db.disconnect()
    """

    # 最大重试次数
    _MAX_RETRIES = 3
    # 重试间隔 (秒)
    _RETRY_DELAY = 1.0

    def __init__(self, db_config: Optional[Dict[str, Any]] = None):
        """
        初始化数据库管理器。

        参数:
            db_config: 数据库连接配置字典，包含 host/port/dbname/user/password。
                      若为 None，则从 env.txt 和环境变量自动加载。
        """
        self._config = db_config if db_config else _load_db_config()
        self._conn: Optional[psycopg2.extensions.connection] = None

    # -------------------------------------------------------------------------
    # 连接管理
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """建立数据库连接，失败时自动重试。"""
        for attempt in range(1, self._MAX_RETRIES + 1):
            try:
                self._conn = psycopg2.connect(
                    host=self._config['host'],
                    port=self._config['port'],
                    dbname=self._config['dbname'],
                    user=self._config['user'],
                    password=self._config['password'],
                )
                # 使用 psycopg2.extras.RealDictCursor 返回字典格式结果
                self._conn.cursor_factory = psycopg2.extras.RealDictCursor
                # 自动提交默认关闭，由方法内部控制事务
                self._conn.autocommit = False
                logger.info("数据库连接成功: %s@%s:%s/%s",
                            self._config['user'], self._config['host'],
                            self._config['port'], self._config['dbname'])
                return
            except OperationalError as e:
                logger.warning("数据库连接失败 (第%d/%d次): %s",
                               attempt, self._MAX_RETRIES, e)
                if attempt < self._MAX_RETRIES:
                    time.sleep(self._RETRY_DELAY * attempt)
                else:
                    raise

    def disconnect(self) -> None:
        """关闭数据库连接。"""
        if self._conn and not self._conn.closed:
            self._conn.close()
            logger.info("数据库连接已关闭")

    @property
    def is_connected(self) -> bool:
        """检查数据库是否已连接。"""
        return self._conn is not None and not self._conn.closed

    # -------------------------------------------------------------------------
    # 上下文管理器 (支持 with 语句)
    # -------------------------------------------------------------------------

    def __enter__(self) -> "DatabaseManager":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            # 发生异常时回滚事务
            if self._conn and not self._conn.closed:
                try:
                    self._conn.rollback()
                except Exception:
                    pass
        else:
            # 正常退出时提交事务
            if self._conn and not self._conn.closed:
                try:
                    self._conn.commit()
                except Exception:
                    pass
        self.disconnect()
        # 返回 False 传播异常
        return False

    @contextmanager
    def transaction(self):
        """
        事务上下文管理器，自动提交/回滚。

        使用示例:
            with db.transaction():
                db.create_task(...)
                db.insert_waypoints(...)
        """
        if not self.is_connected:
            self.connect()
        try:
            yield self._conn
            self._conn.commit()
        except Exception:
            self._conn.rollback()
            raise

    # -------------------------------------------------------------------------
    # 测绘任务 CRUD
    # -------------------------------------------------------------------------

    def create_task(self, task_data: Dict[str, Any]) -> int:
        """
        创建测绘任务。

        参数:
            task_data: 任务数据字典，包含:
                - task_name (str): 任务名称 (必填)
                - task_type (str): 任务类型 'uav_survey'/'ugv_survey'/'fusion'
                - uav_id / ugv_id (str, 可选): 载具编号
                - area_sqm (float, 可选): 测绘面积
                - notes (str, 可选): 备注
                - metadata (dict, 可选): 扩展元数据

        返回:
            int: 新创建任务的 ID
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        import json

        query = """
            INSERT INTO survey_tasks (task_name, task_type, uav_id, ugv_id,
                                      area_sqm, notes, metadata)
            VALUES (%(task_name)s, %(task_type)s, %(uav_id)s, %(ugv_id)s,
                    %(area_sqm)s, %(notes)s, %(metadata)s)
            RETURNING id
        """
        params = {
            'task_name': task_data['task_name'],
            'task_type': task_data.get('task_type', 'fusion'),
            'uav_id': task_data.get('uav_id'),
            'ugv_id': task_data.get('ugv_id'),
            'area_sqm': task_data.get('area_sqm'),
            'notes': task_data.get('notes'),
            'metadata': json.dumps(task_data.get('metadata', {})),
        }

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            result = cur.fetchone()
            task_id = result['id'] if isinstance(result, dict) else result[0]
            logger.info("创建测绘任务: id=%d, name=%s", task_id, params['task_name'])
            return task_id

    def update_task_status(self, task_id: int, status: str,
                           completed_at: Optional[float] = None) -> None:
        """
        更新任务状态。

        参数:
            task_id: 任务 ID
            status: 新状态 ('pending', 'running', 'completed', 'failed')
            completed_at: 完成时间戳 (Unix 秒)，仅 completed/failed 状态有效
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        import datetime

        query = """
            UPDATE survey_tasks
            SET status = %(status)s,
                started_at = CASE WHEN %(status)s = 'running' AND started_at IS NULL
                                  THEN NOW() ELSE started_at END,
                completed_at = CASE WHEN %(status)s IN ('completed', 'failed')
                                    THEN %(completed_at)s ELSE completed_at END
            WHERE id = %(task_id)s
        """
        params = {
            'task_id': task_id,
            'status': status,
            'completed_at': datetime.datetime.fromtimestamp(completed_at) if completed_at else None,
        }

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            logger.info("更新任务状态: id=%d, status=%s", task_id, status)

    def get_task(self, task_id: int) -> Optional[Dict[str, Any]]:
        """
        查询单个任务详情。

        参数:
            task_id: 任务 ID

        返回:
            dict: 任务信息，若不存在则返回 None
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = "SELECT * FROM survey_tasks WHERE id = %(task_id)s"

        with self._conn.cursor() as cur:
            cur.execute(query, {'task_id': task_id})
            result = cur.fetchone()
            if result:
                return dict(result)
            return None

    def list_tasks(self, status: Optional[str] = None,
                   limit: int = 50, offset: int = 0) -> List[Dict[str, Any]]:
        """
        列出任务，支持状态过滤和分页。

        参数:
            status: 按状态过滤 (可选)
            limit: 每页数量
            offset: 偏移量

        返回:
            List[dict]: 任务列表
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        if status:
            query = """
                SELECT * FROM survey_tasks
                WHERE status = %(status)s
                ORDER BY created_at DESC
                LIMIT %(limit)s OFFSET %(offset)s
            """
            params = {'status': status, 'limit': limit, 'offset': offset}
        else:
            query = """
                SELECT * FROM survey_tasks
                ORDER BY created_at DESC
                LIMIT %(limit)s OFFSET %(offset)s
            """
            params = {'limit': limit, 'offset': offset}

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            return [dict(row) for row in cur.fetchall()]

    # -------------------------------------------------------------------------
    # 航点管理
    # -------------------------------------------------------------------------

    def insert_waypoints(self, task_id: int,
                         waypoints: List[Dict[str, Any]]) -> int:
        """
        批量插入航点。

        参数:
            task_id: 所属任务 ID
            waypoints: 航点列表，每个元素包含:
                - sequence_index (int): 序号
                - latitude (float): 纬度
                - longitude (float): 经度
                - altitude (float): 高度 (米)
                - speed (float, 可选): 速度
                - heading (float, 可选): 航向角
                - action (str, 可选): 动作类型

        返回:
            int: 插入的航点数量
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = """
            INSERT INTO waypoints (task_id, sequence_index, latitude, longitude,
                                   altitude, speed, heading, action, geom)
            VALUES (%(task_id)s, %(sequence_index)s, %(latitude)s, %(longitude)s,
                    %(altitude)s, %(speed)s, %(heading)s, %(action)s,
                    ST_SetSRID(ST_MakePoint(%(longitude)s, %(latitude)s), 4326))
        """
        batch = []
        for wp in waypoints:
            batch.append({
                'task_id': task_id,
                'sequence_index': wp['sequence_index'],
                'latitude': wp['latitude'],
                'longitude': wp['longitude'],
                'altitude': wp.get('altitude', 50.0),
                'speed': wp.get('speed'),
                'heading': wp.get('heading'),
                'action': wp.get('action', 'photo'),
            })

        with self._conn.cursor() as cur:
            cur.executemany(query, batch)
            count = cur.rowcount
            logger.info("批量插入航点: task_id=%d, count=%d", task_id, count)
            return count

    def get_waypoints(self, task_id: int) -> List[Dict[str, Any]]:
        """
        获取指定任务的所有航点，按序号排序。

        参数:
            task_id: 任务 ID

        返回:
            List[dict]: 航点列表 (按 sequence_index 升序)
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = """
            SELECT id, task_id, sequence_index, latitude, longitude, altitude,
                   speed, heading, action, reached, reached_at
            FROM waypoints
            WHERE task_id = %(task_id)s
            ORDER BY sequence_index ASC
        """
        with self._conn.cursor() as cur:
            cur.execute(query, {'task_id': task_id})
            return [dict(row) for row in cur.fetchall()]

    # -------------------------------------------------------------------------
    # 轨迹管理
    # -------------------------------------------------------------------------

    def insert_trajectory_point(self, task_id: int, vehicle_type: str,
                                point_data: Dict[str, Any]) -> int:
        """
        插入单个轨迹点。

        参数:
            task_id: 所属任务 ID
            vehicle_type: 载具类型 ('uav' / 'ugv')
            point_data: 轨迹点数据:
                - timestamp (float): Unix 时间戳 (秒)
                - latitude (float): 纬度
                - longitude (float): 经度
                - altitude (float): 海拔高度
                - roll/pitch/yaw (float, 可选): 姿态角

        返回:
            int: 插入的轨迹点 ID
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = """
            INSERT INTO trajectories (task_id, vehicle_type, timestamp,
                                      latitude, longitude, altitude,
                                      roll, pitch, yaw, geom)
            VALUES (%(task_id)s, %(vehicle_type)s, %(timestamp)s,
                    %(latitude)s, %(longitude)s, %(altitude)s,
                    %(roll)s, %(pitch)s, %(yaw)s,
                    ST_SetSRID(ST_MakePoint(%(longitude)s, %(latitude)s, %(altitude)s), 4326))
            RETURNING id
        """
        params = {
            'task_id': task_id,
            'vehicle_type': vehicle_type,
            'timestamp': point_data['timestamp'],
            'latitude': point_data['latitude'],
            'longitude': point_data['longitude'],
            'altitude': point_data.get('altitude', 0.0),
            'roll': point_data.get('roll', 0.0),
            'pitch': point_data.get('pitch', 0.0),
            'yaw': point_data.get('yaw', 0.0),
        }

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            result = cur.fetchone()
            return result['id'] if isinstance(result, dict) else result[0]

    def insert_trajectory_batch(self, task_id: int, vehicle_type: str,
                                points: List[Dict[str, Any]]) -> int:
        """
        批量插入轨迹点。

        参数:
            task_id: 所属任务 ID
            vehicle_type: 载具类型 ('uav' / 'ugv')
            points: 轨迹点列表 (格式同 insert_trajectory_point)

        返回:
            int: 插入的轨迹点数量
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = """
            INSERT INTO trajectories (task_id, vehicle_type, timestamp,
                                      latitude, longitude, altitude,
                                      roll, pitch, yaw, geom)
            VALUES (%(task_id)s, %(vehicle_type)s, %(timestamp)s,
                    %(latitude)s, %(longitude)s, %(altitude)s,
                    %(roll)s, %(pitch)s, %(yaw)s,
                    ST_SetSRID(ST_MakePoint(%(longitude)s, %(latitude)s, %(altitude)s), 4326))
        """
        batch = []
        for pt in points:
            batch.append({
                'task_id': task_id,
                'vehicle_type': vehicle_type,
                'timestamp': pt['timestamp'],
                'latitude': pt['latitude'],
                'longitude': pt['longitude'],
                'altitude': pt.get('altitude', 0.0),
                'roll': pt.get('roll', 0.0),
                'pitch': pt.get('pitch', 0.0),
                'yaw': pt.get('yaw', 0.0),
            })

        with self._conn.cursor() as cur:
            cur.executemany(query, batch)
            count = cur.rowcount
            logger.info("批量插入轨迹: task_id=%d, vehicle=%s, count=%d",
                        task_id, vehicle_type, count)
            return count

    def get_task_trajectory(self, task_id: int,
                            vehicle_type: Optional[str] = None
                            ) -> List[Dict[str, Any]]:
        """
        获取指定任务的轨迹数据。

        参数:
            task_id: 任务 ID
            vehicle_type: 载具类型过滤 (可选, 'uav' / 'ugv')

        返回:
            List[dict]: 轨迹点列表 (按时间戳升序)
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        if vehicle_type:
            query = """
                SELECT id, task_id, vehicle_type, timestamp,
                       latitude, longitude, altitude,
                       roll, pitch, yaw
                FROM trajectories
                WHERE task_id = %(task_id)s AND vehicle_type = %(vehicle_type)s
                ORDER BY timestamp ASC
            """
            params = {'task_id': task_id, 'vehicle_type': vehicle_type}
        else:
            query = """
                SELECT id, task_id, vehicle_type, timestamp,
                       latitude, longitude, altitude,
                       roll, pitch, yaw
                FROM trajectories
                WHERE task_id = %(task_id)s
                ORDER BY vehicle_type, timestamp ASC
            """
            params = {'task_id': task_id}

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            return [dict(row) for row in cur.fetchall()]

    # -------------------------------------------------------------------------
    # 三维模型元数据
    # -------------------------------------------------------------------------

    def save_model_metadata(self, model_data: Dict[str, Any]) -> int:
        """
        保存三维模型元数据。

        参数:
            model_data: 模型数据字典:
                - task_id (int, 可选): 所属任务 ID
                - model_name (str): 模型名称
                - model_type (str): 'pointcloud'/'mesh'/'textured_mesh'
                - file_path (str): 文件系统路径
                - file_size (int, 可选): 文件大小
                - point_count (int, 可选): 点云点数
                - face_count (int, 可选): 面片数
                - bbox_min (tuple, 可选): 包围盒最小值 (x, y, z)
                - bbox_max (tuple, 可选): 包围盒最大值 (x, y, z)
                - registration_rmse (float, 可选): 配准 RMSE
                - metadata (dict, 可选): 扩展元数据

        返回:
            int: 模型元数据记录 ID
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        import json

        # 构建包围盒几何对象
        bbox_min_geom = None
        bbox_max_geom = None
        if model_data.get('bbox_min'):
            x, y, z = model_data['bbox_min']
            bbox_min_geom = f"ST_SetSRID(ST_MakePoint({x}, {y}, {z}), 4326)"
        if model_data.get('bbox_max'):
            x, y, z = model_data['bbox_max']
            bbox_max_geom = f"ST_SetSRID(ST_MakePoint({x}, {y}, {z}), 4326)"

        query = sql.SQL("""
            INSERT INTO model_metadata (task_id, model_name, model_type,
                                        file_path, file_size, point_count,
                                        face_count, bbox_min, bbox_max,
                                        registration_rmse, metadata)
            VALUES (%(task_id)s, %(model_name)s, %(model_type)s,
                    %(file_path)s, %(file_size)s, %(point_count)s,
                    %(face_count)s,
                    {bbox_min},
                    {bbox_max},
                    %(registration_rmse)s, %(metadata)s)
            RETURNING id
        """).format(
            bbox_min=sql.SQL(bbox_min_geom) if bbox_min_geom else sql.SQL('NULL'),
            bbox_max=sql.SQL(bbox_max_geom) if bbox_max_geom else sql.SQL('NULL'),
        )

        params = {
            'task_id': model_data.get('task_id'),
            'model_name': model_data['model_name'],
            'model_type': model_data['model_type'],
            'file_path': model_data['file_path'],
            'file_size': model_data.get('file_size'),
            'point_count': model_data.get('point_count'),
            'face_count': model_data.get('face_count'),
            'registration_rmse': model_data.get('registration_rmse'),
            'metadata': json.dumps(model_data.get('metadata', {})),
        }

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            result = cur.fetchone()
            model_id = result['id'] if isinstance(result, dict) else result[0]
            logger.info("保存模型元数据: id=%d, name=%s", model_id, params['model_name'])
            return model_id

    def get_models(self, task_id: Optional[int] = None,
                   model_type: Optional[str] = None,
                   limit: int = 50) -> List[Dict[str, Any]]:
        """
        查询三维模型列表。

        参数:
            task_id: 按任务过滤 (可选)
            model_type: 按类型过滤 (可选)
            limit: 返回数量限制

        返回:
            List[dict]: 模型元数据列表
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        conditions = []
        params: Dict[str, Any] = {'limit': limit}

        if task_id is not None:
            conditions.append('task_id = %(task_id)s')
            params['task_id'] = task_id
        if model_type:
            conditions.append('model_type = %(model_type)s')
            params['model_type'] = model_type

        where_clause = ('WHERE ' + ' AND '.join(conditions)) if conditions else ''
        query = f"""
            SELECT id, task_id, model_name, model_type, file_path,
                   file_size, point_count, face_count, registration_rmse,
                   created_at, metadata
            FROM model_metadata
            {where_clause}
            ORDER BY created_at DESC
            LIMIT %(limit)s
        """

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            return [dict(row) for row in cur.fetchall()]

    def get_model_by_id(self, model_id: int) -> Optional[Dict[str, Any]]:
        """
        按 ID 查询单个模型元数据。

        参数:
            model_id: 模型 ID

        返回:
            dict: 模型元数据，若不存在则返回 None
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        query = "SELECT * FROM model_metadata WHERE id = %(model_id)s"

        with self._conn.cursor() as cur:
            cur.execute(query, {'model_id': model_id})
            result = cur.fetchone()
            if result:
                return dict(result)
            return None

    # -------------------------------------------------------------------------
    # 融合成果
    # -------------------------------------------------------------------------

    def save_fusion_result(self, fusion_data: Dict[str, Any]) -> int:
        """
        保存空地融合成果。

        参数:
            fusion_data: 融合数据字典:
                - task_id (int): 所属任务 ID
                - uav_pointcloud_path (str, 可选): UAV 点云路径
                - ugv_pointcloud_path (str, 可选): UGV 点云路径
                - fused_pointcloud_path (str, 可选): 融合点云路径
                - mesh_path (str, 可选): 网格文件路径
                - coarse_rmse (float, 可选): 粗配准 RMSE
                - fine_rmse (float, 可选): 精配准 RMSE
                - icp_iterations (int, 可选): ICP 迭代次数
                - transform_matrix (list[float], 可选): 16元素变换矩阵
                - metadata (dict, 可选): 扩展元数据

        返回:
            int: 融合成果记录 ID
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        import json

        query = """
            INSERT INTO fusion_results (task_id, uav_pointcloud_path,
                                        ugv_pointcloud_path,
                                        fused_pointcloud_path, mesh_path,
                                        coarse_rmse, fine_rmse,
                                        icp_iterations, transform_matrix,
                                        metadata)
            VALUES (%(task_id)s, %(uav_pointcloud_path)s,
                    %(ugv_pointcloud_path)s, %(fused_pointcloud_path)s,
                    %(mesh_path)s, %(coarse_rmse)s, %(fine_rmse)s,
                    %(icp_iterations)s, %(transform_matrix)s, %(metadata)s)
            RETURNING id
        """
        params = {
            'task_id': fusion_data.get('task_id'),
            'uav_pointcloud_path': fusion_data.get('uav_pointcloud_path'),
            'ugv_pointcloud_path': fusion_data.get('ugv_pointcloud_path'),
            'fused_pointcloud_path': fusion_data.get('fused_pointcloud_path'),
            'mesh_path': fusion_data.get('mesh_path'),
            'coarse_rmse': fusion_data.get('coarse_rmse'),
            'fine_rmse': fusion_data.get('fine_rmse'),
            'icp_iterations': fusion_data.get('icp_iterations'),
            'transform_matrix': fusion_data.get('transform_matrix'),
            'metadata': json.dumps(fusion_data.get('metadata', {})),
        }

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            result = cur.fetchone()
            fusion_id = result['id'] if isinstance(result, dict) else result[0]
            logger.info("保存融合成果: id=%d, task_id=%s", fusion_id, params['task_id'])
            return fusion_id

    def get_fusion_results(self, task_id: Optional[int] = None,
                           limit: int = 50) -> List[Dict[str, Any]]:
        """
        获取融合成果列表。

        参数:
            task_id: 按任务过滤 (可选)
            limit: 返回数量限制

        返回:
            List[dict]: 融合成果列表
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        if task_id is not None:
            query = """
                SELECT * FROM fusion_results
                WHERE task_id = %(task_id)s
                ORDER BY completed_at DESC
                LIMIT %(limit)s
            """
            params = {'task_id': task_id, 'limit': limit}
        else:
            query = """
                SELECT * FROM fusion_results
                ORDER BY completed_at DESC
                LIMIT %(limit)s
            """
            params = {'limit': limit}

        with self._conn.cursor() as cur:
            cur.execute(query, params)
            return [dict(row) for row in cur.fetchall()]

    # -------------------------------------------------------------------------
    # 统计信息
    # -------------------------------------------------------------------------

    def get_task_statistics(self, task_id: int) -> Dict[str, Any]:
        """
        获取任务统计信息。

        包含:
            - 航点总数和已到达数
            - UAV/UGV 轨迹点数量
            - 轨迹时间范围
            - 关联模型和融合成果数量

        参数:
            task_id: 任务 ID

        返回:
            dict: 统计信息
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        stats: Dict[str, Any] = {
            'task_id': task_id,
            'waypoint_total': 0,
            'waypoint_reached': 0,
            'uav_trajectory_points': 0,
            'ugv_trajectory_points': 0,
            'trajectory_span_seconds': 0,
            'model_count': 0,
            'fusion_count': 0,
        }

        with self._conn.cursor() as cur:
            # 航点统计
            cur.execute("""
                SELECT COUNT(*) AS total,
                       SUM(CASE WHEN reached THEN 1 ELSE 0 END) AS reached
                FROM waypoints WHERE task_id = %(task_id)s
            """, {'task_id': task_id})
            row = cur.fetchone()
            if row:
                stats['waypoint_total'] = row['total'] or 0
                stats['waypoint_reached'] = row['reached'] or 0

            # 轨迹统计
            cur.execute("""
                SELECT vehicle_type, COUNT(*) AS cnt
                FROM trajectories WHERE task_id = %(task_id)s
                GROUP BY vehicle_type
            """, {'task_id': task_id})
            for row in cur.fetchall():
                if row['vehicle_type'] == 'uav':
                    stats['uav_trajectory_points'] = row['cnt']
                elif row['vehicle_type'] == 'ugv':
                    stats['ugv_trajectory_points'] = row['cnt']

            # 轨迹时间跨度
            cur.execute("""
                SELECT MIN(timestamp) AS min_ts, MAX(timestamp) AS max_ts
                FROM trajectories WHERE task_id = %(task_id)s
            """, {'task_id': task_id})
            row = cur.fetchone()
            if row and row['min_ts'] and row['max_ts']:
                stats['trajectory_span_seconds'] = row['max_ts'] - row['min_ts']

            # 模型数量
            cur.execute("""
                SELECT COUNT(*) AS cnt FROM model_metadata
                WHERE task_id = %(task_id)s
            """, {'task_id': task_id})
            row = cur.fetchone()
            if row:
                stats['model_count'] = row['cnt']

            # 融合成果数量
            cur.execute("""
                SELECT COUNT(*) AS cnt FROM fusion_results
                WHERE task_id = %(task_id)s
            """, {'task_id': task_id})
            row = cur.fetchone()
            if row:
                stats['fusion_count'] = row['cnt']

        return stats

    def get_system_statistics(self) -> Dict[str, Any]:
        """
        获取系统整体统计信息。

        返回:
            dict: 包含任务总数、总面积、最近任务、各状态计数
        """
        if not self.is_connected:
            raise RuntimeError("数据库未连接，请先调用 connect()")

        system_stats: Dict[str, Any] = {}

        with self._conn.cursor() as cur:
            # 任务总数
            cur.execute("SELECT COUNT(*) AS cnt FROM survey_tasks")
            system_stats['total_tasks'] = cur.fetchone()['cnt']

            # 测绘总面积 (已完成任务)
            cur.execute("""
                SELECT COALESCE(SUM(area_sqm), 0) AS total_area
                FROM survey_tasks WHERE status = 'completed'
            """)
            system_stats['total_area_sqm'] = cur.fetchone()['total_area']

            # 各状态任务数量
            cur.execute("""
                SELECT status, COUNT(*) AS cnt
                FROM survey_tasks
                GROUP BY status
            """)
            status_counts = {row['status']: row['cnt'] for row in cur.fetchall()}
            system_stats['status_counts'] = status_counts

            # 最近任务 (最新5条)
            cur.execute("""
                SELECT id, task_name, task_type, status, created_at
                FROM survey_tasks
                ORDER BY created_at DESC LIMIT 5
            """)
            system_stats['recent_tasks'] = [dict(row) for row in cur.fetchall()]

            # 轨迹点总数
            cur.execute("SELECT COUNT(*) AS cnt FROM trajectories")
            system_stats['total_trajectory_points'] = cur.fetchone()['cnt']

            # 模型总数
            cur.execute("SELECT COUNT(*) AS cnt FROM model_metadata")
            system_stats['total_models'] = cur.fetchone()['cnt']

            # 融合成果总数
            cur.execute("SELECT COUNT(*) AS cnt FROM fusion_results")
            system_stats['total_fusions'] = cur.fetchone()['cnt']

        return system_stats
