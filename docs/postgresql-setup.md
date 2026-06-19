# PostgreSQL + PostGIS 安装配置文档

空地协同无人化智能测绘系统 —— 数据持久化层数据库安装与配置指南。

---

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS (x86_64) |
| PostgreSQL | 15.x |
| PostGIS | 3.x |
| 内存 | 建议 ≥ 4GB |
| 磁盘 | 建议 ≥ 20GB 可用空间 |
| 网络 | 需要访问 PostgreSQL APT 仓库 |

## 2. 快速安装

使用项目提供的自动化安装脚本：

```bash
sudo bash scripts/setup_postgresql.sh
```

脚本会自动完成以下步骤：
1. 添加 PostgreSQL 官方 APT 仓库
2. 安装 PostgreSQL 15 及相关组件
3. 安装 PostGIS 3 空间扩展
4. 创建数据库用户 `air_survey`
5. 创建数据库 `air_survey_db`
6. 启用 PostGIS 扩展
7. 创建测绘数据表结构和索引
8. 配置 `pg_hba.conf` 认证规则

## 3. 手动安装步骤

### 3.1 添加 PostgreSQL APT 仓库

```bash
# 安装前置依赖
sudo apt-get update
sudo apt-get install -y curl ca-certificates gnupg lsb-release

# 添加 GPG 密钥
curl -fsSL https://www.postgresql.org/media/keys/ACCC4CF8.asc | \
    sudo gpg --dearmor -o /usr/share/keyrings/postgresql-archive-keyring.gpg

# 添加 APT 源
echo "deb [signed-by=/usr/share/keyrings/postgresql-archive-keyring.gpg] \
http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg main" | \
    sudo tee /etc/apt/sources.list.d/pgdg.list

sudo apt-get update
```

### 3.2 安装 PostgreSQL 15

```bash
sudo apt-get install -y postgresql-15 postgresql-contrib
```

安装后服务自动启动，验证：

```bash
sudo systemctl status postgresql
```

### 3.3 安装 PostGIS 3

```bash
sudo apt-get install -y postgresql-15-postgis-3 postgresql-15-postgis-3-scripts
sudo apt-get install -y postgis gdal-bin
```

### 3.4 创建数据库用户和数据库

```bash
# 切换到 postgres 系统用户
sudo -u postgres psql
```

在 psql 中执行：

```sql
-- 创建用户
CREATE USER air_survey WITH PASSWORD 'your_secure_password' CREATEDB;

-- 创建数据库
CREATE DATABASE air_survey_db OWNER air_survey;

-- 授权
GRANT ALL PRIVILEGES ON DATABASE air_survey_db TO air_survey;
```

退出 psql：`\q`

### 3.5 启用 PostGIS 扩展

```bash
sudo -u postgres psql -d air_survey_db
```

```sql
CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS postgis_topology;

-- 验证安装
SELECT PostGIS_Full_Version();
```

### 3.6 创建数据表结构

在 `air_survey_db` 数据库中执行以下 SQL：

```sql
-- 测绘任务表
CREATE TABLE IF NOT EXISTS survey_tasks (
    id SERIAL PRIMARY KEY,
    task_name VARCHAR(255) NOT NULL,
    task_type VARCHAR(50) NOT NULL,
    status VARCHAR(50) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT NOW(),
    started_at TIMESTAMP,
    completed_at TIMESTAMP,
    uav_id VARCHAR(100),
    ugv_id VARCHAR(100),
    area_sqm DOUBLE PRECISION,
    notes TEXT,
    metadata JSONB DEFAULT '{}'
);

-- 航点表
CREATE TABLE IF NOT EXISTS waypoints (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,
    sequence_index INTEGER NOT NULL,
    latitude DOUBLE PRECISION NOT NULL,
    longitude DOUBLE PRECISION NOT NULL,
    altitude DOUBLE PRECISION NOT NULL,
    speed DOUBLE PRECISION,
    heading DOUBLE PRECISION,
    action VARCHAR(50),
    reached BOOLEAN DEFAULT FALSE,
    reached_at TIMESTAMP,
    geom GEOMETRY(Point, 4326)
);

-- 轨迹表
CREATE TABLE IF NOT EXISTS trajectories (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,
    vehicle_type VARCHAR(10) NOT NULL,
    timestamp DOUBLE PRECISION NOT NULL,
    latitude DOUBLE PRECISION NOT NULL,
    longitude DOUBLE PRECISION NOT NULL,
    altitude DOUBLE PRECISION NOT NULL,
    roll DOUBLE PRECISION DEFAULT 0,
    pitch DOUBLE PRECISION DEFAULT 0,
    yaw DOUBLE PRECISION DEFAULT 0,
    geom GEOMETRY(PointZ, 4326)
);

-- 三维模型元数据表
CREATE TABLE IF NOT EXISTS model_metadata (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,
    model_name VARCHAR(255) NOT NULL,
    model_type VARCHAR(50) NOT NULL,
    file_path VARCHAR(500) NOT NULL,
    file_size BIGINT,
    point_count BIGINT,
    face_count BIGINT,
    bbox_min GEOMETRY(PointZ, 4326),
    bbox_max GEOMETRY(PointZ, 4326),
    registration_rmse DOUBLE PRECISION,
    created_at TIMESTAMP DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'
);

-- 融合成果表
CREATE TABLE IF NOT EXISTS fusion_results (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,
    uav_pointcloud_path VARCHAR(500),
    ugv_pointcloud_path VARCHAR(500),
    fused_pointcloud_path VARCHAR(500),
    mesh_path VARCHAR(500),
    coarse_rmse DOUBLE PRECISION,
    fine_rmse DOUBLE PRECISION,
    icp_iterations INTEGER,
    transform_matrix DOUBLE PRECISION[],
    completed_at TIMESTAMP DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'
);

-- 空间索引
CREATE INDEX IF NOT EXISTS idx_trajectories_task_id ON trajectories(task_id);
CREATE INDEX IF NOT EXISTS idx_trajectories_vehicle ON trajectories(vehicle_type);
CREATE INDEX IF NOT EXISTS idx_trajectories_geom ON trajectories USING GIST(geom);
CREATE INDEX IF NOT EXISTS idx_waypoints_task_id ON waypoints(task_id);
CREATE INDEX IF NOT EXISTS idx_waypoints_geom ON waypoints USING GIST(geom);
CREATE INDEX IF NOT EXISTS idx_model_metadata_task_id ON model_metadata(task_id);
CREATE INDEX IF NOT EXISTS idx_fusion_results_task_id ON fusion_results(task_id);
CREATE INDEX IF NOT EXISTS idx_trajectories_task_vehicle ON trajectories(task_id, vehicle_type);
CREATE INDEX IF NOT EXISTS idx_trajectories_timestamp ON trajectories(timestamp);
```

## 4. 用户和权限配置

### 4.1 pg_hba.conf 认证配置

配置文件位置：

```bash
sudo -u postgres psql -t -c 'SHOW hba_file;'
# 默认: /etc/postgresql/15/main/pg_hba.conf
```

推荐配置（在文件末尾添加）：

```conf
# 空地协同测绘系统 - 本地 TCP 连接 (md5 密码认证)
host    all             all             127.0.0.1/32            md5
host    all             all             ::1/128                 md5

# 本地 socket 连接
local   all             postgres                                peer
local   all             air_survey                              md5
```

配置生效：

```bash
sudo systemctl reload postgresql
# 或
sudo pg_ctlcluster 15 main reload
```

### 4.2 权限说明

| 用户 | 角色 | 权限范围 |
|------|------|----------|
| `postgres` | 超级用户 | 数据库管理、备份恢复 |
| `air_survey` | 应用用户 | 数据读写 (CRUD)、创建表 |

## 5. 表结构说明

### 5.1 测绘任务表 (survey_tasks)

测绘任务的核心记录表，每个协同测绘任务在此记录一条。

| 字段 | 类型 | 说明 |
|------|------|------|
| id | SERIAL | 自增主键 |
| task_name | VARCHAR(255) | 任务名称 |
| task_type | VARCHAR(50) | uav_survey / ugv_survey / fusion |
| status | VARCHAR(50) | pending / running / completed / failed |
| created_at | TIMESTAMP | 创建时间 |
| started_at | TIMESTAMP | 开始执行时间 |
| completed_at | TIMESTAMP | 完成时间 |
| uav_id | VARCHAR(100) | 无人机编号 |
| ugv_id | VARCHAR(100) | 无人车编号 |
| area_sqm | DOUBLE | 测绘面积 (平方米) |
| notes | TEXT | 备注 |
| metadata | JSONB | 扩展元数据 |

### 5.2 航点表 (waypoints)

无人机航线规划的航点序列，包含空间位置信息。

| 字段 | 类型 | 说明 |
|------|------|------|
| id | SERIAL | 自增主键 |
| task_id | INTEGER FK | 所属任务 |
| sequence_index | INTEGER | 航点序号 |
| latitude/longitude | DOUBLE | WGS84 坐标 |
| altitude | DOUBLE | 海拔高度 (米) |
| speed | DOUBLE | 速度 (m/s) |
| heading | DOUBLE | 航向角 (度) |
| action | VARCHAR(50) | photo / hover / land |
| reached | BOOLEAN | 是否已到达 |
| geom | GEOMETRY(Point, 4326) | 空间位置 (PostGIS) |

### 5.3 轨迹表 (trajectories)

UAV/UGV 实时轨迹点，支持时空查询和地理围栏分析。

| 字段 | 类型 | 说明 |
|------|------|------|
| id | SERIAL | 自增主键 |
| task_id | INTEGER FK | 所属任务 |
| vehicle_type | VARCHAR(10) | uav / ugv |
| timestamp | DOUBLE | Unix 时间戳 (秒) |
| latitude/longitude | DOUBLE | WGS84 坐标 |
| altitude | DOUBLE | 海拔高度 (米) |
| roll/pitch/yaw | DOUBLE | 姿态角 (度) |
| geom | GEOMETRY(PointZ, 4326) | 三维空间位置 |

### 5.4 三维模型元数据表 (model_metadata)

存储点云、网格、纹理模型的元信息，支持模型检索和管理。

| 字段 | 类型 | 说明 |
|------|------|------|
| id | SERIAL | 自增主键 |
| task_id | INTEGER FK | 所属任务 |
| model_name | VARCHAR(255) | 模型名称 |
| model_type | VARCHAR(50) | pointcloud / mesh / textured_mesh |
| file_path | VARCHAR(500) | 文件系统路径 |
| file_size | BIGINT | 文件大小 (字节) |
| point_count | BIGINT | 点数 (点云模型) |
| face_count | BIGINT | 面数 (网格模型) |
| bbox_min | GEOMETRY(PointZ) | 包围盒最小值 |
| bbox_max | GEOMETRY(PointZ) | 包围盒最大值 |
| registration_rmse | DOUBLE | 配准均方根误差 |

### 5.5 融合成果表 (fusion_results)

空地协同数据配准融合结果，记录 ICP 配准参数和精度信息。

| 字段 | 类型 | 说明 |
|------|------|------|
| id | SERIAL | 自增主键 |
| task_id | INTEGER FK | 所属任务 |
| uav_pointcloud_path | VARCHAR(500) | UAV 点云路径 |
| ugv_pointcloud_path | VARCHAR(500) | UGV 点云路径 |
| fused_pointcloud_path | VARCHAR(500) | 融合点云路径 |
| mesh_path | VARCHAR(500) | 网格文件路径 |
| coarse_rmse | DOUBLE | 粗配准 RMSE |
| fine_rmse | DOUBLE | 精配准 RMSE |
| icp_iterations | INTEGER | ICP 迭代次数 |
| transform_matrix | DOUBLE[] | 4×4 变换矩阵 |

## 6. 连接信息

数据库连接参数配置在项目根目录的 `env.txt` 中：

```ini
# =============================================================================
# 数据库配置 (PostgreSQL 15 + PostGIS 3)
# =============================================================================
DB_HOST=127.0.0.1
DB_PORT=5432
DB_NAME=air_survey_db
DB_USER=air_survey
DB_PASSWORD=air_survey_2024
```

测试连接：

```bash
# 方式一: 使用 psql
PGPASSWORD=air_survey_2024 psql -h 127.0.0.1 -U air_survey -d air_survey_db -c "SELECT PostGIS_Full_Version();"

# 方式二: 使用 Python
python3 -c "
import psycopg2
conn = psycopg2.connect(host='127.0.0.1', port=5432, dbname='air_survey_db', user='air_survey', password='air_survey_2024')
cur = conn.cursor()
cur.execute('SELECT PostGIS_Full_Version();')
print(cur.fetchone()[0][:80])
conn.close()
"
```

## 7. 备份与恢复

### 7.1 全量备份

```bash
# 备份整个数据库 (含 PostGIS 扩展和数据)
pg_dump -h 127.0.0.1 -U air_survey -d air_survey_db \
    -F c -b -v -f air_survey_db_$(date +%Y%m%d_%H%M%S).dump

# 仅备份表结构 (不含数据)
pg_dump -h 127.0.0.1 -U air_survey -d air_survey_db \
    --schema-only -f air_survey_db_schema.sql

# 仅备份数据 (INSERT 语句)
pg_dump -h 127.0.0.1 -U air_survey -d air_survey_db \
    --data-only --inserts -f air_survey_db_data.sql
```

### 7.2 增量备份 (WAL 归档)

```bash
# 启用 WAL 归档 (编辑 postgresql.conf)
# wal_level = replica
# archive_mode = on
# archive_command = 'cp %p /var/lib/postgresql/15/archive/%f'

# 执行基础备份
pg_basebackup -h 127.0.0.1 -U postgres -D /backup/base/ -Ft -z -P
```

### 7.3 恢复

```bash
# 从 dump 文件恢复
pg_restore -h 127.0.0.1 -U air_survey -d air_survey_db \
    -v air_survey_db_20260514_120000.dump

# 从 SQL 文件恢复
psql -h 127.0.0.1 -U air_survey -d air_survey_db -f backup.sql

# 时间点恢复 (PITR)
# 需要完整的 WAL 归档 + 基础备份
```

### 7.4 定时备份 (crontab)

```bash
# 每天凌晨 2:00 自动备份
0 2 * * * /usr/local/bin/pg_backup.sh

# pg_backup.sh 内容:
#!/bin/bash
BACKUP_DIR="/backup/postgresql"
mkdir -p "$BACKUP_DIR"
PGPASSWORD=air_survey_2024 pg_dump -h 127.0.0.1 -U air_survey -d air_survey_db \
    -F c -b -f "$BACKUP_DIR/air_survey_db_$(date +\%Y\%m\%d).dump"
# 保留最近 30 天的备份
find "$BACKUP_DIR" -name "air_survey_db_*.dump" -mtime +30 -delete
```

## 8. 性能调优建议

### 8.1 内存配置 (postgresql.conf)

编辑 `/etc/postgresql/15/main/postgresql.conf`：

```ini
# 共享缓冲区 (建议为系统内存的 25%)
shared_buffers = 1GB                # 默认 128MB

# 有效缓存大小 (建议为系统内存的 50-75%)
effective_cache_size = 3GB          # 默认 4GB

# 工作内存 (单次排序/哈希操作可用内存)
work_mem = 64MB                     # 默认 4MB

# 维护工作内存 (VACUUM/索引创建)
maintenance_work_mem = 256MB        # 默认 64MB

# WAL 缓冲区
wal_buffers = 16MB                  # 默认 -1 (自动)

# 最大连接数 (测绘系统通常不超过 20 个并发)
max_connections = 50                # 默认 100
```

### 8.2 空间查询优化

```sql
-- 更新表统计信息 (帮助查询规划器)
ANALYZE trajectories;
ANALYZE waypoints;

-- 定期 VACUUM (清理死行，更新统计信息)
VACUUM ANALYZE trajectories;
VACUUM ANALYZE survey_tasks;

-- 检查索引使用情况
SELECT 
    schemaname, tablename, indexname, idx_scan, idx_tup_read, idx_tup_fetch
FROM pg_stat_user_indexes
WHERE tablename IN ('trajectories', 'waypoints', 'survey_tasks');
```

### 8.3 轨迹表分区 (数据量大时)

当单表轨迹数据超过 1000 万条时，建议按任务或时间范围分区：

```sql
-- 按任务 ID 范围分区示例
CREATE TABLE trajectories_partitioned (
    id SERIAL,
    task_id INTEGER NOT NULL,
    vehicle_type VARCHAR(10) NOT NULL,
    timestamp DOUBLE PRECISION NOT NULL,
    latitude DOUBLE PRECISION NOT NULL,
    longitude DOUBLE PRECISION NOT NULL,
    altitude DOUBLE PRECISION NOT NULL,
    roll DOUBLE PRECISION DEFAULT 0,
    pitch DOUBLE PRECISION DEFAULT 0,
    yaw DOUBLE PRECISION DEFAULT 0,
    geom GEOMETRY(PointZ, 4326)
) PARTITION BY RANGE (task_id);

-- 创建分区
CREATE TABLE trajectories_p1 PARTITION OF trajectories_partitioned
    FOR VALUES FROM (1) TO (100);

CREATE TABLE trajectories_p2 PARTITION OF trajectories_partitioned
    FOR VALUES FROM (100) TO (200);
```

### 8.4 连接池

生产环境建议使用连接池（如 PgBouncer）：

```bash
sudo apt-get install -y pgbouncer
```

PgBouncer 配置 `/etc/pgbouncer/pgbouncer.ini`：

```ini
[databases]
air_survey_db = host=127.0.0.1 port=5432 dbname=air_survey_db

[pgbouncer]
listen_addr = 127.0.0.1
listen_port = 6432
auth_type = md5
auth_file = /etc/pgbouncer/userlist.txt
pool_mode = transaction
default_pool_size = 20
max_client_conn = 100
```

## 9. 常用查询示例

### 9.1 查询任务统计

```sql
-- 各类测绘任务数量
SELECT task_type, status, COUNT(*) 
FROM survey_tasks 
GROUP BY task_type, status 
ORDER BY task_type, status;

-- 最近 7 天的测绘总面积
SELECT COALESCE(SUM(area_sqm), 0) AS total_area
FROM survey_tasks
WHERE created_at > NOW() - INTERVAL '7 days'
  AND status = 'completed';
```

### 9.2 空间查询

```sql
-- 查询指定范围内的轨迹点 (1km 半径)
SELECT * FROM trajectories
WHERE ST_DWithin(
    geom::geography,
    ST_SetSRID(ST_MakePoint(120.0, 30.0), 4326)::geography,
    1000  -- 1000米
);

-- 计算任务轨迹总长度
SELECT 
    task_id,
    vehicle_type,
    ST_Length(ST_MakeLine(geom ORDER BY timestamp)::geography) AS total_distance_m
FROM trajectories
WHERE task_id = 1
GROUP BY task_id, vehicle_type;
```

### 9.3 时间范围查询

```sql
-- 查询某任务特定时间段的轨迹
SELECT * FROM trajectories
WHERE task_id = 1
  AND timestamp BETWEEN 1715673600 AND 1715677200
  AND vehicle_type = 'uav'
ORDER BY timestamp;
```

## 10. 故障排查

### 10.1 无法连接数据库

```bash
# 检查服务状态
sudo systemctl status postgresql

# 查看日志
sudo tail -f /var/log/postgresql/postgresql-15-main.log

# 检查监听端口
sudo netstat -tlnp | grep 5432
```

### 10.2 认证失败

```bash
# 查看 pg_hba.conf 配置
sudo cat /etc/postgresql/15/main/pg_hba.conf | grep -v '^#' | grep -v '^$'

# 重新加载配置
sudo systemctl reload postgresql
```

### 10.3 PostGIS 扩展不可用

```sql
-- 检查 PostGIS 是否正确安装
SELECT * FROM pg_available_extensions WHERE name LIKE '%postgis%';

-- 手动安装扩展
CREATE EXTENSION IF NOT EXISTS postgis;
ALTER EXTENSION postgis UPDATE;
```

## 11. 安全建议

1. **修改默认密码**：安装后立即修改 `air_survey` 用户密码
2. **限制监听地址**：`postgresql.conf` 中设置 `listen_addresses = 'localhost'`（仅本地访问）
3. **SSL 连接**：生产环境启用 SSL 加密连接
4. **定期审计**：启用 `pgaudit` 扩展记录敏感操作
5. **最小权限**：`air_survey` 用户仅授予必要的 SELECT/INSERT/UPDATE/DELETE 权限

## 12. 卸载

```bash
# 完全卸载 PostgreSQL 15 和 PostGIS
sudo apt-get remove --purge -y postgresql-15 postgresql-15-postgis-3 postgis
sudo apt-get autoremove -y
sudo rm -rf /etc/postgresql/15/
sudo rm -rf /var/lib/postgresql/15/
sudo userdel -r postgres 2>/dev/null || true
```
