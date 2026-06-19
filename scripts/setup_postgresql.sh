#!/bin/bash
# =============================================================================
# 空地协同无人化智能测绘系统 - PostgreSQL + PostGIS 安装配置脚本
# 
# 环境要求: Ubuntu 22.04 LTS (x86_64)
# 安装内容: PostgreSQL 15, PostGIS 3, 创建数据库和表结构
# 使用方式: sudo bash scripts/setup_postgresql.sh
#
# 注意: 本脚本使用 apt 直接安装，非 Docker 方式
# =============================================================================

set -e  # 任何命令失败则退出

# =============================================================================
# 颜色输出定义
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # 无颜色

info()  { echo -e "${BLUE}[信息]${NC} $*"; }
ok()    { echo -e "${GREEN}[成功]${NC} $*"; }
warn()  { echo -e "${YELLOW}[警告]${NC} $*"; }
err()   { echo -e "${RED}[错误]${NC} $*"; }

# =============================================================================
# 检查是否为 root 或具有 sudo 权限
# =============================================================================
if [[ $EUID -ne 0 ]]; then
    err "请使用 sudo 运行此脚本"
    exit 1
fi

# =============================================================================
# 一、安装 PostgreSQL 15
# =============================================================================
info "步骤 1/6: 添加 PostgreSQL 官方 APT 仓库..."
if ! dpkg -l | grep -q postgresql-common; then
    # 安装前置依赖
    apt-get update -qq
    apt-get install -y -qq curl ca-certificates gnupg lsb-release

    # 添加 PostgreSQL 官方 GPG 密钥
    curl -fsSL https://www.postgresql.org/media/keys/ACCC4CF8.asc | \
        gpg --dearmor -o /usr/share/keyrings/postgresql-archive-keyring.gpg

    # 添加 APT 源
    echo "deb [signed-by=/usr/share/keyrings/postgresql-archive-keyring.gpg] \
http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg main" \
        > /etc/apt/sources.list.d/pgdg.list

    apt-get update -qq
fi
ok "PostgreSQL APT 仓库已就绪"

# 安装 PostgreSQL 15
info "安装 PostgreSQL 15..."
apt-get install -y -qq postgresql-15 postgresql-contrib
ok "PostgreSQL 15 安装完成"

# =============================================================================
# 二、安装 PostGIS 3
# =============================================================================
info "步骤 2/6: 安装 PostGIS 3..."
apt-get install -y -qq postgresql-15-postgis-3 postgresql-15-postgis-3-scripts
apt-get install -y -qq postgis gdal-bin  # gdal-bin 提供 ogr2ogr 等工具
ok "PostGIS 3 安装完成"

# =============================================================================
# 三、启动 PostgreSQL 服务
# =============================================================================
info "步骤 3/6: 启动 PostgreSQL 服务..."
pg_ctlcluster 15 main start 2>/dev/null || systemctl start postgresql 2>/dev/null || true
systemctl enable postgresql 2>/dev/null || true
ok "PostgreSQL 服务已启动"

# =============================================================================
# 四、创建数据库用户和数据库
# =============================================================================
info "步骤 4/6: 创建数据库用户 air_survey 和数据库 air_survey_db..."

# 创建数据库用户 (如果已存在则跳过)
su - postgres -c "psql -tc \"SELECT 1 FROM pg_roles WHERE rolname='air_survey'\" | grep -q 1" 2>/dev/null && {
    warn "用户 air_survey 已存在，跳过创建"
} || {
    # 创建用户，密码设为 air_survey_2024 (可根据需要修改)
    su - postgres -c "psql -c \"CREATE USER air_survey WITH PASSWORD 'air_survey_2024' CREATEDB;\""
    ok "用户 air_survey 创建成功 (密码: air_survey_2024)"
}

# 创建数据库 (如果已存在则跳过)
su - postgres -c "psql -tc \"SELECT 1 FROM pg_database WHERE datname='air_survey_db'\" | grep -q 1" 2>/dev/null && {
    warn "数据库 air_survey_db 已存在，跳过创建"
} || {
    su - postgres -c "psql -c \"CREATE DATABASE air_survey_db OWNER air_survey;\""
    ok "数据库 air_survey_db 创建成功"
}

# 授权用户对数据库的所有权限
su - postgres -c "psql -c \"GRANT ALL PRIVILEGES ON DATABASE air_survey_db TO air_survey;\""
ok "用户权限已配置"

# =============================================================================
# 五、启用 PostGIS 扩展并创建表结构
# =============================================================================
info "步骤 5/6: 启用 PostGIS 扩展，创建测绘数据表结构..."

su - postgres -c "psql -d air_survey_db" << 'EOSQL'
-- 启用 PostGIS 扩展
CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS postgis_topology;

-- =============================================================================
-- 测绘任务表
-- 记录每一次空地协同测绘任务的元信息
-- =============================================================================
CREATE TABLE IF NOT EXISTS survey_tasks (
    id SERIAL PRIMARY KEY,
    task_name VARCHAR(255) NOT NULL,           -- 任务名称
    task_type VARCHAR(50) NOT NULL,            -- 任务类型: 'uav_survey', 'ugv_survey', 'fusion'
    status VARCHAR(50) DEFAULT 'pending',      -- 任务状态: 'pending', 'running', 'completed', 'failed'
    created_at TIMESTAMP DEFAULT NOW(),        -- 创建时间
    started_at TIMESTAMP,                      -- 开始执行时间
    completed_at TIMESTAMP,                    -- 完成时间
    uav_id VARCHAR(100),                       -- 无人机编号
    ugv_id VARCHAR(100),                       -- 无人车编号
    area_sqm DOUBLE PRECISION,                 -- 测绘面积 (平方米)
    notes TEXT,                                -- 备注
    metadata JSONB DEFAULT '{}'                -- 扩展元数据 (JSON格式)
);

-- =============================================================================
-- 航点表
-- 无人机航线规划中的航点序列
-- =============================================================================
CREATE TABLE IF NOT EXISTS waypoints (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,  -- 所属任务
    sequence_index INTEGER NOT NULL,          -- 航点序号 (从0开始)
    latitude DOUBLE PRECISION NOT NULL,       -- 纬度 (WGS84)
    longitude DOUBLE PRECISION NOT NULL,      -- 经度 (WGS84)
    altitude DOUBLE PRECISION NOT NULL,       -- 海拔高度 (米)
    speed DOUBLE PRECISION,                   -- 航点速度 (m/s)
    heading DOUBLE PRECISION,                 -- 航向角 (度)
    action VARCHAR(50),                       -- 动作: 'photo', 'hover', 'land'
    reached BOOLEAN DEFAULT FALSE,            -- 是否已到达
    reached_at TIMESTAMP,                     -- 到达时间
    geom GEOMETRY(Point, 4326)                -- 几何列 (WGS84坐标系)
);

-- =============================================================================
-- 轨迹表
-- UAV/UGV 实时轨迹点，支持 2D/3D 空间查询
-- =============================================================================
CREATE TABLE IF NOT EXISTS trajectories (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,  -- 所属任务
    vehicle_type VARCHAR(10) NOT NULL,        -- 载具类型: 'uav', 'ugv'
    timestamp DOUBLE PRECISION NOT NULL,      -- 时间戳 (Unix时间,秒)
    latitude DOUBLE PRECISION NOT NULL,       -- 纬度 (WGS84)
    longitude DOUBLE PRECISION NOT NULL,      -- 经度 (WGS84)
    altitude DOUBLE PRECISION NOT NULL,       -- 海拔高度 (米)
    roll DOUBLE PRECISION DEFAULT 0,          -- 横滚角 (度)
    pitch DOUBLE PRECISION DEFAULT 0,         -- 俯仰角 (度)
    yaw DOUBLE PRECISION DEFAULT 0,           -- 偏航角 (度)
    geom GEOMETRY(PointZ, 4326)               -- 三维几何列 (带Z值的WGS84点)
);

-- =============================================================================
-- 三维模型元数据表
-- 存储点云/网格/纹理模型的文件信息和空间范围
-- =============================================================================
CREATE TABLE IF NOT EXISTS model_metadata (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,  -- 所属任务
    model_name VARCHAR(255) NOT NULL,         -- 模型名称
    model_type VARCHAR(50) NOT NULL,          -- 模型类型: 'pointcloud', 'mesh', 'textured_mesh'
    file_path VARCHAR(500) NOT NULL,          -- 文件系统路径
    file_size BIGINT,                         -- 文件大小 (字节)
    point_count BIGINT,                       -- 点云点数 (仅点云模型)
    face_count BIGINT,                        -- 面片数 (仅网格模型)
    bbox_min GEOMETRY(PointZ, 4326),          -- 包围盒最小点
    bbox_max GEOMETRY(PointZ, 4326),          -- 包围盒最大点
    registration_rmse DOUBLE PRECISION,        -- 配准均方根误差
    created_at TIMESTAMP DEFAULT NOW(),       -- 创建时间
    metadata JSONB DEFAULT '{}'               -- 扩展元数据
);

-- =============================================================================
-- 融合成果表
-- 空地数据配准融合的结果记录
-- =============================================================================
CREATE TABLE IF NOT EXISTS fusion_results (
    id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES survey_tasks(id) ON DELETE CASCADE,  -- 所属任务
    uav_pointcloud_path VARCHAR(500),         -- UAV点云路径
    ugv_pointcloud_path VARCHAR(500),         -- UGV点云路径
    fused_pointcloud_path VARCHAR(500),       -- 融合后点云路径
    mesh_path VARCHAR(500),                   -- 网格文件路径
    coarse_rmse DOUBLE PRECISION,             -- 粗配准均方根误差
    fine_rmse DOUBLE PRECISION,               -- 精配准均方根误差
    icp_iterations INTEGER,                   -- ICP迭代次数
    transform_matrix DOUBLE PRECISION[],       -- 4x4变换矩阵 (16个元素, 行主序)
    completed_at TIMESTAMP DEFAULT NOW(),     -- 融合完成时间
    metadata JSONB DEFAULT '{}'               -- 扩展元数据
);

-- =============================================================================
-- 创建空间索引 (加速地理查询)
-- =============================================================================
CREATE INDEX IF NOT EXISTS idx_trajectories_task_id ON trajectories(task_id);
CREATE INDEX IF NOT EXISTS idx_trajectories_vehicle ON trajectories(vehicle_type);
CREATE INDEX IF NOT EXISTS idx_trajectories_geom ON trajectories USING GIST(geom);
CREATE INDEX IF NOT EXISTS idx_waypoints_task_id ON waypoints(task_id);
CREATE INDEX IF NOT EXISTS idx_waypoints_geom ON waypoints USING GIST(geom);
CREATE INDEX IF NOT EXISTS idx_model_metadata_task_id ON model_metadata(task_id);
CREATE INDEX IF NOT EXISTS idx_fusion_results_task_id ON fusion_results(task_id);

-- 创建复合索引加速轨迹查询
CREATE INDEX IF NOT EXISTS idx_trajectories_task_vehicle 
    ON trajectories(task_id, vehicle_type);

-- 创建时间戳索引用于时间范围查询
CREATE INDEX IF NOT EXISTS idx_trajectories_timestamp 
    ON trajectories(timestamp);

EOSQL

ok "表结构和索引创建完成"

# =============================================================================
# 六、配置 pg_hba.conf 允许本地连接
# =============================================================================
info "步骤 6/6: 配置 pg_hba.conf 允许本地连接..."

PG_HBA_FILE=$(su - postgres -c "psql -t -c 'SHOW hba_file;'" 2>/dev/null | tr -d '[:space:]')

if [[ -z "$PG_HBA_FILE" ]]; then
    # 默认路径
    PG_HBA_FILE="/etc/postgresql/15/main/pg_hba.conf"
fi

if [[ -f "$PG_HBA_FILE" ]]; then
    # 备份原配置
    cp "$PG_HBA_FILE" "${PG_HBA_FILE}.bak.$(date +%Y%m%d_%H%M%S)"
    
    # 确保存在本地连接规则 (md5 密码认证)
    if ! grep -q "host.*all.*all.*127.0.0.1/32.*md5" "$PG_HBA_FILE"; then
        echo "# 空地协同测绘系统 - 本地 TCP 连接 (md5 密码认证)" >> "$PG_HBA_FILE"
        echo "host    all             all             127.0.0.1/32            md5" >> "$PG_HBA_FILE"
        echo "# 空地协同测绘系统 - 本地 TCP 连接 (IPv6)" >> "$PG_HBA_FILE"
        echo "host    all             all             ::1/128                 md5" >> "$PG_HBA_FILE"
        ok "已添加本地 TCP 连接规则到 pg_hba.conf"
    fi
    
    # 确保本地 socket 连接使用 peer 认证 (postgres 用户)
    if ! grep -q "local.*all.*postgres.*peer" "$PG_HBA_FILE"; then
        echo "local   all             postgres                                peer" >> "$PG_HBA_FILE"
        ok "已添加 postgres 用户本地 peer 认证"
    fi
else
    warn "未找到 pg_hba.conf 文件: $PG_HBA_FILE"
    warn "请手动配置认证规则"
fi

# 重载配置使修改生效
pg_ctlcluster 15 main reload 2>/dev/null || systemctl reload postgresql 2>/dev/null || true
ok "pg_hba.conf 配置完成，PostgreSQL 已重新加载配置"

# =============================================================================
# 验证安装
# =============================================================================
info "============================================"
info "验证安装结果..."
info "============================================"

# 检查 PostgreSQL 版本
PG_VERSION=$(su - postgres -c "psql -t -c 'SELECT version();'" 2>/dev/null | head -1)
echo -e "  ${GREEN}PostgreSQL 版本:${NC} ${PG_VERSION}"

# 检查 PostGIS 版本
POSTGIS_VERSION=$(su - postgres -c "psql -d air_survey_db -t -c 'SELECT PostGIS_Full_Version();'" 2>/dev/null | head -1)
echo -e "  ${GREEN}PostGIS 版本:${NC} ${POSTGIS_VERSION}"

# 检查数据库连接
TABLE_COUNT=$(su - postgres -c "psql -d air_survey_db -t -c \"SELECT count(*) FROM information_schema.tables WHERE table_schema='public';\"" 2>/dev/null | tr -d '[:space:]')
echo -e "  ${GREEN}数据库表数量:${NC} ${TABLE_COUNT}"

# 检查用户
USER_EXISTS=$(su - postgres -c "psql -t -c \"SELECT 1 FROM pg_roles WHERE rolname='air_survey';\"" 2>/dev/null | tr -d '[:space:]')
if [[ "$USER_EXISTS" == "1" ]]; then
    echo -e "  ${GREEN}用户 air_survey:${NC} 已创建"
else
    echo -e "  ${RED}用户 air_survey:${NC} 未创建"
fi

echo ""
ok "============================================"
ok "  PostgreSQL + PostGIS 安装配置完成!"
ok "============================================"
echo ""
info "数据库连接信息:"
info "  主机:     localhost (127.0.0.1)"
info "  端口:     5432"
info "  数据库:   air_survey_db"
info "  用户名:   air_survey"
info "  密码:     air_survey_2024"
echo ""
info "测试连接命令:"
echo "  PGPASSWORD=air_survey_2024 psql -h 127.0.0.1 -U air_survey -d air_survey_db"
echo ""
warn "生产环境请在 env.txt 中修改密码!"
