/** 载具位置 */
export interface VehiclePosition {
  id: string
  latitude: number
  longitude: number
  altitude: number
  heading: number
  speed: number
  timestamp: number
}

/** 载具状态 */
export interface VehicleStatus {
  id: string
  connected: boolean
  armed: boolean
  flight_mode: number
  battery: number
  battery_voltage: number
  status_text: string
  latitude: number
  longitude: number
  altitude: number
  last_update: number
}

/** 航点数据 */
export interface Waypoint {
  lat: number
  lon: number
  alt: number
  speed: number
  heading: number
  action: 'photo' | 'hover' | 'land'
}

/** 航点任务状态 */
export interface MissionStatus {
  state: number   // 0=空闲 1=上传中 2=就绪 3=执行中 4=暂停 5=完成 6=失败 7=取消
  current_waypoint_index: number
  total_waypoints: number
  progress: number
  current_waypoint_lat: number
  current_waypoint_lon: number
  current_waypoint_alt: number
  photos_taken: number
  mission_id: string
  status_text: string
}

/** UGV 导航状态 */
export interface NavStatus {
  state: number
  distance_remaining: number
  yaw_error: number
  path_length: number
  current_path_index: number
  total_path_points: number
  status_text: string
}

/** 系统告警 */
export interface SystemAlert {
  id: string
  source: string
  level: number       // 0=info 1=warning 2=error 3=critical
  message: string
  timestamp: number
}

/** 回放会话信息 */
export interface ReplaySession {
  session_id: string
  filename: string
  start_time: number
  end_time: number
  duration: number
  total_frames: number
  has_images: boolean
  has_pointcloud: boolean
}

/** 回放帧 */
export interface ReplayFrame {
  timestamp: number
  uav_lat: number
  uav_lon: number
  uav_alt: number
  uav_heading: number
  ugv_lat: number
  ugv_lon: number
  ugv_heading: number
}

/** 回放状态 */
export interface ReplayState {
  session: ReplaySession | null
  current_index: number
  total_frames: number
  playing: boolean
  speed: number
}

/** WebSocket 消息 */
export interface WSMessage {
  type: 'init' | 'state_update' | 'status_update' | 'pong' | 'alert'
  data: SystemState | SystemAlert
  timestamp?: number
}

/** 系统状态 */
export interface SystemState {
  uav_position: VehiclePosition
  ugv_position: VehiclePosition
  uav_status: VehicleStatus
  ugv_status: VehicleStatus
  clients_count: number
  server_time: number
  // 二阶段新增
  system_mode: number
  system_mode_name: string
  uav_mission_status: MissionStatus
  ugv_nav_status: NavStatus
  replay: ReplayState
}

/** 飞行模式映射 */
export const FlightModeNames: Record<number, string> = {
  0: '待机',
  1: '起飞',
  2: '悬停',
  3: '航线',
  4: '降落',
  5: '返航',
}

/** 任务状态映射 */
export const MissionStateNames: Record<number, string> = {
  0: '空闲', 1: '上传中', 2: '就绪', 3: '执行中',
  4: '暂停', 5: '完成', 6: '失败', 7: '取消',
}

/** 导航状态映射 */
export const NavStateNames: Record<number, string> = {
  0: '空闲', 1: '规划中', 2: '执行中', 3: '到达目标', 4: '失败', 5: '取消',
}

/** 告警级别映射 */
export const AlertLevelNames: Record<number, string> = {
  0: '信息', 1: '警告', 2: '错误', 3: '严重',
}

/** 3D 场景信息 */
export interface SceneInfo {
  name: string
  filename: string
  path: string
  size: number
  format: 'glb'
}

/** 3D 场景元数据 */
export interface SceneMetadata {
  displayName?: string
  description?: string
  geoOrigin?: { lat: number; lng: number; alt: number }
  scale?: number
  rotation?: { x: number; y: number; z: number }
}

/** 采集进度信息 */
export interface CollectionProgress {
  task_id: number
  task_name: string
  total_area_sqm: number
  surveyed_area_sqm: number
  progress_percent: number
  estimated_completion_time: number
  elapsed_seconds: number
  uav_photos_taken: number
  ugv_distance_m: number
  status: string
}

/** 融合成果预览 */
export interface FusionResult {
  id: number
  task_id: number
  task_name: string
  model_type: string
  thumbnail_url?: string
  coarse_rmse: number
  fine_rmse: number
  point_count: number
  face_count: number
  completed_at: string
}

/** 数据库任务记录 */
export interface TaskRecord {
  id: number
  task_name: string
  task_type: string
  status: string
  created_at: string
  completed_at?: string
  area_sqm: number
  uav_id?: string
  ugv_id?: string
  fusion_result?: FusionResult
}

/** 任务列表查询参数 */
export interface TaskQuery {
  status?: string
  limit?: number
  offset?: number
}

/** 世界配准参数 */
export interface WorldRegistration {
  imageWidth: number
  imageHeight: number
  topLeftLng: number
  topLeftLat: number
  pixelSizeX: number
  pixelSizeY: number
  rotation: number
}

/** 统计概览 */
export interface SystemStatistics {
  total_tasks: number
  total_area_sqm: number
  completed_tasks: number
  recent_tasks: TaskRecord[]
  total_fusion_results: number
}
