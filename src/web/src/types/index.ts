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

/** WebSocket 消息 */
export interface WSMessage {
  type: 'init' | 'state_update' | 'status_update' | 'pong'
  data: SystemState
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
