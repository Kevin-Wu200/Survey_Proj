/**
 * useSimulation.ts — 前端仿真引擎
 *
 * 替代后端 mock_data_generator，在前端 Three.js 场景中直接运行仿真：
 * - UAV：椭圆航线飞行，高度由用户通过 WaypointToolbar 输入
 * - UGV：GPS坐标转世界坐标，Raycaster 查询地形高度贴地运行
 * - UGV 物理约束：坡度 > 30° 时减速阻塞，尝试沿等高线绕行
 *
 * 仿真直接更新响应式状态，MapView 通过 watch 消费
 */
import { ref, type Ref } from 'vue'
import type { TerrainQueryInstance } from './useTerrainQuery'

// =============================================================================
// 类型定义
// =============================================================================

/** 仿真车辆状态（前端本地） */
export interface SimVehicleState {
  latitude: number
  longitude: number
  altitude: number
  heading: number
  speed: number
  // UGV 特有
  slopeAngle?: number    // 当前坡度角度
  blocked?: boolean      // 是否被陡坡阻塞
  blockedReason?: string
}

export interface SimulationInstance {
  uavState: Ref<SimVehicleState>
  ugvState: Ref<SimVehicleState>
  uavTargetAlt: Ref<number>     // UAV 目标飞行高度（用户输入）
  isRunning: Ref<boolean>
  start: () => void
  stop: () => void
  setGeoOrigin: (lat: number, lng: number, alt: number) => void
}

// =============================================================================
// Haversine 坐标转换
// =============================================================================

function haversineOffset(
  originLat: number, originLng: number,
  targetLat: number, targetLng: number,
): { dx: number; dz: number } {
  const R = 6371000
  const toRad = (deg: number) => (deg * Math.PI) / 180

  const dLat = toRad(targetLat - originLat)
  const dLng = toRad(targetLng - originLng)
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(originLat)) * Math.cos(toRad(targetLat)) * Math.sin(dLng / 2) ** 2
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
  const totalDist = R * c

  const y = Math.sin(dLng) * Math.cos(toRad(targetLat))
  const x = Math.cos(toRad(originLat)) * Math.sin(toRad(targetLat)) -
    Math.sin(toRad(originLat)) * Math.cos(toRad(targetLat)) * Math.cos(dLng)
  const bearing = Math.atan2(y, x)

  return {
    dx: totalDist * Math.sin(bearing),
    dz: totalDist * Math.cos(bearing),
  }
}

function worldToLatLng(
  originLat: number, originLng: number,
  worldX: number, worldZ: number,
): { lat: number; lng: number } {
  const R = 6371000
  const toDeg = (rad: number) => rad * 180 / Math.PI
  const dist = Math.sqrt(worldX * worldX + worldZ * worldZ)
  const bearing = Math.atan2(worldX, worldZ)
  const angularDist = dist / R
  const lat = Math.asin(
    Math.sin(originLat * Math.PI / 180) * Math.cos(angularDist) +
    Math.cos(originLat * Math.PI / 180) * Math.sin(angularDist) * Math.cos(bearing)
  )
  const lng = originLng * Math.PI / 180 + Math.atan2(
    Math.sin(bearing) * Math.sin(angularDist) * Math.cos(originLat * Math.PI / 180),
    Math.cos(angularDist) - Math.sin(originLat * Math.PI / 180) * Math.sin(lat)
  )
  return { lat: toDeg(lat), lng: toDeg(lng) }
}

// =============================================================================
// useSimulation
// =============================================================================

export function useSimulation(
  terrainQuery: TerrainQueryInstance,
): SimulationInstance {
  // 地理坐标原点（与 GLB 场景绑定）
  let _geoOrigin = { lat: 30.0, lng: 120.0, alt: 0.0 }

  // 仿真状态
  const uavState = ref<SimVehicleState>({
    latitude: 30.0, longitude: 120.0, altitude: 80,
    heading: 0, speed: 0,
  })
  const ugvState = ref<SimVehicleState>({
    latitude: 30.0, longitude: 120.0, altitude: 0,
    heading: 0, speed: 0,
    slopeAngle: 0, blocked: false,
  })

  const uavTargetAlt = ref(80)  // 默认 UAV 飞行高度 80m
  const isRunning = ref(false)

  let _animationId: number | null = null
  let _simTime = 0.0
  let _lastTimestamp = 0
  const _SIM_STEP = 0.5            // 仿真步长（秒）
  const MAX_CLIMB_ANGLE = 30       // 最大爬坡角度
  const UGV_SPEED = 3.0            // UGV 基础速度 (m/s)
  const UAV_SPEED = 10.0           // UAV 基础速度 (m/s)

  // UGV 阻塞绕行状态
  let _blockedTime = 0.0
  let _contourDirection = 0        // 等高线绕行方向

  /**
   * 设置地理坐标原点
   */
  function setGeoOrigin(lat: number, lng: number, alt: number): void {
    _geoOrigin = { lat, lng, alt }
  }

  /**
   * GPS 坐标 → 3D 世界坐标
   */
  function latLngToWorld(lat: number, lng: number, alt: number): { x: number; y: number; z: number } {
    const { dx, dz } = haversineOffset(_geoOrigin.lat, _geoOrigin.lng, lat, lng)
    return { x: dx, y: alt - _geoOrigin.alt, z: dz }
  }

  /**
   * 3D 世界坐标 → GPS 坐标
   */
  function worldToGPS(x: number, z: number, alt: number): { lat: number; lng: number; alt: number } {
    const { lat, lng } = worldToLatLng(_geoOrigin.lat, _geoOrigin.lng, x, z)
    return { lat, lng, alt }
  }

  /**
   * UGV 移动逻辑：沿 heading 方向前进，实时查询地形高度和坡度
   */
  function stepUGV(headingDeg: number, speedMps: number): void {
    const current = ugvState.value
    const headingRad = (headingDeg * Math.PI) / 180

    // 当前世界坐标
    const curWorld = latLngToWorld(current.latitude, current.longitude, current.altitude)

    // 目标位置（沿 heading 方向移动一步）
    const stepDist = speedMps * _SIM_STEP
    const targetX = curWorld.x + Math.sin(headingRad) * stepDist
    const targetZ = curWorld.z + Math.cos(headingRad) * stepDist

    // 查询目标位置地形高度
    const terrainHeight = terrainQuery.getHeightAt(targetX, targetZ)

    if (terrainHeight === null) {
      // 无地形数据：保持当前高度，正常移动
      const targetGPS = worldToGPS(targetX, targetZ, current.altitude)
      current.latitude = targetGPS.lat
      current.longitude = targetGPS.lng
      current.heading = headingDeg
      return
    }

    // 查询目标位置坡度
    const slopeInfo = terrainQuery.getSlopeAt(targetX, targetZ, MAX_CLIMB_ANGLE)

    if (slopeInfo && slopeInfo.steep && speedMps > 0.1) {
      // 陡坡阻塞
      _blockedTime += _SIM_STEP
      current.blocked = true
      current.blockedReason = `陡坡阻塞 (${slopeInfo.angle.toFixed(1)}°)`
      current.slopeAngle = slopeInfo.angle
      current.speed = 0

      // 尝试沿等高线绕行
      if (_blockedTime > 2.0) {
        // 阻塞超过 2 秒，沿等高线方向小幅移动
        const contourStep = 1.0 // 小幅移动
        // 等高线方向 = 垂直于法线的水平分量
        const contourX = slopeInfo.normal.z  // 法线 Z 分量 = 等高线 X 方向
        const contourZ = -slopeInfo.normal.x // 法线 X 分量 = 等高线 -Z 方向
        const contourMag = Math.sqrt(contourX * contourX + contourZ * contourZ)
        if (contourMag > 0.01) {
          const newX = curWorld.x + (contourX / contourMag) * contourStep * (_contourDirection > 0 ? 1 : -1)
          const newZ = curWorld.z + (contourZ / contourMag) * contourStep * (_contourDirection > 0 ? 1 : -1)
          _contourDirection *= -1  // 来回尝试

          const contourHeight = terrainQuery.getHeightAt(newX, newZ)
          if (contourHeight !== null) {
            const contourSlope = terrainQuery.getSlopeAt(newX, newZ, MAX_CLIMB_ANGLE)
            if (contourSlope && !contourSlope.steep) {
              // 等高线方向可行
              const gps = worldToGPS(newX, newZ, contourHeight)
              current.latitude = gps.lat
              current.longitude = gps.lng
              current.altitude = contourHeight
              current.blocked = false
              current.blockedReason = undefined
              current.slopeAngle = contourSlope.angle
              _blockedTime = 0
              current.speed = UGV_SPEED * 0.5
              return
            }
          }
        }
      }
      // 阻塞中：不更新位置
      return
    }

    // 正常前进：更新位置到地形表面
    _blockedTime = 0
    current.blocked = false
    current.blockedReason = undefined
    current.slopeAngle = slopeInfo?.angle ?? 0

    const targetGPS = worldToGPS(targetX, targetZ, terrainHeight)
    current.latitude = targetGPS.lat
    current.longitude = targetGPS.lng
    current.altitude = terrainHeight
    current.heading = headingDeg
    current.speed = speedMps
  }

  /**
   * UAV 移动逻辑：椭圆航线，高度使用用户输入的目标高度
   */
  function stepUAV(): void {
    const centerLat = _geoOrigin.lat
    const centerLon = _geoOrigin.lng
    const targetAlt = uavTargetAlt.value

    // 椭圆航线（同后端逻辑）
    const uavLat = centerLat + 0.001 * Math.sin(_simTime * 0.5)
    const uavLon = centerLon + 0.0015 * Math.cos(_simTime * 0.3)
    const uavAlt = targetAlt + 10.0 * Math.sin(_simTime * 0.4) // 在目标高度附近微小波动
    const uavHeading = Math.atan2(
      Math.cos(_simTime * 0.3) * 0.0015 * 0.3,
      -Math.cos(_simTime * 0.5) * 0.001 * 0.5
    ) * 180 / Math.PI % 360
    const uavSpeed = UAV_SPEED + 3.0 * Math.abs(Math.sin(_simTime * 0.35))

    uavState.value = {
      latitude: uavLat,
      longitude: uavLon,
      altitude: uavAlt,
      heading: uavHeading < 0 ? uavHeading + 360 : uavHeading,
      speed: uavSpeed,
    }
  }

  /**
   * 仿真主循环
   */
  function simulationLoop(timestamp: number): void {
    if (!isRunning.value) return

    // 计算时间增量
    if (_lastTimestamp === 0) _lastTimestamp = timestamp
    const dt = Math.min((timestamp - _lastTimestamp) / 1000, 1.0) // 上限 1 秒
    _lastTimestamp = timestamp
    _simTime += dt

    // UAV 步进
    stepUAV()

    // UGV 步进（基于当前 heading 和速度）
    const ugvHeading = ((_simTime * 20) % 360 + 180) % 360  // 8字形转向
    stepUGV(ugvHeading, UGV_SPEED)

    _animationId = requestAnimationFrame(simulationLoop)
  }

  /**
   * 启动仿真
   */
  function start(): void {
    if (isRunning.value) return
    isRunning.value = true
    _lastTimestamp = 0
    _simTime = 0
    _blockedTime = 0
    _contourDirection = 1
    console.log('[Simulation] 前端仿真已启动')
    _animationId = requestAnimationFrame(simulationLoop)
  }

  /**
   * 停止仿真
   */
  function stop(): void {
    isRunning.value = false
    if (_animationId !== null) {
      cancelAnimationFrame(_animationId)
      _animationId = null
    }
    console.log('[Simulation] 前端仿真已停止')
  }

  return {
    uavState,
    ugvState,
    uavTargetAlt,
    isRunning,
    start,
    stop,
    setGeoOrigin,
  }
}
