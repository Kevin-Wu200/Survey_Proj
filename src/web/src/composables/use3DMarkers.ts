/**
 * use3DMarkers.ts — 3D 场景中的 UAV/UGV 标记和轨迹线管理
 *
 * 功能：
 * - UAV/UGV Sprite 标记（Canvas 动态绘制，始终面向相机）
 * - 轨迹线（增量更新）
 * - 航点标记
 * - GPS 坐标与 3D 世界坐标转换（Haversine 公式）
 */
import { Scene, Sprite, SpriteMaterial, CanvasTexture, Line, BufferGeometry, BufferAttribute, LineBasicMaterial, SphereGeometry, Mesh, MeshBasicMaterial, Vector3, type Object3D } from 'three'

// =============================================================================
// 坐标转换工具
// =============================================================================

/**
 * 使用 Haversine 公式计算两点间的水平距离 (米)
 */
function haversineDistance(
  lat1: number, lng1: number,
  lat2: number, lng2: number,
): { dx: number; dz: number } {
  const R = 6371000 // 地球半径 (米)
  const toRad = (deg: number) => (deg * Math.PI) / 180

  const dLat = toRad(lat2 - lat1)
  const dLng = toRad(lng2 - lng1)
  const a1 = Math.sin(dLat / 2) ** 2
  const a2 = Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLng / 2) ** 2

  // 南北方向距离
  const dz = R * 2 * Math.atan2(Math.sqrt(a1), Math.sqrt(1 - a1)) * Math.sign(lat2 - lat1)

  // 东西方向距离
  const meanLat = toRad((lat1 + lat2) / 2)
  const dx = R * Math.cos(meanLat) * 2 * Math.atan2(Math.sqrt(a2 - a1), Math.sqrt(1 - (a2 - a1))) * Math.sign(lng2 - lng1)

  // 简化: 使用更精确的公式
  const sdLat = Math.sin(dLat / 2)
  const sdLng = Math.sin(dLng / 2)
  const aa = sdLat * sdLat + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * sdLng * sdLng
  const c = 2 * Math.atan2(Math.sqrt(aa), Math.sqrt(1 - aa))
  const totalDist = R * c

  // 方位角
  const y = Math.sin(dLng) * Math.cos(toRad(lat2))
  const x = Math.cos(toRad(lat1)) * Math.sin(toRad(lat2)) - Math.sin(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.cos(dLng)
  const bearing = Math.atan2(y, x)

  return {
    dx: totalDist * Math.sin(bearing),   // 东方向 (X轴)
    dz: totalDist * Math.cos(bearing),   // 北方向 (Z轴)
  }
}

// =============================================================================
// Canvas 图标绘制
// =============================================================================

/**
 * 通过 Canvas 动态绘制 Sprite 纹理
 */
function createSpriteTexture(
  shape: 'triangle' | 'square' | 'circle',
  color: string,
  size: number = 64,
): CanvasTexture {
  const canvas = document.createElement('canvas')
  canvas.width = size
  canvas.height = size
  const ctx = canvas.getContext('2d')!

  ctx.clearRect(0, 0, size, size)

  const half = size / 2
  const margin = size * 0.1

  ctx.fillStyle = color
  ctx.strokeStyle = '#ffffff'
  ctx.lineWidth = 2

  if (shape === 'triangle') {
    ctx.beginPath()
    ctx.moveTo(half, margin)
    ctx.lineTo(size - margin, size - margin)
    ctx.lineTo(margin, size - margin)
    ctx.closePath()
    ctx.fill()
    ctx.stroke()
    // 中心小圆
    ctx.fillStyle = '#ffffff'
    ctx.beginPath()
    ctx.arc(half, half + size * 0.05, size * 0.08, 0, Math.PI * 2)
    ctx.fill()
  } else if (shape === 'square') {
    ctx.fillStyle = color
    ctx.fillRect(margin, margin, size - 2 * margin, size - 2 * margin)
    ctx.strokeStyle = '#ffffff'
    ctx.strokeRect(margin, margin, size - 2 * margin, size - 2 * margin)
    // 中心小圆
    ctx.fillStyle = '#ffffff'
    ctx.beginPath()
    ctx.arc(half, half, size * 0.08, 0, Math.PI * 2)
    ctx.fill()
  } else if (shape === 'circle') {
    ctx.beginPath()
    ctx.arc(half, half, half - margin, 0, Math.PI * 2)
    ctx.fill()
    ctx.stroke()
  }

  const texture = new CanvasTexture(canvas)
  texture.needsUpdate = true
  return texture
}

// =============================================================================
// use3DMarkers
// =============================================================================

export interface ThreeMarkersInstance {
  updateUAV: (lat: number, lng: number, alt: number, heading: number) => void
  updateUGV: (lat: number, lng: number, heading: number) => void
  addWaypoint: (lat: number, lng: number) => void
  clearWaypoints: () => void
  setGeoOrigin: (lat: number, lng: number, alt: number) => void
  latLngToWorld: (lat: number, lng: number, alt: number) => Vector3
  dispose: () => void
}

export function use3DMarkers(scene: Scene): ThreeMarkersInstance {
  // 地理坐标原点，WGS84 赤道原点，高程 100m
  let _geoOrigin = { lat: 0.0, lng: 0.0, alt: 100.0 }
  let _scale = 1.0

  // 标记对象
  let _uavSprite: Sprite | null = null
  let _ugvSprite: Sprite | null = null

  // 轨迹线
  const _uavTrackPoints: Vector3[] = []
  const _ugvTrackPoints: Vector3[] = []
  const MAX_TRACK_POINTS = 200
  let _uavTrackLine: Line | null = null
  let _ugvTrackLine: Line | null = null

  // 航点
  const _waypointMeshes: Mesh[] = []

  // 缓存
  const _vec3 = new Vector3()

  // =========================================================================
  // 坐标转换
  // =========================================================================

  function setGeoOrigin(lat: number, lng: number, alt: number): void {
    _geoOrigin = { lat, lng, alt }
  }

  function latLngToWorld(lat: number, lng: number, alt: number): Vector3 {
    const { dx, dz } = haversineDistance(_geoOrigin.lat, _geoOrigin.lng, lat, lng)
    return new Vector3(
      dx * _scale,            // X: 东方向
      (alt - _geoOrigin.alt) * _scale,  // Y: 高度
      dz * _scale,            // Z: 北方向
    )
  }

  // =========================================================================
  // UAV/UGV 标记
  // =========================================================================

  function updateUAV(lat: number, lng: number, alt: number, heading: number): void {
    const pos = latLngToWorld(lat, lng, alt)

    // 创建或更新 Sprite
    if (!_uavSprite) {
      const texture = createSpriteTexture('triangle', '#00bcd4')
      const material = new SpriteMaterial({ map: texture, depthTest: false, depthWrite: false })
      _uavSprite = new Sprite(material)
      _uavSprite.scale.set(1.5, 1.5, 1)
      scene.add(_uavSprite)
    }
    _uavSprite.position.copy(pos)
    // Sprite 始终面向相机，通过旋转来表示朝向
    // 注意：Sprite 不能旋转，如需朝向显示，未来可替换为 3D 模型

    // 更新轨迹
    _uavTrackPoints.push(pos.clone())
    if (_uavTrackPoints.length > MAX_TRACK_POINTS) {
      _uavTrackPoints.shift()
    }
    updateUAVTrackLine()
  }

  function updateUGV(lat: number, lng: number, heading: number): void {
    const pos = latLngToWorld(lat, lng, 0)

    if (!_ugvSprite) {
      const texture = createSpriteTexture('square', '#ff9800')
      const material = new SpriteMaterial({ map: texture, depthTest: false, depthWrite: false })
      _ugvSprite = new Sprite(material)
      _ugvSprite.scale.set(1.2, 1.2, 1)
      scene.add(_ugvSprite)
    }
    _ugvSprite.position.copy(pos)

    // 更新轨迹
    _ugvTrackPoints.push(pos.clone())
    if (_ugvTrackPoints.length > MAX_TRACK_POINTS) {
      _ugvTrackPoints.shift()
    }
    updateUGVTrackLine()
  }

  // =========================================================================
  // 轨迹线
  // =========================================================================

  function updateUAVTrackLine(): void {
    if (_uavTrackPoints.length < 2) return
    if (_uavTrackLine) {
      scene.remove(_uavTrackLine)
      _uavTrackLine.geometry.dispose()
      ;(_uavTrackLine.material as LineBasicMaterial).dispose()
    }
    const positions = new Float32Array(_uavTrackPoints.length * 3)
    for (let i = 0; i < _uavTrackPoints.length; i++) {
      positions[i * 3] = _uavTrackPoints[i].x
      positions[i * 3 + 1] = _uavTrackPoints[i].y
      positions[i * 3 + 2] = _uavTrackPoints[i].z
    }
    const geometry = new BufferGeometry()
    geometry.setAttribute('position', new BufferAttribute(positions, 3))
    const material = new LineBasicMaterial({ color: 0x00bcd4, opacity: 0.6, transparent: true })
    _uavTrackLine = new Line(geometry, material)
    scene.add(_uavTrackLine)
  }

  function updateUGVTrackLine(): void {
    if (_ugvTrackPoints.length < 2) return
    if (_ugvTrackLine) {
      scene.remove(_ugvTrackLine)
      _ugvTrackLine.geometry.dispose()
      ;(_ugvTrackLine.material as LineBasicMaterial).dispose()
    }
    const positions = new Float32Array(_ugvTrackPoints.length * 3)
    for (let i = 0; i < _ugvTrackPoints.length; i++) {
      positions[i * 3] = _ugvTrackPoints[i].x
      positions[i * 3 + 1] = _ugvTrackPoints[i].y
      positions[i * 3 + 2] = _ugvTrackPoints[i].z
    }
    const geometry = new BufferGeometry()
    geometry.setAttribute('position', new BufferAttribute(positions, 3))
    const material = new LineBasicMaterial({ color: 0xff9800, opacity: 0.6, transparent: true })
    _ugvTrackLine = new Line(geometry, material)
    scene.add(_ugvTrackLine)
  }

  // =========================================================================
  // 航点
  // =========================================================================

  function addWaypoint(lat: number, lng: number): void {
    const pos = latLngToWorld(lat, lng, 0)
    const geometry = new SphereGeometry(0.15, 8, 8)
    const material = new MeshBasicMaterial({ color: 0xffeb3b })
    const mesh = new Mesh(geometry, material)
    mesh.position.copy(pos)
    scene.add(mesh)
    _waypointMeshes.push(mesh)
  }

  function clearWaypoints(): void {
    for (const mesh of _waypointMeshes) {
      scene.remove(mesh)
      mesh.geometry.dispose()
      ;(mesh.material as MeshBasicMaterial).dispose()
    }
    _waypointMeshes.length = 0
  }

  // =========================================================================
  // 资源释放
  // =========================================================================

  function dispose(): void {
    // UAV Sprite
    if (_uavSprite) {
      scene.remove(_uavSprite)
      ;(_uavSprite.material as SpriteMaterial).map?.dispose()
      _uavSprite.material.dispose()
      _uavSprite = null
    }

    // UGV Sprite
    if (_ugvSprite) {
      scene.remove(_ugvSprite)
      ;(_ugvSprite.material as SpriteMaterial).map?.dispose()
      _ugvSprite.material.dispose()
      _ugvSprite = null
    }

    // 轨迹线
    if (_uavTrackLine) {
      scene.remove(_uavTrackLine)
      _uavTrackLine.geometry.dispose()
      ;(_uavTrackLine.material as LineBasicMaterial).dispose()
      _uavTrackLine = null
    }
    if (_ugvTrackLine) {
      scene.remove(_ugvTrackLine)
      _ugvTrackLine.geometry.dispose()
      ;(_ugvTrackLine.material as LineBasicMaterial).dispose()
      _ugvTrackLine = null
    }

    // 航点
    clearWaypoints()

    _uavTrackPoints.length = 0
    _ugvTrackPoints.length = 0
  }

  return {
    updateUAV,
    updateUGV,
    addWaypoint,
    clearWaypoints,
    setGeoOrigin,
    latLngToWorld,
    dispose,
  }
}
