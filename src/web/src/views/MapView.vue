<template>
  <div class="map-container">
    <!-- 视图模式切换工具栏 -->
    <div class="view-toolbar">
      <button
        class="mode-btn"
        :class="{ active: viewMode === 'amap' }"
        @click="switchMode('amap')"
      >
        <span class="mode-icon">🗺️</span> 2D 地图
      </button>
      <button
        class="mode-btn"
        :class="{ active: viewMode === 'glb' }"
        @click="switchMode('glb')"
      >
        <span class="mode-icon">🌐</span> 3D 场景
      </button>
      <template v-if="viewMode === 'glb'">
        <select
          v-model="selectedScene"
          class="scene-select"
          @change="onSceneChange"
        >
          <option value="">-- 选择场景 --</option>
          <option v-for="s in scenes" :key="s.filename" :value="s.filename">
            {{ s.name }} ({{ formatSize(s.size) }})
          </option>
        </select>
        <span v-if="isLoading" class="loading-spinner">⏳ 加载中...</span>
      </template>
    </div>

    <!-- AMap 容器 -->
    <div v-show="viewMode === 'amap'" id="amap-container" class="map-canvas"></div>
    <!-- Three.js Canvas 容器 -->
    <div v-show="viewMode === 'glb'" id="three-container" class="map-canvas"></div>
    <!-- 加载错误提示 -->
    <div v-if="loadError" class="load-error" @click="loadError = ''">
      ⚠️ {{ loadError }} (点击关闭)
    </div>

    <!-- UGV 导航目标提示 (仅 2D 模式) -->
    <div class="map-overlay top-right" v-if="ugvNavTarget && viewMode === 'amap'">
      <div class="overlay-box nav-target">
        🎯 UGV 目标: {{ ugvNavTarget.lat.toFixed(6) }}°, {{ ugvNavTarget.lng.toFixed(6) }}°
        <button @click="cancelNavTarget" class="cancel-btn">✕</button>
      </div>
    </div>

    <!-- 地图信息叠加 (仅 2D 模式) -->
    <div class="map-overlay top-left" v-if="viewMode === 'amap'">
      <div class="overlay-box">
        <span>地图中心: {{ centerLng.toFixed(4) }}°, {{ centerLat.toFixed(4) }}°</span>
        <span>缩放: {{ zoomLevel }}</span>
      </div>
    </div>

    <!-- 延迟指示器 -->
    <div class="map-overlay bottom-right">
      <div class="overlay-box delay-indicator" :class="delayClass">
        <span v-if="wsConnected">延迟: {{ estimatedDelay }}ms</span>
        <span v-else>未连接</span>
      </div>
    </div>

    <!-- 图例 -->
    <div class="map-overlay bottom-left">
      <div class="overlay-box legend">
        <div class="legend-item">
          <span class="legend-icon uav">▲</span>
          <span>UAV 无人机</span>
        </div>
        <div class="legend-item">
          <span class="legend-icon ugv">■</span>
          <span>UGV 无人车</span>
        </div>
        <div class="legend-item">
          <span class="legend-icon waypoint" style="color:#ffeb3b;">●</span>
          <span>航点</span>
        </div>
        <div class="legend-item">
          <span class="legend-icon target" style="color:#f44336;">⦿</span>
          <span>目标点</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, computed } from 'vue'
import { useSystemStore } from '@/stores/system'
import { useThreeScene } from '@/composables/useThreeScene'
import { use3DMarkers } from '@/composables/use3DMarkers'
import type { SceneInfo } from '@/types'

const store = useSystemStore()

// Three.js 3D 场景实例（按需初始化）
const threeScene = useThreeScene()
let threeMarkers: ReturnType<typeof use3DMarkers> | null = null
let threeInitialized = false

// 视图模式状态
const viewMode = ref<'amap' | 'glb'>('amap')
const scenes = ref<SceneInfo[]>([])
const selectedScene = ref('')
const isLoading = ref(false)
const loadError = ref('')

// 地图实例
let map: any = null
let uavMarker: any = null
let ugvMarker: any = null
let uavLabel: any = null
let ugvLabel: any = null
let uavHistory: any = null   // UAV 轨迹线
let ugvHistory: any = null   // UGV 轨迹线
let waypointMarkers: any[] = []  // 航点标记列表
let waypointPath: any = null     // 航点间连线
let navTargetMarker: any = null  // UGV 导航目标标记
let replayUavMarker: any = null  // 回放 UAV 标记
let replayUgvMarker: any = null  // 回放 UGV 标记
let navPathLine: any = null      // 导航规划路径

// 绘制模式状态（由 WaypointToolbar 通过 drawing-state-changed 事件同步）
const drawingActive = ref(false)

function onDrawingStateChanged(e: CustomEvent) {
  drawingActive.value = e.detail?.drawingEnabled ?? false
}

function onWaypointsUpdated(e: CustomEvent) {
  // 简化处理：航点数量变化时，重建地图标记
  // 从最新事件获取数量
  const count = e.detail?.count ?? 0
  if (count === 0) {
    clearWaypoints()
  }
  // 对于单个删除，简单重建：清除所有标记并重新添加（保留前 count 个）
  // 由于无法获取精确坐标，这里只处理全部清空的情况
  // 精确的逐点删除需要更复杂的跨组件通信
}

// 轨迹历史点
const uavTrackPoints: { lng: number; lat: number }[] = []
const ugvTrackPoints: { lng: number; lat: number }[] = []
const MAX_TRACK_POINTS = 200

// UGV 导航目标
const ugvNavTarget = ref<{ lat: number; lng: number } | null>(null)

// 地图状态
const centerLng = ref(120.0)
const centerLat = ref(30.0)
const zoomLevel = ref(16)
const wsConnected = computed(() => store.connected)

// 延迟估算
const lastUpdateTime = ref(0)
const estimatedDelay = ref(0)
const delayClass = computed(() => {
  if (estimatedDelay.value < 200) return 'good'
  if (estimatedDelay.value < 500) return 'warning'
  return 'danger'
})

// 高德地图 API 密钥 (由 Vite 插件从 env.txt 注入)
declare const __AMAP_KEY__: string
const AMAP_KEY: string = typeof __AMAP_KEY__ !== 'undefined' ? __AMAP_KEY__ : ''

// 导出给 App 使用
defineExpose({
  addWaypointToMap,
})

// =========================================================================
// 3D 场景管理
// =========================================================================

/**
 * 获取可用 3D 场景列表
 */
async function fetchScenes(): Promise<void> {
  try {
    const res = await fetch('/api/scenes')
    if (!res.ok) throw new Error(`HTTP ${res.status}`)
    scenes.value = await res.json()
    console.log(`[MapView] 场景列表已获取: ${scenes.value.length} 个场景`)
  } catch (e) {
    console.error('[MapView] 获取场景列表失败:', e)
    // 静默失败，不影响 2D 地图正常使用
  }
}

/**
 * 切换视图模式
 */
function switchMode(mode: 'amap' | 'glb'): void {
  if (viewMode.value === mode) return

  if (mode === 'glb') {
    // 切换到 3D 模式
    if (!threeInitialized) {
      try {
        threeScene.initScene('three-container')
        threeInitialized = true

        // 初始化 3D 标记
        if (threeScene.scene.value) {
          threeMarkers = use3DMarkers(threeScene.scene.value)
        }
      } catch (e) {
        console.error('[MapView] Three.js 初始化失败:', e)
        loadError.value = 'WebGL 初始化失败，请检查浏览器是否支持 WebGL'
        return
      }
    }
    viewMode.value = 'glb'
    console.log('[MapView] 切换到 3D 场景模式')
  } else {
    // 切换回 2D AMap 模式
    viewMode.value = 'amap'
    console.log('[MapView] 切换到 2D 地图模式')
  }
}

/**
 * 场景选择变更处理
 */
async function onSceneChange(): Promise<void> {
  if (!selectedScene.value) {
    loadError.value = ''
    return
  }

  isLoading.value = true
  loadError.value = ''

  try {
    const sceneInfo = scenes.value.find(s => s.filename === selectedScene.value)
    if (!sceneInfo) {
      throw new Error('场景信息未找到')
    }

    // 加载场景元数据（用于设置坐标原点）
    const metaPath = selectedScene.value.replace(/\.glb$/i, '') + '/metadata.json'
    try {
      const metaRes = await fetch(`/api/scenes/${metaPath}`)
      if (metaRes.ok) {
        const meta = await metaRes.json()
        if (meta.geoOrigin && threeMarkers) {
          threeMarkers.setGeoOrigin(
            meta.geoOrigin.lat,
            meta.geoOrigin.lng,
            meta.geoOrigin.alt ?? 0,
          )
        }
      }
    } catch {
      // metadata.json 不存在，使用默认原点
    }

    // 加载 GLB 模型
    await threeScene.loadModel(`/api/scenes/${sceneInfo.path}`)
  } catch (e: any) {
    console.error('[MapView] 场景加载失败:', e)
    loadError.value = `场景加载失败: ${e.message || '未知错误'}`
    selectedScene.value = ''
  } finally {
    isLoading.value = false
  }
}

/**
 * 格式化文件大小显示
 */
function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
}

// =========================================================================
// 生命周期
// =========================================================================

onMounted(() => {
  // 监听 WaypointToolbar 的绘制状态变化
  window.addEventListener('drawing-state-changed', onDrawingStateChanged as EventListener)
  // 监听航点清空事件
  window.addEventListener('waypoints-cleared', clearWaypoints)
  // 监听航点更新事件（用于同步删除操作）
  window.addEventListener('waypoints-updated', onWaypointsUpdated as EventListener)

  // 获取 3D 场景列表
  fetchScenes()

  waitForAMap()
})

onUnmounted(() => {
  window.removeEventListener('drawing-state-changed', onDrawingStateChanged as EventListener)
  window.removeEventListener('waypoints-cleared', clearWaypoints)
  window.removeEventListener('waypoints-updated', onWaypointsUpdated as EventListener)

  // 清理 3D 资源
  if (threeMarkers) {
    threeMarkers.dispose()
    threeMarkers = null
  }
  if (threeInitialized) {
    threeScene.destroyScene()
    threeInitialized = false
  }

  if (map) {
    map.destroy()
    map = null
  }
})

function waitForAMap(retries = 30, interval = 200): void {
  if (typeof AMap !== 'undefined' && AMap.Map && AMap.TileLayer) {
    console.log('[Map] 高德地图 API 已就绪，开始初始化地图')
    initMap()
    return
  }

  if (retries <= 0) {
    console.error('[Map] 高德地图 API 加载超时')
    const container = document.getElementById('amap-container')
    if (container) {
      container.innerHTML = `
        <div style="display:flex;align-items:center;justify-content:center;height:100%;color:#f44336;font-size:16px;flex-direction:column;gap:12px;">
          <span style="font-size:32px;">🗺️</span>
          <span>地图加载失败</span>
          <span style="font-size:12px;opacity:0.6;">请检查网络连接或刷新页面重试</span>
        </div>`
    }
    return
  }

  console.log(`[Map] 等待高德地图 API 就绪... (剩余重试 ${retries} 次)`)
  setTimeout(() => waitForAMap(retries - 1, interval), interval)
}

function initMap() {
  const container = document.getElementById('amap-container')
  if (!container) return

  map = new AMap.Map('amap-container', {
    center: [120.0, 30.0],
    zoom: 16,
    layers: [
      new AMap.TileLayer.Satellite(),
      new AMap.TileLayer.RoadNet(),
    ],
  })

  // 创建 UAV 标记
  uavMarker = new AMap.Marker({
    position: [120.0, 30.0],
    icon: new AMap.Icon({
      image: createUAVIcon('#00bcd4'),
      size: new AMap.Size(32, 32),
      imageSize: new AMap.Size(32, 32),
    }),
    offset: new AMap.Pixel(-16, -16),
    zIndex: 100,
  })
  map.add(uavMarker)

  uavLabel = new AMap.Text({
    text: 'UAV',
    position: [120.0, 30.0005],
    anchor: 'center',
    offset: new AMap.Pixel(0, -20),
    style: {
      color: '#00bcd4', fontSize: '12px', fontWeight: 'bold',
      backgroundColor: 'rgba(0,0,0,0.6)', padding: '2px 6px',
      borderRadius: '3px', border: '1px solid #00bcd4',
    },
  })
  map.add(uavLabel)

  // 创建 UGV 标记
  ugvMarker = new AMap.Marker({
    position: [120.0, 30.0],
    icon: new AMap.Icon({
      image: createUGVIcon('#ff9800'),
      size: new AMap.Size(28, 28),
      imageSize: new AMap.Size(28, 28),
    }),
    offset: new AMap.Pixel(-14, -14),
    zIndex: 100,
  })
  map.add(ugvMarker)

  ugvLabel = new AMap.Text({
    text: 'UGV',
    position: [120.0, 30.0005],
    anchor: 'center',
    offset: new AMap.Pixel(0, -20),
    style: {
      color: '#ff9800', fontSize: '12px', fontWeight: 'bold',
      backgroundColor: 'rgba(0,0,0,0.6)', padding: '2px 6px',
      borderRadius: '3px', border: '1px solid #ff9800',
    },
  })
  map.add(ugvLabel)

  // 地图事件
  map.on('moveend', () => {
    const center = map.getCenter()
    centerLng.value = center.lng
    centerLat.value = center.lat
    zoomLevel.value = map.getZoom()
  })

  // 地图点击事件: UGV 导航目标下发 / 航点添加
  map.on('click', (e: any) => {
    handleMapClick(e.lnglat.lng, e.lnglat.lat)
  })

  console.log('[Map] 高德地图初始化完成 (二阶段增强)')
}

/**
 * 地图点击处理 - 根据上下文决定行为:
 * - Ctrl+Click: 发送 UGV 导航目标
 * - 否则: 如果 WaypointToolbar 处于绘制模式，添加航点
 */
function handleMapClick(lng: number, lat: number) {
  // Ctrl 键 + 点击 → UGV 导航目标
  if ((window as any)._ctrlKeyPressed) {
    sendUgvNavTarget(lng, lat)
    return
  }

  // 添加航点到工具栏
  addWaypointToMap(lat, lng)
}

function addWaypointToMap(lat: number, lng: number) {
  // 仅在绘制模式下才在地图上添加航点标记
  if (!drawingActive.value) return

  // 尝试通知 WaypointToolbar
  // 通过全局事件通信
  window.dispatchEvent(new CustomEvent('map-waypoint-click', {
    detail: { lat, lon: lng },
  }))

  // 直接在地图上添加标记
  const marker = new AMap.Marker({
    position: [lng, lat],
    icon: new AMap.Icon({
      image: createCircleIcon('#ffeb3b'),
      size: new AMap.Size(14, 14),
      imageSize: new AMap.Size(14, 14),
    }),
    offset: new AMap.Pixel(-7, -7),
    zIndex: 90,
  })
  map.add(marker)
  waypointMarkers.push(marker)

  // 更新航点间连线
  updateWaypointPath()

  // 通知 WaypointToolbar
  const toolbar = document.querySelector('.waypoint-toolbar')
  // The WaypointToolbar component listens for this event
}

function updateWaypointPath() {
  if (waypointPath) { map.remove(waypointPath); waypointPath = null; }
  if (waypointMarkers.length < 2) return

  const positions = waypointMarkers.map((m: any) => {
    const pos = m.getPosition()
    return [pos.lng, pos.lat]
  })

  waypointPath = new AMap.Polyline({
    path: positions,
    strokeColor: '#ffeb3b',
    strokeWeight: 2,
    strokeOpacity: 0.6,
    strokeStyle: 'dashed',
    zIndex: 50,
  })
  map.add(waypointPath)
}

function clearWaypoints() {
  waypointMarkers.forEach((m: any) => {
    if (m && map) map.remove(m)
  })
  waypointMarkers = []
  if (waypointPath) { map?.remove(waypointPath); waypointPath = null; }
}

async function sendUgvNavTarget(lng: number, lat: number) {
  ugvNavTarget.value = { lat, lng }

  // 添加目标标记
  if (navTargetMarker) { map?.remove(navTargetMarker); }
  navTargetMarker = new AMap.Marker({
    position: [lng, lat],
    icon: new AMap.Icon({
      image: createCircleIcon('#f44336'),
      size: new AMap.Size(20, 20),
      imageSize: new AMap.Size(20, 20),
    }),
    offset: new AMap.Pixel(-10, -10),
    zIndex: 95,
    label: {
      content: '目标',
      offset: new AMap.Pixel(12, -5),
      direction: 'right',
    },
  })
  map.add(navTargetMarker)

  // 发送到后端
  try {
    await fetch('/api/ugv/nav/goal', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        target_lat: lat,
        target_lon: lng,
        target_yaw: 0,
        max_linear_speed: 2.0,
        max_angular_speed: 1.5,
      }),
    })
  } catch (e) {
    console.error('发送导航目标失败:', e)
  }
}

function cancelNavTarget() {
  ugvNavTarget.value = null
  if (navTargetMarker) { map?.remove(navTargetMarker); navTargetMarker = null; }
}

// SVG 图标生成
function createUAVIcon(color: string): string {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
    <polygon points="16,2 30,28 2,28" fill="${color}" stroke="#fff" stroke-width="1.5"/>
    <circle cx="16" cy="20" r="3" fill="#fff"/>
  </svg>`
  return 'data:image/svg+xml;base64,' + btoa(svg)
}

function createUGVIcon(color: string): string {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="28" height="28" viewBox="0 0 28 28">
    <rect x="3" y="3" width="22" height="22" rx="3" fill="${color}" stroke="#fff" stroke-width="1.5"/>
    <circle cx="14" cy="14" r="3" fill="#fff"/>
  </svg>`
  return 'data:image/svg+xml;base64,' + btoa(svg)
}

function createCircleIcon(color: string): string {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 20 20">
    <circle cx="10" cy="10" r="7" fill="${color}" stroke="#fff" stroke-width="2"/>
    <circle cx="10" cy="10" r="2" fill="#000" opacity="0.5"/>
  </svg>`
  return 'data:image/svg+xml;base64,' + btoa(svg)
}

// 监听 UAV 位置变化
watch(
  () => store.uavPosition,
  (pos) => {
    if (!pos) return

    // 3D 模式：更新 Sprite 标记
    if (viewMode.value === 'glb' && threeMarkers) {
      threeMarkers.updateUAV(pos.latitude, pos.longitude, pos.altitude, pos.heading)
    }

    // AMap 模式：更新地图标记
    if (viewMode.value === 'amap' && map) {
      const lngLat: [number, number] = [pos.longitude, pos.latitude]

      uavMarker?.setPosition(lngLat)
      uavLabel?.setPosition([pos.longitude, pos.latitude + 0.0005])
      uavLabel?.setText(
        `UAV ${pos.altitude.toFixed(0)}m ${pos.speed.toFixed(1)}m/s`
      )

      if (pos.speed > 0.1) {
        uavTrackPoints.push({ lng: pos.longitude, lat: pos.latitude })
        if (uavTrackPoints.length > MAX_TRACK_POINTS) {
          uavTrackPoints.shift()
        }
        updateUAVTrack()
      }
    }

    const now = Date.now()
    if (lastUpdateTime.value > 0) {
      estimatedDelay.value = now - pos.timestamp * 1000
    }
    lastUpdateTime.value = now
  },
  { deep: true }
)

// 监听 UGV 位置变化
watch(
  () => store.ugvPosition,
  (pos) => {
    if (!pos) return

    // 3D 模式：更新 Sprite 标记
    if (viewMode.value === 'glb' && threeMarkers) {
      threeMarkers.updateUGV(pos.latitude, pos.longitude, pos.heading)
    }

    // AMap 模式：更新地图标记
    if (viewMode.value === 'amap' && map) {
      const lngLat: [number, number] = [pos.longitude, pos.latitude]

      ugvMarker?.setPosition(lngLat)
      ugvLabel?.setPosition([pos.longitude, pos.latitude + 0.0005])
      ugvLabel?.setText(`UGV ${pos.speed.toFixed(1)}m/s`)

      if (pos.speed > 0.1) {
        ugvTrackPoints.push({ lng: pos.longitude, lat: pos.latitude })
        if (ugvTrackPoints.length > MAX_TRACK_POINTS) {
          ugvTrackPoints.shift()
        }
        updateUGVTrack()
      }
    }
  },
  { deep: true }
)

// 监听回放帧变化
watch(
  () => store.replayState?.current_index,
  (idx) => {
    if (idx === undefined || !map) return
    updateReplayMarkers()
  },
)

function updateReplayMarkers() {
  // 简化: 从后端获取当前回放帧
  fetch('/api/replay/frame')
    .then(r => r.json())
    .then((frame: any) => {
      if (!frame || !map) return

      // 更新回放 UAV 标记
      if (!replayUavMarker) {
        replayUavMarker = new AMap.Marker({
          position: [frame.uav_lon, frame.uav_lat],
          icon: new AMap.Icon({
            image: createUAVIcon('#e91e63'),
            size: new AMap.Size(24, 24),
            imageSize: new AMap.Size(24, 24),
          }),
          offset: new AMap.Pixel(-12, -12),
          zIndex: 80,
        })
        map.add(replayUavMarker)
      } else {
        replayUavMarker.setPosition([frame.uav_lon, frame.uav_lat])
      }

      // 更新回放 UGV 标记
      if (!replayUgvMarker) {
        replayUgvMarker = new AMap.Marker({
          position: [frame.ugv_lon, frame.ugv_lat],
          icon: new AMap.Icon({
            image: createUGVIcon('#9c27b0'),
            size: new AMap.Size(20, 20),
            imageSize: new AMap.Size(20, 20),
          }),
          offset: new AMap.Pixel(-10, -10),
          zIndex: 80,
        })
        map.add(replayUgvMarker)
      } else {
        replayUgvMarker.setPosition([frame.ugv_lon, frame.ugv_lat])
      }
    })
    .catch(() => {})
}

function updateUAVTrack() {
  if (!map || uavTrackPoints.length < 2) return
  if (uavHistory) { map.remove(uavHistory) }
  const points: [number, number][] = uavTrackPoints.map(p => [p.lng, p.lat])
  uavHistory = new AMap.Polyline({
    path: points,
    strokeColor: '#00bcd4',
    strokeWeight: 2,
    strokeOpacity: 0.6,
    strokeStyle: 'dashed',
    zIndex: 50,
  })
  map.add(uavHistory)
}

function updateUGVTrack() {
  if (!map || ugvTrackPoints.length < 2) return
  if (ugvHistory) { map.remove(ugvHistory) }
  const points: [number, number][] = ugvTrackPoints.map(p => [p.lng, p.lat])
  ugvHistory = new AMap.Polyline({
    path: points,
    strokeColor: '#ff9800',
    strokeWeight: 2,
    strokeOpacity: 0.6,
    strokeStyle: 'dashed',
    zIndex: 50,
  })
  map.add(ugvHistory)
}

// 全局键盘监听: Ctrl 键用于 UGV 目标选择
if (typeof window !== 'undefined') {
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Control') (window as any)._ctrlKeyPressed = true
  })
  window.addEventListener('keyup', (e) => {
    if (e.key === 'Control') (window as any)._ctrlKeyPressed = false
  })
}
</script>

<style scoped>
.map-container {
  width: 100%;
  height: 100%;
  position: relative;
}

.map-canvas {
  width: 100%;
  height: 100%;
}

/* 视图模式切换工具栏 */
.view-toolbar {
  position: absolute;
  top: 10px;
  left: 50%;
  transform: translateX(-50%);
  z-index: 100;
  display: flex;
  align-items: center;
  gap: 8px;
  background: rgba(0, 0, 0, 0.8);
  padding: 6px 12px;
  border-radius: 8px;
  border: 1px solid rgba(255, 255, 255, 0.15);
  backdrop-filter: blur(8px);
}

.mode-btn {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 6px 14px;
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 6px;
  background: transparent;
  color: rgba(255, 255, 255, 0.7);
  font-size: 13px;
  cursor: pointer;
  transition: all 0.2s;
  white-space: nowrap;
}

.mode-btn:hover {
  border-color: rgba(255, 255, 255, 0.4);
  color: #fff;
}

.mode-btn.active {
  background: rgba(0, 188, 212, 0.25);
  border-color: #00bcd4;
  color: #00bcd4;
}

.mode-icon {
  font-size: 16px;
}

.scene-select {
  padding: 6px 10px;
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.1);
  color: #fff;
  font-size: 13px;
  cursor: pointer;
  min-width: 180px;
  outline: none;
}

.scene-select option {
  background: #1a1a2e;
  color: #fff;
}

.scene-select:focus {
  border-color: #00bcd4;
}

.loading-spinner {
  color: #00bcd4;
  font-size: 13px;
  white-space: nowrap;
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.4; }
}

.load-error {
  position: absolute;
  bottom: 60px;
  left: 50%;
  transform: translateX(-50%);
  z-index: 100;
  background: rgba(244, 67, 54, 0.9);
  color: #fff;
  padding: 8px 20px;
  border-radius: 6px;
  font-size: 13px;
  cursor: pointer;
  white-space: nowrap;
}

.map-overlay {
  position: absolute;
  z-index: 50;
  pointer-events: none;
}

.map-overlay.top-left { top: 10px; left: 10px; }
.map-overlay.top-right { top: 10px; right: 10px; pointer-events: auto; }
.map-overlay.bottom-right { bottom: 10px; right: 10px; }
.map-overlay.bottom-left { bottom: 10px; left: 10px; }

.overlay-box {
  background: rgba(0, 0, 0, 0.75);
  color: #fff;
  padding: 6px 12px;
  border-radius: 4px;
  font-size: 12px;
  font-family: 'Courier New', monospace;
  display: flex;
  gap: 12px;
  backdrop-filter: blur(5px);
  border: 1px solid rgba(255, 255, 255, 0.15);
}

.nav-target {
  color: #f44336;
  border-color: #f44336;
  align-items: center;
}

.cancel-btn {
  background: rgba(244, 67, 54, 0.2);
  border: 1px solid #f44336;
  color: #f44336;
  border-radius: 50%;
  width: 20px;
  height: 20px;
  cursor: pointer;
  font-size: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  pointer-events: auto;
}

.delay-indicator.good { border-color: #4caf50; }
.delay-indicator.warning { border-color: #ff9800; }
.delay-indicator.danger { border-color: #f44336; }

.legend {
  display: flex;
  flex-direction: column;
  gap: 4px;
  font-size: 11px;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 6px;
}

.legend-icon {
  font-size: 14px;
  width: 20px;
  text-align: center;
}

.legend-icon.uav { color: #00bcd4; }
.legend-icon.ugv { color: #ff9800; }
</style>
