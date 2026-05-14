<template>
  <div class="map-container">
    <div id="tianditu-map" class="map-canvas"></div>

    <!-- 地图信息叠加 -->
    <div class="map-overlay top-left">
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
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, computed } from 'vue'
import { useSystemStore } from '@/stores/system'

const store = useSystemStore()

// 地图实例
let map: any = null
let uavMarker: any = null
let ugvMarker: any = null
let uavLabel: any = null
let ugvLabel: any = null
let uavHistory: any = null   // UAV 轨迹线
let ugvHistory: any = null   // UGV 轨迹线

// 轨迹历史点
const uavTrackPoints: { lng: number; lat: number }[] = []
const ugvTrackPoints: { lng: number; lat: number }[] = []
const MAX_TRACK_POINTS = 200

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

// 天地图 API 密钥
const TIANDITU_KEY = '66773e61397afa9c511d62fc259f3e70'

onMounted(() => {
  // 等待天地图 API 加载完成后再初始化地图
  waitForTianditu()
})

onUnmounted(() => {
  if (map) {
    map.destroy()
    map = null
  }
})

/**
 * 等待天地图 API 加载完成，带超时和重试机制
 */
function waitForTianditu(retries = 30, interval = 200): void {
  // 检查 T 全局对象是否已加载
  if (typeof T !== 'undefined' && T.Map && T.TileLayer) {
    console.log('[Map] 天地图 API 已就绪，开始初始化地图')
    initMap()
    return
  }

  if (retries <= 0) {
    console.error('[Map] 天地图 API 加载超时，请检查网络连接或 API Key 是否有效')
    // 显示错误提示
    const container = document.getElementById('tianditu-map')
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

  console.log(`[Map] 等待天地图 API 就绪... (剩余重试 ${retries} 次)`)
  setTimeout(() => waitForTianditu(retries - 1, interval), interval)
}

function initMap() {
  const container = document.getElementById('tianditu-map')
  if (!container) return

  // 创建天地图实例
  map = new T.Map('tianditu-map', {
    projection: 'EPSG:4326',    // WGS84 经纬度投影
    center: new T.LngLat(120.0, 30.0),
    zoom: 16,
  })

  // 添加卫星影像图层
  const sateLayer = new T.TileLayer(
    `https://t0.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&tk=${TIANDITU_KEY}`
  )
  map.addLayer(sateLayer)

  // 添加注记图层 (可半透明叠加)
  const cvaLayer = new T.TileLayer(
    `https://t0.tianditu.gov.cn/cva_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=cva&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&tk=${TIANDITU_KEY}`
  )
  map.addLayer(cvaLayer)

  // 添加缩放控件
  const zoomControl = new T.Control.Zoom()
  map.addControl(zoomControl)

  // 创建 UAV 标记 (三角形图标)
  uavMarker = new T.Marker(new T.LngLat(120.0, 30.0), {
    icon: new T.Icon({
      iconUrl: createUAVIcon('#00bcd4'),
      iconSize: new T.Point(32, 32),
      iconAnchor: new T.Point(16, 16),
    }),
  })
  map.addOverlay(uavMarker)

  // UAV 标签
  uavLabel = new T.Label({
    text: 'UAV',
    position: new T.LngLat(120.0, 30.0005),
    offset: new T.Point(-12, -20),
  })
  uavLabel.setStyle({
    color: '#00bcd4',
    fontSize: 12,
    fontWeight: 'bold',
    backgroundColor: 'rgba(0,0,0,0.6)',
    padding: '2px 6px',
    borderRadius: '3px',
    border: '1px solid #00bcd4',
  })
  map.addOverlay(uavLabel)

  // 创建 UGV 标记 (方形图标)
  ugvMarker = new T.Marker(new T.LngLat(120.0, 30.0), {
    icon: new T.Icon({
      iconUrl: createUGVIcon('#ff9800'),
      iconSize: new T.Point(28, 28),
      iconAnchor: new T.Point(14, 14),
    }),
  })
  map.addOverlay(ugvMarker)

  // UGV 标签
  ugvLabel = new T.Label({
    text: 'UGV',
    position: new T.LngLat(120.0, 30.0005),
    offset: new T.Point(-12, -20),
  })
  ugvLabel.setStyle({
    color: '#ff9800',
    fontSize: 12,
    fontWeight: 'bold',
    backgroundColor: 'rgba(0,0,0,0.6)',
    padding: '2px 6px',
    borderRadius: '3px',
    border: '1px solid #ff9800',
  })
  map.addOverlay(ugvLabel)

  // 监听地图事件
  map.addEventListener('moveend', () => {
    const center = map.getCenter()
    centerLng.value = center.lng
    centerLat.value = center.lat
    zoomLevel.value = map.getZoom()
  })

  console.log('[Map] 天地图初始化完成')
}

// 生成 UAV 图标 (三角形 SVG data URL)
function createUAVIcon(color: string): string {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
    <polygon points="16,2 30,28 2,28" fill="${color}" stroke="#fff" stroke-width="1.5"/>
    <circle cx="16" cy="20" r="3" fill="#fff"/>
  </svg>`
  return 'data:image/svg+xml;base64,' + btoa(svg)
}

// 生成 UGV 图标 (方形 SVG data URL)
function createUGVIcon(color: string): string {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="28" height="28" viewBox="0 0 28 28">
    <rect x="3" y="3" width="22" height="22" rx="3" fill="${color}" stroke="#fff" stroke-width="1.5"/>
    <circle cx="14" cy="14" r="3" fill="#fff"/>
  </svg>`
  return 'data:image/svg+xml;base64,' + btoa(svg)
}

// 监听 UAV 位置变化
watch(
  () => store.uavPosition,
  (pos) => {
    if (!pos || !map) return
    const lngLat = new T.LngLat(pos.longitude, pos.latitude)

    // 更新标记位置
    uavMarker?.setLngLat(lngLat)
    uavLabel?.setPosition(new T.LngLat(pos.longitude, pos.latitude + 0.0005))

    // 更新标签内容
    uavLabel?.setLabel(
      `UAV ${pos.altitude.toFixed(0)}m ${pos.speed.toFixed(1)}m/s`
    )

    // 添加轨迹点
    if (pos.speed > 0.1) {
      uavTrackPoints.push({ lng: pos.longitude, lat: pos.latitude })
      if (uavTrackPoints.length > MAX_TRACK_POINTS) {
        uavTrackPoints.shift()
      }
      updateUAVTrack()
    }

    // 延迟估算
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
    if (!pos || !map) return
    const lngLat = new T.LngLat(pos.longitude, pos.latitude)

    ugvMarker?.setLngLat(lngLat)
    ugvLabel?.setPosition(new T.LngLat(pos.longitude, pos.latitude + 0.0005))
    ugvLabel?.setLabel(
      `UGV ${pos.speed.toFixed(1)}m/s`
    )

    if (pos.speed > 0.1) {
      ugvTrackPoints.push({ lng: pos.longitude, lat: pos.latitude })
      if (ugvTrackPoints.length > MAX_TRACK_POINTS) {
        ugvTrackPoints.shift()
      }
      updateUGVTrack()
    }
  },
  { deep: true }
)

// 更新 UAV 轨迹线
function updateUAVTrack() {
  if (!map || uavTrackPoints.length < 2) return

  if (uavHistory) {
    map.removeOverlay(uavHistory)
  }

  const points = uavTrackPoints.map(p => new T.LngLat(p.lng, p.lat))
  uavHistory = new T.Polyline(points, {
    color: '#00bcd4',
    weight: 2,
    opacity: 0.6,
    lineStyle: 'dashed',
  })
  map.addOverlay(uavHistory)
}

// 更新 UGV 轨迹线
function updateUGVTrack() {
  if (!map || ugvTrackPoints.length < 2) return

  if (ugvHistory) {
    map.removeOverlay(ugvHistory)
  }

  const points = ugvTrackPoints.map(p => new T.LngLat(p.lng, p.lat))
  ugvHistory = new T.Polyline(points, {
    color: '#ff9800',
    weight: 2,
    opacity: 0.6,
    lineStyle: 'dashed',
  })
  map.addOverlay(ugvHistory)
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

.map-overlay {
  position: absolute;
  z-index: 50;
  pointer-events: none;
}

.map-overlay.top-left {
  top: 10px;
  left: 10px;
}

.map-overlay.bottom-right {
  bottom: 10px;
  right: 10px;
}

.map-overlay.bottom-left {
  bottom: 10px;
  left: 10px;
}

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
