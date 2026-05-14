<template>
  <div class="waypoint-toolbar">
    <div class="toolbar-header">
      <span>📍 航线工具</span>
    </div>

    <!-- 航线类型选择 -->
    <div class="tool-section">
      <label>航线类型</label>
      <select v-model="routeType">
        <option :value="0">自定义航点</option>
        <option :value="1">多边形航线</option>
        <option :value="2">蛇形航线</option>
      </select>
    </div>

    <!-- 多边形参数 -->
    <div class="tool-section" v-if="routeType === 1">
      <div class="param-row">
        <label>边数</label>
        <input type="number" v-model.number="polygonSides" min="3" max="12" />
      </div>
      <div class="param-row">
        <label>半径(m)</label>
        <input type="number" v-model.number="polygonRadius" min="10" max="2000" step="10" />
      </div>
    </div>

    <!-- 蛇形参数 -->
    <div class="tool-section" v-if="routeType === 2">
      <div class="param-row">
        <label>线间距(m)</label>
        <input type="number" v-model.number="snakeSpacing" min="10" max="500" step="10" />
      </div>
      <div class="param-row">
        <label>主方向(°)</label>
        <input type="number" v-model.number="snakeHeading" min="0" max="360" step="5" />
      </div>
    </div>

    <!-- 通用参数 -->
    <div class="tool-section">
      <div class="param-row">
        <label>飞行高度(m)</label>
        <input type="number" v-model.number="altitude" min="5" max="500" step="5" />
      </div>
      <div class="param-row">
        <label>飞行速度(m/s)</label>
        <input type="number" v-model.number="speed" min="1" max="15" step="0.5" />
      </div>
    </div>

    <!-- 拍照设置 -->
    <div class="tool-section">
      <label>拍照模式</label>
      <select v-model="cameraTriggerMode">
        <option :value="0">等距触发</option>
        <option :value="1">等时触发</option>
      </select>
      <div class="param-row" v-if="cameraTriggerMode === 0">
        <label>{{ cameraTriggerMode === 0 ? '间距(m)' : '间隔(s)' }}</label>
        <input type="number" v-model.number="cameraInterval"
               :min="cameraTriggerMode === 0 ? 5 : 1"
               :step="cameraTriggerMode === 0 ? 5 : 1" />
      </div>
    </div>

    <!-- 航点列表 (自定义模式) -->
    <div class="tool-section" v-if="routeType === 0">
      <div class="waypoint-hint">
        点击地图添加航点 (已添加 {{ customWaypoints.length }})<br />
        <span v-if="!drawingEnabled" style="color:#ff9800">
          请先在下方点击「开始绘制」
        </span>
      </div>
      <div class="wp-actions">
        <button @click="toggleDrawing" :class="{ active: drawingEnabled }">
          {{ drawingEnabled ? '⏹ 停止绘制' : '✏️ 开始绘制' }}
        </button>
        <button @click="clearWaypoints" :disabled="customWaypoints.length === 0">
          🗑 清除
        </button>
      </div>
      <div class="wp-list" v-if="customWaypoints.length">
        <div v-for="(wp, i) in customWaypoints" :key="i" class="wp-item">
          <span>WP{{ i + 1 }}</span>
          <span>{{ wp.lat.toFixed(6) }}, {{ wp.lon.toFixed(6) }}</span>
          <button @click="removeWaypoint(i)" class="wp-remove">×</button>
        </div>
      </div>
    </div>

    <!-- 操作按钮 -->
    <div class="tool-actions">
      <button
        class="btn-upload"
        @click="uploadMission"
        :disabled="routeType === 0 ? customWaypoints.length === 0 : false"
      >
        📤 上传任务
      </button>
      <button class="btn-start" @click="startMission">▶ 启动</button>
      <button class="btn-pause" @click="pauseMission">⏸ 暂停</button>
      <button class="btn-stop" @click="stopMission">⏹ 停止</button>
    </div>

    <!-- 任务状态 -->
    <div class="mission-status" v-if="store.uavMissionStatus">
      <div class="status-bar">
        <div class="status-fill" :style="{ width: (store.uavMissionStatus.progress || 0) + '%' }"></div>
      </div>
      <div class="status-info">
        <span>{{ store.uavMissionStatus.status_text }}</span>
        <span>📷 {{ store.uavMissionStatus.photos_taken || 0 }}</span>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted } from 'vue'
import { useSystemStore } from '@/stores/system'
import type { Waypoint } from '@/types'

const store = useSystemStore()

// 航线配置
const routeType = ref(0)
const polygonSides = ref(6)
const polygonRadius = ref(200)
const snakeSpacing = ref(50)
const snakeHeading = ref(0)
const altitude = ref(50)
const speed = ref(8)
const cameraTriggerMode = ref(0)
const cameraInterval = ref(50)

// 自定义航点
const customWaypoints = ref<Waypoint[]>([])
const drawingEnabled = ref(false)

// 暴露给 MapView 使用
defineExpose({
  customWaypoints,
  drawingEnabled,
  addWaypoint,
})

function addWaypoint(lat: number, lon: number) {
  if (!drawingEnabled.value || routeType.value !== 0) return
  customWaypoints.value.push({
    lat, lon, alt: altitude.value,
    speed: speed.value, heading: 0,
    action: 'photo',
  })
}

function removeWaypoint(index: number) {
  customWaypoints.value.splice(index, 1)
  // 通知地图更新标记
  window.dispatchEvent(new CustomEvent('waypoints-updated', {
    detail: { count: customWaypoints.value.length },
  }))
}

function clearWaypoints() {
  customWaypoints.value = []
  // 通知地图清除标记
  window.dispatchEvent(new CustomEvent('waypoints-cleared'))
}

function toggleDrawing() {
  drawingEnabled.value = !drawingEnabled.value
}

// -- 监听地图点击事件，接收航点坐标 --
function onMapWaypointClick(e: Event) {
  const customEvent = e as CustomEvent<{ lat: number; lon: number }>
  if (customEvent.detail) {
    addWaypoint(customEvent.detail.lat, customEvent.detail.lon)
  }
}

onMounted(() => {
  window.addEventListener('map-waypoint-click', onMapWaypointClick)
})

onUnmounted(() => {
  window.removeEventListener('map-waypoint-click', onMapWaypointClick)
})

// -- 观察 drawingEnabled 变化，通知地图组件 --
watch(drawingEnabled, (val) => {
  window.dispatchEvent(new CustomEvent('drawing-state-changed', {
    detail: { drawingEnabled: val },
  }))
})

// API 调用
async function uploadMission() {
  const payload: any = {
    route_type: routeType.value,
    camera_trigger_mode: cameraTriggerMode.value,
    camera_trigger_interval: cameraInterval.value,
  }

  if (routeType.value === 0) {
    payload.waypoints = customWaypoints.value
  } else if (routeType.value === 1) {
    payload.mission_params = {
      center_lat: store.uavPosition?.latitude ?? 0.0,
      center_lon: store.uavPosition?.longitude ?? 0.0,
      radius: polygonRadius.value,
      sides: polygonSides.value,
      altitude: altitude.value,
      speed: speed.value,
    }
  } else {
    const center = store.mapCenter
    payload.mission_params = {
      start_lat: center.lat - 0.001,
      start_lon: center.lng - 0.001,
      end_lat: center.lat + 0.001,
      end_lon: center.lng + 0.001,
      line_spacing: snakeSpacing.value,
      altitude: altitude.value,
      speed: speed.value,
      heading: snakeHeading.value,
    }
  }

  try {
    await fetch('/api/uav/mission/upload', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    })
  } catch (e) {
    console.error('上传任务失败:', e)
  }
}

async function startMission() {
  try {
    await fetch('/api/uav/mission/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({}),
    })
  } catch (e) {
    console.error('启动任务失败:', e)
  }
}

async function pauseMission() {
  try {
    await fetch('/api/uav/mission/pause', { method: 'POST' })
  } catch (e) {
    console.error('暂停任务失败:', e)
  }
}

async function stopMission() {
  try {
    await fetch('/api/uav/mission/stop', { method: 'POST' })
  } catch (e) {
    console.error('停止任务失败:', e)
  }
}
</script>

<style scoped>
.waypoint-toolbar {
  background: rgba(0, 0, 0, 0.85);
  border-radius: 8px;
  padding: 10px;
  color: #fff;
  font-size: 12px;
  min-width: 220px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.15);
}

.toolbar-header {
  font-weight: 600;
  font-size: 13px;
  margin-bottom: 8px;
  padding-bottom: 6px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.15);
}

.tool-section {
  margin-bottom: 8px;
}

.tool-section label {
  display: block;
  font-size: 11px;
  opacity: 0.6;
  margin-bottom: 2px;
}

.tool-section select,
.tool-section input {
  width: 100%;
  background: rgba(255, 255, 255, 0.08);
  color: #fff;
  border: 1px solid rgba(255, 255, 255, 0.15);
  padding: 4px 8px;
  border-radius: 3px;
  font-size: 12px;
}

.param-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin: 3px 0;
}

.param-row input {
  width: 80px;
}

.waypoint-hint {
  font-size: 11px;
  opacity: 0.6;
  margin-bottom: 6px;
  line-height: 1.4;
}

.wp-actions {
  display: flex;
  gap: 4px;
  margin-bottom: 6px;
}

.wp-actions button {
  flex: 1;
  background: rgba(255, 255, 255, 0.1);
  border: 1px solid rgba(255, 255, 255, 0.2);
  color: #fff;
  padding: 4px 8px;
  border-radius: 3px;
  cursor: pointer;
  font-size: 11px;
}

.wp-actions button:hover:not(:disabled) { background: rgba(255, 255, 255, 0.2); }
.wp-actions button.active { background: #ff9800; border-color: #ff9800; }
.wp-actions button:disabled { opacity: 0.4; }

.wp-list {
  max-height: 120px;
  overflow-y: auto;
  margin-bottom: 6px;
}

.wp-item {
  display: flex;
  align-items: center;
  gap: 4px;
  padding: 2px 0;
  font-size: 11px;
  font-family: monospace;
}

.wp-item span:first-child { color: #00bcd4; min-width: 30px; }
.wp-item span:nth-child(2) { flex: 1; overflow: hidden; text-overflow: ellipsis; }

.wp-remove {
  background: none;
  border: none;
  color: #f44336;
  cursor: pointer;
  font-size: 14px;
}

.tool-actions {
  display: flex;
  gap: 4px;
  flex-wrap: wrap;
}

.tool-actions button {
  flex: 1;
  min-width: 45px;
  background: rgba(255, 255, 255, 0.1);
  border: 1px solid rgba(255, 255, 255, 0.2);
  color: #fff;
  padding: 5px 6px;
  border-radius: 4px;
  cursor: pointer;
  font-size: 11px;
}

.tool-actions button:hover:not(:disabled) { background: rgba(255, 255, 255, 0.2); }
.tool-actions button:disabled { opacity: 0.4; cursor: default; }
.btn-upload { border-color: #00bcd4 !important; }
.btn-start { border-color: #4caf50 !important; }
.btn-pause { border-color: #ff9800 !important; }
.btn-stop { border-color: #f44336 !important; }

.mission-status {
  margin-top: 8px;
  padding-top: 6px;
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

.status-bar {
  height: 4px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 2px;
  margin-bottom: 4px;
}

.status-fill {
  height: 100%;
  background: #00bcd4;
  border-radius: 2px;
  transition: width 0.5s;
}

.status-info {
  display: flex;
  justify-content: space-between;
  font-size: 11px;
  opacity: 0.8;
}
</style>
