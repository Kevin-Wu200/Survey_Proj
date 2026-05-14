<template>
  <div class="map-container">
    <!-- 场景选择工具栏 -->
    <div class="view-toolbar">
      <span class="toolbar-label">🌐 3D 场景</span>
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
      <span class="sim-badge" :class="{ running: simRunning }">
        {{ simRunning ? '▶ 仿真运行中' : '⏸ 仿真已暂停' }}
      </span>
    </div>

    <!-- Three.js Canvas 容器 -->
    <div id="three-container" class="map-canvas"></div>

    <!-- 加载错误提示 -->
    <div v-if="loadError" class="load-error" @click="loadError = ''">
      ⚠️ {{ loadError }} (点击关闭)
    </div>

    <!-- UGV 状态叠加 -->
    <div class="map-overlay top-right" v-if="ugvBlocked">
      <div class="overlay-box ugv-status blocked">
        🚫 UGV 陡坡阻塞: {{ ugvBlockedReason }}
      </div>
    </div>

    <!-- UAV 高度显示 -->
    <div class="map-overlay top-left">
      <div class="overlay-box">
        <span>🛸 UAV 高于 UGV: {{ uavTargetAlt }}m (当前: {{ simUavAlt.toFixed(0) }}m)</span>
      </div>
    </div>

    <!-- UGV 坡度信息 -->
    <div class="map-overlay bottom-right">
      <div class="overlay-box" :class="ugvSlopeClass">
        <span>🚙 UGV 坡度: {{ ugvSlopeAngle.toFixed(1) }}°</span>
        <span v-if="ugvBlocked" class="blocked-warn">⚠ 阻塞</span>
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
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, computed } from 'vue'
import { useThreeScene } from '@/composables/useThreeScene'
import { use3DMarkers } from '@/composables/use3DMarkers'
import { useTerrainQuery } from '@/composables/useTerrainQuery'
import { useSimulation } from '@/composables/useSimulation'
import type { SceneInfo } from '@/types'
import { Raycaster, Vector2 } from 'three'

const store = null as any // 保留引用但不使用 WebSocket 数据

// Three.js 场景
const threeScene = useThreeScene()
let threeMarkers: ReturnType<typeof use3DMarkers> | null = null

// 地形查询
const terrainQuery = useTerrainQuery()

// 仿真引擎
const simulation = useSimulation(terrainQuery)

// 场景列表
const scenes = ref<SceneInfo[]>([])
const selectedScene = ref('')
const isLoading = ref(false)
const loadError = ref('')

// 仿真状态计算
const simRunning = computed(() => simulation.isRunning.value)
const uavTargetAlt = computed(() => simulation.uavTargetAlt.value)
const simUavAlt = computed(() => simulation.uavState.value.altitude)
const ugvBlocked = computed(() => simulation.ugvState.value.blocked)
const ugvBlockedReason = computed(() => simulation.ugvState.value.blockedReason ?? '')
const ugvSlopeAngle = computed(() => simulation.ugvState.value.slopeAngle ?? 0)
const ugvSlopeClass = computed(() => {
  if (ugvSlopeAngle.value > 30) return 'slope-danger'
  if (ugvSlopeAngle.value > 15) return 'slope-warning'
  return 'slope-good'
})

// 绘制模式状态（与 WaypointToolbar 通信）
const drawingActive = ref(false)

function onDrawingStateChanged(e: CustomEvent) {
  drawingActive.value = e.detail?.drawingEnabled ?? false
}

function onWaypointsUpdated(e: CustomEvent) {
  const count = e.detail?.count ?? 0
  if (count === 0 && threeMarkers) {
    threeMarkers.clearWaypoints()
  }
}

function onWaypointsCleared() {
  threeMarkers?.clearWaypoints()
}

// 导出给父组件
defineExpose({
  addWaypointToMap,
})

// =========================================================================
// 场景管理
// =========================================================================

async function fetchScenes(): Promise<void> {
  try {
    const res = await fetch('/api/scenes')
    if (!res.ok) throw new Error(`HTTP ${res.status}`)
    scenes.value = await res.json()
    console.log(`[MapView] 场景列表已获取: ${scenes.value.length} 个场景`)
  } catch (e) {
    console.error('[MapView] 获取场景列表失败:', e)
  }
}

async function onSceneChange(): Promise<void> {
  if (!selectedScene.value) {
    loadError.value = ''
    // 停止仿真
    simulation.stop()
    terrainQuery.clear()
    if (threeMarkers) {
      threeMarkers.dispose()
      threeMarkers = null
    }
    return
  }

  isLoading.value = true
  loadError.value = ''
  simulation.stop()

  try {
    const sceneInfo = scenes.value.find(s => s.filename === selectedScene.value)
    if (!sceneInfo) throw new Error('场景信息未找到')

    // 加载场景元数据
    let geoOrigin = { lat: 0.0, lng: 0.0, alt: 0.0 }
    const metaPath = selectedScene.value.replace(/\.glb$/i, '_metadata.json')
      .replace(/(.+)\.glb$/, '$1/metadata.json')
    try {
      const metaRes = await fetch(`/api/scenes/${metaPath}`)
      if (metaRes.ok) {
        const meta = await metaRes.json()
        if (meta.geoOrigin) {
          geoOrigin = {
            lat: meta.geoOrigin.lat ?? 0.0,
            lng: meta.geoOrigin.lng ?? 0.0,
            alt: meta.geoOrigin.alt ?? 0.0,
          }
        }
      }
    } catch { /* 无 metadata，使用默认值 */ }

    // 加载 GLB 模型
    const model = await threeScene.loadModel(`/api/scenes/${sceneInfo.path}`)

    // 注册地形 mesh 到 terrainQuery
    if (model) {
      terrainQuery.setTerrainObjects([model])

      // 初始化 3D 标记（在模型加载后，确保场景中有内容）
      if (threeScene.scene.value) {
        if (threeMarkers) threeMarkers.dispose()
        threeMarkers = use3DMarkers(threeScene.scene.value)
      }
    }

    // 设置仿真坐标原点
    simulation.setGeoOrigin(geoOrigin.lat, geoOrigin.lng, geoOrigin.alt)

    // 启动仿真
    simulation.start()

    console.log(`[MapView] 场景加载完成，仿真已启动 (原点: ${geoOrigin.lat}, ${geoOrigin.lng})`)
  } catch (e: any) {
    console.error('[MapView] 场景加载失败:', e)
    loadError.value = `场景加载失败: ${e.message || '未知错误'}`
    selectedScene.value = ''
  } finally {
    isLoading.value = false
  }
}

function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
}

// =========================================================================
// 3D 场景点击 → 航点添加
// =========================================================================

/**
 * 在 3D 场景中添加航点（通过 WaypointToolbar 事件通信）
 */
function addWaypointToMap(lat: number, lng: number): void {
  if (!drawingActive.value || !threeMarkers) return

  window.dispatchEvent(new CustomEvent('map-waypoint-click', {
    detail: { lat, lon: lng },
  }))

  threeMarkers.addWaypoint(lat, lng)
}

/**
 * 处理 3D 场景中的点击事件
 * 使用 Raycaster 检测与地形 mesh 的交点
 */
function handle3DClick(event: MouseEvent): void {
  if (!drawingActive.value) return
  if (!threeScene.scene.value || !threeScene.camera.value) return

  const container = document.getElementById('three-container')
  if (!container) return

  // 计算鼠标在 canvas 中的归一化坐标
  const rect = container.getBoundingClientRect()
  const mouseX = ((event.clientX - rect.left) / rect.width) * 2 - 1
  const mouseY = -((event.clientY - rect.top) / rect.height) * 2 + 1

  // 使用 Three.js Raycaster
  const raycaster = new Raycaster()
  raycaster.setFromCamera(new Vector2(mouseX, mouseY), threeScene.camera.value)

  const terrainMeshes = terrainQuery.getAllMeshes()
  if (terrainMeshes.length === 0) return

  const intersects = raycaster.intersectObjects(terrainMeshes, false)
  if (intersects.length === 0) return

  // 获取交点世界坐标
  const point = intersects[0].point

  // 世界坐标 → GPS 坐标（需要逆向转换）
  // 使用 simulation 中的 geoOrigin 信息
  // 简化：通过已知的原点和世界坐标反算 GPS
  const geoLat = simulation.uavState.value.latitude
  const geoLng = simulation.uavState.value.longitude

  // 这里需要反向计算 GPS，简化处理：用仿真当前原点 + 偏移
  // 实际使用时会通过 setGeoOrigin 设置原点
  // 目前使用仿真 UAV 位置附近的插值
  window.dispatchEvent(new CustomEvent('map-waypoint-click', {
    detail: {
      lat: geoLat + point.z / 111320,
      lon: geoLng + point.x / (111320 * Math.cos(geoLat * Math.PI / 180)),
      worldX: point.x,
      worldY: point.y,
      worldZ: point.z,
    },
  }))

  if (threeMarkers) {
    // 直接用世界坐标添加航点标记
    threeMarkers.clearWaypoints()
    // 使用 lat/lng 方式添加
  }
}

// =========================================================================
// 仿真状态监听 → 更新 3D 标记
// =========================================================================

watch(
  () => simulation.uavState.value,
  (state) => {
    if (!threeMarkers) return
    threeMarkers.updateUAV(
      state.latitude, state.longitude,
      state.altitude, state.heading,
    )
  },
  { deep: true },
)

watch(
  () => simulation.ugvState.value,
  (state) => {
    if (!threeMarkers) return
    threeMarkers.updateUGV(
      state.latitude, state.longitude,
      state.heading,
    )
  },
  { deep: true },
)

// =========================================================================
// 生命周期
// =========================================================================

onMounted(() => {
  // 监听 WaypointToolbar 事件
  window.addEventListener('drawing-state-changed', onDrawingStateChanged as EventListener)
  window.addEventListener('waypoints-cleared', onWaypointsCleared)
  window.addEventListener('waypoints-updated', onWaypointsUpdated as EventListener)

  // 初始化 Three.js 场景
  threeScene.initScene('three-container')

  // 监听 3D 场景点击
  const container = document.getElementById('three-container')
  if (container) {
    container.addEventListener('click', handle3DClick)
  }

  // 获取场景列表
  fetchScenes()

  console.log('[MapView] 纯3D模式初始化完成')
})

onUnmounted(() => {
  window.removeEventListener('drawing-state-changed', onDrawingStateChanged as EventListener)
  window.removeEventListener('waypoints-cleared', onWaypointsCleared)
  window.removeEventListener('waypoints-updated', onWaypointsUpdated as EventListener)

  const container = document.getElementById('three-container')
  if (container) {
    container.removeEventListener('click', handle3DClick)
  }

  simulation.stop()
  terrainQuery.clear()
  if (threeMarkers) {
    threeMarkers.dispose()
    threeMarkers = null
  }
  threeScene.destroyScene()
})
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

/* 工具栏 */
.view-toolbar {
  position: absolute;
  top: 10px;
  left: 50%;
  transform: translateX(-50%);
  z-index: 100;
  display: flex;
  align-items: center;
  gap: 10px;
  background: rgba(0, 0, 0, 0.8);
  padding: 8px 16px;
  border-radius: 8px;
  border: 1px solid rgba(255, 255, 255, 0.15);
  backdrop-filter: blur(8px);
}

.toolbar-label {
  color: #00bcd4;
  font-size: 14px;
  font-weight: bold;
  white-space: nowrap;
}

.scene-select {
  padding: 6px 12px;
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.1);
  color: #fff;
  font-size: 13px;
  cursor: pointer;
  min-width: 200px;
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

.sim-badge {
  color: rgba(255, 255, 255, 0.5);
  font-size: 12px;
  white-space: nowrap;
  padding: 4px 10px;
  border-radius: 4px;
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.sim-badge.running {
  color: #4caf50;
  border-color: #4caf50;
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

/* Overlay */
.map-overlay {
  position: absolute;
  z-index: 50;
  pointer-events: none;
}

.map-overlay.top-left { top: 60px; left: 10px; }
.map-overlay.top-right { top: 60px; right: 10px; }
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

.ugv-status.blocked {
  border-color: #f44336;
  color: #f44336;
}

.slope-good { border-color: #4caf50; }
.slope-warning { border-color: #ff9800; }
.slope-danger { border-color: #f44336; }

.blocked-warn {
  color: #f44336;
  font-weight: bold;
  animation: pulse 0.8s infinite;
}

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
