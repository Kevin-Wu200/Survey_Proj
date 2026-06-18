<template>
  <div class="app-container">
    <!-- 顶部状态栏 -->
    <header class="top-bar">
      <div class="title">
        <img src="/logo.svg" alt="logo" class="logo" />
        <span>空地协同无人化智能测绘系统</span>
      </div>
      <div class="connection-status">
        <span :class="['dot', store.connected ? 'online' : 'offline']"></span>
        {{ store.connected ? '已连接地面站' : '地面站离线' }}
      </div>
      <div class="server-time">
        {{ timeString }}
      </div>
    </header>

    <!-- 主内容区 -->
    <main class="main-content">
      <!-- 地图区域 -->
      <MapView class="map-area" />

      <!-- 侧边栏 -->
      <aside class="sidebar">
        <StatusPanel
          title="🛸 UAV - DJI M300 RTK"
          :connected="uavStatus?.connected ?? false"
          :mode-text="uavModeText"
          :lat="uavStatus?.latitude ?? 30.0"
          :lon="uavStatus?.longitude ?? 120.0"
          :alt="uavStatus?.altitude ?? 0"
          :speed="uavPos?.speed ?? 0"
          :heading="uavPos?.heading ?? 0"
          :battery="uavStatus?.battery ?? 100"
          :battery-v="uavStatus?.battery_voltage ?? 0"
        />

        <StatusPanel
          title="🚗 UGV - 四轮差速底盘"
          :connected="ugvStatus?.connected ?? false"
          :mode-text="ugvModeText"
          :lat="ugvStatus?.latitude ?? 30.0"
          :lon="ugvStatus?.longitude ?? 120.0"
          :alt="ugvStatus?.altitude ?? 0"
          :speed="ugvPos?.speed ?? 0"
          :heading="ugvPos?.heading ?? 0"
          :battery="ugvStatus?.battery ?? 100"
          :battery-v="ugvStatus?.battery_voltage ?? 0"
        />
      </aside>
    </main>
  </div>
</template>

<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import MapView from '@/views/MapView.vue'
import StatusPanel from '@/components/StatusPanel.vue'
import { useSystemStore } from '@/stores/system'
import { FlightModeNames } from '@/types'

const store = useSystemStore()

const uavPos = computed(() => store.uavPosition)
const uavStatus = computed(() => store.uavStatus)
const ugvPos = computed(() => store.ugvPosition)
const ugvStatus = computed(() => store.ugvStatus)

const uavModeText = computed(() => {
  if (!uavStatus.value?.connected) return '离线'
  return FlightModeNames[uavStatus.value.flight_mode] || '未知'
})

const ugvModeText = computed(() => {
  if (!ugvStatus.value?.connected) return '离线'
  return ugvStatus.value.status_text || '待机'
})

const timeString = ref('')
let timeTimer: ReturnType<typeof setInterval> | null = null

onMounted(() => {
  timeTimer = setInterval(() => {
    timeString.value = new Date().toLocaleString('zh-CN')
  }, 1000)
})

onUnmounted(() => {
  if (timeTimer) clearInterval(timeTimer)
})
</script>

<style>
/* 全局样式 */
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto,
    'Helvetica Neue', Arial, 'Noto Sans SC', sans-serif;
  background: #0a0e17;
  color: #e0e0e0;
  overflow: hidden;
}

.app-container {
  width: 100vw;
  height: 100vh;
  display: flex;
  flex-direction: column;
}

.top-bar {
  height: 48px;
  background: rgba(15, 20, 35, 0.95);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  display: flex;
  align-items: center;
  padding: 0 16px;
  z-index: 100;
  backdrop-filter: blur(10px);
}

.top-bar .title {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 16px;
  font-weight: 600;
  flex: 1;
}

.top-bar .logo {
  width: 28px;
  height: 28px;
}

.connection-status {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 13px;
  margin-right: 20px;
}

.connection-status .dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  display: inline-block;
}

.dot.online {
  background: #4caf50;
  box-shadow: 0 0 6px #4caf50;
}

.dot.offline {
  background: #f44336;
}

.server-time {
  font-size: 12px;
  opacity: 0.6;
  font-family: 'Courier New', monospace;
}

.main-content {
  flex: 1;
  display: flex;
  position: relative;
  overflow: hidden;
}

.map-area {
  flex: 1;
  position: relative;
}

.sidebar {
  width: 260px;
  background: rgba(15, 20, 35, 0.9);
  border-left: 1px solid rgba(255, 255, 255, 0.1);
  display: flex;
  flex-direction: column;
  gap: 12px;
  padding: 12px;
  overflow-y: auto;
  backdrop-filter: blur(10px);
}
</style>
