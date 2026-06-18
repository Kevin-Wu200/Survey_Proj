<template>
  <div class="status-panel">
    <div class="panel-header">
      <h2>{{ title }}</h2>
      <span :class="['status-dot', connected ? 'online' : 'offline']"></span>
      <span class="status-text">{{ connected ? '在线' : '离线' }}</span>
    </div>
    <div class="panel-body">
      <div class="info-row">
        <span class="label">操作模式</span>
        <span class="value">{{ modeText }}</span>
      </div>
      <div class="info-row">
        <span class="label">经度</span>
        <span class="value">{{ lon.toFixed(6) }}°</span>
      </div>
      <div class="info-row">
        <span class="label">纬度</span>
        <span class="value">{{ lat.toFixed(6) }}°</span>
      </div>
      <div class="info-row">
        <span class="label">高度</span>
        <span class="value">{{ alt.toFixed(1) }} m</span>
      </div>
      <div class="info-row">
        <span class="label">速度</span>
        <span class="value">{{ speed.toFixed(1) }} m/s</span>
      </div>
      <div class="info-row">
        <span class="label">航向</span>
        <span class="value">{{ heading.toFixed(1) }}°</span>
      </div>
      <div class="battery-section">
        <span class="label">电池</span>
        <div class="battery-bar">
          <div
            class="battery-fill"
            :style="{ width: battery + '%' }"
            :class="batteryClass"
          ></div>
        </div>
        <span class="battery-text">{{ battery.toFixed(0) }}%</span>
      </div>
      <div class="info-row">
        <span class="label">电池电压</span>
        <span class="value">{{ batteryV.toFixed(1) }}V</span>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue'

const props = defineProps<{
  title: string
  connected: boolean
  modeText: string
  lat: number
  lon: number
  alt: number
  speed: number
  heading: number
  battery: number
  batteryV: number
}>()

const batteryClass = computed(() => {
  if (props.battery > 50) return 'good'
  if (props.battery > 20) return 'warning'
  return 'danger'
})
</script>

<style scoped>
.status-panel {
  background: rgba(0, 0, 0, 0.85);
  border-radius: 8px;
  padding: 12px 16px;
  color: #fff;
  font-size: 13px;
  min-width: 220px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.15);
}

.panel-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 10px;
  padding-bottom: 8px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.2);
}

.panel-header h2 {
  margin: 0;
  font-size: 15px;
  font-weight: 600;
  flex: 1;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  display: inline-block;
}

.status-dot.online {
  background: #4caf50;
  box-shadow: 0 0 6px #4caf50;
}

.status-dot.offline {
  background: #f44336;
}

.status-text {
  font-size: 12px;
  opacity: 0.8;
}

.info-row {
  display: flex;
  justify-content: space-between;
  padding: 3px 0;
}

.label {
  opacity: 0.6;
}

.value {
  font-family: 'Courier New', monospace;
  font-weight: 500;
}

.battery-section {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 3px 0;
}

.battery-bar {
  flex: 1;
  height: 8px;
  background: rgba(255, 255, 255, 0.15);
  border-radius: 4px;
  overflow: hidden;
}

.battery-fill {
  height: 100%;
  border-radius: 4px;
  transition: width 0.5s ease;
}

.battery-fill.good { background: #4caf50; }
.battery-fill.warning { background: #ff9800; }
.battery-fill.danger { background: #f44336; }

.battery-text {
  font-size: 12px;
  min-width: 36px;
  text-align: right;
}
</style>
