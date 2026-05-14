<template>
  <div class="alert-panel">
    <div class="panel-header">
      <span>🔔 告警信息</span>
      <span class="alert-count" v-if="alerts.length">{{ alerts.length }}</span>
    </div>
    <div class="alert-list" v-if="alerts.length">
      <div
        v-for="alert in reversedAlerts"
        :key="alert.id"
        :class="['alert-item', levelClass(alert.level)]"
      >
        <span class="alert-icon">{{ levelIcon(alert.level) }}</span>
        <div class="alert-content">
          <span class="alert-source">{{ sourceLabel(alert.source) }}</span>
          <span class="alert-message">{{ alert.message }}</span>
        </div>
        <span class="alert-time">{{ formatTime(alert.timestamp) }}</span>
      </div>
    </div>
    <div class="alert-empty" v-else>
      暂无告警
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue'
import type { SystemAlert } from '@/types'

const props = defineProps<{
  alerts: SystemAlert[]
}>()

const reversedAlerts = computed(() => [...props.alerts].reverse().slice(0, 10))

function levelClass(level: number): string {
  return ['info', 'warning', 'error', 'critical'][level] || 'info'
}

function levelIcon(level: number): string {
  return ['ℹ️', '⚠️', '❌', '🚨'][level] || 'ℹ️'
}

function sourceLabel(source: string): string {
  const map: Record<string, string> = {
    uav: 'UAV', ugv: 'UGV',
    ground_station: '地面站', system: '系统',
  }
  return map[source] || source
}

function formatTime(ts: number): string {
  const d = new Date(ts * 1000)
  return d.toLocaleTimeString('zh-CN')
}
</script>

<style scoped>
.alert-panel {
  background: rgba(0, 0, 0, 0.85);
  border-radius: 8px;
  padding: 10px;
  color: #fff;
  font-size: 12px;
  max-height: 250px;
  display: flex;
  flex-direction: column;
  min-width: 220px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.15);
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 6px;
  font-weight: 600;
  font-size: 13px;
}

.alert-count {
  background: #f44336;
  border-radius: 10px;
  padding: 1px 7px;
  font-size: 11px;
}

.alert-list {
  overflow-y: auto;
  flex: 1;
}

.alert-item {
  display: flex;
  align-items: flex-start;
  gap: 6px;
  padding: 4px 0;
  border-bottom: 1px solid rgba(255, 255, 255, 0.08);
}

.alert-item.info { border-left: 2px solid #2196f3; padding-left: 6px; }
.alert-item.warning { border-left: 2px solid #ff9800; padding-left: 6px; }
.alert-item.error { border-left: 2px solid #f44336; padding-left: 6px; }
.alert-item.critical { border-left: 2px solid #ff1744; padding-left: 6px; animation: blink 1s infinite; }

@keyframes blink {
  50% { opacity: 0.5; }
}

.alert-icon { font-size: 14px; }

.alert-content {
  flex: 1;
  display: flex;
  flex-direction: column;
}

.alert-source {
  font-size: 10px;
  opacity: 0.6;
}

.alert-message {
  font-size: 11px;
}

.alert-time {
  font-size: 10px;
  opacity: 0.5;
  white-space: nowrap;
}

.alert-empty {
  text-align: center;
  opacity: 0.4;
  padding: 12px 0;
  font-size: 12px;
}
</style>
