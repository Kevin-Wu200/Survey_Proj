<template>
  <div class="timeline-container" v-if="store.replayState?.total_frames">
    <!-- 回放控制栏 -->
    <div class="replay-controls">
      <button class="ctrl-btn" @click="replayControl('play')" :disabled="store.replayState?.playing">
        ▶ 播放
      </button>
      <button class="ctrl-btn" @click="replayControl('pause')" :disabled="!store.replayState?.playing">
        ⏸ 暂停
      </button>
      <button class="ctrl-btn" @click="replayControl('stop')">
        ⏹ 停止
      </button>
      <div class="speed-control">
        <span>倍速:</span>
        <select v-model.number="selectedSpeed" @change="changeSpeed">
          <option :value="0.5">0.5×</option>
          <option :value="1">1×</option>
          <option :value="2">2×</option>
          <option :value="4">4×</option>
          <option :value="8">8×</option>
        </select>
      </div>
      <span class="frame-info">
        {{ store.replayState?.current_index ?? 0 }} / {{ store.replayState?.total_frames ?? 0 }}
      </span>
    </div>

    <!-- 时间轴 -->
    <div class="timeline-track" ref="trackRef" @click="seekTimeline">
      <div
        class="timeline-progress"
        :style="{ width: progressPercent + '%' }"
      ></div>
      <div
        class="timeline-thumb"
        :style="{ left: progressPercent + '%' }"
      ></div>
    </div>

    <!-- 时间标签 -->
    <div class="timeline-labels">
      <span>{{ formatTime(store.replayState?.session?.start_time ?? 0) }}</span>
      <span>{{ formatTime(store.replayState?.session?.end_time ?? 0) }}</span>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import { useSystemStore } from '@/stores/system'

const store = useSystemStore()
const trackRef = ref<HTMLElement | null>(null)
const selectedSpeed = ref(1)

const progressPercent = computed(() => {
  const current = store.replayState?.current_index ?? 0
  const total = store.replayState?.total_frames ?? 1
  return (current / total) * 100
})

async function replayControl(action: string) {
  try {
    await fetch('/api/replay/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action, speed: selectedSpeed.value }),
    })
  } catch (e) {
    console.error('回放控制失败:', e)
  }
}

function changeSpeed() {
  replayControl('play')
}

function seekTimeline(e: MouseEvent) {
  if (!trackRef.value) return
  const rect = trackRef.value.getBoundingClientRect()
  const pct = (e.clientX - rect.left) / rect.width
  fetch('/api/replay/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'seek', position: pct }),
  }).catch(() => {})
}

function formatTime(ts: number): string {
  const d = new Date(ts * 1000)
  return d.toLocaleTimeString('zh-CN', { hour12: false })
}
</script>

<style scoped>
.timeline-container {
  background: rgba(0, 0, 0, 0.85);
  border-radius: 8px;
  padding: 8px 12px;
  color: #fff;
  font-size: 12px;
  min-width: 220px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.15);
}

.replay-controls {
  display: flex;
  align-items: center;
  gap: 6px;
  margin-bottom: 6px;
  flex-wrap: wrap;
}

.ctrl-btn {
  background: rgba(255, 255, 255, 0.1);
  border: 1px solid rgba(255, 255, 255, 0.2);
  color: #fff;
  padding: 4px 10px;
  border-radius: 4px;
  cursor: pointer;
  font-size: 12px;
}

.ctrl-btn:hover:not(:disabled) { background: rgba(255, 255, 255, 0.2); }
.ctrl-btn:disabled { opacity: 0.4; cursor: default; }

.speed-control {
  display: flex;
  align-items: center;
  gap: 4px;
}

.speed-control select {
  background: rgba(255, 255, 255, 0.1);
  color: #fff;
  border: 1px solid rgba(255, 255, 255, 0.2);
  padding: 2px 4px;
  border-radius: 3px;
  font-size: 11px;
}

.frame-info {
  font-family: 'Courier New', monospace;
  opacity: 0.7;
  margin-left: 4px;
}

.timeline-track {
  height: 6px;
  background: rgba(255, 255, 255, 0.15);
  border-radius: 3px;
  cursor: pointer;
  position: relative;
  margin: 4px 0;
}

.timeline-progress {
  height: 100%;
  background: #00bcd4;
  border-radius: 3px;
  transition: width 0.2s;
}

.timeline-thumb {
  position: absolute;
  top: -3px;
  width: 12px;
  height: 12px;
  background: #00bcd4;
  border-radius: 50%;
  margin-left: -6px;
  box-shadow: 0 0 6px rgba(0, 188, 212, 0.6);
}

.timeline-labels {
  display: flex;
  justify-content: space-between;
  font-size: 10px;
  opacity: 0.5;
}
</style>
