<template>
  <div class="data-panel">
    <div class="panel-header">
      <span>📊 数据管理</span>
    </div>

    <!-- Tab 切换 -->
    <div class="tabs">
      <button
        v-for="tab in tabs"
        :key="tab.key"
        :class="['tab-btn', { active: activeTab === tab.key }]"
        @click="activeTab = tab.key"
      >
        {{ tab.label }}
      </button>
    </div>

    <!-- Tab 内容 -->
    <div class="tab-content">
      <!-- 采集进度 -->
      <div v-if="activeTab === 'progress'" class="progress-view">
        <div v-if="progressLoading" class="loading">加载中...</div>
        <div v-else-if="!progress" class="empty">暂无采集任务</div>
        <div v-else class="progress-detail">
          <div class="task-name">{{ progress.task_name || '任务 #' + progress.task_id }}</div>
          <div class="progress-bar-wrap">
            <div class="progress-bar">
              <div
                class="progress-fill"
                :style="{ width: progress.progress_percent + '%' }"
                :class="{ complete: progress.progress_percent >= 100 }"
              ></div>
            </div>
            <span class="progress-text">{{ progress.progress_percent }}%</span>
          </div>
          <div class="progress-stats">
            <div class="stat-row">
              <span>测绘面积</span>
              <span>{{ (progress.surveyed_area_sqm / 10000).toFixed(2) }} / {{ (progress.total_area_sqm / 10000).toFixed(2) }} ha</span>
            </div>
            <div class="stat-row">
              <span>UAV 照片</span>
              <span>{{ progress.uav_photos_taken }} 张</span>
            </div>
            <div class="stat-row">
              <span>UGV 里程</span>
              <span>{{ progress.ugv_distance_m.toFixed(1) }} m</span>
            </div>
            <div class="stat-row">
              <span>已用时间</span>
              <span>{{ formatDuration(progress.elapsed_seconds) }}</span>
            </div>
            <div class="stat-row" v-if="progress.estimated_completion_time > 0">
              <span>预计完成</span>
              <span>{{ formatTime(progress.estimated_completion_time) }}</span>
            </div>
          </div>
          <div class="status-badge" :class="progress.status">
            {{ statusLabels[progress.status] || progress.status }}
          </div>
        </div>
        <button class="refresh-btn" @click="fetchProgress">🔄 刷新</button>
      </div>

      <!-- 任务历史 -->
      <div v-if="activeTab === 'history'" class="history-view">
        <div class="filter-row">
          <select v-model="taskFilter" @change="fetchTasks">
            <option value="">全部状态</option>
            <option value="pending">待执行</option>
            <option value="running">执行中</option>
            <option value="completed">已完成</option>
            <option value="failed">失败</option>
          </select>
        </div>
        <div v-if="tasksLoading" class="loading">加载中...</div>
        <div v-else-if="tasks.length === 0" class="empty">暂无任务记录</div>
        <div v-else class="task-list">
          <div
            v-for="task in tasks"
            :key="task.id"
            class="task-item"
            :class="{ selected: selectedTaskId === task.id }"
            @click="selectTask(task.id)"
          >
            <div class="task-header">
              <span class="task-name">{{ task.task_name }}</span>
              <span :class="['task-status', task.status]">{{ statusLabels[task.status] || task.status }}</span>
            </div>
            <div class="task-meta">
              <span>{{ task.task_type }}</span>
              <span v-if="task.area_sqm">{{ (task.area_sqm / 10000).toFixed(2) }} ha</span>
              <span>{{ formatTime(task.created_at) }}</span>
            </div>
            <!-- 融合成果概要 -->
            <div v-if="task.fusion_result" class="fusion-brief">
              <span>📐 粗RMSE: {{ (task.fusion_result.coarse_rmse * 100).toFixed(1) }}cm</span>
              <span>🎯 精RMSE: {{ (task.fusion_result.fine_rmse * 100).toFixed(1) }}cm</span>
            </div>
          </div>
        </div>
        <button class="refresh-btn" @click="fetchTasks">🔄 刷新</button>
      </div>

      <!-- 融合成果 -->
      <div v-if="activeTab === 'fusion'" class="fusion-view">
        <div v-if="selectedTaskId == null" class="empty">请先在「任务历史」中选择一个任务</div>
        <div v-else-if="fusionLoading" class="loading">加载中...</div>
        <div v-else-if="fusionResults.length === 0" class="empty">该任务暂无融合成果</div>
        <div v-else class="fusion-list">
          <div
            v-for="result in fusionResults"
            :key="result.id"
            class="fusion-item"
          >
            <div class="fusion-header">
              <span class="model-type">{{ modelTypeLabels[result.model_type] || result.model_type }}</span>
              <span class="fusion-points">{{ result.point_count.toLocaleString() }} 点</span>
            </div>
            <div class="fusion-rmse">
              <span>粗配准 RMSE: {{ (result.coarse_rmse * 100).toFixed(1) }} cm</span>
              <span :class="{ pass: result.fine_rmse < 0.05, fail: result.fine_rmse >= 0.05 }">
                精配准 RMSE: {{ (result.fine_rmse * 100).toFixed(1) }} cm
              </span>
            </div>
            <div class="fusion-meta">
              <span>{{ result.face_count.toLocaleString() }} 面片</span>
              <span>{{ formatTime(result.completed_at) }}</span>
            </div>
            <!-- 缩略图预览（V2.0 简略实现） -->
            <div v-if="result.thumbnail_url" class="thumbnail">
              <img :src="result.thumbnail_url" alt="融合缩略图" />
            </div>
          </div>
        </div>
        <button class="refresh-btn" @click="fetchFusionResults">🔄 刷新</button>
      </div>

      <!-- PGW 世界文件工具 -->
      <div v-if="activeTab === 'pgw'" class="pgw-view">
        <div v-if="pgwLoading" class="loading">加载中...</div>
        <div v-else-if="pgwModels.length === 0" class="empty">暂无可用的 GLB 模型</div>
        <div v-else class="pgw-form">
          <label class="pgw-label">选择模型</label>
          <select v-model="pgwSelectedModel" class="pgw-select">
            <option value="">-- 选择 GLB 模型 --</option>
            <option v-for="m in pgwModels" :key="m.filename" :value="m.filename">
              {{ m.filename }} ({{ (m.range[0] * m.range[1] / 10000).toFixed(2) }} ha)
            </option>
          </select>

          <label class="pgw-label">图像宽度 (px)</label>
          <input v-model.number="pgwImageWidth" type="number" min="256" max="8192" class="pgw-input" />

          <label class="pgw-label">图像高度 (px)</label>
          <input v-model.number="pgwImageHeight" type="number" min="256" max="8192" class="pgw-input" />

          <label class="pgw-label">边距比例</label>
          <input v-model.number="pgwMargin" type="number" min="0" max="0.5" step="0.01" class="pgw-input" />

          <button class="pgw-generate-btn" @click="generatePgw" :disabled="!pgwSelectedModel || pgwGenerating">
            {{ pgwGenerating ? '生成中...' : '🎯 生成 .pgw 世界文件' }}
          </button>

          <div v-if="pgwResult" class="pgw-result">
            <div class="pgw-result-header">生成结果</div>
            <div class="pgw-stat">模型中心: ({{ pgwResult.model_center[0].toFixed(2) }}, {{ pgwResult.model_center[1].toFixed(2) }})</div>
            <div class="pgw-stat">像素尺寸: {{ pgwResult.pixel_size.toFixed(6) }} 世界单位/px</div>
            <div class="pgw-stat">覆盖范围: {{ pgwResult.world_coverage[0] }} × {{ pgwResult.world_coverage[1] }} 世界单位</div>
            <div class="pgw-content-label">.pgw 文件内容 (6行):</div>
            <pre class="pgw-content">{{ pgwResult.pgw_content }}</pre>
            <button class="pgw-copy-btn" @click="copyPgwContent">📋 复制内容</button>
            <div v-if="pgwCopied" class="pgw-copied">✅ 已复制到剪贴板</div>
          </div>

          <div v-if="pgwError" class="pgw-error">❌ {{ pgwError }}</div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, watch } from 'vue'
import type { CollectionProgress, TaskRecord, FusionResult } from '@/types'

const activeTab = ref<'progress' | 'history' | 'fusion' | 'pgw'>('progress')
const tabs = [
  { key: 'progress' as const, label: '📡 采集进度' },
  { key: 'history' as const, label: '📋 任务历史' },
  { key: 'fusion' as const, label: '🔬 融合成果' },
  { key: 'pgw' as const, label: '🗺️ PGW工具' },
]

const statusLabels: Record<string, string> = {
  pending: '待执行',
  running: '执行中',
  surveying: '采集中',
  completed: '已完成',
  failed: '失败',
}

const modelTypeLabels: Record<string, string> = {
  pointcloud: '点云',
  mesh: '网格',
  textured_mesh: '纹理网格',
  fusion: '融合成果',
}

// 采集进度
const progress = ref<CollectionProgress | null>(null)
const progressLoading = ref(false)

async function fetchProgress() {
  progressLoading.value = true
  try {
    // 尝试获取最近运行中任务的进度
    const tasksResp = await fetch('/api/tasks?status=running&limit=1')
    const tasks = await tasksResp.json()
    if (Array.isArray(tasks) && tasks.length > 0) {
      const taskId = tasks[0].id
      const resp = await fetch(`/api/collection/progress/${taskId}`)
      if (resp.ok) {
        progress.value = await resp.json()
      }
    }
  } catch (e) {
    console.error('获取采集进度失败:', e)
  } finally {
    progressLoading.value = false
  }
}

// 任务历史
const tasks = ref<TaskRecord[]>([])
const tasksLoading = ref(false)
const taskFilter = ref('')
const selectedTaskId = ref<number | null>(null)

async function fetchTasks() {
  tasksLoading.value = true
  try {
    const params = new URLSearchParams()
    if (taskFilter.value) params.set('status', taskFilter.value)
    params.set('limit', '50')
    const resp = await fetch(`/api/tasks?${params}`)
    if (resp.ok) {
      tasks.value = await resp.json()
    }
  } catch (e) {
    console.error('获取任务列表失败:', e)
  } finally {
    tasksLoading.value = false
  }
}

function selectTask(taskId: number) {
  selectedTaskId.value = taskId
  activeTab.value = 'fusion'
  fetchFusionResults()
}

// 融合成果
const fusionResults = ref<FusionResult[]>([])
const fusionLoading = ref(false)

async function fetchFusionResults() {
  if (selectedTaskId.value == null) return
  fusionLoading.value = true
  try {
    const resp = await fetch(`/api/tasks/${selectedTaskId.value}/fusion`)
    if (resp.ok) {
      fusionResults.value = await resp.json()
    }
  } catch (e) {
    console.error('获取融合成果失败:', e)
  } finally {
    fusionLoading.value = false
  }
}

// PGW 世界文件工具
interface PgwModel {
  filename: string
  bbox_min: number[]
  bbox_max: number[]
  range: number[]
  center: number[]
}

interface PgwResult {
  model_center: number[]
  pixel_size: number
  world_coverage: number[]
  pgw_content: string
}

const pgwLoading = ref(false)
const pgwModels = ref<PgwModel[]>([])
const pgwSelectedModel = ref('')
const pgwImageWidth = ref(2048)
const pgwImageHeight = ref(2048)
const pgwMargin = ref(0.05)
const pgwGenerating = ref(false)
const pgwResult = ref<PgwResult | null>(null)
const pgwError = ref('')
const pgwCopied = ref(false)

async function fetchPgwModels() {
  pgwLoading.value = true
  try {
    const resp = await fetch('/api/pgw/list')
    if (resp.ok) {
      pgwModels.value = await resp.json()
    }
  } catch (e) {
    console.error('获取模型列表失败:', e)
  } finally {
    pgwLoading.value = false
  }
}

async function generatePgw() {
  if (!pgwSelectedModel.value) return
  pgwGenerating.value = true
  pgwError.value = ''
  pgwResult.value = null
  try {
    const formData = new FormData()
    formData.append('model_filename', pgwSelectedModel.value)
    formData.append('image_width', String(pgwImageWidth.value))
    formData.append('image_height', String(pgwImageHeight.value))
    formData.append('margin', String(pgwMargin.value))
    const resp = await fetch('/api/pgw/generate', { method: 'POST', body: formData })
    if (resp.ok) {
      pgwResult.value = await resp.json()
    } else {
      const err = await resp.json()
      pgwError.value = err.error || '生成失败'
    }
  } catch (e) {
    pgwError.value = '请求失败: ' + String(e)
  } finally {
    pgwGenerating.value = false
  }
}

function copyPgwContent() {
  if (!pgwResult.value) return
  navigator.clipboard.writeText(pgwResult.value.pgw_content).then(() => {
    pgwCopied.value = true
    setTimeout(() => { pgwCopied.value = false }, 2000)
  })
}

// 工具函数
function formatDuration(seconds: number): string {
  if (!seconds || seconds <= 0) return '-'
  const m = Math.floor(seconds / 60)
  const s = Math.floor(seconds % 60)
  if (m >= 60) {
    const h = Math.floor(m / 60)
    return `${h}h ${m % 60}m ${s}s`
  }
  return `${m}m ${s}s`
}

function formatTime(ts: number | string): string {
  if (!ts) return '-'
  const d = new Date(typeof ts === 'string' ? ts : ts * 1000)
  if (isNaN(d.getTime())) return '-'
  return d.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

onMounted(() => {
  fetchProgress()
  fetchTasks()
  fetchPgwModels()
})

// 当切换tab时自动刷新
watch(activeTab, (tab) => {
  if (tab === 'progress') fetchProgress()
  else if (tab === 'history') fetchTasks()
  else if (tab === 'fusion' && selectedTaskId.value != null) fetchFusionResults()
  else if (tab === 'pgw') fetchPgwModels()
})
</script>

<style scoped>
.data-panel {
  background: rgba(20, 25, 45, 0.85);
  border-radius: 8px;
  border: 1px solid rgba(255, 255, 255, 0.08);
  overflow: hidden;
  display: flex;
  flex-direction: column;
}

.panel-header {
  padding: 8px 12px;
  font-size: 13px;
  font-weight: 600;
  border-bottom: 1px solid rgba(255, 255, 255, 0.08);
  color: #90caf9;
}

.tabs {
  display: flex;
  border-bottom: 1px solid rgba(255, 255, 255, 0.06);
}

.tab-btn {
  flex: 1;
  padding: 7px 4px;
  font-size: 11px;
  background: transparent;
  border: none;
  color: #888;
  cursor: pointer;
  border-bottom: 2px solid transparent;
  transition: all 0.2s;
}

.tab-btn:hover {
  color: #ccc;
}

.tab-btn.active {
  color: #64b5f6;
  border-bottom-color: #64b5f6;
}

.tab-content {
  flex: 1;
  overflow-y: auto;
  padding: 8px;
  max-height: 320px;
}

.loading, .empty {
  padding: 20px;
  text-align: center;
  color: #666;
  font-size: 12px;
}

/* 采集进度 */
.progress-detail {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.task-name {
  font-size: 12px;
  font-weight: 500;
  color: #e0e0e0;
}

.progress-bar-wrap {
  display: flex;
  align-items: center;
  gap: 8px;
}

.progress-bar {
  flex: 1;
  height: 8px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  background: linear-gradient(90deg, #2196f3, #4caf50);
  border-radius: 4px;
  transition: width 0.5s ease;
}

.progress-fill.complete {
  background: #4caf50;
}

.progress-text {
  font-size: 12px;
  font-weight: 600;
  color: #64b5f6;
  min-width: 36px;
}

.progress-stats {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.stat-row {
  display: flex;
  justify-content: space-between;
  font-size: 11px;
  color: #999;
}

.stat-row span:last-child {
  color: #ccc;
}

.status-badge {
  align-self: flex-start;
  font-size: 10px;
  padding: 2px 8px;
  border-radius: 8px;
  text-transform: uppercase;
}

.status-badge.running, .status-badge.surveying {
  background: rgba(33, 150, 243, 0.2);
  color: #64b5f6;
}

.status-badge.completed {
  background: rgba(76, 175, 80, 0.2);
  color: #81c784;
}

.status-badge.failed {
  background: rgba(244, 67, 54, 0.2);
  color: #ef5350;
}

.status-badge.pending {
  background: rgba(255, 193, 7, 0.2);
  color: #ffd54f;
}

/* 任务历史 */
.filter-row {
  margin-bottom: 8px;
}

.filter-row select {
  width: 100%;
  padding: 5px 8px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  color: #ccc;
  font-size: 11px;
}

.task-list {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.task-item {
  padding: 8px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 6px;
  cursor: pointer;
  border: 1px solid transparent;
  transition: all 0.2s;
}

.task-item:hover {
  background: rgba(255, 255, 255, 0.06);
}

.task-item.selected {
  border-color: rgba(100, 181, 246, 0.4);
  background: rgba(100, 181, 246, 0.08);
}

.task-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 4px;
}

.task-header .task-name {
  font-size: 12px;
  color: #e0e0e0;
}

.task-status {
  font-size: 10px;
  padding: 1px 6px;
  border-radius: 6px;
}

.task-status.running {
  background: rgba(33, 150, 243, 0.2);
  color: #64b5f6;
}

.task-status.completed {
  background: rgba(76, 175, 80, 0.2);
  color: #81c784;
}

.task-status.failed {
  background: rgba(244, 67, 54, 0.2);
  color: #ef5350;
}

.task-status.pending {
  background: rgba(255, 193, 7, 0.2);
  color: #ffd54f;
}

.task-meta {
  display: flex;
  gap: 10px;
  font-size: 10px;
  color: #777;
}

.fusion-brief {
  display: flex;
  gap: 8px;
  margin-top: 4px;
  font-size: 10px;
  color: #81c784;
}

/* 融合成果 */
.fusion-list {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.fusion-item {
  padding: 8px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 6px;
}

.fusion-header {
  display: flex;
  justify-content: space-between;
  margin-bottom: 4px;
}

.model-type {
  font-size: 12px;
  color: #90caf9;
  font-weight: 500;
}

.fusion-points {
  font-size: 10px;
  color: #888;
}

.fusion-rmse {
  display: flex;
  flex-direction: column;
  gap: 2px;
  font-size: 10px;
  color: #aaa;
}

.fusion-rmse .pass {
  color: #81c784;
}

.fusion-rmse .fail {
  color: #ef5350;
}

.fusion-meta {
  display: flex;
  justify-content: space-between;
  font-size: 10px;
  color: #666;
  margin-top: 4px;
}

.thumbnail {
  margin-top: 6px;
  border-radius: 4px;
  overflow: hidden;
  max-height: 80px;
}

.thumbnail img {
  width: 100%;
  height: auto;
  object-fit: cover;
}

.refresh-btn {
  width: 100%;
  padding: 5px;
  margin-top: 8px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  color: #888;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.refresh-btn:hover {
  background: rgba(255, 255, 255, 0.1);
  color: #ccc;
}

/* PGW 工具 */
.pgw-form {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.pgw-label {
  font-size: 10px;
  color: #888;
  margin-top: 4px;
}

.pgw-select, .pgw-input {
  width: 100%;
  padding: 5px 8px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  color: #ccc;
  font-size: 11px;
}

.pgw-generate-btn {
  width: 100%;
  padding: 7px;
  margin-top: 8px;
  background: rgba(33, 150, 243, 0.2);
  border: 1px solid rgba(33, 150, 243, 0.3);
  border-radius: 4px;
  color: #64b5f6;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
}

.pgw-generate-btn:hover:not(:disabled) {
  background: rgba(33, 150, 243, 0.3);
}

.pgw-generate-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.pgw-result {
  margin-top: 8px;
  padding: 8px;
  background: rgba(76, 175, 80, 0.05);
  border: 1px solid rgba(76, 175, 80, 0.15);
  border-radius: 4px;
}

.pgw-result-header {
  font-size: 12px;
  font-weight: 600;
  color: #81c784;
  margin-bottom: 6px;
}

.pgw-stat {
  font-size: 10px;
  color: #aaa;
  margin-bottom: 2px;
}

.pgw-content-label {
  font-size: 10px;
  color: #888;
  margin-top: 6px;
  margin-bottom: 4px;
}

.pgw-content {
  font-size: 10px;
  font-family: 'Courier New', monospace;
  color: #ccc;
  background: rgba(0, 0, 0, 0.3);
  padding: 6px;
  border-radius: 4px;
  white-space: pre-wrap;
  max-height: 100px;
  overflow-y: auto;
}

.pgw-copy-btn {
  width: 100%;
  padding: 5px;
  margin-top: 4px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  color: #888;
  font-size: 11px;
  cursor: pointer;
}

.pgw-copy-btn:hover {
  background: rgba(255, 255, 255, 0.1);
}

.pgw-copied {
  font-size: 10px;
  color: #81c784;
  text-align: center;
  margin-top: 4px;
}

.pgw-error {
  font-size: 11px;
  color: #ef5350;
  margin-top: 8px;
  padding: 6px;
  background: rgba(244, 67, 54, 0.05);
  border-radius: 4px;
}
</style>
