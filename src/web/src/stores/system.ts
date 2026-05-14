import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { SystemState, SystemAlert } from '@/types'
import { useWebSocket } from '@/composables/useWebSocket'

/**
 * 系统状态 Store — 二阶段增强版
 * 管理 UAV/UGV 数据、航点任务、导航、告警、回放
 */
export const useSystemStore = defineStore('system', () => {
  // WebSocket 连接
  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const wsUrl = `${wsProtocol}//${window.location.host}/ws`
  const { connected, systemState, error: wsError, alerts } = useWebSocket(wsUrl)

  // 状态数据
  const serverTime = ref(0)
  const clientsCount = ref(0)

  // UAV 数据
  const uavPosition = computed(() => systemState.value?.uav_position)
  const uavStatus = computed(() => systemState.value?.uav_status)

  // UGV 数据
  const ugvPosition = computed(() => systemState.value?.ugv_position)
  const ugvStatus = computed(() => systemState.value?.ugv_status)

  // 二阶段: 系统模式
  const systemMode = computed(() => systemState.value?.system_mode ?? 0)
  const systemModeName = computed(() => systemState.value?.system_mode_name ?? '仿真模式')

  // 二阶段: UAV 航点任务
  const uavMissionStatus = computed(() => systemState.value?.uav_mission_status)

  // 二阶段: UGV 导航
  const ugvNavStatus = computed(() => systemState.value?.ugv_nav_status)

  // 二阶段: 回放状态
  const replayState = computed(() => systemState.value?.replay)

  // 地图中心 (UAV 和 UGV 位置中点)
  const mapCenter = computed(() => {
    const uav = uavPosition.value
    const ugv = ugvPosition.value
    if (uav && ugv) {
      return {
        lng: (uav.longitude + ugv.longitude) / 2,
        lat: (uav.latitude + ugv.latitude) / 2,
      }
    }
    if (uav) return { lng: uav.longitude, lat: uav.latitude }
    if (ugv) return { lng: ugv.longitude, lat: ugv.latitude }
    return { lng: 120.0, lat: 30.0 }
  })

  return {
    connected,
    wsError,
    serverTime,
    clientsCount,
    uavPosition,
    uavStatus,
    ugvPosition,
    ugvStatus,
    mapCenter,
    // 二阶段新增
    systemMode,
    systemModeName,
    uavMissionStatus,
    ugvNavStatus,
    replayState,
    alerts,
  }
})
