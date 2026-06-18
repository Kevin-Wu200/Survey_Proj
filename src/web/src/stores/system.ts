import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { SystemState } from '@/types'
import { useWebSocket } from '@/composables/useWebSocket'

/**
 * 系统状态 Store
 * 管理 UAV/UGV 数据和 WebSocket 连接
 */
export const useSystemStore = defineStore('system', () => {
  // WebSocket 连接
  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const wsUrl = `${wsProtocol}//${window.location.host}/ws`
  const { connected, systemState, error: wsError } = useWebSocket(wsUrl)

  // 状态数据
  const serverTime = ref(0)
  const clientsCount = ref(0)

  // UAV 数据
  const uavPosition = computed(() => systemState.value?.uav_position)
  const uavStatus = computed(() => systemState.value?.uav_status)

  // UGV 数据
  const ugvPosition = computed(() => systemState.value?.ugv_position)
  const ugvStatus = computed(() => systemState.value?.ugv_status)

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
  }
})
