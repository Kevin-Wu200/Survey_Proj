import { ref, onUnmounted } from 'vue'
import type { SystemState, WSMessage } from '@/types'

/** WebSocket 连接管理 composable */
export function useWebSocket(url: string) {
  const connected = ref(false)
  const systemState = ref<SystemState | null>(null)
  const error = ref<string | null>(null)
  const reconnectDelay = 3000

  let ws: WebSocket | null = null
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null

  function connect() {
    if (ws) {
      ws.close()
    }

    try {
      ws = new WebSocket(url)
    } catch (e) {
      error.value = 'WebSocket 连接失败'
      scheduleReconnect()
      return
    }

    ws.onopen = () => {
      console.log('[WS] 已连接')
      connected.value = true
      error.value = null
    }

    ws.onmessage = (event) => {
      try {
        const msg: WSMessage = JSON.parse(event.data)
        if (msg.type === 'init' || msg.type === 'state_update') {
          systemState.value = msg.data
        }
      } catch (e) {
        console.error('[WS] 消息解析失败:', e)
      }
    }

    ws.onclose = () => {
      console.log('[WS] 已断开')
      connected.value = false
      ws = null
      scheduleReconnect()
    }

    ws.onerror = () => {
      error.value = 'WebSocket 通信错误'
    }
  }

  function scheduleReconnect() {
    if (reconnectTimer) return
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null
      console.log('[WS] 尝试重连...')
      connect()
    }, reconnectDelay)
  }

  function sendPing() {
    if (ws && connected.value) {
      ws.send(JSON.stringify({ type: 'ping' }))
    }
  }

  function disconnect() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer)
      reconnectTimer = null
    }
    if (ws) {
      ws.close()
      ws = null
    }
  }

  // 定时心跳
  const pingInterval = setInterval(sendPing, 10000)

  onUnmounted(() => {
    clearInterval(pingInterval)
    disconnect()
  })

  // 初始连接
  connect()

  return {
    connected,
    systemState,
    error,
    sendPing,
    reconnect: connect,
  }
}
