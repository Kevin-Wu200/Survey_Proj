import { fileURLToPath, URL } from 'node:url'
import { readFileSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { envPlugin } from './vite-env-plugin'

const __dirname = dirname(fileURLToPath(import.meta.url))

/** 从 env.txt 读取配置，返回解析后的键值对 */
function loadEnv(): Record<string, string> {
  const env: Record<string, string> = {}
  try {
    const envPath = resolve(__dirname, '..', '..', 'env.txt')
    const content = readFileSync(envPath, 'utf-8')
    for (const line of content.split('\n')) {
      const trimmed = line.trim()
      if (!trimmed || trimmed.startsWith('#')) continue
      const eqIndex = trimmed.indexOf('=')
      if (eqIndex === -1) continue
      const key = trimmed.slice(0, eqIndex).trim()
      const value = trimmed.slice(eqIndex + 1).trim()
      if (key) env[key] = value
    }
  } catch {
    console.warn('[vite] 未能读取 env.txt，使用默认值')
  }
  return env
}

const env = loadEnv()

export default defineConfig({
  plugins: [vue(), envPlugin()],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  server: {
    host: env['FRONTEND_HOST'] || '0.0.0.0',
    port: parseInt(env['FRONTEND_PORT'] || '3000', 10),
    proxy: {
      '/api': {
        target: `http://${env['BACKEND_HOST'] || 'localhost'}:${env['BACKEND_PORT'] || '8000'}`,
        changeOrigin: true,
      },
      '/ws': {
        target: `ws://${env['BACKEND_HOST'] || 'localhost'}:${env['BACKEND_PORT'] || '8000'}`,
        ws: true,
      },
    },
  },
})
