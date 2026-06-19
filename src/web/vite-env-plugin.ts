/**
 * Vite 插件: 从项目根目录 env.txt 读取环境配置并注入前端
 *
 * 功能:
 * - 读取 env.txt (key=value 格式)
 * - 通过 define 注入全局常量 (__BACKEND_HOST__ 等)
 *
 * 用法:
 *   import { envPlugin } from './vite-env-plugin'
 *   // 在 vite.config.ts 的 plugins 中添加 envPlugin()
 */
import type { Plugin, ResolvedConfig } from 'vite'
import { readFileSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const __dirname = dirname(fileURLToPath(import.meta.url))

/** 解析 env.txt key=value 格式，忽略注释行和空行 */
function parseEnvFile(filePath: string): Record<string, string> {
  const env: Record<string, string> = {}

  try {
    const content = readFileSync(filePath, 'utf-8')
    for (const line of content.split('\n')) {
      const trimmed = line.trim()
      // 跳过空行和注释行
      if (!trimmed || trimmed.startsWith('#')) continue

      const eqIndex = trimmed.indexOf('=')
      if (eqIndex === -1) continue

      const key = trimmed.slice(0, eqIndex).trim()
      const value = trimmed.slice(eqIndex + 1).trim()
      if (key) {
        env[key] = value
      }
    }
  } catch {
    console.warn('[env-plugin] 未能读取 env.txt，使用默认值')
  }

  return env
}

export function envPlugin(
  envFilePath: string = resolve(__dirname, '..', '..', 'env.txt'),
): Plugin {
  let config: ResolvedConfig | undefined

  return {
    name: 'vite-plugin-env-txt',
    enforce: 'pre',

    /** 将 env.txt 中的值注入为全局常量 */
    config(userConfig, { mode }) {
      const envVars = parseEnvFile(envFilePath)

      // 定义全局常量，注入到客户端代码
      const define: Record<string, string> = {}
      for (const [key, value] of Object.entries(envVars)) {
        define[`__${key}__`] = JSON.stringify(value)
      }

      return {
        define,
      }
    },

    configResolved(resolvedConfig) {
      config = resolvedConfig
    },
  }
}
