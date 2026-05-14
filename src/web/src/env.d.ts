/// <reference types="vite/client" />

declare module '*.vue' {
  import type { DefineComponent } from 'vue'
  const component: DefineComponent<{}, {}, any>
  export default component
}

// 高德地图全局变量声明
declare const AMap: any

// env.txt 注入的全局常量 (由 vite-env-plugin 注入)
declare const __AMAP_KEY__: string
declare const __BACKEND_HOST__: string
declare const __BACKEND_PORT__: string
declare const __FRONTEND_HOST__: string
declare const __FRONTEND_PORT__: string
declare const __CENTER_LAT__: string
declare const __CENTER_LNG__: string
