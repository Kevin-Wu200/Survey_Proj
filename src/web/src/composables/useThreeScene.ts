/**
 * useThreeScene.ts — Three.js 场景全生命周期管理
 *
 * 功能：
 * - WebGL 渲染器初始化与销毁
 * - GLB 模型加载（GLTFLoader）
 * - OrbitControls 视角控制
 * - 自适应窗口大小
 * - 资源释放（防止内存泄漏）
 */
import { ref, shallowRef, type Ref } from 'vue'
import {
  Scene,
  WebGLRenderer,
  PerspectiveCamera,
  AmbientLight,
  DirectionalLight,
  Color,
  Box3,
  Vector3,
  type Object3D,
} from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

export interface ThreeSceneInstance {
  scene: Ref<Scene | null>
  camera: Ref<PerspectiveCamera | null>
  controls: Ref<OrbitControls | null>
  renderer: Ref<WebGLRenderer | null>
  initScene: (containerId: string) => void
  loadModel: (url: string) => Promise<void>
  destroyScene: () => void
  resize: () => void
}

export function useThreeScene(): ThreeSceneInstance {
  // 使用 shallowRef 避免 Vue 响应式系统深度追踪 Three.js 内部对象
  const scene = shallowRef<Scene | null>(null)
  const camera = shallowRef<PerspectiveCamera | null>(null)
  const controls = shallowRef<OrbitControls | null>(null)
  const renderer = shallowRef<WebGLRenderer | null>(null)

  // 内部变量
  let _container: HTMLElement | null = null
  let _animationId: number | null = null
  let _resizeObserver: ResizeObserver | null = null
  let _currentModel: Object3D | null = null
  const _gltfLoader = new GLTFLoader()

  /**
   * 渲染循环
   */
  function animate() {
    _animationId = requestAnimationFrame(animate)
    if (controls.value) {
      controls.value.update()
    }
    if (renderer.value && scene.value && camera.value) {
      renderer.value.render(scene.value, camera.value)
    }
  }

  /**
   * 递归释放 Object3D 及其子对象的所有资源
   */
  function disposeObject3D(obj: Object3D): void {
    obj.traverse((child: any) => {
      if (child.geometry) {
        child.geometry.dispose()
      }
      if (child.material) {
        const materials = Array.isArray(child.material) ? child.material : [child.material]
        for (const mat of materials) {
          // 释放材质中的纹理
          for (const key of Object.keys(mat)) {
            const value = mat[key]
            if (value && typeof value === 'object' && 'isTexture' in value) {
              value.dispose()
            }
          }
          mat.dispose()
        }
      }
    })
  }

  /**
   * 初始化 Three.js 场景
   */
  function initScene(containerId: string): void {
    // 如果已初始化，先销毁
    if (scene.value) {
      destroyScene()
    }

    _container = document.getElementById(containerId)
    if (!_container) {
      console.error(`[ThreeScene] 容器元素 "#${containerId}" 不存在`)
      return
    }

    const width = _container.clientWidth
    const height = _container.clientHeight

    // 渲染器
    const r = new WebGLRenderer({
      antialias: true,
      alpha: true,
    })
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2)) // 限制像素比，节省性能
    r.setSize(width, height)
    r.shadowMap.enabled = false
    _container.appendChild(r.domElement)
    renderer.value = r

    // 场景
    const s = new Scene()
    s.background = new Color(0x1a1a2e) // 深色背景，类似太空蓝
    scene.value = s

    // 相机
    const cam = new PerspectiveCamera(
      60, // FOV
      width / height, // 宽高比
      0.1, // 近裁剪面
      10000, // 远裁剪面
    )
    // 默认俯视角度
    cam.position.set(5, 8, 5)
    cam.lookAt(0, 0, 0)
    camera.value = cam

    // 轨道控制器
    const ctrl = new OrbitControls(cam, r.domElement)
    ctrl.enableDamping = true
    ctrl.dampingFactor = 0.08
    ctrl.minDistance = 0.5
    ctrl.maxDistance = 500
    ctrl.maxPolarAngle = Math.PI / 2 + 0.3 // 限制俯角，防止翻到底部
    ctrl.target.set(0, 0, 0)
    ctrl.update()
    controls.value = ctrl

    // 光照
    const ambientLight = new AmbientLight(0xffffff, 0.6)
    s.add(ambientLight)

    const directionalLight = new DirectionalLight(0xffffff, 0.8)
    directionalLight.position.set(10, 20, 5)
    directionalLight.castShadow = false
    s.add(directionalLight)

    // 辅助元素（开发时可启用，生产环境可注释掉）
    // import { AxesHelper, GridHelper } from 'three'
    // s.add(new AxesHelper(2))
    // s.add(new GridHelper(10, 10))

    // 开始渲染循环
    animate()

    // 监听容器大小变化
    if (_resizeObserver) {
      _resizeObserver.disconnect()
    }
    _resizeObserver = new ResizeObserver(() => {
      resize()
    })
    _resizeObserver.observe(_container)

    console.log('[ThreeScene] 3D 场景初始化完成')
  }

  /**
   * 加载 GLB 模型
   */
  function loadModel(url: string): Promise<void> {
    return new Promise((resolve, reject) => {
      if (!scene.value) {
        reject(new Error('场景未初始化，请先调用 initScene()'))
        return
      }

      // 移除之前的模型
      if (_currentModel) {
        scene.value.remove(_currentModel)
        disposeObject3D(_currentModel)
        _currentModel = null
      }

      _gltfLoader.load(
        url,
        (gltf) => {
          _currentModel = gltf.scene
          scene.value!.add(_currentModel)

          // 自动调整视角以适配模型
          if (controls.value && camera.value) {
            const box = new Box3().setFromObject(_currentModel)
            const size = box.getSize(new Vector3())
            const center = box.getCenter(new Vector3())
            const maxDim = Math.max(size.x, size.y, size.z)
            const fov = camera.value.fov * (Math.PI / 180)
            const distance = maxDim / (2 * Math.tan(fov / 2)) * 1.5

            camera.value.position.set(
              center.x + distance * 0.5,
              center.y + distance * 0.8,
              center.z + distance * 0.5,
            )
            controls.value.target.copy(center)
            controls.value.update()
          }

          console.log(`[ThreeScene] GLB 模型加载完成: ${url}`)
          resolve()
        },
        (progress) => {
          if (progress.total > 0) {
            const pct = Math.round((progress.loaded / progress.total) * 100)
            console.log(`[ThreeScene] 加载进度: ${pct}%`)
          }
        },
        (error) => {
          console.error(`[ThreeScene] GLB 加载失败: ${url}`, error)
          reject(error)
        },
      )
    })
  }

  /**
   * 响应窗口尺寸变化
   */
  function resize(): void {
    if (!_container || !renderer.value || !camera.value) return

    const width = _container.clientWidth
    const height = _container.clientHeight

    if (width === 0 || height === 0) return

    renderer.value.setSize(width, height)
    camera.value.aspect = width / height
    camera.value.updateProjectionMatrix()
  }

  /**
   * 销毁场景，释放所有资源
   */
  function destroyScene(): void {
    // 停止动画
    if (_animationId !== null) {
      cancelAnimationFrame(_animationId)
      _animationId = null
    }

    // 停止 ResizeObserver
    if (_resizeObserver) {
      _resizeObserver.disconnect()
      _resizeObserver = null
    }

    // 释放模型
    if (_currentModel && scene.value) {
      scene.value.remove(_currentModel)
      disposeObject3D(_currentModel)
      _currentModel = null
    }

    // 释放场景中所有对象
    if (scene.value) {
      while (scene.value.children.length > 0) {
        const child = scene.value.children[0]
        scene.value.remove(child)
        if ((child as any).isLight) {
          // 灯光无需 dispose
        } else {
          disposeObject3D(child)
        }
      }
    }

    // 释放 OrbitControls
    if (controls.value) {
      controls.value.dispose()
      controls.value = null
    }

    // 释放渲染器
    if (renderer.value) {
      renderer.value.dispose()
      if (_container && renderer.value.domElement.parentElement === _container) {
        _container.removeChild(renderer.value.domElement)
      }
      renderer.value = null
    }

    scene.value = null
    camera.value = null
    _container = null
    console.log('[ThreeScene] 3D 场景已销毁，资源已释放')
  }

  return {
    scene,
    camera,
    controls,
    renderer,
    initScene,
    loadModel,
    destroyScene,
    resize,
  }
}
