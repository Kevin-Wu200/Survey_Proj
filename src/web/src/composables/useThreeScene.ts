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
  OrthographicCamera,
  AmbientLight,
  DirectionalLight,
  Color,
  Box3,
  Vector3,
  Mesh,
  MeshBasicMaterial,
  PlaneGeometry,
  TextureLoader,
  DoubleSide,
  type Object3D,
} from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

// =============================================================================
// 2D 地图世界配准参数（来自 2D_Map.pgw 世界文件）
// =============================================================================
const MAP_WORLD_CONFIG = {
  pixelSize: 0.3478653194,       // 像素尺寸 (世界单位/像素)
  imageWidth: 1750,               // 图像宽度 (像素)
  imageHeight: 1654,              // 图像高度 (像素)
  worldWidth: 608.76,             // 覆盖世界宽度 (pixelSize * imageWidth)
  worldHeight: 575.37,            // 覆盖世界高度 (pixelSize * imageHeight)
  worldCenterX: 56.229256,        // 模型中心 X (东)
  worldCenterZ: 75.805115,        // 模型中心 Z (北，对应 .pgw Y)
  planeYOffset: 0.5,              // 平面高度偏移（稍高于地形）
}

export interface ThreeSceneInstance {
  scene: Ref<Scene | null>
  camera: Ref<PerspectiveCamera | OrthographicCamera | null>
  controls: Ref<OrbitControls | null>
  renderer: Ref<WebGLRenderer | null>
  initScene: (containerId: string) => void
  loadModel: (url: string) => Promise<Object3D | null>
  load2DMap: (imageUrl: string) => Promise<void>
  set2DMapVisible: (visible: boolean) => void
  setOrthographicCamera: (enabled: boolean) => void
  destroyScene: () => void
  resize: () => void
  getCurrentModel: () => Object3D | null
}

export function useThreeScene(): ThreeSceneInstance {
  // 使用 shallowRef 避免 Vue 响应式系统深度追踪 Three.js 内部对象
  const scene = shallowRef<Scene | null>(null)
  const camera = shallowRef<PerspectiveCamera | OrthographicCamera | null>(null)
  const controls = shallowRef<OrbitControls | null>(null)
  const renderer = shallowRef<WebGLRenderer | null>(null)

  // 内部变量
  let _container: HTMLElement | null = null
  let _animationId: number | null = null
  let _resizeObserver: ResizeObserver | null = null
  let _currentModel: Object3D | null = null
  const _gltfLoader = new GLTFLoader()
  const _textureLoader = new TextureLoader()

  // 2D 地图平面
  let _2DMapPlane: Mesh | null = null
  let _2DMapTexture: any = null
  let _orthoCamera: OrthographicCamera | null = null
  let _is2DMapEnabled = false

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
   * @returns 加载的模型根节点
   */
  function loadModel(url: string): Promise<Object3D | null> {
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
            const cam = camera.value
            if ('isPerspectiveCamera' in cam && cam.isPerspectiveCamera) {
              const fov = (cam as PerspectiveCamera).fov * (Math.PI / 180)
              const distance = maxDim / (2 * Math.tan(fov / 2)) * 1.5

              cam.position.set(
                center.x + distance * 0.5,
                center.y + distance * 0.8,
                center.z + distance * 0.5,
              )
            }
            controls.value.target.copy(center)
            controls.value.update()
          }

          console.log(`[ThreeScene] GLB 模型加载完成: ${url}`)
          resolve(_currentModel)
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
   * 获取当前加载的模型根节点
   */
  function getCurrentModel(): Object3D | null {
    return _currentModel
  }

  /**
   * 加载 2D 地图纹理平面
   * 使用 .pgw 世界配准参数将平面精确放置在模型上方
   */
  function load2DMap(imageUrl: string): Promise<void> {
    return new Promise((resolve, reject) => {
      if (!scene.value) {
        reject(new Error('场景未初始化，请先调用 initScene()'))
        return
      }

      // 移除旧的地图平面
      _remove2DMapPlane()

      const cfg = MAP_WORLD_CONFIG

      _textureLoader.load(
        imageUrl,
        (texture) => {
          _2DMapTexture = texture

          // 创建水平平面（默认在 XY 平面，需旋转到 XZ 平面）
          const geometry = new PlaneGeometry(cfg.worldWidth, cfg.worldHeight)
          const material = new MeshBasicMaterial({
            map: texture,
            side: DoubleSide,
            transparent: true,
            opacity: 1.0,
            depthTest: false,
            depthWrite: false,
          })
          const plane = new Mesh(geometry, material)

          // 旋转使平面水平（默认 XY → 绕 X 轴 -90° 变为 XZ）
          plane.rotation.x = -Math.PI / 2

          // 定位到世界坐标 (中心 X, Y偏移, 中心 Z)
          plane.position.set(
            cfg.worldCenterX,
            cfg.planeYOffset,
            cfg.worldCenterZ,
          )

          plane.name = '_2DMapPlane'
          plane.renderOrder = -1 // 最先渲染，作为背景层
          plane.visible = false // 默认隐藏
          scene.value!.add(plane)
          _2DMapPlane = plane

          console.log(`[ThreeScene] 2D 地图平面已加载: ${cfg.worldWidth.toFixed(1)} × ${cfg.worldHeight.toFixed(1)} 世界单位`)
          resolve()
        },
        undefined, // 进度回调（不需要）
        (error) => {
          console.error('[ThreeScene] 2D 地图纹理加载失败:', error)
          reject(error)
        },
      )
    })
  }

  /**
   * 切换 2D 地图平面的可见性
   */
  function set2DMapVisible(visible: boolean): void {
    if (_2DMapPlane) {
      _2DMapPlane.visible = visible
      console.log(`[ThreeScene] 2D 地图平面: ${visible ? '显示' : '隐藏'}`)
    }
  }

  /**
   * 切换正交/透视相机模式
   *
   * 正交模式用于 2D 俯视地图；
   * 透视模式用于 3D 地形浏览。
   */
  function setOrthographicCamera(enabled: boolean): void {
    if (!renderer.value || !scene.value || !_container || !controls.value) return

    const oldCamera = camera.value
    if (!oldCamera) return

    const width = _container.clientWidth
    const height = _container.clientHeight
    const aspect = width / height

    if (enabled) {
      // 切换到正交相机（俯视）
      if (!_orthoCamera) {
        const halfH = MAP_WORLD_CONFIG.worldHeight / 2 * 1.1
        const halfW = halfH * aspect
        _orthoCamera = new OrthographicCamera(
          -halfW, halfW, halfH, -halfH, 0.1, 500,
        )
      }

      // 保存当前透视相机的位置和目标
      const savedPos = oldCamera.position.clone()
      const savedTarget = controls.value.target.clone()

      // 正交相机从正上方俯视地图中心
      const cfg = MAP_WORLD_CONFIG
      const camHeight = Math.max(cfg.worldWidth, cfg.worldHeight) * 0.6
      _orthoCamera.position.set(cfg.worldCenterX, camHeight, cfg.worldCenterZ)
      _orthoCamera.lookAt(cfg.worldCenterX, 0, cfg.worldCenterZ)

      camera.value = _orthoCamera
      _is2DMapEnabled = true

      // 调整 OrbitControls 附加到新相机
      controls.value.object = _orthoCamera
      controls.value.target.set(cfg.worldCenterX, 0, cfg.worldCenterZ)
      controls.value.enableRotate = false // 2D 模式禁用旋转
      controls.value.update()

      console.log('[ThreeScene] 已切换到正交俯视相机 (2D 地图模式)')
    } else {
      // 恢复到透视相机
      if (_orthoCamera) {
        _orthoCamera = null
      }
      // 恢复原始透视相机
      camera.value = oldCamera
      _is2DMapEnabled = false

      // 恢复 OrbitControls
      controls.value.object = oldCamera
      controls.value.enableRotate = true
      controls.value.update()

      console.log('[ThreeScene] 已恢复到透视相机 (3D 地形模式)')
    }
  }

  /**
   * 内部：移除 2D 地图平面并释放资源
   */
  function _remove2DMapPlane(): void {
    if (_2DMapPlane) {
      if (scene.value) {
        scene.value.remove(_2DMapPlane)
      }
      _2DMapPlane.geometry.dispose()
      ;(_2DMapPlane.material as MeshBasicMaterial).dispose()
      _2DMapPlane = null
    }
    if (_2DMapTexture) {
      _2DMapTexture.dispose()
      _2DMapTexture = null
    }
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

    if (_is2DMapEnabled && _orthoCamera) {
      // 正交相机：按比例调整视口
      const aspect = width / height
      const halfH = MAP_WORLD_CONFIG.worldHeight / 2 * 1.1
      const halfW = halfH * aspect
      _orthoCamera.left = -halfW
      _orthoCamera.right = halfW
      _orthoCamera.top = halfH
      _orthoCamera.bottom = -halfH
      _orthoCamera.updateProjectionMatrix()
    } else if ('aspect' in camera.value) {
      // 透视相机
      ;(camera.value as PerspectiveCamera).aspect = width / height
      camera.value.updateProjectionMatrix()
    }
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

    // 清理 2D 地图平面
    _remove2DMapPlane()
    _orthoCamera = null
    _is2DMapEnabled = false

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
    load2DMap,
    set2DMapVisible,
    setOrthographicCamera,
    destroyScene,
    resize,
    getCurrentModel,
  }
}
