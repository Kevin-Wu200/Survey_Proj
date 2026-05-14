/**
 * useTerrainQuery.ts — 3D 地形高度查询与坡度分析
 *
 * 功能：
 * - Raycaster 垂直向下查询地形高度
 * - 多点采样计算坡度（角度 + 法线方向）
 * - 支持多个 mesh 的场景（合并所有 mesh 参与碰撞检测）
 */
import { Raycaster, Vector3, type Mesh, type Object3D } from 'three'

/** 坡度查询结果 */
export interface SlopeInfo {
  height: number        // 地形高度 (Y 坐标)
  angle: number         // 坡度角度 (度, 0=水平)
  normal: Vector3       // 地表法线 (单位向量)
  steep: boolean        // 是否超过最大爬坡角度
}

export interface TerrainQueryInstance {
  setTerrainObjects: (objects: Object3D[]) => void
  getHeightAt: (x: number, z: number) => number | null
  getSlopeAt: (x: number, z: number, maxAngle?: number) => SlopeInfo | null
  getAllMeshes: () => Mesh[]
  clear: () => void
}

export function useTerrainQuery(): TerrainQueryInstance {
  // 参与碰撞检测的所有 mesh
  const _meshes: Mesh[] = []

  // 共用的 Raycaster（避免重复创建）
  const _raycaster = new Raycaster()
  // 射线起点方向：从上往下 (0, -1, 0)
  const _downDir = new Vector3(0, -1, 0)
  // 最大查询高度（射线起点 Y 坐标）
  const RAY_ORIGIN_Y = 10000

  /**
   * 设置参与地形查询的 3D 对象列表
   * 会自动遍历提取所有 Mesh
   */
  function setTerrainObjects(objects: Object3D[]): void {
    _meshes.length = 0
    for (const obj of objects) {
      obj.traverse((child: Object3D) => {
        if ((child as any).isMesh) {
          _meshes.push(child as Mesh)
        }
      })
    }
    console.log(`[TerrainQuery] 已注册 ${_meshes.length} 个地形 mesh`)
  }

  /**
   * 获取指定世界坐标 (x, z) 处的地形高度
   * @returns 地形表面的 Y 坐标，无交点返回 null
   */
  function getHeightAt(x: number, z: number): number | null {
    if (_meshes.length === 0) return null

    // 射线从高处垂直向下
    const origin = new Vector3(x, RAY_ORIGIN_Y, z)
    _raycaster.set(origin, _downDir)
    _raycaster.far = RAY_ORIGIN_Y * 2

    const intersections = _raycaster.intersectObjects(_meshes, false)

    if (intersections.length > 0) {
      return intersections[0].point.y
    }
    return null
  }

  /**
   * 获取指定位置的坡度和地形信息
   * 在目标点周围采样 5 个点（中心 + 前后左右），拟合法线计算坡度
   *
   * @param x - 世界 X 坐标
   * @param z - 世界 Z 坐标
   * @param maxAngle - 最大爬坡角度（度），默认 30
   * @returns 坡度信息，无地形时返回 null
   */
  function getSlopeAt(x: number, z: number, maxAngle: number = 30): SlopeInfo | null {
    const sampleRadius = 0.5 // 采样半径（米）

    // 5 点采样：中心 + 东/西/南/北
    const center = getHeightAt(x, z)
    const east = getHeightAt(x + sampleRadius, z)
    const west = getHeightAt(x - sampleRadius, z)
    const north = getHeightAt(x, z + sampleRadius)
    const south = getHeightAt(x, z - sampleRadius)

    if (center === null) return null

    // 用周围点计算法线（如果周围点缺失，用中心代替）
    const hEast = east ?? center
    const hWest = west ?? center
    const hNorth = north ?? center
    const hSouth = south ?? center

    // 计算梯度向量
    // dx: 每米的高度变化（东西方向）
    const dz = (hEast - hWest) / (2 * sampleRadius)
    // dz: 每米的高度变化（南北方向）
    const dx = (hNorth - hSouth) / (2 * sampleRadius)

    // 法线 = normalize(-gradient, 1) 即 (-dx, 1, -dz) 的归一化
    // 坡度角度 = acos(normal.y) = atan(sqrt(dx² + dz²))
    const gradMag = Math.sqrt(dx * dx + dz * dz)
    const angleRad = Math.atan(gradMag)
    const angleDeg = angleRad * (180 / Math.PI)

    // 法线向量
    const normal = new Vector3(-dx, 1, -dz).normalize()

    return {
      height: center,
      angle: angleDeg,
      normal,
      steep: angleDeg > maxAngle,
    }
  }

  /**
   * 获取所有注册的地形 mesh
   */
  function getAllMeshes(): Mesh[] {
    return _meshes
  }

  /**
   * 清空所有地形数据
   */
  function clear(): void {
    _meshes.length = 0
  }

  return {
    setTerrainObjects,
    getHeightAt,
    getSlopeAt,
    getAllMeshes,
    clear,
  }
}
