# GLB 3D 场景功能设计文档

## 概述

系统采用**纯 3D 场景模式**运行，使用 Three.js 渲染 GLB 三维地形模型。仿真引擎已从前端接管（替代后端 `mock_data_generator`），UAV 和 UGV 在 3D 地形上进行仿真运动。

## 架构设计

### 纯 3D 前端仿真架构

```
用户选择场景 → GET /api/scenes → 返回 GLB 文件列表
→ 选中场景 → GET /api/scenes/{filename}
→ Three.js GLTFLoader 加载 → 渲染到 Canvas
→ terrainQuery 注册地形 mesh → 启动前端仿真

前端仿真循环 (useSimulation.ts):
  UAV: GPS椭圆航线 + 用户输入高度 → watch → 更新3D Sprite
  UGV: GPS坐标 → 世界坐标 → Raycaster查询地形高度 → 坡度检测 → watch → 更新3D Sprite
```

### 后端职责

后端仅提供场景文件服务（`/api/scenes` 端点），仿真完全由前端 Three.js 驱动。

- 仿真模式可通过环境变量 `SIM_MODE=frontend` 切换（禁用后端 mock_data_generator）

```
┌─────────────────────────────────────────────────────────┐
│  工具栏：[🗺️ 2D 地图] [🌐 3D 场景]  [场景: ▼ 场景A]  │
├─────────────────────────────────────────────────────────┤
│              .map-container (100%宽高)                  │
│  AMap 模式：<div id="amap-container">                 │
│  GLB 模式：<div id="three-container"> (WebGL Canvas)  │
│     ├── 3D 地形 (GLB 模型，可选隐藏)                     │
│     ├── 2D 地图平面 (2D_Map.png 纹理，可选显示)          │
│     ├── UAV/UGV 标记 (Sprite)                           │
│     ├── 轨迹线 (Line)                                   │
│     └── 航点 (SphereGeometry)                           │
├─────────────────────────────────────────────────────────┤
│  状态面板 / 航点工具栏 / 告警面板 / 时间轴（不变）     │
└─────────────────────────────────────────────────────────┘
```

### 数据流

```
选择场景 → loadModel → terrainQuery.setTerrainObjects(model)
         → simulation.setGeoOrigin → simulation.start()

仿真循环 (requestAnimationFrame, ~60fps):
  stepUAV(): GPS椭圆航线 → 更新 uavState
  stepUGV(): GPS坐标 → 世界坐标
    → terrainQuery.getHeightAt(x, z) → 获取地形高度
    → terrainQuery.getSlopeAt(x, z) → 坡度检测
    → 陡坡(>30°)则减速阻塞，尝试等高线绕行
    → 正常则贴地移动 → 更新 ugvState

watch(uavState) → threeMarkers.updateUAV() → 更新 Sprite + 轨迹
watch(ugvState) → threeMarkers.updateUGV() → 更新 Sprite + 轨迹
```

### UGV 物理约束

- **最大爬坡角度**：30°
- **坡度检测**：5 点采样拟合法线（中心 + 前后左右 0.5m）
- **陡坡阻塞**：坡度 > 30° 时减速到 0，状态标记为"陡坡阻塞"
- **等高线绕行**：阻塞 > 2 秒后，沿等高线方向（垂直于法线的水平分量）小幅试探移动
- **双方向尝试**：来回切换绕行方向以找到可行路径

## 文件变更清单

| 操作 | 文件 | 说明 |
|------|------|------|
| 新增 | `3D_Model/` | GLB 模型存储目录 |
| 新增 | `src/web/src/composables/useThreeScene.ts` | Three.js 场景管理 |
| 新增 | `src/web/src/composables/use3DMarkers.ts` | 3D 标记管理 |
| 新增 | `src/web/src/composables/useTerrainQuery.ts` | 地形高度与坡度查询 |
| 新增 | `src/web/src/composables/useSimulation.ts` | 前端仿真引擎 |
| 新增 | `docs/3d-scene-feature.md` | 本文档 |
| 修改 | `src/web/src/views/MapView.vue` | 纯3D模式（已移除AMap） |
| 修改 | `src/web/package.json` | 添加 three 依赖 |
| 修改 | `src/backend/main.py` | 新增 /api/scenes 端点 + SIM_MODE 控制 |
| 修改 | `.gitignore` | 添加 3D_Model GLB 规则 |
| 修改 | `src/web/src/types/index.ts` | 新增 SceneInfo/SceneMetadata 类型 |

## GLB 模型准备指南

### Blender 导出设置

1. 打开 Blender 模型文件
2. File → Export → glTF 2.0 (.glb/.gltf)
3. 格式选择 **glTF Binary (.glb)**
4. 推荐设置：
   - ✅ Include → Selected Objects
   - ✅ Transform → +Y Up
   - ✅ Geometry → Apply Modifiers
   - ✅ Geometry → UVs, Normals, Vertex Colors
   - ✅ Animation（如有动画需求）
   - ✅ Compression（推荐开启 DRACO 压缩）

### 目录结构规范

```
3D_Model/
├── scene_a/
│   ├── scene_a.glb          # GLB 模型文件
│   ├── metadata.json        # 场景元数据（可选）
│   └── textures/            # 纹理贴图
└── scene_b/
    ├── scene_b.glb
    └── metadata.json
```

### 场景元数据规范 (metadata.json)

```json
{
  "displayName": "城市测绘场景",
  "description": "杭州市某区域三维模型",
  "geoOrigin": { "lat": 0.0, "lng": 0.0, "alt": 0 },
  "scale": 1.0,
  "rotation": { "x": 0, "y": 0, "z": 0 }
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| displayName | string | 否 | 场景显示名称（默认使用文件名） |
| description | string | 否 | 场景描述 |
| geoOrigin.lat | number | 否 | GPS 纬度坐标原点 |
| geoOrigin.lng | number | 否 | GPS 经度坐标原点 |
| geoOrigin.alt | number | 否 | GPS 海拔高度（默认 0） |
| scale | number | 否 | 坐标缩放比例（默认 1.0） |
| rotation | object | 否 | 模型初始旋转角度 |

## GPS 坐标与 3D 坐标系映射

使用 Haversine 公式将 GPS 偏移量转换为米制距离：

- **X 轴**（东方向）：`worldX = haversineX(geoOrigin, targetLng)`
- **Y 轴**（高度）：`worldY = (targetAlt - geoOrigin.alt) * scale`
- **Z 轴**（北方向）：`worldZ = haversineZ(geoOrigin, targetLat)`

坐标转换由 `use3DMarkers` composable 中的 `latLngToWorld()` 方法实现。

## API 接口文档

### GET /api/scenes

获取可用 3D 场景列表。

**响应示例：**
```json
[
  {
    "name": "测绘场景A（示例）",
    "filename": "scene_a.glb",
    "path": "scene_a/scene_a.glb",
    "size": 630,
    "format": "glb"
  }
]
```

### GET /api/scenes/{filename}

获取指定场景文件（GLB 模型、纹理等）。

- 支持路径子目录（如 `scene_a/scene_a.glb`）
- 安全防护：路径遍历防护、文件类型白名单
- Content-Type 根据文件类型自动设置（.glb → `model/gltf-binary`）

### 安全说明

- 路径遍历防护：禁止 `..` 和绝对路径
- 文件类型白名单：仅允许 `.glb`, `.bin`, `.png`, `.jpg`, `.jpeg`, `.webp`, `.json`
- 文件范围限制：仅限 `3D_Model/` 目录内

## 使用说明

### 添加新场景

1. 在 `3D_Model/` 下创建场景子目录
2. 放入 `.glb` 模型文件
3. （可选）创建 `metadata.json` 设置显示名称和地理原点
4. 刷新页面，下拉列表自动显示新场景

### 操作指南

1. 页面加载后自动初始化 3D 场景
2. 从顶部工具栏下拉列表选择场景模型
3. 场景加载后仿真自动开始
4. 鼠标操作：
   - **左键拖拽**：旋转视角
   - **滚轮**：缩放
   - **右键拖拽**：平移
5. UAV 按椭圆航线飞行，高度可调整
6. UGV 贴地形运行，陡坡自动阻塞绕行
7. 开启 WaypointToolbar 绘制模式可在 3D 场景中点击添加航点

### 仿真控制说明

- 工具栏显示仿真运行状态（▶ 运行中 / ⏸ 已暂停）
- 左上角显示 UAV 当前高度和目标高度
- 右下角显示 UGV 当前坡度角度
- UGV 阻塞时右上角显示红色警告

## 技术栈

- **3D 引擎**：Three.js 0.184+
- **模型格式**：GLB（glTF 2.0 Binary）
- **视角控制**：OrbitControls
- **前端框架**：Vue 3 Composition API
- **后端**：FastAPI + FileResponse

## 风险评估

| 风险 | 应对措施 |
|------|---------|
| 大模型加载性能 | GLB 二进制格式 + DRACO 压缩 + 加载进度提示 |
| GPS 坐标精度损失 | 相对坐标偏移 + 双精度浮点数 |
| 低性能设备兼容 | WebGL 检测 + 降级为 AMap 模式 |
| Three.js 与 Vue 冲突 | shallowRef + markRaw 避免响应式追踪 |
| 内存泄漏 | 严格的 dispose() 资源释放 + ResizeObserver 清理 |

## 俯视地图摄影

### 功能说明

`scripts/topdown_map_render.py` 将 3D GLB 模型渲染为俯视视角的地图影像（类似卫星遥感影像）。

核心特性：
- **正交投影**：模拟卫星摄影的平行光线，无透视变形
- **正交投影**：模拟卫星摄影的平行光线，无透视变形
- **模型居中**：自动将模型中心平移至地图中心
- **Numba 加速**：使用 JIT 编译的软件光栅化器，341K 三角形 2048×2048 渲染仅需 ~0.5 秒
- **双线性纹理采样**：像素级精确纹理映射

### 使用方法

```bash
source venv/bin/activate
python3 scripts/topdown_map_render.py "3D_Model/丘陵.glb" -r 2048 -o "3D_Model/丘陵_topdown.png"
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `input` | 输入 GLB 模型路径（必填） | - |
| `-o, --output` | 输出图像路径 | 模型同目录下 `{name}_topdown.png` |
| `-r, --resolution` | 输出图像分辨率（正方形） | 4096 |
| `-m, --margin` | 模型边缘留白比例 | 0.05 |

### 输出文件

| 文件 | 说明 |
|------|------|
| `{name}_topdown.png` | 卫星遥感影像风格的地图 |
| `{name}_topdown.depth.png` | 归一化高程深度图（灰度） |

### 依赖

- `trimesh`：GLB 模型加载与解析
- `numpy`：数值计算
- `numba`：JIT 编译加速光栅化
- `Pillow`：图像读写

### 2D 地图世界文件配准

`scripts/create_world_file.py` 为 2D 地图影像生成 `.pgw` 世界文件，使图像与 3D 模型的地理坐标系对齐。

**核心功能**：
- 读取 GLB 模型中心坐标和 XY 包围盒范围
- 计算像素到世界坐标的仿射变换参数
- 生成标准 ESRI World File 格式的 `.pgw` 文件
- 确保图像像素中心对应模型 XY 世界坐标中心
- 确保图像覆盖模型的完整 XY 范围（含边距）

**使用方法**：

```bash
source venv/bin/activate
python3 scripts/create_world_file.py "3D_Model/2D_Map.png" "3D_Model/丘陵.glb" -m 0.05
```

**参数说明**：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `image` | 2D 地图图像路径（必填） | - |
| `model` | GLB 3D 模型路径（必填） | - |
| `-m, --margin` | 边缘留白比例 | 0.05 |
| `-o, --output` | 输出世界文件路径 | 图像同目录同名 `.pgw` |

**世界文件格式**（6行文本）：

```
A  — 像素 X 方向世界单位尺寸
D  — Y 方向旋转参数（正北为 0）
B  — X 方向旋转参数（正北为 0）
E  — 像素 Y 方向世界单位尺寸（负值 = 正北向上）
C  — 左上角像素中心的 X 世界坐标
F  — 左上角像素中心的 Y 世界坐标
```

**配准结果** (`丘陵.glb` + `2D_Map.png`, 1750×1654 px)：

| 参数 | 值 |
|------|-----|
| 模型中心 | (56.23, 75.81) |
| 像素尺寸 | 0.3479 世界单位/像素 |
| 图像覆盖世界范围 | 608.76 × 575.37 |
| 左上角世界坐标 | (-247.98, 363.32) |
| 右下角世界坐标 | (360.79, -212.05) |

### 2D 地图叠加层（前端）

前端支持在 Three.js 3D 场景中叠加 `2D_Map.png` 作为 2D 地图背景层，在此之上绘制航点、UAV/UGV 轨迹等。

**核心机制**：

| 组件 | 说明 |
|------|------|
| `useThreeScene.load2DMap()` | 加载 2D_Map.png 纹理，创建 PlaneGeometry 水平平面 |
| `useThreeScene.set2DMapVisible()` | 切换 2D 地图平面可见性 |
| `useThreeScene.setOrthographicCamera()` | 切换正交俯视（2D）/ 透视（3D）相机 |
| `MapView.vue` 切换按钮 | 工具栏 "🗺️ 2D 地图" / "🌐 3D 地形" 切换 |

**坐标对齐**：平面位置由 .pgw 世界配准参数计算：

```
平面位置: (56.23, 0.50, 75.81)   ← Three.js (X=east, Y=高, Z=north)
平面尺寸: 608.76 × 575.37 世界单位
renderOrder: -1 (最先渲染，作为背景层)
depthTest: false (不参与深度测试，3D 标记始终在上层)
```

**视图切换行为**：

| 模式 | 相机 | 2D 地图 | 3D 地形 | OrbitControls |
|------|------|---------|---------|---------------|
| 2D 地图 | 正交俯视 | 显示 | 隐藏 | 旋转锁定 |
| 3D 地形 | 透视 | 隐藏 | 显示 | 正常操作 |

**数据流**：

```
API: /api/scenes/2D_Map.png → TextureLoader → MeshBasicMaterial
                                                      ↓
.pgw 参数 → PlaneGeometry 位置/尺寸 → Mesh → Scene (renderOrder=-1)
                                                      ↓
             use3DMarkers: UAV/UGV Sprite + 轨迹 Line → 自动叠加在上层
             use3DMarkers: 航点 SphereGeometry → 自动叠加在上层
```

### 渲染质量修复 (2024-06)

`scripts/topdown_map_render.py` 已完成以下关键修复，确保植被、树木、树叶等透明纹理的正确渲染：

| 修复项 | 问题描述 | 解决方案 |
|--------|----------|----------|
| 预乘 Alpha 双线性采样 | 透明像素 (alpha=0) 的 RGB 颜色 (如白色) 在插值边界处渗出，产生白边伪影 | `_sample_tex_bilinear` 中实现 Porter-Duff 预乘 Alpha 插值：每个纹素的 RGB 乘以其 Alpha 后再插值，确保透明像素颜色贡献为零 |
| 全局 Z-Buffer 深度归一化 | 各 mesh 独立计算深度范围归一化，导致不同 mesh 间 Z-buffer 比较失效（如 fence 和 rock 深度值不可比较） | `orthographic_projection` 改为接收全局 `global_z_min/max`，所有 mesh 使用统一深度尺度 |
| Numba parallel 竞争写入 | `prange` 多线程并发写入 `depth_buf` 和 `color_buf`，Z-buffer test-write 非原子操作导致像素丢失/闪烁 | 移除 `parallel=True` 及 `prange`，改为单线程 `range` 光栅化（4096×4096 仍仅需 ~1.5s） |
| glTF Alpha Mode 支持 | 未区分 OPAQUE/BLEND/MASK 三种模式，所有 RGBA 纹理统一做简单 Alpha 混合 | 新增 `alpha_mode` 参数：BLEND (预乘混合)、MASK (低于 `alphaCutoff` 丢弃)、OPAQUE (忽略 Alpha) |
| baseColorFactor 应用 | 材质的 `baseColorFactor` 被忽略，导致颜色偏差 | 采样纹理颜色后乘以 `baseColorFactor`（trimesh 4.x 已转为 uint8 [0-255]） |

**测试结果** (`丘陵.glb`, 27 mesh, 341K 三角形, 4096×4096):
- 零白边像素 (>250 RGB): 0 / 9,086,806
- 深度边缘白像素: 0
- 纯白像素 (255,255,255): 0
- 绿色植被像素占比: 31.0%
- 渲染耗时: 1.5s

## 未来扩展

- 3D 无人机/无人车模型替换 Sprites
- 实时点云数据叠加
- 3D 场景测量工具
- 增量式 Mesh 更新（V2.2 实时三维重建预览）
