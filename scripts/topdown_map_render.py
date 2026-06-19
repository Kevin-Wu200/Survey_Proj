#!/usr/bin/env python3
"""
俯视地图摄影脚本
将3D GLB模型渲染为俯视视角的地图影像（类似卫星遥感影像）。
模型中心即为地图中心，使用正交投影 + DEM地形可视化方案：
- 深度图提取高程信息
- 地形渐变色（绿→黄→棕→白）
- 山体阴影（hillshade）增强立体感
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh
import pyrender
from PIL import Image, ImageFilter


# ============================================================
# 地形配色方案（类似卫星遥感影像/地形图）
# ============================================================
def terrain_colormap():
    """构建地形高程渐变色表：低处绿色→高处灰白色"""
    return [
        (0.00, np.array([34, 94, 43])),       # 低处：深绿（森林/植被）
        (0.20, np.array([86, 138, 53])),       # 浅绿
        (0.40, np.array([168, 178, 98])),      # 黄绿
        (0.55, np.array([194, 163, 101])),     # 土黄
        (0.70, np.array([161, 118, 76])),      # 棕色
        (0.85, np.array([189, 170, 150])),     # 浅棕
        (1.00, np.array([235, 230, 225])),     # 高处：灰白（岩石/雪）
    ]


def apply_terrain_colormap(normalized_height, stops):
    """将归一化高程映射到地形渐变色"""
    h, w = normalized_height.shape
    result = np.zeros((h, w, 3), dtype=np.float32)
    for i in range(len(stops) - 1):
        pos0, color0 = stops[i]
        pos1, color1 = stops[i + 1]
        mask = (normalized_height >= pos0) & (normalized_height < pos1)
        if i == len(stops) - 2:
            mask = (normalized_height >= pos0) & (normalized_height <= pos1)
        if not np.any(mask):
            continue
        t = (normalized_height[mask] - pos0) / (pos1 - pos0 + 1e-8)
        t = t[..., np.newaxis]
        result[mask] = color0 * (1 - t) + color1 * t
    return result.astype(np.uint8)


def generate_hillshade(depth, azimuth_deg=315.0, altitude_deg=45.0, z_factor=1.0):
    """
    从深度图生成山体阴影。
    azimuth_deg: 太阳方位角 0=北 90=东 180=南 270=西
    altitude_deg: 太阳高度角 0=地平线 90=天顶
    """
    gy, gx = np.gradient(depth)
    dzdx = -gx * z_factor
    dzdy = -gy * z_factor
    azimuth_rad = np.deg2rad(360.0 - azimuth_deg + 90.0)
    altitude_rad = np.deg2rad(altitude_deg)
    slope = np.arctan(z_factor * np.hypot(dzdx, dzdy))
    aspect = np.arctan2(dzdy, -dzdx)
    hillshade = (np.cos(altitude_rad) * np.cos(slope) +
                 np.sin(altitude_rad) * np.sin(slope) *
                 np.cos(azimuth_rad - aspect))
    return np.clip(hillshade, 0, 1).astype(np.float32)


# ============================================================
# 核心处理流程
# ============================================================

def load_and_center_model(glb_path):
    """加载GLB模型，计算中心并平移到原点"""
    scene = trimesh.load(glb_path)
    if isinstance(scene, trimesh.Trimesh):
        scene = trimesh.Scene([scene])

    all_vertices = []
    for g in scene.geometry.values():
        if hasattr(g, 'vertices') and g.vertices is not None:
            all_vertices.append(g.vertices)
    if not all_vertices:
        raise ValueError("模型中没有找到有效的顶点数据")

    all_vertices = np.vstack(all_vertices)
    center = (all_vertices.min(axis=0) + all_vertices.max(axis=0)) / 2.0

    transform = trimesh.transformations.translation_matrix(-center)
    scene.apply_transform(transform)
    vertices_centered = all_vertices - center

    print(f"原始模型中心: {center}")
    print(f"bbox范围: {vertices_centered.max(axis=0) - vertices_centered.min(axis=0)}")
    return scene, vertices_centered


def setup_pyrender_scene(mesh_scene):
    """构建pyrender场景（白色材质用于深度渲染）"""
    render_scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0, 1.0])
    for geometry in mesh_scene.geometry.values():
        if isinstance(geometry, trimesh.Trimesh):
            mesh = pyrender.Mesh(
                primitives=[pyrender.Primitive(
                    positions=geometry.vertices.astype(np.float32),
                    indices=geometry.faces.astype(np.int32),
                    color_0=np.ones((len(geometry.vertices), 3), dtype=np.float32),
                )])
            render_scene.add(mesh)
    return render_scene


def setup_topdown_camera(scene, vertices, resolution=2048, margin=0.05):
    """设置俯视正交相机"""
    bbox_range = vertices.max(axis=0) - vertices.min(axis=0)
    half_size = max(bbox_range[0], bbox_range[1]) / 2 * (1 + margin)
    camera_z = vertices.max(axis=0)[2] + bbox_range[2] * 2

    camera = pyrender.OrthographicCamera(
        xmag=half_size, ymag=half_size, znear=0.01, zfar=camera_z * 5)
    # 相机从正上方俯视XY平面
    camera_pose = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, camera_z],
        [0, 0, 0, 1]
    ])
    scene.add(camera, pose=camera_pose)
    print(f"正交相机: half_size={half_size:.2f}, z={camera_z:.2f}")
    return resolution


def render_depth(scene, resolution):
    """渲染深度图"""
    renderer = pyrender.OffscreenRenderer(resolution, resolution)
    scene.add(pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=3.0),
              pose=np.eye(4))
    _, depth = renderer.render(scene)
    renderer.delete()
    return depth


def create_satellite_image(depth, output_path, azimuth=315.0, altitude=45.0):
    """将深度图转换为卫星遥感影像风格的地图"""
    # Y轴翻转匹配图像坐标（OpenGL渲染时Y轴朝上，图像坐标Y轴朝下）
    depth = np.flipud(depth)

    valid_mask = depth > 0
    if not np.any(valid_mask):
        raise ValueError("深度图中没有有效数据")

    valid_depth = depth[valid_mask]
    d_min, d_max = valid_depth.min(), valid_depth.max()
    print(f"深度范围: [{d_min:.6f}, {d_max:.6f}]")

    # 归一化高程（深度越大=离相机越远=高程越低）
    normalized = np.zeros_like(depth)
    normalized[valid_mask] = 1.0 - (depth[valid_mask] - d_min) / (d_max - d_min + 1e-8)

    # 地形渐变色
    stops = terrain_colormap()
    colored = apply_terrain_colormap(normalized, stops)

    # 山体阴影叠加
    hillshade = generate_hillshade(depth, azimuth_deg=azimuth,
                                   altitude_deg=altitude, z_factor=2.0)
    shadow_factor = 0.5
    result = np.zeros_like(colored, dtype=np.float32)
    for c in range(3):
        channel = colored[:, :, c].astype(np.float32)
        shaded = channel * (1.0 - shadow_factor + shadow_factor * hillshade)
        result[:, :, c] = np.clip(shaded, 0, 255)

    result[~valid_mask] = [220, 220, 220]  # 背景浅灰
    result = result.astype(np.uint8)

    # 轻微锐化并保存
    img = Image.fromarray(result)
    img = img.filter(ImageFilter.SHARPEN)
    img.save(output_path)
    print(f"卫星影像已保存至: {output_path}")

    # 保存归一化深度图（已翻转，直接保存）
    depth_vis = np.zeros_like(depth, dtype=np.uint8)
    depth_vis[valid_mask] = (normalized[valid_mask] * 255).astype(np.uint8)
    depth_path = str(Path(output_path).with_suffix('.depth.png'))
    Image.fromarray(depth_vis).save(depth_path)
    print(f"深度图已保存至: {depth_path}")
    return result


def main():
    parser = argparse.ArgumentParser(description='3D模型俯视地图摄影')
    parser.add_argument('input', type=str, help='输入GLB模型路径')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出图像路径')
    parser.add_argument('-r', '--resolution', type=int, default=2048,
                        help='输出分辨率 (默认2048)')
    parser.add_argument('-m', '--margin', type=float, default=0.05,
                        help='边缘留白比例 (默认0.05)')
    parser.add_argument('--azimuth', type=float, default=315.0,
                        help='太阳方位角 (默认315=西北)')
    parser.add_argument('--altitude', type=float, default=45.0,
                        help='太阳高度角 (默认45)')

    args = parser.parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"错误: 文件不存在: {args.input}", file=sys.stderr)
        sys.exit(1)

    output_path = args.output or str(input_path.parent / f"{input_path.stem}_topdown.png")

    print(f"加载模型: {args.input}")
    mesh_scene, vertices = load_and_center_model(args.input)

    print("构建渲染场景...")
    render_scene = setup_pyrender_scene(mesh_scene)

    print("设置俯视正交相机...")
    setup_topdown_camera(render_scene, vertices,
                          resolution=args.resolution, margin=args.margin)

    print("渲染深度图...")
    depth = render_depth(render_scene, args.resolution)
    print(f"深度图: {depth.shape}, 有效像素: {(depth > 0).sum()}")

    print("生成卫星遥感影像...")
    create_satellite_image(depth, output_path,
                            azimuth=args.azimuth, altitude=args.altitude)
    print("\n完成! 模型中心即为地图中心。")


if __name__ == '__main__':
    main()
