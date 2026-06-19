#!/usr/bin/env python3
"""
俯视地图摄影脚本（纹理版）
将3D GLB模型渲染为俯视视角的高清地图影像。
使用numba加速的软件光栅化，完整支持PBR纹理、UV贴图、Alpha混合。
模型中心即为地图中心，正交投影模拟卫星摄影。
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image
from numba import njit, prange


# ============================================================
# Numba加速的软件光栅化渲染器
# ============================================================

@njit(cache=True)
def _sample_tex_bilinear(tex, h, w, c, u, v):
    """双线性纹理采样 (numba 加速)"""
    u = max(0.0, min(1.0, u))
    v = max(0.0, min(1.0, v))
    fx = u * (w - 1)
    fy = v * (h - 1)
    x0 = int(fx)
    y0 = int(fy)
    x1 = min(x0 + 1, w - 1)
    y1 = min(y0 + 1, h - 1)
    tx = fx - x0
    ty = fy - y0
    result = np.zeros(c, dtype=np.float64)
    for ch in range(c):
        p00 = tex[y0, x0, ch]
        p10 = tex[y0, x1, ch]
        p01 = tex[y1, x0, ch]
        p11 = tex[y1, x1, ch]
        result[ch] = (p00 * (1 - tx) * (1 - ty) +
                      p10 * tx * (1 - ty) +
                      p01 * (1 - tx) * ty +
                      p11 * tx * ty)
    return result


@njit(cache=True, parallel=True)
def _rasterize_mesh_numba(px, py, depth_norm, faces, uv_coords,
                           tex_array, tex_h, tex_w, tex_c,
                           color_buf, depth_buf, has_alpha):
    """Numba 加速的光栅化"""
    n_tri = len(faces)
    img_h, img_w = color_buf.shape[:2]

    for i in prange(n_tri):
        f = faces[i]
        v0, v1, v2 = f[0], f[1], f[2]

        x0, x1, x2 = px[v0], px[v1], px[v2]
        y0, y1, y2 = py[v0], py[v1], py[v2]
        z0, z1, z2 = depth_norm[v0], depth_norm[v1], depth_norm[v2]

        # 包围盒
        x_min = max(0, min(x0, x1, x2))
        x_max = min(img_w - 1, max(x0, x1, x2))
        y_min = max(0, min(y0, y1, y2))
        y_max = min(img_h - 1, max(y0, y1, y2))

        if x_min > x_max or y_min > y_max:
            continue

        # 边函数的分母
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if denom == 0.0:
            continue
        inv_denom = 1.0 / denom

        # 遍历包围盒内的像素
        for py_idx in range(y_min, y_max + 1):
            for px_idx in range(x_min, x_max + 1):
                # 重心坐标
                w0 = ((y1 - y2) * (px_idx - x2) + (x2 - x1) * (py_idx - y2)) * inv_denom
                w1 = ((y2 - y0) * (px_idx - x2) + (x0 - x2) * (py_idx - y2)) * inv_denom
                w2 = 1.0 - w0 - w1

                if w0 < -1e-6 or w1 < -1e-6 or w2 < -1e-6:
                    continue

                # 深度插值
                z_interp = w0 * z0 + w1 * z1 + w2 * z2

                # Z-buffer测试
                if z_interp <= depth_buf[py_idx, px_idx]:
                    continue

                depth_buf[py_idx, px_idx] = z_interp

                # 纹理采样
                if uv_coords is not None and tex_array is not None:
                    uv0x, uv0y = uv_coords[v0, 0], uv_coords[v0, 1]
                    uv1x, uv1y = uv_coords[v1, 0], uv_coords[v1, 1]
                    uv2x, uv2y = uv_coords[v2, 0], uv_coords[v2, 1]
                    u = w0 * uv0x + w1 * uv1x + w2 * uv2x
                    v = w0 * uv0y + w1 * uv1y + w2 * uv2y
                    color = _sample_tex_bilinear(tex_array, tex_h, tex_w, tex_c, u, v)
                    if has_alpha and tex_c >= 4:
                        alpha = color[3] / 255.0
                        for ch in range(3):
                            old = color_buf[py_idx, px_idx, ch]
                            color_buf[py_idx, px_idx, ch] = np.uint8(
                                color[ch] * alpha + old * (1 - alpha))
                    else:
                        for ch in range(3):
                            color_buf[py_idx, px_idx, ch] = np.uint8(color[ch])
                else:
                    g = np.uint8(z_interp * 255)
                    color_buf[py_idx, px_idx, 0] = g
                    color_buf[py_idx, px_idx, 1] = g
                    color_buf[py_idx, px_idx, 2] = g


def orthographic_projection(vertices, bbox_range, margin=0.05, resolution=2048):
    """正交投影：3D顶点 → 2D屏幕坐标"""
    half_size = max(bbox_range[0], bbox_range[1]) / 2 * (1 + margin)
    scale = (resolution - 1) / (2 * half_size)
    px = (vertices[:, 0] * scale + resolution / 2).astype(np.int32)
    py = (resolution - 1 - (vertices[:, 1] * scale + resolution / 2)).astype(np.int32)
    z_min, z_max = vertices[:, 2].min(), vertices[:, 2].max()
    z_range = max(z_max - z_min, 1e-6)
    depth_norm = (vertices[:, 2] - z_min) / z_range
    return px, py, depth_norm.astype(np.float32)


def prepare_texture(tex_img):
    """准备纹理：转换各种模式为numpy数组 (uint8)"""
    if tex_img is None:
        return None, False

    mode = tex_img.mode
    if mode == 'LA':
        arr = np.array(tex_img)
        rgba = np.zeros((arr.shape[0], arr.shape[1], 4), dtype=np.uint8)
        rgba[:, :, 0] = arr[:, :, 0]
        rgba[:, :, 1] = arr[:, :, 0]
        rgba[:, :, 2] = arr[:, :, 0]
        rgba[:, :, 3] = arr[:, :, 1]
        return rgba, True
    elif mode == 'RGBA':
        return np.array(tex_img), True
    elif mode == 'RGB':
        return np.array(tex_img), False
    elif mode == 'P':
        rgba = np.array(tex_img.convert('RGBA'))
        has_alpha = (rgba[:, :, 3] < 255).any()
        return rgba, has_alpha
    elif mode == 'L':
        arr = np.array(tex_img)
        return np.stack([arr, arr, arr], axis=-1), False
    else:
        return np.array(tex_img.convert('RGB')), False


# ============================================================
# 主处理流程
# ============================================================

def load_and_center_model(glb_path):
    """加载并居中模型"""
    print(f"加载模型: {glb_path}")
    scene = trimesh.load(glb_path)
    if isinstance(scene, trimesh.Trimesh):
        scene = trimesh.Scene([scene])

    all_verts = []
    for g in scene.geometry.values():
        if hasattr(g, 'vertices') and g.vertices is not None:
            all_verts.append(g.vertices)
    if not all_verts:
        raise ValueError("模型中没有找到有效的顶点数据")

    all_verts = np.vstack(all_verts)
    center = (all_verts.min(axis=0) + all_verts.max(axis=0)) / 2.0
    transform = trimesh.transformations.translation_matrix(-center)
    scene.apply_transform(transform)
    vertices_centered = all_verts - center

    bbox_range = vertices_centered.max(axis=0) - vertices_centered.min(axis=0)
    print(f"模型中心: {center}")
    print(f"平移后范围: {bbox_range}")
    return scene, vertices_centered, bbox_range


def render_topdown(scene, vertices, bbox_range, resolution=4096, margin=0.05):
    """主渲染函数：对所有mesh逐三角形光栅化"""
    h, w = resolution, resolution
    color_buf = np.full((h, w, 3), 220, dtype=np.uint8)
    depth_buf = np.full((h, w), -1.0, dtype=np.float32)

    meshes = []
    for name, geom in scene.geometry.items():
        if isinstance(geom, trimesh.Trimesh) and geom.faces is not None and len(geom.faces) > 0:
            meshes.append((name, geom))

    total_tris = sum(len(m[1].faces) for m in meshes)
    print(f"共 {len(meshes)} 个几何体, {total_tris:,} 个三角形")

    rendered = 0
    t0 = time.time()
    for name, geom in meshes:
        verts = geom.vertices
        faces = geom.faces.astype(np.int32)
        uv = geom.visual.uv

        # 投影
        px, py, depth_norm = orthographic_projection(
            verts, bbox_range, margin, resolution)

        # 纹理
        tex_img = None
        has_alpha = False
        if hasattr(geom.visual, 'material') and geom.visual.material is not None:
            mat = geom.visual.material
            if hasattr(mat, 'baseColorTexture') and mat.baseColorTexture is not None:
                tex_img = mat.baseColorTexture

        tex_arr_np = np.zeros((1, 1, 3), dtype=np.uint8)  # dummy
        tex_h = tex_w = tex_c = 1
        if tex_img is not None:
            tex_arr_np, has_alpha = prepare_texture(tex_img)
            tex_h, tex_w = tex_arr_np.shape[:2]
            tex_c = tex_arr_np.shape[2] if tex_arr_np.ndim == 3 else 1

        uv_arr = uv.astype(np.float64) if uv is not None else np.zeros((1, 2))

        # Numba 光栅化
        _rasterize_mesh_numba(
            px, py, depth_norm, faces, uv_arr,
            tex_arr_np, tex_h, tex_w, tex_c,
            color_buf, depth_buf, has_alpha)

        rendered += 1
        if rendered % 5 == 0:
            elapsed = time.time() - t0
            print(f"  进度: {rendered}/{len(meshes)} ({elapsed:.1f}s)")

    elapsed = time.time() - t0
    print(f"光栅化完成: {elapsed:.1f}s")
    return color_buf, depth_buf


def main():
    parser = argparse.ArgumentParser(description='3D模型俯视地图摄影（纹理版）')
    parser.add_argument('input', type=str, help='输入GLB模型路径')
    parser.add_argument('-o', '--output', type=str, default=None, help='输出图像路径')
    parser.add_argument('-r', '--resolution', type=int, default=4096,
                        help='输出分辨率 (默认4096)')
    parser.add_argument('-m', '--margin', type=float, default=0.05,
                        help='边缘留白比例 (默认0.05)')

    args = parser.parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"错误: 文件不存在: {args.input}", file=sys.stderr)
        sys.exit(1)

    output_path = args.output or str(
        input_path.parent / f"{input_path.stem}_topdown.png")

    scene, vertices, bbox_range = load_and_center_model(args.input)

    print(f"开始渲染 ({args.resolution}x{args.resolution})...")
    t_start = time.time()
    color_buf, depth_buf = render_topdown(
        scene, vertices, bbox_range, resolution=args.resolution, margin=args.margin)

    # 保存
    img = Image.fromarray(color_buf)
    img.save(output_path)
    size_kb = Path(output_path).stat().st_size / 1024
    print(f"地图影像已保存至: {output_path} ({size_kb:.0f} KB)")

    depth_vis = np.zeros_like(depth_buf, dtype=np.uint8)
    valid = depth_buf >= 0
    if valid.any():
        dmin, dmax = depth_buf[valid].min(), depth_buf[valid].max()
        depth_vis[valid] = ((depth_buf[valid] - dmin) / max(dmax - dmin, 1e-8) * 255).astype(np.uint8)
    depth_path = str(Path(output_path).with_suffix('.depth.png'))
    Image.fromarray(depth_vis).save(depth_path)
    print(f"深度图已保存至: {depth_path}")

    covered = valid.sum()
    total_px = args.resolution * args.resolution
    print(f"覆盖像素: {covered:,} / {total_px:,} ({covered / total_px * 100:.1f}%)")
    print(f"总耗时: {time.time() - t_start:.1f}s")
    print("\n完成! 模型中心即为地图中心。")


if __name__ == '__main__':
    main()
