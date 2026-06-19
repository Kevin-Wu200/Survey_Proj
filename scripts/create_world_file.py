#!/usr/bin/env python3
"""
为 2D 地图影像生成 .pgw 世界文件（地理配准）。

读取 GLB 模型的中心坐标和 XY 范围，计算世界文件参数，
使图像像素中心对应模型的 XY 世界坐标中心，
图像覆盖模型的完整 XY 范围（含边距）。

世界文件格式（.pgw, 6行文本）:
  Line 1: A — 像素 X 方向世界单位尺寸
  Line 2: D — Y 方向旋转参数（正北为 0）
  Line 3: B — X 方向旋转参数（正北为 0）
  Line 4: E — 像素 Y 方向世界单位尺寸（负值，正北向上）
  Line 5: C — 左上角像素中心的 X 世界坐标
  Line 6: F — 左上角像素中心的 Y 世界坐标
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image


def load_model_center(glb_path: Path) -> tuple:
    """加载 GLB 模型，返回 (中心坐标, XY范围)"""
    scene = trimesh.load(str(glb_path))
    if isinstance(scene, trimesh.Trimesh):
        scene = trimesh.Scene([scene])

    all_verts = []
    for g in scene.geometry.values():
        if hasattr(g, 'vertices') and g.vertices is not None:
            all_verts.append(g.vertices)

    if not all_verts:
        raise ValueError("模型中没有找到有效的顶点数据")

    all_verts = np.vstack(all_verts)
    bbox_min = all_verts.min(axis=0)
    bbox_max = all_verts.max(axis=0)
    center = (bbox_min + bbox_max) / 2.0
    bbox_range = bbox_max - bbox_min

    return center, bbox_range


def compute_world_file(image_path: Path, model_center: np.ndarray,
                       model_bbox_range: np.ndarray,
                       margin: float = 0.05) -> str:
    """
    计算世界文件内容。

    参数:
        image_path: 图像文件路径
        model_center: 模型中心 [x, y, z]
        model_bbox_range: 模型包围盒范围 [dx, dy, dz]
        margin: 边缘留白比例（默认 5%）

    返回:
        世界文件内容字符串（6行）
    """
    img = Image.open(image_path)
    img_w, img_h = img.size

    center_x = model_center[0]
    center_y = model_center[1]
    model_dx = model_bbox_range[0]
    model_dy = model_bbox_range[1]

    # 确保图片覆盖模型 XY 范围（含边距）
    required_world_w = model_dx * (1.0 + margin)
    required_world_h = model_dy * (1.0 + margin)

    # 计算正方形像素尺寸，确保两个方向都覆盖
    pixel_size_x = required_world_w / img_w
    pixel_size_y = required_world_h / img_h
    pixel_size = max(pixel_size_x, pixel_size_y)

    # 图像覆盖的世界范围
    world_w = pixel_size * img_w
    world_h = pixel_size * img_h

    # 左上角像素中心的世界坐标
    # 像素采用 0-indexed，中心像素位于 (img_w/2 - 0.5, img_h/2 - 0.5)
    center_px = (img_w / 2.0) - 0.5
    center_py = (img_h / 2.0) - 0.5
    ul_cx = center_x - center_px * pixel_size  # C: 左上角 X
    ul_cy = center_y + center_py * pixel_size  # F: 左上角 Y (正值，因 Y 轴向上)

    # 世界文件 6 行
    lines = [
        f"{pixel_size:.10f}",   # A: 像素 X 尺寸
        "0.0",                   # D: 旋转参数
        "0.0",                   # B: 旋转参数
        f"{-pixel_size:.10f}",  # E: 像素 Y 尺寸（负值 = 正北向上）
        f"{ul_cx:.10f}",        # C: 左上角 X
        f"{ul_cy:.10f}",        # F: 左上角 Y
    ]

    # 打印配准信息
    print(f"图像尺寸: {img_w} × {img_h} px")
    print(f"模型中心: ({center_x:.6f}, {center_y:.6f})")
    print(f"模型XY范围: {model_dx:.2f} × {model_dy:.2f}")
    print(f"像素尺寸: {pixel_size:.6f} 世界单位/像素")
    print(f"图像覆盖世界范围: {world_w:.2f} × {world_h:.2f}")
    print(f"左上角世界坐标: ({ul_cx:.6f}, {ul_cy:.6f})")
    print(f"右下角世界坐标: ({ul_cx + world_w:.6f}, {ul_cy - world_h:.6f})")

    # 验证中心点
    verify_cx = ul_cx + center_px * pixel_size
    verify_cy = ul_cy - center_py * pixel_size
    print(f"验证图像中心 → 世界坐标: ({verify_cx:.6f}, {verify_cy:.6f})")
    print(f"  对应模型中心: ({center_x:.6f}, {center_y:.6f})")
    print(f"  误差: X={abs(verify_cx - center_x):.2e}, Y={abs(verify_cy - center_y):.2e}")

    # 验证模型范围是否被覆盖
    world_x_min = ul_cx
    world_x_max = ul_cx + world_w
    world_y_min = ul_cy - world_h
    world_y_max = ul_cy
    model_x_min = center_x - model_dx / 2
    model_x_max = center_x + model_dx / 2
    model_y_min = center_y - model_dy / 2
    model_y_max = center_y + model_dy / 2
    print(f"图像世界范围: X=[{world_x_min:.2f}, {world_x_max:.2f}], "
          f"Y=[{world_y_min:.2f}, {world_y_max:.2f}]")
    print(f"模型XY范围:   X=[{model_x_min:.2f}, {model_x_max:.2f}], "
          f"Y=[{model_y_min:.2f}, {model_y_max:.2f}]")
    x_ok = world_x_min <= model_x_min and world_x_max >= model_x_max
    y_ok = world_y_min <= model_y_min and world_y_max >= model_y_max
    print(f"覆盖检查: X={'✓' if x_ok else '✗'}, Y={'✓' if y_ok else '✗'}")

    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description='为 2D 地图影像生成 .pgw 世界文件（地理配准）')
    parser.add_argument('image', type=str, help='2D 地图图像路径（PNG）')
    parser.add_argument('model', type=str, help='GLB 3D 模型路径')
    parser.add_argument('-m', '--margin', type=float, default=0.05,
                        help='边缘留白比例（默认 0.05）')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出世界文件路径（默认与图像同目录同名的 .pgw）')

    args = parser.parse_args()
    image_path = Path(args.image)
    model_path = Path(args.model)

    if not image_path.exists():
        print(f"错误: 图像文件不存在: {args.image}", file=sys.stderr)
        sys.exit(1)
    if not model_path.exists():
        print(f"错误: 模型文件不存在: {args.model}", file=sys.stderr)
        sys.exit(1)

    # 确定输出路径
    pgw_path = args.output or str(image_path.with_suffix('.pgw'))
    # 如果是 .png，使用 .pgw（而非 .pngw）
    if image_path.suffix.lower() == '.png':
        pgw_path = str(image_path.with_suffix('')) + '.pgw'

    print(f"输入图像: {image_path}")
    print(f"输入模型: {model_path}")

    # 获取模型中心与范围
    center, bbox_range = load_model_center(model_path)

    # 计算并写入世界文件
    world_content = compute_world_file(image_path, center, bbox_range,
                                       margin=args.margin)

    with open(pgw_path, 'w') as f:
        f.write(world_content)
    print(f"\n世界文件已保存至: {pgw_path}")


if __name__ == '__main__':
    main()
