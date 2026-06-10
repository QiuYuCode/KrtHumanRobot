"""将 FAST-LIO 保存的 PCD 点云地图按高度切片投影为 2D 栅格地图 (pgm + yaml)。

坐标说明：PCD 的 z=0 位于建图起点的雷达高度（camera_init 系），
地面大约在 z = -lidar_height 处。

用法:
    ros2 run ranger_nav pcd2pgm --pcd ~/maps/scans.pcd --out ~/maps/map \
        --lidar-height 0.30 --z-min 0.15 --z-max 1.2 --resolution 0.05

去噪参数（地图上出现杂乱孤立点时调整，详见 README「地图去噪调参」）:
    --occ-thresh   栅格内点数达到该值才视为占据，调大可滤稀疏噪点
    --min-blob     剔除面积小于 N 格的孤立占据块（连通域分析）
    --ror-radius / --ror-min-pts  3D 半径离群点滤波（默认关闭）
"""
from __future__ import annotations

import argparse
import os
import re
import struct
import sys

import numpy as np

try:
    from scipy import ndimage
except ImportError:
    ndimage = None

PCD_TYPE_MAP = {
    ('F', 4): 'f4', ('F', 8): 'f8',
    ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4',
    ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
}


def read_pcd_xyz(path: str) -> np.ndarray:
    """读取 PCD 文件（ascii / binary），返回 N x 3 的 xyz 数组。"""
    with open(path, 'rb') as f:
        header: dict[str, list[str]] = {}
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('#') or not line:
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == 'DATA':
                break
        data_start = f.tell()
        raw = f.read()

    fields = header['FIELDS']
    sizes = [int(s) for s in header['SIZE']]
    types = header['TYPE']
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
    n_points = int(header['POINTS'][0])
    data_mode = header['DATA'][0].lower()

    dtype_fields = []
    for name, size, typ, cnt in zip(fields, sizes, types, counts):
        base = PCD_TYPE_MAP[(typ, size)]
        if cnt == 1:
            dtype_fields.append((name, base))
        else:
            dtype_fields.append((name, base, (cnt,)))
    dtype = np.dtype(dtype_fields)

    if data_mode == 'binary':
        arr = np.frombuffer(raw[:n_points * dtype.itemsize], dtype=dtype)
    elif data_mode == 'ascii':
        text = raw.decode('ascii', errors='ignore')
        arr = np.loadtxt(text.splitlines(), dtype=np.float64, max_rows=n_points)
        xyz_idx = [fields.index(k) for k in ('x', 'y', 'z')]
        return arr[:, xyz_idx].astype(np.float32)
    else:
        raise RuntimeError(f'不支持的 PCD DATA 类型: {data_mode}'
                           '（binary_compressed 请先用 pcl_convert_pcd_ascii_binary 转换）')

    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def radius_outlier_removal(pts: np.ndarray, radius: float, min_pts: int) -> np.ndarray:
    """半径离群点滤波（体素哈希近似，不依赖 PCL）。

    将点云按边长 radius 的体素分桶，每个点的邻居数近似为其所在体素
    及周围 26 个体素内的点数总和，少于 min_pts 的点被剔除。

    Args:
        pts: N x 3 点云。
        radius: 邻域半径（米），同时作为体素边长。
        min_pts: 邻域内最少点数（含自身）。

    Returns:
        过滤后的点云。
    """
    voxel_idx = np.floor(pts / radius).astype(np.int64)
    # 体素坐标编码为单个 int64 键
    keys, inverse, counts = np.unique(
        voxel_idx, axis=0, return_inverse=True, return_counts=True)

    key_map = {tuple(k): c for k, c in zip(keys, counts)}
    neighbor_counts = np.zeros(len(keys), dtype=np.int64)
    offsets = [(dx, dy, dz)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
    for i, k in enumerate(keys):
        total = 0
        for off in offsets:
            total += key_map.get((k[0] + off[0], k[1] + off[1], k[2] + off[2]), 0)
        neighbor_counts[i] = total

    keep = neighbor_counts[inverse] >= min_pts
    return pts[keep]


def remove_small_blobs(occupied: np.ndarray, min_size: int) -> tuple[np.ndarray, int]:
    """剔除占据栅格中面积小于 min_size 的 4 连通孤立块。

    Returns:
        (过滤后的占据掩码, 剔除的格数)。
    """
    if ndimage is None:
        print('警告: 未安装 scipy，跳过 --min-blob 连通域剔除 '
              '(pip install scipy)', file=sys.stderr)
        return occupied, 0
    labels, n = ndimage.label(occupied)  # 默认 4 连通
    if n == 0:
        return occupied, 0
    sizes = ndimage.sum_labels(occupied, labels, index=np.arange(1, n + 1))
    small = np.flatnonzero(sizes < min_size) + 1
    if len(small) == 0:
        return occupied, 0
    removed_mask = np.isin(labels, small)
    return occupied & ~removed_mask, int(removed_mask.sum())


def write_pgm(path: str, grid: np.ndarray) -> None:
    h, w = grid.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True, help='输入 PCD 文件路径')
    parser.add_argument('--out', required=True, help='输出前缀，生成 <out>.pgm 与 <out>.yaml')
    parser.add_argument('--resolution', type=float, default=0.05, help='栅格分辨率 (m)')
    parser.add_argument('--lidar-height', type=float, default=0.30,
                        help='建图时雷达离地高度 (m)，用于推算地面位置')
    parser.add_argument('--z-min', type=float, default=0.15,
                        help='障碍物切片下限，相对地面 (m)')
    parser.add_argument('--z-max', type=float, default=1.2,
                        help='障碍物切片上限，相对地面 (m)')
    parser.add_argument('--occ-thresh', type=int, default=2,
                        help='栅格内点数达到该值视为占据，调大可滤稀疏噪点')
    parser.add_argument('--min-blob', type=int, default=3,
                        help='剔除面积小于 N 格的孤立占据块，0 关闭')
    parser.add_argument('--ror-radius', type=float, default=0.0,
                        help='3D 半径离群点滤波的邻域半径 (m)，0 关闭')
    parser.add_argument('--ror-min-pts', type=int, default=5,
                        help='半径滤波邻域内最少点数（含自身）')
    args = parser.parse_args(argv)

    pcd_path = os.path.expanduser(args.pcd)
    out_prefix = os.path.expanduser(args.out)

    xyz = read_pcd_xyz(pcd_path)
    print(f'读取点云: {len(xyz)} 个点')

    ground_z = -args.lidar_height
    obs = xyz[(xyz[:, 2] > ground_z + args.z_min) & (xyz[:, 2] < ground_z + args.z_max)]
    ground = xyz[(xyz[:, 2] > ground_z - 0.1) & (xyz[:, 2] < ground_z + args.z_min)]
    print(f'障碍物切片点数: {len(obs)}，地面点数: {len(ground)}')
    if len(obs) == 0:
        print('错误: 切片内没有点，请检查 --lidar-height / --z-min / --z-max', file=sys.stderr)
        return 1

    if args.ror_radius > 0:
        before = len(obs)
        obs = radius_outlier_removal(obs, args.ror_radius, args.ror_min_pts)
        print(f'半径离群点滤波 (r={args.ror_radius}, k={args.ror_min_pts}): '
              f'剔除 {before - len(obs)} 点，剩余 {len(obs)}')
        if len(obs) == 0:
            print('错误: 离群点滤波后没有剩余点，请调小 --ror-min-pts', file=sys.stderr)
            return 1

    res = args.resolution
    all_pts = np.vstack([obs[:, :2], ground[:, :2]])
    x_min, y_min = all_pts.min(axis=0) - 1.0
    x_max, y_max = all_pts.max(axis=0) + 1.0
    width = int(np.ceil((x_max - x_min) / res))
    height = int(np.ceil((y_max - y_min) / res))
    print(f'地图尺寸: {width} x {height} ({res} m/格)')

    def to_idx(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = ((pts[:, 0] - x_min) / res).astype(int).clip(0, width - 1)
        iy = ((pts[:, 1] - y_min) / res).astype(int).clip(0, height - 1)
        return ix, iy

    # 205=未知, 254=空闲, 0=占据（ROS map_server 标准灰度）
    grid = np.full((height, width), 205, dtype=np.uint8)

    gx, gy = to_idx(ground)
    grid[gy, gx] = 254

    ox, oy = to_idx(obs)
    occ_count = np.zeros((height, width), dtype=np.int32)
    np.add.at(occ_count, (oy, ox), 1)
    occupied = occ_count >= args.occ_thresh
    print(f'占据栅格数 (occ-thresh={args.occ_thresh}): {int(occupied.sum())}')

    if args.min_blob > 0:
        occupied, removed = remove_small_blobs(occupied, args.min_blob)
        # 被剔除的栅格保持切片前的状态：有地面点的为空闲 254，否则未知 205
        print(f'孤立块剔除 (min-blob={args.min_blob}): 剔除 {removed} 格，'
              f'剩余占据 {int(occupied.sum())} 格')

    grid[occupied] = 0

    # pgm 的行从上往下，y 轴需要翻转
    grid = np.flipud(grid)

    pgm_path = out_prefix + '.pgm'
    yaml_path = out_prefix + '.yaml'
    write_pgm(pgm_path, grid)
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(
            f'image: {os.path.basename(pgm_path)}\n'
            f'mode: trinary\n'
            f'resolution: {res}\n'
            f'origin: [{x_min:.3f}, {y_min:.3f}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )
    print(f'已生成: {pgm_path}\n已生成: {yaml_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
