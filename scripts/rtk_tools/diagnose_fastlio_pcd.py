#!/usr/bin/env python3
"""Diagnose FAST-LIO PCD height distribution and local ground drift."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", help="Input PCD path")
    parser.add_argument("--grid-size", type=float, default=1.0)
    parser.add_argument("--ground-percentile", type=float, default=10.0)
    parser.add_argument("--min-points-per-cell", type=int, default=20)
    args = parser.parse_args()

    path = Path(args.pcd).expanduser()
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
      raise RuntimeError(f"empty point cloud: {path}")

    xyz_min = points.min(axis=0)
    xyz_max = points.max(axis=0)
    z = points[:, 2]
    percentiles = [0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100]

    xy = points[:, :2]
    min_xy = xy.min(axis=0)
    ij = np.floor((xy - min_xy) / args.grid_size).astype(np.int64)
    order = np.lexsort((ij[:, 1], ij[:, 0]))
    sorted_ij = ij[order]
    sorted_z = z[order]

    grounds = []
    counts = []
    start = 0
    while start < len(sorted_z):
        end = start + 1
        while end < len(sorted_z) and np.array_equal(sorted_ij[end], sorted_ij[start]):
            end += 1
        count = end - start
        if count >= args.min_points_per_cell:
            grounds.append(float(np.percentile(sorted_z[start:end], args.ground_percentile)))
            counts.append(count)
        start = end

    grounds_arr = np.asarray(grounds, dtype=np.float64)
    counts_arr = np.asarray(counts, dtype=np.int64)

    print("========== FAST-LIO PCD Diagnose ==========")
    print(f"pcd: {path}")
    print(f"points: {len(points)}")
    print("x_range: [%.6f, %.6f]" % (xyz_min[0], xyz_max[0]))
    print("y_range: [%.6f, %.6f]" % (xyz_min[1], xyz_max[1]))
    print("z_range: [%.6f, %.6f]" % (xyz_min[2], xyz_max[2]))
    print("z_percentiles:")
    for p, value in zip(percentiles, np.percentile(z, percentiles)):
        print("  p%03d: %.6f" % (p, value))
    print(f"ground_grid_size: {args.grid_size}")
    print(f"ground_cells: {len(grounds_arr)}")
    if len(grounds_arr):
        print("ground_z_percentiles:")
        for p, value in zip(percentiles, np.percentile(grounds_arr, percentiles)):
            print("  p%03d: %.6f" % (p, value))
        print("cell_count_percentiles:")
        for p, value in zip([0, 10, 50, 90, 100], np.percentile(counts_arr, [0, 10, 50, 90, 100])):
            print("  p%03d: %.1f" % (p, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
