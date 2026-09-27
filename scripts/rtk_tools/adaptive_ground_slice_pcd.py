#!/usr/bin/env python3
"""Slice a FAST-LIO PCD by local ground height and optionally clear near robot path."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def load_points(path: Path) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
        raise RuntimeError(f"empty point cloud: {path}")
    return points


def save_points(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points[:, :3])
    if not o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=False):
        raise RuntimeError(f"failed to write PCD: {path}")


def load_trajectory_xy(path: Path, stride: int) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if stride > 1 and i % stride != 0:
                continue
            rows.append((float(row["x"]), float(row["y"])))
    if not rows:
        raise RuntimeError(f"empty trajectory: {path}")
    return np.asarray(rows, dtype=np.float64)


def estimate_local_ground(
    points: np.ndarray,
    grid_size: float,
    percentile: float,
    min_points_per_cell: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    xy = points[:, :2]
    z = points[:, 2]
    min_xy = xy.min(axis=0)
    ij = np.floor((xy - min_xy) / grid_size).astype(np.int64)

    order = np.lexsort((ij[:, 1], ij[:, 0]))
    sorted_ij = ij[order]
    sorted_z = z[order]

    cell_keys = []
    ground_values = []
    start = 0
    while start < len(sorted_z):
        end = start + 1
        while end < len(sorted_z) and np.array_equal(sorted_ij[end], sorted_ij[start]):
            end += 1
        count = end - start
        if count >= min_points_per_cell:
            cell_keys.append((int(sorted_ij[start, 0]), int(sorted_ij[start, 1])))
            ground_values.append(float(np.percentile(sorted_z[start:end], percentile)))
        start = end

    if not cell_keys:
        raise RuntimeError("no grid cell has enough points for ground estimation")

    key_to_ground = {key: value for key, value in zip(cell_keys, ground_values)}
    valid = np.zeros(len(points), dtype=bool)
    ground_z = np.zeros(len(points), dtype=np.float64)

    for idx, key_arr in enumerate(ij):
        key = (int(key_arr[0]), int(key_arr[1]))
        value = key_to_ground.get(key)
        if value is None:
            continue
        valid[idx] = True
        ground_z[idx] = value

    return valid, ground_z, ij, min_xy


def filter_near_path(
    points: np.ndarray,
    ground_z: np.ndarray,
    trajectory_xy: np.ndarray,
    radius: float,
    rel_z_max: float,
) -> np.ndarray:
    tree = cKDTree(trajectory_xy)
    distances, _ = tree.query(points[:, :2], k=1, workers=-1)
    rel_z = points[:, 2] - ground_z
    return ~((distances <= radius) & (rel_z <= rel_z_max))


def filter_sparse_columns(
    points: np.ndarray,
    rel_z: np.ndarray,
    grid_size: float,
    min_points: int,
    min_height_thickness: float,
    z_bin_size: float,
    min_z_bins: int,
    neighbor_support: bool,
    neighbor_min_columns: int,
) -> tuple[np.ndarray, dict[str, int]]:
    xy = points[:, :2]
    min_xy = xy.min(axis=0)
    ij = np.floor((xy - min_xy) / grid_size).astype(np.int64)

    columns: dict[tuple[int, int], list[int]] = {}
    for idx, key_arr in enumerate(ij):
        key = (int(key_arr[0]), int(key_arr[1]))
        columns.setdefault(key, []).append(idx)

    strong_columns: set[tuple[int, int]] = set()
    column_stats: dict[tuple[int, int], tuple[int, float, int]] = {}
    for key, indices in columns.items():
        values = rel_z[indices]
        point_count = len(indices)
        height_thickness = float(values.max() - values.min()) if point_count else 0.0
        if z_bin_size > 0.0:
            z_bins = int(np.unique(np.floor(values / z_bin_size).astype(np.int64)).size)
        else:
            z_bins = 1
        column_stats[key] = (point_count, height_thickness, z_bins)
        if (
            point_count >= min_points
            or height_thickness >= min_height_thickness
            or z_bins >= min_z_bins
        ):
            strong_columns.add(key)

    keep_columns = set(strong_columns)
    if neighbor_support:
        for key in columns:
            if key in keep_columns:
                continue
            i, j = key
            support = 0
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    if (i + di, j + dj) in strong_columns:
                        support += 1
            if support >= neighbor_min_columns:
                keep_columns.add(key)

    keep = np.zeros(len(points), dtype=bool)
    removed_points = 0
    for key, indices in columns.items():
        if key in keep_columns:
            keep[indices] = True
        else:
            removed_points += len(indices)

    return keep, {
        "column_total": len(columns),
        "column_kept": len(keep_columns),
        "column_removed": len(columns) - len(keep_columns),
        "column_strong": len(strong_columns),
        "column_removed_points": removed_points,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Input raw PCD")
    parser.add_argument("--output", required=True, help="Output sliced PCD")
    parser.add_argument("--grid-size", type=float, default=0.5)
    parser.add_argument("--ground-percentile", type=float, default=10.0)
    parser.add_argument("--min-points-per-cell", type=int, default=20)
    parser.add_argument("--rel-z-min", type=float, default=0.06)
    parser.add_argument("--rel-z-max", type=float, default=1.0)
    parser.add_argument("--trajectory-csv", default="")
    parser.add_argument("--filter-near-path", action="store_true")
    parser.add_argument("--path-clear-radius", type=float, default=0.45)
    parser.add_argument("--path-clear-rel-z-max", type=float, default=1.0)
    parser.add_argument("--path-sample-stride", type=int, default=5)
    parser.add_argument("--column-filter", action="store_true")
    parser.add_argument("--column-grid-size", type=float, default=0.03)
    parser.add_argument("--column-min-points", type=int, default=8)
    parser.add_argument("--column-min-height-thickness", type=float, default=0.12)
    parser.add_argument("--column-z-bin-size", type=float, default=0.04)
    parser.add_argument("--column-min-z-bins", type=int, default=3)
    parser.add_argument("--column-neighbor-support", action="store_true")
    parser.add_argument("--column-neighbor-min-columns", type=int, default=2)
    args = parser.parse_args()

    input_path = Path(args.input).expanduser()
    output_path = Path(args.output).expanduser()

    print("========== Adaptive Ground Slice PCD ==========")
    print(f"input: {input_path}")
    print(f"output: {output_path}")
    print(f"grid_size: {args.grid_size}")
    print(f"ground_percentile: {args.ground_percentile}")
    print(f"rel_z_range: [{args.rel_z_min}, {args.rel_z_max}]")
    print(f"filter_near_path: {args.filter_near_path}")
    print(f"path_clear_radius: {args.path_clear_radius}")
    print(f"path_clear_rel_z_max: {args.path_clear_rel_z_max}")
    print(f"column_filter: {args.column_filter}")
    print(
        "column_filter_params: grid=%.3f min_points=%d min_height_thickness=%.3f z_bin=%.3f min_z_bins=%d neighbor_support=%s neighbor_min_columns=%d"
        % (
            args.column_grid_size,
            args.column_min_points,
            args.column_min_height_thickness,
            args.column_z_bin_size,
            args.column_min_z_bins,
            args.column_neighbor_support,
            args.column_neighbor_min_columns,
        )
    )

    points = load_points(input_path)
    original_count = len(points)
    valid_ground, ground_z, _, _ = estimate_local_ground(
        points,
        args.grid_size,
        args.ground_percentile,
        args.min_points_per_cell,
    )
    rel_z = points[:, 2] - ground_z
    height_mask = valid_ground & (rel_z >= args.rel_z_min) & (rel_z <= args.rel_z_max)
    after_height = int(height_mask.sum())

    mask = height_mask
    near_path_removed = 0
    if args.filter_near_path:
        if not args.trajectory_csv:
            raise RuntimeError("--filter-near-path requires --trajectory-csv")
        trajectory_xy = load_trajectory_xy(Path(args.trajectory_csv).expanduser(), args.path_sample_stride)
        keep_near_path = filter_near_path(
            points,
            ground_z,
            trajectory_xy,
            args.path_clear_radius,
            args.path_clear_rel_z_max,
        )
        near_path_removed = int((mask & ~keep_near_path).sum())
        mask = mask & keep_near_path

    column_stats = {
        "column_total": 0,
        "column_kept": 0,
        "column_removed": 0,
        "column_strong": 0,
        "column_removed_points": 0,
    }
    if args.column_filter:
        candidate_indices = np.nonzero(mask)[0]
        candidate_points = points[candidate_indices]
        candidate_rel_z = rel_z[candidate_indices]
        keep_columns, column_stats = filter_sparse_columns(
            candidate_points,
            candidate_rel_z,
            args.column_grid_size,
            args.column_min_points,
            args.column_min_height_thickness,
            args.column_z_bin_size,
            args.column_min_z_bins,
            args.column_neighbor_support,
            args.column_neighbor_min_columns,
        )
        column_mask = np.zeros(len(points), dtype=bool)
        column_mask[candidate_indices[keep_columns]] = True
        mask = mask & column_mask

    sliced = points[mask]
    if len(sliced) == 0:
        raise RuntimeError("adaptive slice produced empty point cloud")
    save_points(output_path, sliced)

    print(f"original_points: {original_count}")
    print(f"valid_ground_points: {int(valid_ground.sum())}")
    print(f"after_relative_height: {after_height}")
    print(f"near_path_removed: {near_path_removed}")
    for key, value in column_stats.items():
        print(f"{key}: {value}")
    print(f"output_points: {len(sliced)}")
    print(
        "output_z_range: [%.6f, %.6f]" % (float(sliced[:, 2].min()), float(sliced[:, 2].max()))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
