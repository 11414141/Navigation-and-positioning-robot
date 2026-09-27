#!/usr/bin/env python3
"""Remove tiny isolated occupied components from a Nav2 PGM map."""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image


def resolve_image_path(yaml_path: Path) -> Path:
    image_value = ""
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("image:"):
            image_value = stripped.split(":", 1)[1].strip().strip("'\"")
            break
    if not image_value:
        raise RuntimeError(f"map yaml missing image field: {yaml_path}")
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return image_path


def read_resolution(yaml_path: Path) -> float:
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("resolution:"):
            return float(stripped.split(":", 1)[1].strip())
    raise RuntimeError(f"map yaml missing resolution field: {yaml_path}")


def component_filter(
    occupied: np.ndarray,
    resolution: float,
    min_cells: int,
    min_length: float,
    connectivity: int,
) -> tuple[np.ndarray, dict[str, int]]:
    height, width = occupied.shape
    visited = np.zeros_like(occupied, dtype=bool)
    keep = np.zeros_like(occupied, dtype=bool)
    if connectivity == 4:
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    elif connectivity == 8:
        neighbors = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]
    else:
        raise ValueError("connectivity must be 4 or 8")

    total_components = 0
    removed_components = 0
    removed_cells = 0
    kept_components = 0

    ys, xs = np.nonzero(occupied)
    for y0, x0 in zip(ys, xs):
        if visited[y0, x0]:
            continue
        total_components += 1
        queue = deque([(y0, x0)])
        visited[y0, x0] = True
        cells = []
        min_x = max_x = x0
        min_y = max_y = y0

        while queue:
            y, x = queue.popleft()
            cells.append((y, x))
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            for dy, dx in neighbors:
                ny = y + dy
                nx = x + dx
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                if visited[ny, nx] or not occupied[ny, nx]:
                    continue
                visited[ny, nx] = True
                queue.append((ny, nx))

        count = len(cells)
        width_m = (max_x - min_x + 1) * resolution
        height_m = (max_y - min_y + 1) * resolution
        length_m = max(width_m, height_m)
        remove = count < min_cells and length_m < min_length
        if remove:
            removed_components += 1
            removed_cells += count
        else:
            kept_components += 1
            for y, x in cells:
                keep[y, x] = True

    return keep, {
        "total_components": total_components,
        "kept_components": kept_components,
        "removed_components": removed_components,
        "removed_cells": removed_cells,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", required=True, help="Nav2 map yaml path")
    parser.add_argument("--output-yaml", default="", help="Optional output yaml path")
    parser.add_argument("--output-image", default="", help="Optional output image path")
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--min-component-cells", type=int, default=4)
    parser.add_argument("--min-component-length", type=float, default=0.12)
    parser.add_argument("--connectivity", type=int, default=8, choices=[4, 8])
    args = parser.parse_args()

    yaml_path = Path(args.yaml).expanduser()
    image_path = resolve_image_path(yaml_path)
    resolution = read_resolution(yaml_path)

    out_yaml = Path(args.output_yaml).expanduser() if args.output_yaml else yaml_path
    out_image = Path(args.output_image).expanduser() if args.output_image else image_path

    image = Image.open(image_path).convert("L")
    data = np.asarray(image, dtype=np.uint8)
    occupied = data <= args.occupied_threshold
    original_occupied = int(occupied.sum())

    keep_occupied, stats = component_filter(
        occupied,
        resolution,
        args.min_component_cells,
        args.min_component_length,
        args.connectivity,
    )

    filtered = data.copy()
    filtered[occupied & ~keep_occupied] = 254
    final_occupied = int((filtered <= args.occupied_threshold).sum())

    Image.fromarray(filtered, mode="L").save(out_image)
    if out_yaml != yaml_path:
        text = yaml_path.read_text(encoding="utf-8")
        image_name = out_image.name
        lines = []
        for line in text.splitlines():
            if line.strip().startswith("image:"):
                lines.append(f"image: {image_name}")
            else:
                lines.append(line)
        out_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("========== Nav2 Map Speckle Filter ==========")
    print(f"yaml: {yaml_path}")
    print(f"image: {image_path}")
    print(f"output_yaml: {out_yaml}")
    print(f"output_image: {out_image}")
    print(f"resolution: {resolution}")
    print(f"occupied_threshold: {args.occupied_threshold}")
    print(f"min_component_cells: {args.min_component_cells}")
    print(f"min_component_length: {args.min_component_length}")
    print(f"connectivity: {args.connectivity}")
    print(f"original_occupied_cells: {original_occupied}")
    print(f"final_occupied_cells: {final_occupied}")
    for key, value in stats.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
