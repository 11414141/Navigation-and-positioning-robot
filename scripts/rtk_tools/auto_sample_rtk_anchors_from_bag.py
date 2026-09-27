#!/usr/bin/env python3
"""Sample RTK anchors from a mapping bag and an offline FAST-LIO trajectory."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from myrobot_rtk_anchor import compute_transform_from_anchors, save_yaml


@dataclass(frozen=True)
class TrajectorySample:
    stamp_sec: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float


@dataclass(frozen=True)
class RtkSample:
    stamp_sec: float
    frame_id: str
    latitude: float
    longitude: float
    altitude: float
    status: int
    service: int
    covariance_type: int
    horizontal_stddev: float | None


def horizontal_stddev(msg: Any) -> float | None:
    cov_x = float(msg.position_covariance[0])
    cov_y = float(msg.position_covariance[4])
    if not (math.isfinite(cov_x) and math.isfinite(cov_y)):
        return None
    if cov_x < 0.0 or cov_y < 0.0:
        return None
    return math.sqrt(max(cov_x, cov_y))


def load_trajectory(path: Path) -> list[TrajectorySample]:
    rows: list[TrajectorySample] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                TrajectorySample(
                    stamp_sec=float(row["stamp_sec"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    z=float(row["z"]),
                    qx=float(row["qx"]),
                    qy=float(row["qy"]),
                    qz=float(row["qz"]),
                    qw=float(row["qw"]),
                    yaw=float(row["yaw"]),
                )
            )
    rows.sort(key=lambda item: item.stamp_sec)
    if not rows:
        raise RuntimeError(f"empty trajectory CSV: {path}")
    return rows


def read_bag_topic(bag_path: Path, topic_name: str) -> list[RtkSample]:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    topic_type = topic_types.get(topic_name)
    if topic_type is None:
        raise RuntimeError(f"bag topic not found: {topic_name}")
    msg_type = get_message(topic_type)

    samples: list[RtkSample] = []
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = float(bag_time_ns) * 1e-9
        latitude = float(msg.latitude)
        longitude = float(msg.longitude)
        if not (math.isfinite(latitude) and math.isfinite(longitude)):
            continue
        samples.append(
            RtkSample(
                stamp_sec=stamp,
                frame_id=str(msg.header.frame_id),
                latitude=latitude,
                longitude=longitude,
                altitude=float(msg.altitude),
                status=int(msg.status.status),
                service=int(msg.status.service),
                covariance_type=int(msg.position_covariance_type),
                horizontal_stddev=horizontal_stddev(msg),
            )
        )
    samples.sort(key=lambda item: item.stamp_sec)
    if not samples:
        raise RuntimeError(f"no usable RTK samples in bag topic: {topic_name}")
    return samples


def nearest_trajectory(
    trajectory: list[TrajectorySample],
    stamps: list[float],
    stamp_sec: float,
) -> tuple[TrajectorySample, float]:
    pos = bisect.bisect_left(stamps, stamp_sec)
    candidates = []
    if pos < len(trajectory):
        candidates.append(trajectory[pos])
    if pos > 0:
        candidates.append(trajectory[pos - 1])
    if not candidates:
        raise RuntimeError("trajectory is empty")
    best = min(candidates, key=lambda item: abs(item.stamp_sec - stamp_sec))
    return best, abs(best.stamp_sec - stamp_sec)


def distance_xy(a: TrajectorySample, b: TrajectorySample) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def build_anchor(site: str, name: str, traj: TrajectorySample, rtk: RtkSample, time_diff: float) -> dict:
    return {
        "name": name,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "map": {
            "frame_id": "map",
            "child_frame_id": "base_footprint",
            "x": float(traj.x),
            "y": float(traj.y),
            "z": float(traj.z),
            "yaw": float(traj.yaw),
            "source": "offline_fastlio_trajectory",
            "stamp_sec": float(traj.stamp_sec),
        },
        "rtk": {
            "frame_id": rtk.frame_id,
            "source": "/gps/fix_filtered",
            "latitude": float(rtk.latitude),
            "longitude": float(rtk.longitude),
            "altitude": float(rtk.altitude),
            "status": int(rtk.status),
            "service": int(rtk.service),
            "position_covariance_type": int(rtk.covariance_type),
            "horizontal_stddev": rtk.horizontal_stddev,
            "stamp_sec": float(rtk.stamp_sec),
        },
        "auto_sample": {
            "site": site,
            "time_diff_sec": float(time_diff),
        },
    }


def sample_anchors(
    site: str,
    trajectory: list[TrajectorySample],
    rtk_samples: list[RtkSample],
    min_spacing: float,
    max_time_diff: float,
    min_count: int,
    max_count: int,
    max_horizontal_stddev: float,
) -> tuple[list[dict], dict[str, int]]:
    stamps = [item.stamp_sec for item in trajectory]
    anchors: list[dict] = []
    last_kept: TrajectorySample | None = None
    stats = {
        "rtk_samples": len(rtk_samples),
        "trajectory_samples": len(trajectory),
        "rejected_time_diff": 0,
        "rejected_spacing": 0,
        "rejected_covariance": 0,
    }

    for rtk in rtk_samples:
        if rtk.horizontal_stddev is not None and rtk.horizontal_stddev > max_horizontal_stddev:
            stats["rejected_covariance"] += 1
            continue
        traj, time_diff = nearest_trajectory(trajectory, stamps, rtk.stamp_sec)
        if time_diff > max_time_diff:
            stats["rejected_time_diff"] += 1
            continue
        if last_kept is not None and distance_xy(traj, last_kept) < min_spacing:
            stats["rejected_spacing"] += 1
            continue
        name = f"auto_{len(anchors) + 1:03d}"
        anchors.append(build_anchor(site, name, traj, rtk, time_diff))
        last_kept = traj
        if max_count > 0 and len(anchors) >= max_count:
            break

    if len(anchors) < min_count:
        raise RuntimeError(f"not enough RTK anchors: {len(anchors)} < {min_count}")
    return anchors, stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True)
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--trajectory-csv", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--fix-topic", default="/gps/fix_filtered")
    parser.add_argument("--min-spacing", type=float, default=5.0)
    parser.add_argument("--max-time-diff", type=float, default=0.2)
    parser.add_argument("--min-count", type=int, default=3)
    parser.add_argument("--max-count", type=int, default=30)
    parser.add_argument("--max-horizontal-stddev", type=float, default=0.2)
    parser.add_argument("--max-rms", type=float, default=1.0)
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser()
    trajectory_csv = Path(args.trajectory_csv).expanduser()
    map_yaml = Path(args.map_yaml).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    if not bag_path.is_dir():
        print(f"ERROR: bag directory not found: {bag_path}", file=sys.stderr)
        return 2
    if not trajectory_csv.is_file():
        print(f"ERROR: trajectory CSV not found: {trajectory_csv}", file=sys.stderr)
        return 2
    if not map_yaml.is_file():
        print(f"ERROR: map yaml not found: {map_yaml}", file=sys.stderr)
        return 2

    try:
        trajectory = load_trajectory(trajectory_csv)
        rtk_samples = read_bag_topic(bag_path, args.fix_topic)
        anchors, stats = sample_anchors(
            args.site,
            trajectory,
            rtk_samples,
            args.min_spacing,
            args.max_time_diff,
            args.min_count,
            args.max_count,
            args.max_horizontal_stddev,
        )
        transform = compute_transform_from_anchors(args.site, anchors, map_yaml)
        if float(transform["rms_error_m"]) > args.max_rms:
            raise RuntimeError(
                f"RTK anchor RMS too large: {transform['rms_error_m']:.3f}m > {args.max_rms:.3f}m"
            )
        output_dir.mkdir(parents=True, exist_ok=True)
        anchors_path = output_dir / "rtk_anchors.yaml"
        transform_path = output_dir / "rtk_map_transform.yaml"
        anchors_data = {
            "site": args.site,
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source_bag": str(bag_path),
            "trajectory_csv": str(trajectory_csv),
            "map_yaml": str(map_yaml),
            "sampling": {
                "fix_topic": args.fix_topic,
                "min_spacing": args.min_spacing,
                "max_time_diff": args.max_time_diff,
                "min_count": args.min_count,
                "max_count": args.max_count,
                "max_horizontal_stddev": args.max_horizontal_stddev,
            },
            "stats": stats,
            "anchors": anchors,
        }
        save_yaml(anchors_path, anchors_data)
        save_yaml(transform_path, transform)
        result = {
            "site": args.site,
            "anchors_file": str(anchors_path),
            "transform_file": str(transform_path),
            "anchor_count": len(anchors),
            "rms_error_m": float(transform["rms_error_m"]),
            "stats": stats,
        }
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
