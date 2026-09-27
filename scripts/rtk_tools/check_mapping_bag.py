#!/usr/bin/env python3
"""Check a mapping rosbag before offline FAST-LIO2 reconstruction."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml


CORE_TOPICS = {
    "/livox/lidar": {
        "type": "livox_ros_driver2/msg/CustomMsg",
        "min_count": 10,
        "reason": "FAST-LIO2 raw Livox CustomMsg",
    },
    "/livox/imu": {
        "type": "sensor_msgs/msg/Imu",
        "min_count": 100,
        "reason": "FAST-LIO2 IMU",
    },
    "/tf": {
        "type": "tf2_msgs/msg/TFMessage",
        "min_count": 1,
        "reason": "dynamic TF replay/debug",
    },
    "/tf_static": {
        "type": "tf2_msgs/msg/TFMessage",
        "min_count": 1,
        "reason": "static TF replay/debug",
    },
}

STRONGLY_RECOMMENDED_TOPICS = {
    "/odom": {"type": "nav_msgs/msg/Odometry", "min_count": 10, "reason": "wheel odom"},
    "/imu/data_raw": {"type": "sensor_msgs/msg/Imu", "min_count": 10, "reason": "robot IMU"},
    "/gps/fix": {"type": "sensor_msgs/msg/NavSatFix", "min_count": 1, "reason": "raw RTK/GNSS fix"},
    "/rtk/status": {
        "type": "diagnostic_msgs/msg/DiagnosticArray",
        "min_count": 1,
        "reason": "RTK fixed/NTRIP status",
    },
}

OPTIONAL_TOPICS = {
    "/gps/fix_filtered": {
        "type": "sensor_msgs/msg/NavSatFix",
        "min_count": 1,
        "reason": "quality-filtered RTK fixed fixes for automatic anchors",
    },
    "/odometry/global": {
        "type": "nav_msgs/msg/Odometry",
        "min_count": 1,
        "reason": "dual-EKF global odometry",
    },
    "/odometry/gps": {
        "type": "nav_msgs/msg/Odometry",
        "min_count": 1,
        "reason": "navsat_transform output",
    },
    "/livox/lidar_points": {
        "type": "sensor_msgs/msg/PointCloud2",
        "min_count": 1,
        "reason": "PointCloud2 debug view",
    },
    "/livox/lidar_points_nav": {
        "type": "sensor_msgs/msg/PointCloud2",
        "min_count": 1,
        "reason": "Nav2 point cloud slice",
    },
    "/livox/lidar_points_glim": {
        "type": "sensor_msgs/msg/PointCloud2",
        "min_count": 1,
        "reason": "GLIM/debug point cloud",
    },
    "/scan": {"type": "sensor_msgs/msg/LaserScan", "min_count": 1, "reason": "2D scan debug"},
    "/nmea_sentence": {"type": "nmea_msgs/msg/Sentence", "min_count": 1, "reason": "raw NMEA"},
    "/gnss/gpgga": {"type": "nmea_msgs/msg/Gpgga", "min_count": 1, "reason": "GGA diagnostics"},
}


def load_metadata(bag_path: Path) -> dict[str, Any]:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    info = data.get("rosbag2_bagfile_information") if isinstance(data, dict) else None
    if not isinstance(info, dict):
        raise ValueError(f"invalid rosbag2 metadata: {metadata_path}")
    return info


def topic_index(info: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for item in info.get("topics_with_message_count", []):
        metadata = item.get("topic_metadata", {})
        name = metadata.get("name")
        if not name:
            continue
        result[name] = {
            "type": metadata.get("type", ""),
            "count": int(item.get("message_count", 0) or 0),
        }
    return result


def check_group(
    title: str,
    topics: dict[str, dict[str, Any]],
    available: dict[str, dict[str, Any]],
    fail_on_error: bool,
) -> int:
    errors = 0
    print()
    print(title)
    bad_label = "FAIL" if fail_on_error else "WARN"
    for name, expected in topics.items():
        actual = available.get(name)
        expected_type = str(expected["type"])
        min_count = int(expected["min_count"])
        reason = expected["reason"]
        if actual is None:
            errors += 1
            print(f"  [{bad_label}] {name} missing ({reason})")
            continue
        actual_type = actual["type"]
        count = int(actual["count"])
        if actual_type != expected_type:
            errors += 1
            print(f"  [{bad_label}] {name} type={actual_type}, expected={expected_type}")
            continue
        if count < min_count:
            errors += 1
            print(f"  [{bad_label}] {name} count={count}, expected>={min_count} ({reason})")
            continue
        print(f"  [OK] {name} type={actual_type} count={count}")
    if errors and not fail_on_error:
        return 0
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="Path to rosbag directory")
    parser.add_argument(
        "--min-duration",
        type=float,
        default=10.0,
        help="Minimum acceptable bag duration in seconds",
    )
    parser.add_argument(
        "--require-rtk-fixed",
        action="store_true",
        help="Fail if /gps/fix_filtered has no messages",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag_path).expanduser()
    if not bag_path.is_dir():
        print(f"[FAIL] bag directory not found: {bag_path}", file=sys.stderr)
        return 1

    try:
        info = load_metadata(bag_path)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    duration_sec = float(info.get("duration", {}).get("nanoseconds", 0) or 0) / 1e9
    message_count = int(info.get("message_count", 0) or 0)
    storage = info.get("storage_identifier", "unknown")
    available = topic_index(info)

    print("========== Mapping Bag Check ==========")
    print(f"bag: {bag_path}")
    print(f"storage: {storage}")
    print(f"duration_sec: {duration_sec:.3f}")
    print(f"message_count: {message_count}")
    print(f"topic_count: {len(available)}")

    errors = 0
    if duration_sec < args.min_duration:
        errors += 1
        print(f"[FAIL] duration {duration_sec:.3f}s < {args.min_duration:.3f}s")
    else:
        print(f"[OK] duration {duration_sec:.3f}s >= {args.min_duration:.3f}s")

    errors += check_group("FAST-LIO2 必需话题", CORE_TOPICS, available, True)
    errors += check_group("强烈建议话题", STRONGLY_RECOMMENDED_TOPICS, available, True)

    optional_errors = check_group("可选/增强话题", OPTIONAL_TOPICS, available, False)
    if optional_errors:
        print(f"  [WARN] optional topic issues: {optional_errors}")

    filtered = available.get("/gps/fix_filtered")
    filtered_count = int(filtered["count"]) if filtered else 0
    if filtered_count > 0:
        print()
        print(f"[OK] /gps/fix_filtered count={filtered_count}; bag contains filtered RTK fixed data.")
    else:
        print()
        message = (
            "/gps/fix_filtered has no messages. Offline FAST-LIO2 mapping can still run, "
            "but automatic RTK anchor sampling should not be trusted."
        )
        if args.require_rtk_fixed:
            errors += 1
            print(f"[FAIL] {message}")
        else:
            print(f"[WARN] {message}")

    print()
    if errors:
        print(f"========== CHECK FAILED: errors={errors} ==========")
        return 1
    print("========== CHECK PASSED ==========")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
