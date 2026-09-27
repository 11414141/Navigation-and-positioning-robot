#!/usr/bin/env python3
"""Publish Nav2 initial pose from current RTK fix and site RTK map transform."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from myrobot_rtk_anchor import RosSnapshot, load_transform, rtk_to_map_pose
from route_common import yaw_to_quaternion


def build_initialpose(site: str, args) -> tuple[PoseWithCovarianceStamped, dict]:
    transform = load_transform(site)
    snapshot = RosSnapshot(args.timeout)
    try:
        fix = snapshot.read_fix(
            args.fix_topic,
            args.status_topic,
            args.raw_fix_topic,
            args.allow_raw_fallback,
        )
    finally:
        snapshot.close()

    if args.require_filtered and fix["source"] != args.fix_topic:
        raise RuntimeError(f"required filtered RTK fix, got {fix['source']}")

    map_pose = rtk_to_map_pose(transform, fix["latitude"], fix["longitude"])
    q = yaw_to_quaternion(float(map_pose["yaw"]))

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = args.frame_id
    msg.pose.pose.position.x = float(map_pose["x"])
    msg.pose.pose.position.y = float(map_pose["y"])
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = float(q["x"])
    msg.pose.pose.orientation.y = float(q["y"])
    msg.pose.pose.orientation.z = float(q["z"])
    msg.pose.pose.orientation.w = float(q["w"])

    xy_var = args.xy_stddev * args.xy_stddev
    yaw_var = args.yaw_stddev * args.yaw_stddev
    msg.pose.covariance[0] = xy_var
    msg.pose.covariance[7] = xy_var
    msg.pose.covariance[35] = yaw_var

    result = {
        "site": site,
        "topic": args.topic,
        "map": map_pose,
        "rtk": fix,
        "covariance": {
            "xy_stddev": args.xy_stddev,
            "yaw_stddev": args.yaw_stddev,
        },
    }
    return msg, result


def publish_once(site: str, args) -> dict:
    msg, result = build_initialpose(site, args)
    rclpy.init()
    node = rclpy.create_node("myrobot_rtk_initialpose_publisher")
    pub = node.create_publisher(PoseWithCovarianceStamped, args.topic, 10)
    try:
        deadline = time.time() + args.publisher_warmup
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        msg.header.stamp = node.get_clock().now().to_msg()
        for _ in range(args.repeat):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(args.repeat_delay)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True)
    parser.add_argument("--topic", default="/initialpose")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--fix-topic", default="/gps/fix_filtered")
    parser.add_argument("--raw-fix-topic", default="/gps/fix")
    parser.add_argument("--status-topic", default="/rtk/status")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--allow-raw-fallback", action="store_true")
    parser.add_argument("--no-require-filtered", dest="require_filtered", action="store_false")
    parser.set_defaults(require_filtered=True)
    parser.add_argument("--xy-stddev", type=float, default=0.5)
    parser.add_argument("--yaw-stddev", type=float, default=0.7)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--repeat-delay", type=float, default=0.2)
    parser.add_argument("--publisher-warmup", type=float, default=0.5)
    parser.add_argument("--period", type=float, default=0.0, help="Seconds between repeated RTK initialpose publications. 0 means publish once.")
    parser.add_argument("--max-count", type=int, default=1, help="Maximum publications in period mode. 0 means unlimited.")
    args = parser.parse_args()

    count = 0
    try:
        while True:
            result = publish_once(args.site, args)
            count += 1
            print(json.dumps({**result, "publish_count": count}, ensure_ascii=False))
            if args.period <= 0.0:
                break
            if args.max_count > 0 and count >= args.max_count:
                break
            time.sleep(args.period)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
