#!/usr/bin/env python3
import argparse
import time

import rclpy
import tf2_ros

from route_common import load_route, quaternion_to_yaw, save_route


def main() -> int:
    parser = argparse.ArgumentParser(description="Append current robot pose to a saved route.")
    parser.add_argument("--site", required=True)
    parser.add_argument("--route", required=True)
    parser.add_argument("--name", default="")
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--source-frame", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("rtk_remote_waypoint_recorder")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)

    deadline = time.time() + args.timeout
    transform = None
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(args.target_frame, args.source_frame, rclpy.time.Time())
            break
        except Exception:
            pass

    if transform is None:
        print(f"ERROR: TF unavailable: {args.target_frame} -> {args.source_frame}")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        route = load_route(args.site, args.route)
    except Exception:
        route = {"name": args.route, "frame_id": args.target_frame, "waypoints": []}

    t = transform.transform.translation
    q_msg = transform.transform.rotation
    q = {"x": q_msg.x, "y": q_msg.y, "z": q_msg.z, "w": q_msg.w}
    waypoint = {
        "name": args.name or f"wp_{len(route.get('waypoints', [])) + 1}",
        "frame_id": args.target_frame,
        "x": float(t.x),
        "y": float(t.y),
        "z": 0.0,
        "yaw": float(quaternion_to_yaw(q)),
    }
    route.setdefault("waypoints", []).append(waypoint)
    path = save_route(args.site, args.route, route)
    print(f"Saved waypoint to {path}: {waypoint}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

