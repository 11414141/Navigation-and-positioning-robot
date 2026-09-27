#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

from route_common import load_route, yaw_to_quaternion


def waypoint_to_pose(waypoint: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = waypoint.get("frame_id", "map")
    pose.pose.position.x = float(waypoint["x"])
    pose.pose.position.y = float(waypoint["y"])
    pose.pose.position.z = float(waypoint.get("z", 0.0))
    if "orientation" in waypoint:
        q = waypoint["orientation"]
    else:
        q = yaw_to_quaternion(float(waypoint.get("yaw", 0.0)))
    pose.pose.orientation.x = float(q.get("x", 0.0))
    pose.pose.orientation.y = float(q.get("y", 0.0))
    pose.pose.orientation.z = float(q.get("z", 0.0))
    pose.pose.orientation.w = float(q.get("w", 1.0))
    return pose


def main() -> int:
    parser = argparse.ArgumentParser(description="Execute a saved Nav2 waypoint route.")
    parser.add_argument("--site", required=True, help="Work site name")
    parser.add_argument("--route", required=True, help="Route name, without or with .yaml")
    parser.add_argument("--action", default="/navigate_through_poses", help="Nav2 action name")
    parser.add_argument("--timeout", type=float, default=20.0, help="Seconds to wait for action server")
    args = parser.parse_args()

    try:
      route = load_route(args.site, args.route)
    except Exception as exc:
      print(f"ERROR: failed to load route: {exc}", file=sys.stderr)
      return 2

    waypoints = route["waypoints"]
    if not waypoints:
        print("ERROR: route has no waypoints", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("rtk_remote_route_executor")
    client = ActionClient(node, NavigateThroughPoses, args.action)

    print(f"Waiting for Nav2 action server: {args.action}")
    if not client.wait_for_server(timeout_sec=args.timeout):
        print(f"ERROR: Nav2 action server not available: {args.action}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    goal = NavigateThroughPoses.Goal()
    now = node.get_clock().now().to_msg()
    for wp in waypoints:
        pose = waypoint_to_pose(wp)
        pose.header.stamp = now
        goal.poses.append(pose)

    print(f"Sending route '{args.route}' for site '{args.site}' with {len(goal.poses)} waypoints")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle or not goal_handle.accepted:
        print("ERROR: route goal rejected by Nav2", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 4

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    status = getattr(result, "status", None)
    print(f"Route finished with status: {status}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == 4 else 5


if __name__ == "__main__":
    raise SystemExit(main())

