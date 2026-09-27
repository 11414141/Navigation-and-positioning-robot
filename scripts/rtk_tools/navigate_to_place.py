#!/usr/bin/env python3
import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from myrobot_places import load_places
from route_common import yaw_to_quaternion


def place_to_pose(place: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = place.get("frame_id", "map")
    pose.pose.position.x = float(place["x"])
    pose.pose.position.y = float(place["y"])
    pose.pose.position.z = float(place.get("z", 0.0))
    q = place.get("orientation") or yaw_to_quaternion(float(place.get("yaw", 0.0)))
    pose.pose.orientation.x = float(q.get("x", 0.0))
    pose.pose.orientation.y = float(q.get("y", 0.0))
    pose.pose.orientation.z = float(q.get("z", 0.0))
    pose.pose.orientation.w = float(q.get("w", 1.0))
    return pose


def main() -> int:
    parser = argparse.ArgumentParser(description="Navigate to a saved semantic place with Nav2.")
    parser.add_argument("--site", required=True)
    parser.add_argument("--place", required=True)
    parser.add_argument("--action", default="/navigate_to_pose")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    data = load_places(args.site)
    place = data.get("places", {}).get(args.place)
    if not place:
        print(f"ERROR: place not found: {args.site}/{args.place}", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("myrobot_place_navigator")
    client = ActionClient(node, NavigateToPose, args.action)

    print(f"Waiting for Nav2 action server: {args.action}")
    if not client.wait_for_server(timeout_sec=args.timeout):
        print(f"ERROR: Nav2 action server not available: {args.action}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    goal = NavigateToPose.Goal()
    goal.pose = place_to_pose(place)
    goal.pose.header.stamp = node.get_clock().now().to_msg()

    print(f"Sending place '{args.place}' for site '{args.site}'")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle or not goal_handle.accepted:
        print("ERROR: place goal rejected by Nav2", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 4

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    status = getattr(result, "status", None)
    print(f"NavigateToPose finished with status: {status}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == 4 else 5


if __name__ == "__main__":
    raise SystemExit(main())
