#!/usr/bin/env python3
"""Record FAST-LIO /Odometry to a CSV file during offline replay."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FastlioTrajectoryRecorder(Node):
    def __init__(self, output_csv: Path, topic: str) -> None:
        super().__init__("fastlio_trajectory_recorder")
        self.output_csv = output_csv
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["stamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw"])
        self.count = 0
        self.subscription = self.create_subscription(Odometry, topic, self.on_odom, 100)
        self.get_logger().info(f"Recording FAST-LIO trajectory: {topic} -> {output_csv}")

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.writer.writerow(
            [
                f"{stamp:.9f}",
                f"{pose.position.x:.9f}",
                f"{pose.position.y:.9f}",
                f"{pose.position.z:.9f}",
                f"{q.x:.9f}",
                f"{q.y:.9f}",
                f"{q.z:.9f}",
                f"{q.w:.9f}",
                f"{yaw:.9f}",
            ]
        )
        self.count += 1
        if self.count % 100 == 0:
            self.file.flush()

    def destroy_node(self) -> bool:
        self.file.flush()
        self.file.close()
        self.get_logger().info(f"Saved FAST-LIO trajectory samples: {self.count}")
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--topic", default="/Odometry", help="FAST-LIO odometry topic")
    args = parser.parse_args()

    rclpy.init()
    node = FastlioTrajectoryRecorder(Path(args.output).expanduser(), args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
