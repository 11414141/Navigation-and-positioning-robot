#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class LivoxScanDelayMonitor(Node):
    def __init__(self, duration: float, window: float) -> None:
        super().__init__("livox_scan_delay_monitor")
        self.duration = duration
        self.window = window
        self.start_time = time.monotonic()
        self.last_report_time = self.start_time

        self.points_count = 0
        self.scan_count = 0
        self.points_delay_sum = 0.0
        self.scan_delay_sum = 0.0
        self.points_delay_max = 0.0
        self.scan_delay_max = 0.0
        self.points_delay_last = None
        self.scan_delay_last = None

        self.create_subscription(
            PointCloud2, "/livox/lidar_points", self.on_points, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.2, self.on_timer)

    def on_points(self, msg: PointCloud2) -> None:
        delay = self.delay_seconds(msg.header.stamp)
        self.points_count += 1
        self.points_delay_sum += delay
        self.points_delay_max = max(self.points_delay_max, delay)
        self.points_delay_last = delay

    def on_scan(self, msg: LaserScan) -> None:
        delay = self.delay_seconds(msg.header.stamp)
        self.scan_count += 1
        self.scan_delay_sum += delay
        self.scan_delay_max = max(self.scan_delay_max, delay)
        self.scan_delay_last = delay

    def delay_seconds(self, stamp) -> float:
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(stamp)
        return (now - msg_time).nanoseconds / 1e9

    def on_timer(self) -> None:
        now = time.monotonic()
        if now - self.last_report_time >= self.window:
            self.report(now)
            self.reset_window(now)
        if self.duration > 0.0 and now - self.start_time >= self.duration:
            rclpy.shutdown()

    def report(self, now: float) -> None:
        elapsed = now - self.start_time
        points_avg = self.points_delay_sum / self.points_count if self.points_count else None
        scan_avg = self.scan_delay_sum / self.scan_count if self.scan_count else None

        print(
            "[%6.1fs] lidar_points=%s avg=%s max=%s | scan=%s avg=%s max=%s"
            % (
                elapsed,
                self.format_delay(self.points_delay_last, self.points_count),
                self.format_delay(points_avg),
                self.format_delay(self.points_delay_max if self.points_count else None),
                self.format_delay(self.scan_delay_last, self.scan_count),
                self.format_delay(scan_avg),
                self.format_delay(self.scan_delay_max if self.scan_count else None),
            ),
            flush=True,
        )

    def reset_window(self, now: float) -> None:
        self.last_report_time = now
        self.points_count = 0
        self.scan_count = 0
        self.points_delay_sum = 0.0
        self.scan_delay_sum = 0.0
        self.points_delay_max = 0.0
        self.scan_delay_max = 0.0
        self.points_delay_last = None
        self.scan_delay_last = None

    @staticmethod
    def format_delay(value, count=None) -> str:
        if count == 0 or value is None:
            return "no_msg"
        return "%.3fs" % value


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Monitor /livox/lidar_points and /scan header timestamp delay.")
    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="Monitoring duration in seconds. Use 0 to run until Ctrl+C.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Report window in seconds.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = LivoxScanDelayMonitor(args.duration, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
