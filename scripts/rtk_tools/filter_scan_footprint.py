#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_matrix(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


class ScanFootprintFilter(Node):
    def __init__(self):
        super().__init__("scan_footprint_filter")
        self.input_scan = self.declare_parameter("input_scan", "/scan").value
        self.output_scan = self.declare_parameter("output_scan", "/scan_nav_filtered").value
        self.target_frame = self.declare_parameter("target_frame", "base_footprint").value
        self.min_x = float(self.declare_parameter("footprint_min_x", -0.15).value)
        self.max_x = float(self.declare_parameter("footprint_max_x", 0.45).value)
        self.min_y = float(self.declare_parameter("footprint_min_y", -0.28).value)
        self.max_y = float(self.declare_parameter("footprint_max_y", 0.28).value)
        self.min_z = float(self.declare_parameter("footprint_min_z", -0.20).value)
        self.max_z = float(self.declare_parameter("footprint_max_z", 0.50).value)
        self.use_inf = bool(self.declare_parameter("use_inf", False).value)
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.05).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            LaserScan, self.output_scan, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan, self.input_scan, self.scan_callback, qos_profile_sensor_data
        )
        self.filtered_count = 0
        self.scan_count = 0
        self.get_logger().info(
            "Filtering %s -> %s, target_frame=%s, footprint=[x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f]",
            self.input_scan,
            self.output_scan,
            self.target_frame,
            self.min_x,
            self.max_x,
            self.min_y,
            self.max_y,
            self.min_z,
            self.max_z,
        )

    def scan_callback(self, msg):
        if not msg.header.frame_id:
            self.publisher.publish(msg)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"TF unavailable {self.target_frame} <- {msg.header.frame_id}: {exc}",
                    throttle_duration_sec=5.0,
                )
                self.publisher.publish(msg)
                return

        t = transform.transform.translation
        q = transform.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = msg.intensities
        filtered.ranges = list(msg.ranges)

        replacement = math.inf if self.use_inf else msg.range_max + 1.0
        removed = 0
        for i, scan_range in enumerate(msg.ranges):
            if not math.isfinite(scan_range):
                continue
            if scan_range < msg.range_min or scan_range > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = scan_range * math.cos(angle)
            y = scan_range * math.sin(angle)
            z = 0.0
            bx = rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + t.x
            by = rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + t.y
            bz = rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + t.z
            if (
                self.min_x <= bx <= self.max_x
                and self.min_y <= by <= self.max_y
                and self.min_z <= bz <= self.max_z
            ):
                filtered.ranges[i] = replacement
                removed += 1

        self.scan_count += 1
        self.filtered_count += removed
        if self.scan_count % 50 == 0:
            self.get_logger().info(
                "scan footprint filter removed=%d total_removed=%d scans=%d",
                removed,
                self.filtered_count,
                self.scan_count,
            )
        self.publisher.publish(filtered)


def main():
    rclpy.init()
    node = ScanFootprintFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
