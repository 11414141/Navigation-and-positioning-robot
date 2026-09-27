#!/usr/bin/env python3
"""Publish map->odom_combined from RTK fixed data anchored to the current site map."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from myrobot_rtk_anchor import diagnostic_values, horizontal_stddev, load_transform, rtk_to_map_pose
from route_common import quaternion_to_yaw, yaw_to_quaternion


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RtkMapTfPublisher:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("myrobot_rtk_map_tf_publisher")
        self.transform_data = load_transform(args.site)
        self.map_odom_yaw = self.resolve_map_odom_yaw()
        self.latest_fix: NavSatFix | None = None
        self.latest_status: dict[str, str] = {}
        self.filtered_x: float | None = None
        self.filtered_y: float | None = None
        self.filtered_yaw: float | None = None
        self.last_log_time = 0.0
        self.last_publish_time = 0.0
        self.publish_count = 0
        self.skip_count = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=args.tf_cache_time))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.fix_sub = self.node.create_subscription(
            NavSatFix,
            args.fix_topic,
            self.on_fix,
            qos_profile_sensor_data,
        )
        self.status_sub = self.node.create_subscription(
            DiagnosticArray,
            args.status_topic,
            self.on_status,
            10,
        )
        self.timer = self.node.create_timer(1.0 / args.publish_rate, self.on_timer)
        self.node.get_logger().info(
            "RTK map TF publisher started: site=%s parent=%s child=%s yaw=%.6f fix_topic=%s"
            % (args.site, args.map_frame, args.odom_frame, self.map_odom_yaw, args.fix_topic)
        )

    def resolve_map_odom_yaw(self) -> float:
        if self.args.map_odom_yaw is not None:
            return float(self.args.map_odom_yaw)
        if self.args.yaw_source == "zero":
            return 0.0
        yaw = self.transform_data.get("enu_to_map", {}).get("yaw")
        if yaw is None:
            raise RuntimeError("rtk_map_transform.yaml missing enu_to_map.yaw")
        return float(yaw)

    def on_fix(self, msg: NavSatFix):
        self.latest_fix = msg

    def on_status(self, msg: DiagnosticArray):
        self.latest_status = diagnostic_values(msg)

    def fix_is_usable(self, msg: NavSatFix) -> tuple[bool, str]:
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return False, "invalid_latlon"
        if self.args.require_filtered and self.args.fix_topic != "/gps/fix_filtered":
            return False, "require_filtered_topic"
        if self.args.require_rtk_fixed:
            quality = self.latest_status.get("fix_quality", "")
            if quality and quality != "4":
                return False, f"fix_quality_{quality}_not_allowed"
        stddev = horizontal_stddev(msg)
        if stddev is not None and stddev > self.args.max_horizontal_stddev:
            return False, "horizontal_stddev_%.3f_too_large" % stddev
        return True, "ok"

    def lookup_odom_base(self):
        return self.tf_buffer.lookup_transform(
            self.args.odom_frame,
            self.args.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.args.lookup_timeout),
        )

    def smooth(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        alpha = max(0.0, min(1.0, self.args.position_alpha))
        yaw_alpha = max(0.0, min(1.0, self.args.yaw_alpha))
        if self.filtered_x is None or alpha >= 1.0:
            self.filtered_x = x
            self.filtered_y = y
        else:
            self.filtered_x = alpha * x + (1.0 - alpha) * self.filtered_x
            self.filtered_y = alpha * y + (1.0 - alpha) * self.filtered_y

        if self.filtered_yaw is None or yaw_alpha >= 1.0:
            self.filtered_yaw = yaw
        else:
            delta = normalize_angle(yaw - self.filtered_yaw)
            self.filtered_yaw = normalize_angle(self.filtered_yaw + yaw_alpha * delta)
        return self.filtered_x, self.filtered_y, self.filtered_yaw

    def on_timer(self):
        now = self.node.get_clock().now()
        wall_now = time.time()
        fix = self.latest_fix
        if fix is None:
            self.skip_count += 1
            self.maybe_log(wall_now, "waiting for RTK fix")
            return

        ok, reason = self.fix_is_usable(fix)
        if not ok:
            self.skip_count += 1
            self.maybe_log(wall_now, "skip RTK fix: %s" % reason)
            return

        try:
            odom_base = self.lookup_odom_base()
        except Exception as exc:
            self.skip_count += 1
            self.maybe_log(wall_now, "waiting for TF %s->%s: %s" % (self.args.odom_frame, self.args.base_frame, exc))
            return

        map_pose = rtk_to_map_pose(self.transform_data, float(fix.latitude), float(fix.longitude))
        odom_translation = odom_base.transform.translation
        rotated_odom_x, rotated_odom_y = rotate_xy(
            float(odom_translation.x),
            float(odom_translation.y),
            self.map_odom_yaw,
        )
        map_odom_x = float(map_pose["x"]) - rotated_odom_x
        map_odom_y = float(map_pose["y"]) - rotated_odom_y
        map_odom_yaw = self.map_odom_yaw
        map_odom_x, map_odom_y, map_odom_yaw = self.smooth(map_odom_x, map_odom_y, map_odom_yaw)

        msg = TransformStamped()
        msg.header.stamp = (now + Duration(seconds=self.args.transform_time_offset)).to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.child_frame_id = self.args.odom_frame
        msg.transform.translation.x = float(map_odom_x)
        msg.transform.translation.y = float(map_odom_y)
        msg.transform.translation.z = 0.0
        q = yaw_to_quaternion(map_odom_yaw)
        msg.transform.rotation.x = float(q["x"])
        msg.transform.rotation.y = float(q["y"])
        msg.transform.rotation.z = float(q["z"])
        msg.transform.rotation.w = float(q["w"])
        self.tf_broadcaster.sendTransform(msg)
        self.publish_count += 1
        self.last_publish_time = wall_now
        self.maybe_log(
            wall_now,
            "published map->odom x=%.3f y=%.3f yaw=%.3f rtk_map_base=(%.3f, %.3f) quality=%s"
            % (
                map_odom_x,
                map_odom_y,
                map_odom_yaw,
                float(map_pose["x"]),
                float(map_pose["y"]),
                self.latest_status.get("fix_quality_text", self.latest_status.get("fix_quality", "?")),
            ),
        )

    def maybe_log(self, wall_now: float, message: str):
        if wall_now - self.last_log_time >= self.args.log_period:
            self.last_log_time = wall_now
            self.node.get_logger().info(
                "%s | published=%d skipped=%d" % (message, self.publish_count, self.skip_count)
            )

    def spin(self):
        rclpy.spin(self.node)

    def close(self):
        self.node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True)
    parser.add_argument("--fix-topic", default="/gps/fix_filtered")
    parser.add_argument("--status-topic", default="/rtk/status")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom_combined")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--lookup-timeout", type=float, default=0.05)
    parser.add_argument("--tf-cache-time", type=float, default=10.0)
    parser.add_argument("--transform-time-offset", type=float, default=0.2)
    parser.add_argument("--position-alpha", type=float, default=0.5)
    parser.add_argument("--yaw-alpha", type=float, default=1.0)
    parser.add_argument("--yaw-source", choices=["transform", "zero"], default="transform")
    parser.add_argument("--map-odom-yaw", type=float, default=None)
    parser.add_argument("--max-horizontal-stddev", type=float, default=0.3)
    parser.add_argument("--require-rtk-fixed", action="store_true", default=True)
    parser.add_argument("--allow-non-fixed", dest="require_rtk_fixed", action="store_false")
    parser.add_argument("--require-filtered", action="store_true", default=True)
    parser.add_argument("--log-period", type=float, default=2.0)
    args = parser.parse_args()

    if args.publish_rate <= 0.0:
        print("ERROR: --publish-rate must be > 0", file=sys.stderr)
        return 2

    rclpy.init()
    node = None
    try:
        node = RtkMapTfPublisher(args)
        node.spin()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
