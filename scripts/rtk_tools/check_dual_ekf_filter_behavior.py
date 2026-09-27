#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class DualEkfFilterMonitor(Node):
    def __init__(self, duration: float, window: float) -> None:
        super().__init__("dual_ekf_filter_monitor")
        self.duration = duration
        self.window = window
        self.start_time = time.monotonic()
        self.last_report_time = self.start_time

        self.fix_quality = "unknown"
        self.fix_quality_text = "unknown"
        self.num_sats = "unknown"
        self.hdop = "unknown"
        self.rtcm_age = "unknown"

        self.total_filtered_fix = 0
        self.total_odom_gps = 0
        self.total_odom_global = 0
        self.total_global_nan = 0

        self.window_filtered_fix = 0
        self.window_odom_gps = 0
        self.window_odom_global = 0
        self.window_global_nan = 0

        self.fixed_windows = 0
        self.fixed_pass_windows = 0
        self.nonfixed_windows = 0
        self.nonfixed_block_windows = 0
        self.global_ok_windows = 0
        self.total_windows = 0
        self.transition_windows = 0
        self.window_fix_qualities = set()

        self.create_subscription(DiagnosticArray, "/rtk/status", self.on_status, 10)
        self.create_subscription(NavSatFix, "/gps/fix_filtered", self.on_fix_filtered, 10)
        self.create_subscription(Odometry, "/odometry/gps", self.on_odom_gps, 10)
        self.create_subscription(Odometry, "/odometry/global", self.on_odom_global, 10)
        self.create_timer(0.5, self.on_timer)

    def on_status(self, msg: DiagnosticArray) -> None:
        values = {}
        for status in msg.status:
            for item in status.values:
                values[item.key] = item.value
        self.fix_quality = values.get("fix_quality", self.fix_quality)
        self.fix_quality_text = values.get("fix_quality_text", self.fix_quality_text)
        self.num_sats = values.get("num_sats", self.num_sats)
        self.hdop = values.get("hdop", self.hdop)
        self.rtcm_age = values.get("seconds_since_rtcm", self.rtcm_age)
        if self.fix_quality != "unknown":
            self.window_fix_qualities.add(self.fix_quality)

    def on_fix_filtered(self, _msg: NavSatFix) -> None:
        self.total_filtered_fix += 1
        self.window_filtered_fix += 1

    def on_odom_gps(self, _msg: Odometry) -> None:
        self.total_odom_gps += 1
        self.window_odom_gps += 1

    def on_odom_global(self, msg: Odometry) -> None:
        self.total_odom_global += 1
        self.window_odom_global += 1
        if self.has_nan(msg):
            self.total_global_nan += 1
            self.window_global_nan += 1

    def on_timer(self) -> None:
        now = time.monotonic()
        if now - self.last_report_time >= self.window:
            self.report_window(now)
            self.reset_window(now)
        if self.duration > 0.0 and now - self.start_time >= self.duration:
            self.print_summary()
            rclpy.shutdown()

    def report_window(self, now: float) -> None:
        self.total_windows += 1
        elapsed = now - self.start_time
        fixed = self.fix_quality == "4"
        nonfixed = self.fix_quality not in ("4", "unknown")
        transition = len(self.window_fix_qualities) > 1
        global_ok = self.window_odom_global > 0 and self.window_global_nan == 0

        if transition:
            self.transition_windows += 1
            filter_state = "TRANSITION"
        elif fixed:
            self.fixed_windows += 1
            fixed_pass = self.window_filtered_fix > 0 and self.window_odom_gps > 0
            if fixed_pass:
                self.fixed_pass_windows += 1
            filter_state = "PASS" if fixed_pass else "FAIL"
        elif nonfixed:
            self.nonfixed_windows += 1
            blocked = self.window_filtered_fix == 0
            if blocked:
                self.nonfixed_block_windows += 1
            filter_state = "PASS" if blocked else "FAIL"
        else:
            filter_state = "WAIT"

        if global_ok:
            self.global_ok_windows += 1

        if transition:
            filter_action = "切换中"
        elif fixed and self.window_filtered_fix > 0:
            filter_action = "放行"
        elif nonfixed and self.window_filtered_fix == 0:
            filter_action = "阻断"
        elif self.fix_quality == "unknown":
            filter_action = "等待"
        else:
            filter_action = "异常"

        if self.window_global_nan > 0:
            global_state = "异常NaN"
        elif self.window_odom_global == 0:
            global_state = "无输出"
        elif self.window_odom_gps > 0:
            global_state = "RTK校正"
        else:
            global_state = "预测维持"

        print(
            "[%6.1fs] RTK=%s | 卫星数(sats)=%s 水平精度因子(HDOP)=%s 差分龄期(rtcm_age)=%s | 滤波器=%s(%d) | global=%s(%d) | nan=%d"
            % (
                elapsed,
                self.fix_quality_text,
                self.num_sats,
                self.hdop,
                self.rtcm_age,
                filter_action,
                self.window_filtered_fix,
                global_state,
                self.window_odom_global,
                self.window_global_nan,
            ),
            flush=True,
        )

    def reset_window(self, now: float) -> None:
        self.last_report_time = now
        self.window_filtered_fix = 0
        self.window_odom_gps = 0
        self.window_odom_global = 0
        self.window_global_nan = 0
        self.window_fix_qualities = set()

    def print_summary(self) -> None:
        print("\n========== dual EKF filter monitor summary ==========")
        print("total_windows: %d" % self.total_windows)
        print("fixed_windows: %d" % self.fixed_windows)
        print("fixed_pass_windows: %d" % self.fixed_pass_windows)
        print("nonfixed_windows: %d" % self.nonfixed_windows)
        print("nonfixed_block_windows: %d" % self.nonfixed_block_windows)
        print("transition_windows: %d" % self.transition_windows)
        print("global_ok_windows: %d" % self.global_ok_windows)
        print("total_filtered_fix: %d" % self.total_filtered_fix)
        print("total_odom_gps: %d" % self.total_odom_gps)
        print("total_odom_global: %d" % self.total_odom_global)
        print("total_global_nan: %d" % self.total_global_nan)

        fixed_ok = self.fixed_windows == 0 or self.fixed_pass_windows == self.fixed_windows
        nonfixed_ok = (
            self.nonfixed_windows == 0
            or self.nonfixed_block_windows == self.nonfixed_windows
        )
        global_ok = self.total_windows > 0 and self.global_ok_windows == self.total_windows
        no_nan = self.total_global_nan == 0

        print("Fixed时是否放行RTK: %s" % ("正常" if fixed_ok else "异常"))
        print("非Fixed时是否阻断RTK: %s" % ("正常" if nonfixed_ok else "异常"))
        print("/odometry/global是否持续输出: %s" % ("正常" if global_ok else "异常"))
        print("/odometry/global是否无NaN: %s" % ("正常" if no_nan else "异常"))

        if fixed_ok and nonfixed_ok and global_ok and no_nan:
            print("总体结论: 符合预期")
        else:
            print("总体结论: 需要检查")

    @staticmethod
    def has_nan(msg: Odometry) -> bool:
        values = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        return any(not math.isfinite(value) for value in values)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Monitor RTK quality filter and dual EKF behavior."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
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
    node = DualEkfFilterMonitor(args.duration, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
