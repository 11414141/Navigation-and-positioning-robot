#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from route_common import quaternion_to_yaw, sanitize_site_name, site_dir


EARTH_RADIUS_M = 6378137.0


def nav2_current_dir(site: str) -> Path:
    return site_dir(site) / "maps/nav2_current"


def current_map_yaml(site: str) -> Path:
    return nav2_current_dir(site) / "current.yaml"


def anchors_file(site: str) -> Path:
    return nav2_current_dir(site) / "rtk_anchors.yaml"


def transform_file(site: str) -> Path:
    return nav2_current_dir(site) / "rtk_map_transform.yaml"


def load_yaml(path: Path, default):
    if not path.is_file():
        return default
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or default


def save_yaml(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return path


def current_map_info(site: str) -> dict:
    path = current_map_yaml(site)
    if not path.is_file():
        return {}
    stat = path.stat()
    return {
        "map_yaml": str(path),
        "map_yaml_mtime": int(stat.st_mtime),
        "map_yaml_size": int(stat.st_size),
    }


def load_anchors(site: str) -> dict:
    data = load_yaml(anchors_file(site), {"site": site, "anchors": []})
    anchors = data.get("anchors", [])
    if not isinstance(anchors, list):
        anchors = []
    return {"site": data.get("site", site), "anchors": anchors}


def save_anchors(site: str, data: dict) -> Path:
    data["site"] = site
    return save_yaml(anchors_file(site), data)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_to_local_xy(latitude: float, longitude: float, origin_latitude: float, origin_longitude: float) -> tuple[float, float]:
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lat0 = math.radians(origin_latitude)
    lon0 = math.radians(origin_longitude)
    x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
    y = (lat - lat0) * EARTH_RADIUS_M
    return x, y


def local_xy_to_latlon(x: float, y: float, origin_latitude: float, origin_longitude: float) -> tuple[float, float]:
    lat0 = math.radians(origin_latitude)
    lon0 = math.radians(origin_longitude)
    lat = y / EARTH_RADIUS_M + lat0
    lon = x / (math.cos(lat0) * EARTH_RADIUS_M) + lon0
    return math.degrees(lat), math.degrees(lon)


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def horizontal_stddev(msg: NavSatFix):
    cov_x = msg.position_covariance[0]
    cov_y = msg.position_covariance[4]
    if not (math.isfinite(cov_x) and math.isfinite(cov_y)):
        return None
    if cov_x < 0.0 or cov_y < 0.0:
        return None
    return math.sqrt(max(cov_x, cov_y))


def diagnostic_values(msg: DiagnosticArray) -> dict:
    values = {}
    for status in msg.status:
        for item in status.values:
            values[item.key] = item.value
    return values


class RosSnapshot:
    def __init__(self, timeout: float):
        rclpy.init()
        self.node = rclpy.create_node("myrobot_rtk_anchor_snapshot")
        self.timeout = timeout
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate):
        deadline = time.time() + self.timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def lookup_pose(self, target_frame: str, source_frame: str, odom_topic: str) -> dict:
        deadline = time.time() + self.timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                t = transform.transform.translation
                q = transform.transform.rotation
                yaw = quaternion_to_yaw({"x": q.x, "y": q.y, "z": q.z, "w": q.w})
                return {
                    "frame_id": target_frame,
                    "child_frame_id": source_frame,
                    "x": float(t.x),
                    "y": float(t.y),
                    "z": float(t.z),
                    "yaw": float(yaw),
                    "source": "tf",
                }
            except Exception:
                pass

        latest = {"msg": None}
        sub = self.node.create_subscription(Odometry, odom_topic, lambda msg: latest.__setitem__("msg", msg), 10)
        try:
            if not self.spin_until(lambda: latest["msg"] is not None):
                raise RuntimeError(f"pose unavailable: TF {target_frame}->{source_frame}, odometry {odom_topic}")
            msg = latest["msg"]
            q = msg.pose.pose.orientation
            yaw = quaternion_to_yaw({"x": q.x, "y": q.y, "z": q.z, "w": q.w})
            return {
                "frame_id": msg.header.frame_id or target_frame,
                "child_frame_id": msg.child_frame_id or source_frame,
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z),
                "yaw": float(yaw),
                "source": odom_topic,
            }
        finally:
            self.node.destroy_subscription(sub)

    def read_fix(self, fix_topic: str, status_topic: str, raw_fix_topic: str, allow_raw_fallback: bool) -> dict:
        latest = {"fix": None, "status": None}
        fix_sub = self.node.create_subscription(NavSatFix, fix_topic, lambda msg: latest.__setitem__("fix", msg), qos_profile_sensor_data)
        status_sub = self.node.create_subscription(DiagnosticArray, status_topic, lambda msg: latest.__setitem__("status", msg), 10)
        try:
            self.spin_until(lambda: latest["fix"] is not None and latest["status"] is not None)
        finally:
            self.node.destroy_subscription(fix_sub)

        fix_source = fix_topic
        if latest["fix"] is None and allow_raw_fallback and raw_fix_topic != fix_topic:
            raw_sub = self.node.create_subscription(NavSatFix, raw_fix_topic, lambda msg: latest.__setitem__("fix", msg), qos_profile_sensor_data)
            try:
                self.spin_until(lambda: latest["fix"] is not None and latest["status"] is not None)
            finally:
                self.node.destroy_subscription(raw_sub)
            fix_source = raw_fix_topic
        self.node.destroy_subscription(status_sub)

        if latest["fix"] is None:
            raise RuntimeError(f"RTK fix unavailable: {fix_topic}")
        msg = latest["fix"]
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            raise RuntimeError("RTK fix has invalid latitude/longitude")

        status = diagnostic_values(latest["status"]) if latest["status"] is not None else {}
        return {
            "frame_id": msg.header.frame_id,
            "source": fix_source,
            "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "altitude": float(msg.altitude),
            "status": int(msg.status.status),
            "service": int(msg.status.service),
            "position_covariance_type": int(msg.position_covariance_type),
            "horizontal_stddev": horizontal_stddev(msg),
            "diagnostics": status,
        }


def upsert_anchor(site: str, anchor: dict) -> Path:
    data = load_anchors(site)
    name = anchor["name"]
    anchors = [item for item in data["anchors"] if item.get("name") != name]
    anchors.append(anchor)
    data["anchors"] = anchors
    return save_anchors(site, data)


def record_anchor(args) -> dict:
    name = sanitize_site_name(args.name).strip()
    if not name:
        raise ValueError("anchor name is empty")

    snapshot = RosSnapshot(args.timeout)
    try:
        pose = snapshot.lookup_pose(args.target_frame, args.source_frame, args.odom_topic)
        fix = snapshot.read_fix(args.fix_topic, args.status_topic, args.raw_fix_topic, args.allow_raw_fallback)
    finally:
        snapshot.close()

    if args.require_map_frame and pose["frame_id"] != args.target_frame:
        raise RuntimeError(f"pose frame is {pose['frame_id']}, expected {args.target_frame}")

    anchor = {
        "name": name,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "map": pose,
        "rtk": fix,
    }
    anchor.update(current_map_info(args.site))
    path = upsert_anchor(args.site, anchor)
    return {"site": args.site, "name": name, "anchor": anchor, "file": str(path)}


def usable_anchors(site: str) -> list[dict]:
    anchors = load_anchors(site)["anchors"]
    return usable_anchors_from_items(anchors)


def usable_anchors_from_items(anchors: list[dict]) -> list[dict]:
    usable = []
    for item in anchors:
        try:
            map_data = item["map"]
            rtk_data = item["rtk"]
            usable.append({
                "name": item.get("name", ""),
                "map_x": float(map_data["x"]),
                "map_y": float(map_data["y"]),
                "map_yaw": float(map_data.get("yaw", 0.0)),
                "lat": float(rtk_data["latitude"]),
                "lon": float(rtk_data["longitude"]),
                "alt": float(rtk_data.get("altitude", 0.0)),
                "raw": item,
            })
        except (KeyError, TypeError, ValueError):
            continue
    return usable


def compute_transform_data(site: str, anchors: list[dict], map_yaml: Path) -> dict:
    if len(anchors) < 2:
        raise RuntimeError("at least two RTK anchors are required to compute map/RTK rotation")

    origin = anchors[0]
    points = []
    for anchor in anchors:
        enu_x, enu_y = latlon_to_local_xy(anchor["lat"], anchor["lon"], origin["lat"], origin["lon"])
        points.append((anchor, enu_x, enu_y, anchor["map_x"], anchor["map_y"]))

    enu_cx = sum(p[1] for p in points) / len(points)
    enu_cy = sum(p[2] for p in points) / len(points)
    map_cx = sum(p[3] for p in points) / len(points)
    map_cy = sum(p[4] for p in points) / len(points)

    cross = 0.0
    dot = 0.0
    for _, enu_x, enu_y, map_x, map_y in points:
        ex = enu_x - enu_cx
        ey = enu_y - enu_cy
        mx = map_x - map_cx
        my = map_y - map_cy
        cross += ex * my - ey * mx
        dot += ex * mx + ey * my

    yaw = math.atan2(cross, dot)
    tx, ty = rotate_xy(enu_cx, enu_cy, yaw)
    translation_x = map_cx - tx
    translation_y = map_cy - ty

    residuals = []
    for anchor, enu_x, enu_y, map_x, map_y in points:
        pred_x, pred_y = rotate_xy(enu_x, enu_y, yaw)
        pred_x += translation_x
        pred_y += translation_y
        error = math.hypot(pred_x - map_x, pred_y - map_y)
        residuals.append({"name": anchor["name"], "error_m": float(error)})

    rms_error = math.sqrt(sum(item["error_m"] ** 2 for item in residuals) / len(residuals))
    data = {
        "site": site,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "map_yaml": str(map_yaml),
        "origin_anchor": origin["name"],
        "origin_latitude": origin["lat"],
        "origin_longitude": origin["lon"],
        "enu_to_map": {
            "yaw": float(yaw),
            "translation_x": float(translation_x),
            "translation_y": float(translation_y),
        },
        "anchor_count": len(points),
        "rms_error_m": float(rms_error),
        "residuals": residuals,
    }
    return data


def compute_transform_from_anchors(site: str, anchor_items: list[dict], map_yaml: Path) -> dict:
    anchors = usable_anchors_from_items(anchor_items)
    return compute_transform_data(site, anchors, map_yaml)


def compute_transform(args) -> dict:
    anchors = usable_anchors(args.site)
    data = compute_transform_data(args.site, anchors, current_map_yaml(args.site))
    path = save_yaml(transform_file(args.site), data)
    return {"site": args.site, "transform": data, "file": str(path)}


def load_transform(site: str) -> dict:
    path = transform_file(site)
    if not path.is_file():
        raise RuntimeError(f"transform file not found: {path}")
    data = load_yaml(path, {})
    if "enu_to_map" not in data:
        raise RuntimeError(f"invalid transform file: {path}")
    return data


def rtk_to_map_pose(transform: dict, latitude: float, longitude: float) -> dict:
    enu_x, enu_y = latlon_to_local_xy(
        latitude,
        longitude,
        float(transform["origin_latitude"]),
        float(transform["origin_longitude"]),
    )
    tf_data = transform["enu_to_map"]
    map_x, map_y = rotate_xy(enu_x, enu_y, float(tf_data["yaw"]))
    map_x += float(tf_data["translation_x"])
    map_y += float(tf_data["translation_y"])
    return {
        "frame_id": "map",
        "x": float(map_x),
        "y": float(map_y),
        "yaw": float(tf_data["yaw"]),
    }


def locate_current(args) -> dict:
    transform = load_transform(args.site)
    snapshot = RosSnapshot(args.timeout)
    try:
        fix = snapshot.read_fix(args.fix_topic, args.status_topic, args.raw_fix_topic, args.allow_raw_fallback)
    finally:
        snapshot.close()

    map_pose = rtk_to_map_pose(transform, fix["latitude"], fix["longitude"])
    return {
        "site": args.site,
        "map": map_pose,
        "rtk": fix,
        "transform_file": str(transform_file(args.site)),
    }


def list_anchors(args) -> dict:
    data = load_anchors(args.site)
    return {"site": args.site, "file": str(anchors_file(args.site)), "anchors": data["anchors"]}


def main() -> int:
    parser = argparse.ArgumentParser(description="Record and use RTK anchors for a MyRobot site map.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list")
    p.add_argument("--site", required=True)

    p = sub.add_parser("record")
    p.add_argument("--site", required=True)
    p.add_argument("--name", required=True)
    p.add_argument("--target-frame", default="map")
    p.add_argument("--source-frame", default="base_footprint")
    p.add_argument("--odom-topic", default="/odometry/global")
    p.add_argument("--fix-topic", default="/gps/fix_filtered")
    p.add_argument("--raw-fix-topic", default="/gps/fix")
    p.add_argument("--status-topic", default="/rtk/status")
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--allow-raw-fallback", action="store_true")
    p.add_argument("--no-require-map-frame", dest="require_map_frame", action="store_false")
    p.set_defaults(require_map_frame=True)

    p = sub.add_parser("compute")
    p.add_argument("--site", required=True)

    p = sub.add_parser("locate")
    p.add_argument("--site", required=True)
    p.add_argument("--fix-topic", default="/gps/fix_filtered")
    p.add_argument("--raw-fix-topic", default="/gps/fix")
    p.add_argument("--status-topic", default="/rtk/status")
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--allow-raw-fallback", action="store_true")

    args = parser.parse_args()
    try:
        if args.cmd == "list":
            result = list_anchors(args)
        elif args.cmd == "record":
            result = record_anchor(args)
        elif args.cmd == "compute":
            result = compute_transform(args)
        elif args.cmd == "locate":
            result = locate_current(args)
        else:
            raise ValueError(f"unknown command: {args.cmd}")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
