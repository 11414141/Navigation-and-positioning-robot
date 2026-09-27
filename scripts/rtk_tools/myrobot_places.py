#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from nav_msgs.msg import Odometry

from myrobot_rtk_anchor import RosSnapshot, load_transform, rtk_to_map_pose, transform_file
from route_common import quaternion_to_yaw, sanitize_site_name, site_dir


def current_map_info(site: str) -> dict:
    path = site_dir(site) / "maps/nav2_current/current.yaml"
    if not path.is_file():
        return {}
    stat = path.stat()
    info = {
        "current_map": str(path),
        "current_map_mtime": int(stat.st_mtime),
        "current_map_size": int(stat.st_size),
    }
    tf_path = transform_file(site)
    if tf_path.is_file():
        tf_stat = tf_path.stat()
        info.update({
            "rtk_transform": str(tf_path),
            "rtk_transform_mtime": int(tf_stat.st_mtime),
            "rtk_transform_size": int(tf_stat.st_size),
        })
    return info


def places_dir(site: str) -> Path:
    return site_dir(site) / "semantic_map"


def places_file(site: str) -> Path:
    return places_dir(site) / "places.yaml"


def load_places(site: str) -> dict:
    path = places_file(site)
    if not path.is_file():
        return {"site": site, "places": {}}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    places = data.get("places", {})
    if not isinstance(places, dict):
        places = {}
    return {"site": data.get("site", site), "places": places}


def save_places(site: str, data: dict) -> Path:
    path = places_file(site)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return path


def list_places(site: str) -> dict:
    data = load_places(site)
    items = []
    for name, place in sorted(data.get("places", {}).items()):
        items.append({
            "name": name,
            "frame_id": place.get("frame_id", "map"),
            "x": float(place.get("x", 0.0)),
            "y": float(place.get("y", 0.0)),
            "z": float(place.get("z", 0.0)),
            "yaw": float(place.get("yaw", 0.0)),
            "created_at": place.get("created_at", ""),
            "rtk_available": bool(place.get("rtk")),
        })
    return {"site": site, "file": str(places_file(site)), "places": items}


def lookup_current_pose(target_frame: str, source_frame: str, timeout: float):
    rclpy.init()
    node = rclpy.create_node("myrobot_place_recorder")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    deadline = time.time() + timeout
    transform = None
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            break
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()
    if transform is None:
        raise RuntimeError(f"TF unavailable: {target_frame} -> {source_frame}")
    return transform


def lookup_odometry(topic: str, timeout: float):
    rclpy.init()
    node = rclpy.create_node("myrobot_place_odom_recorder")
    latest = {"msg": None}
    node.create_subscription(Odometry, topic, lambda msg: latest.__setitem__("msg", msg), 10)

    deadline = time.time() + timeout
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if latest["msg"] is not None:
            break

    msg = latest["msg"]
    node.destroy_node()
    rclpy.shutdown()
    if msg is None:
        raise RuntimeError(f"odometry unavailable: {topic}")
    return msg


def read_current_rtk(timeout: float, fix_topic: str, status_topic: str, raw_fix_topic: str, allow_raw_fallback: bool) -> dict:
    snapshot = RosSnapshot(timeout)
    try:
        return snapshot.read_fix(fix_topic, status_topic, raw_fix_topic, allow_raw_fallback)
    finally:
        snapshot.close()


def pose_from_tf_or_odom(target_frame: str, source_frame: str, timeout: float, odom_topic: str) -> dict:
    source = "tf"
    try:
        transform = lookup_current_pose(target_frame, source_frame, timeout)
        frame_id = target_frame
        t = transform.transform.translation
        q_msg = transform.transform.rotation
    except Exception:
        odom = lookup_odometry(odom_topic, timeout)
        source = odom_topic
        frame_id = odom.header.frame_id or "camera_init"
        t = odom.pose.pose.position
        q_msg = odom.pose.pose.orientation
    yaw = quaternion_to_yaw({"x": q_msg.x, "y": q_msg.y, "z": q_msg.z, "w": q_msg.w})
    return {
        "frame_id": frame_id,
        "x": float(t.x),
        "y": float(t.y),
        "z": 0.0,
        "yaw": float(yaw),
        "source": source,
    }


def pose_from_rtk(site: str, rtk: dict) -> dict:
    transform = load_transform(site)
    map_pose = rtk_to_map_pose(transform, float(rtk["latitude"]), float(rtk["longitude"]))
    return {
        "frame_id": "map",
        "x": float(map_pose["x"]),
        "y": float(map_pose["y"]),
        "z": 0.0,
        "yaw": float(map_pose["yaw"]),
        "source": "rtk_map_transform",
    }


def record_place(
    site: str,
    name: str,
    target_frame: str,
    source_frame: str,
    timeout: float,
    odom_topic: str,
    fix_topic: str,
    status_topic: str,
    raw_fix_topic: str,
    allow_raw_fallback: bool,
    require_rtk: bool,
    pose_source: str,
) -> dict:
    name = sanitize_site_name(name).strip()
    if not name:
        raise ValueError("place name is empty")

    rtk = None
    rtk_error = None
    if pose_source in ("auto", "rtk") or require_rtk:
        try:
            rtk = read_current_rtk(timeout, fix_topic, status_topic, raw_fix_topic, allow_raw_fallback)
        except Exception as exc:
            rtk_error = str(exc)
            if require_rtk or pose_source == "rtk":
                raise RuntimeError(f"RTK unavailable for semantic place: {exc}")

    pose = None
    pose_error = None
    if pose_source in ("auto", "rtk") and rtk is not None:
        try:
            pose = pose_from_rtk(site, rtk)
        except Exception as exc:
            pose_error = str(exc)
            if pose_source == "rtk":
                raise RuntimeError(f"RTK->map pose unavailable: {exc}")

    if pose is None:
        if pose_source == "rtk":
            raise RuntimeError("RTK pose source selected but no RTK map pose was produced")
        try:
            pose = pose_from_tf_or_odom(target_frame, source_frame, timeout, odom_topic)
        except Exception as exc:
            if pose_error:
                raise RuntimeError(f"pose unavailable: RTK error={pose_error}; TF/odom error={exc}")
            raise

    place = {
        "frame_id": pose["frame_id"],
        "x": pose["x"],
        "y": pose["y"],
        "z": pose["z"],
        "yaw": pose["yaw"],
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": pose["source"],
        "pose_source": pose_source,
    }
    if pose_error:
        place["pose_error"] = pose_error
    if rtk_error:
        place["rtk_error"] = rtk_error
    if rtk is not None:
        place["rtk"] = rtk

    place.update(current_map_info(site))

    data = load_places(site)
    data["site"] = site
    data.setdefault("places", {})[name] = place
    path = save_places(site, data)
    return {"site": site, "name": name, "place": place, "file": str(path)}


def record_origin(site: str, name: str, frame_id: str) -> dict:
    name = sanitize_site_name(name).strip()
    if not name:
        raise ValueError("place name is empty")
    place = {
        "frame_id": frame_id,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": "mapping_start_origin",
    }
    place.update(current_map_info(site))
    data = load_places(site)
    data["site"] = site
    data.setdefault("places", {})[name] = place
    path = save_places(site, data)
    return {"site": site, "name": name, "place": place, "file": str(path)}


def refresh_places_map(site: str, keep_yaw: bool = True) -> dict:
    transform = load_transform(site)
    data = load_places(site)
    places = data.setdefault("places", {})
    refreshed = []
    skipped = []
    for name, place in places.items():
        rtk = place.get("rtk")
        if not isinstance(rtk, dict):
            skipped.append({"name": name, "reason": "missing_rtk"})
            continue
        try:
            map_pose = rtk_to_map_pose(transform, float(rtk["latitude"]), float(rtk["longitude"]))
            old_yaw = float(place.get("yaw", 0.0))
            place["frame_id"] = "map"
            place["x"] = map_pose["x"]
            place["y"] = map_pose["y"]
            place["z"] = 0.0
            place["yaw"] = old_yaw if keep_yaw else map_pose["yaw"]
            place["source"] = "rtk_reprojected"
            place["reprojected_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            place.update(current_map_info(site))
            refreshed.append(name)
        except Exception as exc:
            skipped.append({"name": name, "reason": str(exc)})
    path = save_places(site, data)
    return {
        "site": site,
        "file": str(path),
        "refreshed": refreshed,
        "refreshed_count": len(refreshed),
        "skipped": skipped,
        "skipped_count": len(skipped),
    }


def delete_place(site: str, name: str) -> dict:
    data = load_places(site)
    places = data.setdefault("places", {})
    existed = name in places
    if existed:
        del places[name]
        save_places(site, data)
    return {"site": site, "name": name, "deleted": existed, "file": str(places_file(site))}


def main() -> int:
    parser = argparse.ArgumentParser(description="Manage MyRobot semantic places for a work site.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("list")
    p.add_argument("--site", required=True)
    p = sub.add_parser("record")
    p.add_argument("--site", required=True)
    p.add_argument("--name", required=True)
    p.add_argument("--target-frame", default="map")
    p.add_argument("--source-frame", default="base_footprint")
    p.add_argument("--odom-topic", default="/Odometry")
    p.add_argument("--fix-topic", default="/gps/fix_filtered")
    p.add_argument("--raw-fix-topic", default="/gps/fix")
    p.add_argument("--status-topic", default="/rtk/status")
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--allow-raw-fallback", action="store_true")
    p.add_argument("--require-rtk", action="store_true")
    p.add_argument(
        "--pose-source",
        choices=["auto", "rtk", "tf"],
        default="auto",
        help="auto prefers RTK->map when available, rtk requires RTK->map, tf uses map->base_footprint or odometry",
    )
    p = sub.add_parser("record-origin")
    p.add_argument("--site", required=True)
    p.add_argument("--name", default="充电站")
    p.add_argument("--frame-id", default="map")
    p = sub.add_parser("refresh-map")
    p.add_argument("--site", required=True)
    p.add_argument("--use-transform-yaw", dest="keep_yaw", action="store_false")
    p.set_defaults(keep_yaw=True)
    p = sub.add_parser("delete")
    p.add_argument("--site", required=True)
    p.add_argument("--name", required=True)
    args = parser.parse_args()

    try:
        if args.cmd == "list":
            result = list_places(args.site)
        elif args.cmd == "record":
            result = record_place(
                args.site,
                args.name,
                args.target_frame,
                args.source_frame,
                args.timeout,
                args.odom_topic,
                args.fix_topic,
                args.status_topic,
                args.raw_fix_topic,
                args.allow_raw_fallback,
                args.require_rtk,
                args.pose_source,
            )
        elif args.cmd == "record-origin":
            result = record_origin(args.site, args.name, args.frame_id)
        elif args.cmd == "refresh-map":
            result = refresh_places_map(args.site, args.keep_yaw)
        elif args.cmd == "delete":
            result = delete_place(args.site, args.name)
        else:
            raise ValueError(f"unknown command: {args.cmd}")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
