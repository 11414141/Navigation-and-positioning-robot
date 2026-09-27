#!/usr/bin/env python3
import math
import os
from pathlib import Path

import yaml


SITE_ROOT = Path(os.environ.get("SITE_ROOT", "/home/wheeltec/field_sites"))


def sanitize_site_name(name: str) -> str:
    return name.replace("/", "_").replace("\\", "_").replace(":", "_").replace("\n", "_")


def site_dir(site: str) -> Path:
    return SITE_ROOT / sanitize_site_name(site)


def routes_dir_for_site(site: str) -> Path:
    return site_dir(site) / "routes"


def route_file(site: str, route_name: str) -> Path:
    safe_name = sanitize_site_name(route_name)
    if not safe_name.endswith(".yaml"):
        safe_name = f"{safe_name}.yaml"
    return routes_dir_for_site(site) / safe_name


def yaw_to_quaternion(yaw: float) -> dict:
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def quaternion_to_yaw(q) -> float:
    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_route(site: str, route_name: str) -> dict:
    path = route_file(site, route_name)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "waypoints" not in data or not isinstance(data["waypoints"], list):
        raise ValueError(f"route has no waypoints list: {path}")
    return data


def save_route(site: str, route_name: str, data: dict) -> Path:
    path = route_file(site, route_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return path

