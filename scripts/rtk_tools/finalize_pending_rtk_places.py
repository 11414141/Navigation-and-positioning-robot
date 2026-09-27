#!/usr/bin/env python3
"""Project pending RTK semantic places into map coordinates after offline mapping."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import yaml

from myrobot_places import current_map_info, load_places, save_places
from myrobot_rtk_anchor import load_yaml, rtk_to_map_pose
from record_pending_rtk_place import load_pending, pending_file, save_pending


def load_transform_file(path: Path) -> dict:
    data = load_yaml(path, {})
    if "enu_to_map" not in data:
        raise RuntimeError(f"invalid RTK map transform: {path}")
    return data


def finalize(args) -> dict:
    transform_path = Path(args.transform).expanduser()
    if not transform_path.is_file():
        raise FileNotFoundError(str(transform_path))
    transform = load_transform_file(transform_path)

    pending = load_pending(args.site)
    places_data = load_places(args.site)
    places = places_data.setdefault("places", {})

    finalized = []
    skipped = []
    for item in pending.get("pending_places", []):
        name = item.get("name", "")
        if not name:
            skipped.append({"name": "", "reason": "missing_name"})
            continue
        if item.get("finalized", False) and not args.reprocess:
            skipped.append({"name": name, "reason": "already_finalized"})
            continue
        rtk = item.get("rtk")
        if not isinstance(rtk, dict):
            skipped.append({"name": name, "reason": "missing_rtk"})
            continue
        try:
            map_pose = rtk_to_map_pose(transform, float(rtk["latitude"]), float(rtk["longitude"]))
            old_place = places.get(name, {})
            yaw = float(old_place.get("yaw", map_pose["yaw"]))
            place = {
                "frame_id": "map",
                "x": float(map_pose["x"]),
                "y": float(map_pose["y"]),
                "z": 0.0,
                "yaw": yaw,
                "created_at": old_place.get("created_at", item.get("created_at", time.strftime("%Y-%m-%d %H:%M:%S"))),
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "source": "pending_rtk_finalized",
                "pose_source": "rtk",
                "rtk": rtk,
                "pending_record": {
                    "created_at": item.get("created_at", ""),
                    "record": item.get("record", {}),
                    "bag_hint": item.get("bag_hint", ""),
                },
                "rtk_transform": str(transform_path),
            }
            place.update(current_map_info(args.site))
            if name in places and not args.overwrite:
                skipped.append({"name": name, "reason": "place_exists"})
                continue
            places[name] = place
            item["finalized"] = True
            item["finalized_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            item["finalized_transform"] = str(transform_path)
            finalized.append(name)
        except Exception as exc:
            skipped.append({"name": name, "reason": str(exc)})

    places_path = save_places(args.site, places_data)
    pending_path = save_pending(args.site, pending)
    return {
        "site": args.site,
        "transform": str(transform_path),
        "places_file": str(places_path),
        "pending_file": str(pending_path),
        "finalized": finalized,
        "finalized_count": len(finalized),
        "skipped": skipped,
        "skipped_count": len(skipped),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True)
    parser.add_argument("--transform", required=True, help="rtk_map_transform.yaml path")
    parser.add_argument("--overwrite", action="store_true", default=True)
    parser.add_argument("--no-overwrite", dest="overwrite", action="store_false")
    parser.add_argument("--reprocess", action="store_true")
    args = parser.parse_args()

    try:
        result = finalize(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
