#!/usr/bin/env python3
"""Record a pending semantic place from RTK fixed data during bag recording."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import yaml

from myrobot_rtk_anchor import RosSnapshot
from route_common import sanitize_site_name, site_dir


def pending_file(site: str) -> Path:
    return site_dir(site) / "semantic_map" / "pending_rtk_places.yaml"


def load_pending(site: str) -> dict:
    path = pending_file(site)
    if not path.is_file():
        return {"site": site, "pending_places": []}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    places = data.get("pending_places", [])
    if not isinstance(places, list):
        places = []
    return {"site": data.get("site", site), "pending_places": places}


def save_pending(site: str, data: dict) -> Path:
    path = pending_file(site)
    path.parent.mkdir(parents=True, exist_ok=True)
    data["site"] = site
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return path


def record_pending(args) -> dict:
    name = sanitize_site_name(args.name).strip()
    if not name:
        raise ValueError("place name is empty")

    snapshot = RosSnapshot(args.timeout)
    try:
        fix = snapshot.read_fix(args.fix_topic, args.status_topic, args.raw_fix_topic, False)
    finally:
        snapshot.close()

    stamp_sec = fix.get("stamp_sec")
    if stamp_sec is None or float(stamp_sec) <= 0.0:
        stamp_sec = time.time()

    item = {
        "name": name,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": "pending_rtk_record",
        "bag_hint": args.bag_hint,
        "rtk": fix,
        "record": {
            "stamp_sec": float(stamp_sec),
            "note": args.note,
        },
        "finalized": False,
    }

    data = load_pending(args.site)
    places = data.setdefault("pending_places", [])
    overwritten = False
    if args.overwrite:
        kept = []
        for existing in places:
            if existing.get("name") == name and not existing.get("finalized", False):
                overwritten = True
                continue
            kept.append(existing)
        places = kept
    places.append(item)
    data["pending_places"] = places
    path = save_pending(args.site, data)
    return {
        "site": args.site,
        "name": name,
        "file": str(path),
        "overwritten": overwritten,
        "pending": item,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--fix-topic", default="/gps/fix_filtered")
    parser.add_argument("--raw-fix-topic", default="/gps/fix")
    parser.add_argument("--status-topic", default="/rtk/status")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--bag-hint", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--no-overwrite", dest="overwrite", action="store_false")
    parser.set_defaults(overwrite=True)
    args = parser.parse_args()

    try:
        result = record_pending(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
