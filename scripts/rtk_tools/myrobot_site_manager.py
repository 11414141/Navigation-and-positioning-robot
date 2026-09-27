#!/usr/bin/env python3
import argparse
import json
import shutil
import time
import re
from pathlib import Path

import yaml


SITE_ROOT = Path("/home/wheeltec/field_sites")
DELETED_ROOT = Path("/home/wheeltec/field_sites_deleted")


def sanitize(name):
    return name.replace("/", "_").replace("\\", "_").replace(":", "_").replace("\n", "_").strip()


def decode_site_name(name):
    name = sanitize(name)
    def repl(match):
        return chr(int(match.group(1), 16))

    return re.sub(r"u([0-9A-Fa-f]{4})", repl, name)


def registry_file():
    return SITE_ROOT / "sites.txt"


def site_dir(name):
    return SITE_ROOT / decode_site_name(name)


def ensure_site_dirs(path):
    for rel in ("maps/fast_lio_sessions", "maps/nav2_current", "bags", "logs"):
        (path / rel).mkdir(parents=True, exist_ok=True)


def read_sites():
    path = registry_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch(exist_ok=True)
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def write_sites(sites):
    path = registry_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(sites) + ("\n" if sites else ""), encoding="utf-8")


def register_site(name):
    name = decode_site_name(name)
    if not name:
        raise ValueError("site name is empty")
    sites = read_sites()
    if name not in sites:
        sites.append(name)
        write_sites(sites)
    ensure_site_dirs(site_dir(name))
    return name


def archive_site(name):
    name = decode_site_name(name)
    sites = [site for site in read_sites() if site != name]
    write_sites(sites)
    src = site_dir(name)
    if not src.exists():
        return ""
    DELETED_ROOT.mkdir(parents=True, exist_ok=True)
    dst = DELETED_ROOT / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.move(str(src), str(dst))
    return str(dst)


def resolve_image_path(yaml_path):
    for line in yaml_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("image:"):
            image = line.split(":", 1)[1].strip()
            path = Path(image)
            return path if path.is_absolute() else yaml_path.parent / path
    raise ValueError(f"map yaml has no image field: {yaml_path}")


def install_current_map(site, yaml_file):
    site = register_site(site)
    yaml_path = Path(yaml_file)
    if not yaml_path.is_file():
        raise FileNotFoundError(str(yaml_path))
    image_path = resolve_image_path(yaml_path)
    if not image_path.is_file():
        raise FileNotFoundError(str(image_path))
    current_dir = site_dir(site) / "maps/nav2_current"
    current_dir.mkdir(parents=True, exist_ok=True)
    current_yaml = current_dir / "current.yaml"
    current_pgm = current_dir / "current.pgm"
    shutil.copy2(image_path, current_pgm)
    shutil.copy2(yaml_path, current_yaml)
    lines = current_yaml.read_text(encoding="utf-8", errors="replace").splitlines()
    lines = ["image: current.pgm" if line.startswith("image:") else line for line in lines]
    current_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")
    for sidecar_name in ("rtk_anchors.yaml", "rtk_map_transform.yaml"):
        sidecar_src = yaml_path.parent / sidecar_name
        sidecar_dst = current_dir / sidecar_name
        if sidecar_src.is_file():
            shutil.copy2(sidecar_src, sidecar_dst)
        elif sidecar_dst.is_file():
            sidecar_dst.unlink()
    return str(current_yaml)


def refresh_semantic_places(site):
    result = {}
    try:
        from finalize_pending_rtk_places import finalize
        from argparse import Namespace

        transform = site_dir(site) / "maps/nav2_current/rtk_map_transform.yaml"
        if transform.is_file():
            result["pending_finalize"] = finalize(Namespace(
                site=site,
                transform=str(transform),
                overwrite=True,
                reprocess=False,
            ))
        else:
            result["pending_finalize"] = {"skipped": "missing_rtk_map_transform"}
    except Exception as exc:
        result["pending_finalize"] = {"error": str(exc)}

    try:
        from myrobot_places import refresh_places_map

        result["places_refresh"] = refresh_places_map(site)
    except Exception as exc:
        result["places_refresh"] = {"error": str(exc)}
    return result


def latest_candidate(site):
    path = site_dir(site)
    latest_file = path / "maps/nav2_latest_candidate.txt"
    candidates = []
    if latest_file.is_file():
        candidate = latest_file.read_text(encoding="utf-8", errors="replace").strip()
        if candidate and Path(candidate).is_file():
            candidates.append(Path(candidate))
    candidates.extend(path.glob("maps/fast_lio_sessions/*/nav2/fast_lio_2d.yaml"))
    valid = [candidate for candidate in candidates if candidate.is_file()]
    if not valid:
        return ""
    return str(max(valid, key=lambda item: item.stat().st_mtime))


def best_nav2_map(site):
    path = site_dir(site)
    current = path / "maps/nav2_current/current.yaml"
    if current.is_file():
        return str(current)
    return latest_candidate(site)


def semantic_places(site):
    path = site_dir(site) / "semantic_map/places.yaml"
    if not path.is_file():
        return []
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return []
    places = data.get("places", {})
    if not isinstance(places, dict):
        return []
    result = []
    for name, place in sorted(places.items()):
        if not isinstance(place, dict):
            continue
        result.append({
            "name": name,
            "frame_id": place.get("frame_id", "map"),
            "x": float(place.get("x", 0.0)),
            "y": float(place.get("y", 0.0)),
            "yaw": float(place.get("yaw", 0.0)),
            "created_at": place.get("created_at", ""),
            "current_map": place.get("current_map", ""),
            "current_map_mtime": place.get("current_map_mtime", 0),
        })
    return result


def site_info(name):
    path = site_dir(name)
    current = path / "maps/nav2_current/current.yaml"
    candidate = latest_candidate(name)
    usable = best_nav2_map(name)
    places = semantic_places(name)
    return {
        "name": name,
        "dir": str(path),
        "current_map": str(current),
        "has_current_map": current.is_file(),
        "latest_candidate": candidate,
        "has_latest_candidate": bool(candidate),
        "usable_map": usable,
        "has_usable_map": bool(usable),
        "places": places,
        "place_count": len(places),
    }


def list_state():
    sites = read_sites()
    return {
        "site_root": str(SITE_ROOT),
        "sites": [site_info(name) for name in sites],
        "sites_with_maps": [name for name in sites if (site_dir(name) / "maps/nav2_current/current.yaml").is_file()],
        "sites_with_usable_maps": [name for name in sites if best_nav2_map(name)],
    }


def main():
    parser = argparse.ArgumentParser(description="MyRobot site/map manager.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("create")
    p.add_argument("--site", required=True)
    p = sub.add_parser("delete")
    p.add_argument("--site", required=True)
    p = sub.add_parser("update-candidate")
    p.add_argument("--site", required=True)
    p = sub.add_parser("borrow-map")
    p.add_argument("--site", required=True)
    p.add_argument("--from-site", required=True)
    p = sub.add_parser("import-map")
    p.add_argument("--site", required=True)
    p.add_argument("--yaml", required=True)
    p = sub.add_parser("install-uploaded-map")
    p.add_argument("--site", required=True)
    p.add_argument("--yaml", required=True)
    p.add_argument("--image", required=True)
    args = parser.parse_args()

    if args.cmd == "list":
        result = list_state()
    elif args.cmd == "create":
        result = {"site": register_site(args.site)}
    elif args.cmd == "delete":
        result = {"site": sanitize(args.site), "archived": archive_site(args.site)}
    elif args.cmd == "update-candidate":
        candidate = latest_candidate(args.site)
        if not candidate:
            raise SystemExit("no latest candidate map")
        current_map = install_current_map(args.site, candidate)
        result = {"current_map": current_map, "places_refresh": refresh_semantic_places(args.site)}
    elif args.cmd == "borrow-map":
        source = best_nav2_map(args.from_site)
        if not source:
            raise SystemExit("source site has no current or candidate map")
        current_map = install_current_map(args.site, source)
        result = {"current_map": current_map, "places_refresh": refresh_semantic_places(args.site)}
    elif args.cmd == "import-map":
        current_map = install_current_map(args.site, args.yaml)
        result = {"current_map": current_map, "places_refresh": refresh_semantic_places(args.site)}
    elif args.cmd == "install-uploaded-map":
        current_map = install_current_map(args.site, args.yaml)
        result = {"current_map": current_map, "places_refresh": refresh_semantic_places(args.site)}
    else:
        raise SystemExit(f"unknown command: {args.cmd}")

    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
