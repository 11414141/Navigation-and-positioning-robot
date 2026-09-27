#!/usr/bin/env python3
import argparse
import json
import sys

from myrobot_places import load_places, refresh_places_map
from route_common import save_route


def create_route(
    site: str,
    route_name: str,
    place_names: list[str],
    home_name: str = "充电站",
    refresh_map: bool = True,
) -> dict:
    refresh_result = None
    if refresh_map:
        try:
            refresh_result = refresh_places_map(site)
        except Exception as exc:
            refresh_result = {"error": str(exc)}
    data = load_places(site)
    places = data.get("places", {})
    if home_name:
        if home_name not in places:
            raise ValueError(f"home place not found: {home_name}")
        route_names = [home_name] + [name for name in place_names if name != home_name] + [home_name]
    else:
        route_names = place_names
    waypoints = []
    missing = []
    for name in route_names:
        place = places.get(name)
        if not place:
            missing.append(name)
            continue
        waypoints.append({
            "name": name,
            "frame_id": "map",
            "x": float(place["x"]),
            "y": float(place["y"]),
            "z": float(place.get("z", 0.0)),
            "yaw": float(place.get("yaw", 0.0)),
        })
    if missing:
        raise ValueError(f"places not found: {', '.join(missing)}")
    if not waypoints:
        raise ValueError("route has no places")
    path = save_route(site, route_name, {"name": route_name, "frame_id": "map", "waypoints": waypoints})
    return {
        "site": site,
        "route": route_name,
        "waypoint_count": len(waypoints),
        "file": str(path),
        "refresh_map": refresh_result,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a Nav2 route from saved semantic places.")
    parser.add_argument("--site", required=True)
    parser.add_argument("--route", required=True)
    parser.add_argument("--places", required=True, help="Comma-separated place names in navigation order")
    parser.add_argument("--home", default="充电站", help="Fixed start/end place name. Empty disables fixed home.")
    parser.add_argument("--no-refresh-map", dest="refresh_map", action="store_false")
    parser.set_defaults(refresh_map=True)
    args = parser.parse_args()
    place_names = [item.strip() for item in args.places.split(",") if item.strip()]
    try:
        result = create_route(args.site, args.route, place_names, args.home, args.refresh_map)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
