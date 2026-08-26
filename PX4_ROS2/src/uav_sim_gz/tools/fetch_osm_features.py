#!/usr/bin/env python3
"""Downloads all scene map layers via Overpass API.

Data (c) OpenStreetMap contributors, ODbL 1.0.
"""

import argparse
import collections
import json
import sys

import requests

# Main instance 504s when busy; mirrors run the same software.
OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.jp/api/interpreter",
)

# "out geom" inlines node coords, else Overpass returns bare ids.
QUERY_TEMPLATE = """[out:json][timeout:{timeout}];
(
  way["building"](around:{radius},{latitude},{longitude});
  way["highway"](around:{radius},{latitude},{longitude});
  way["landuse"](around:{radius},{latitude},{longitude});
  way["leisure"](around:{radius},{latitude},{longitude});
  way["natural"](around:{radius},{latitude},{longitude});
  way["amenity"="parking"](around:{radius},{latitude},{longitude});
  way["barrier"](around:{radius},{latitude},{longitude});
  node["natural"="tree"](around:{radius},{latitude},{longitude});
);
out geom;
"""


def summarise(payload: dict) -> None:
    counts = collections.Counter()
    for element in payload.get("elements", []):
        tags = element.get("tags", {})
        for key in ("building", "highway", "landuse", "leisure", "natural",
                    "amenity", "barrier"):
            if key in tags:
                counts[f"{key}={tags[key]}" if key != "building" else "building"] += 1
                break

    print("layers found:")
    for label, count in counts.most_common(24):
        print(f"  {count:5d}  {label}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latitude", type=float, default=10.7729)
    parser.add_argument("--longitude", type=float, default=106.6577)
    parser.add_argument("--radius", type=float, default=400.0, help="metres")
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--out", default="osm_features.json")
    args = parser.parse_args()

    query = QUERY_TEMPLATE.format(
        timeout=args.timeout, radius=int(args.radius),
        latitude=args.latitude, longitude=args.longitude)
    print(f"querying Overpass within {args.radius:.0f} m of "
          f"{args.latitude}, {args.longitude}")

    payload = None
    for url in OVERPASS_URLS:
        try:
            response = requests.post(
                url, data={"data": query}, timeout=args.timeout + 60,
                headers={"User-Agent": "uav_sim_gz world generator"})
            response.raise_for_status()
            payload = response.json()
            print(f"answered by {url}")
            break
        except requests.RequestException as error:
            print(f"  {url}: {error}", file=sys.stderr)

    if payload is None:
        print("every Overpass endpoint failed", file=sys.stderr)
        raise SystemExit(1)

    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle)

    print(f"{len(payload.get('elements', []))} elements -> {args.out}")
    summarise(payload)


if __name__ == "__main__":
    main()
