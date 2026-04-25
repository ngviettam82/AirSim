#!/usr/bin/env python

import argparse
import csv
from datetime import datetime
import os
import random
import re
import sys

import setup_path
import cosysairsim as airsim
import numpy as np


def build_parser():
    parser = argparse.ArgumentParser(
        description="Randomize instance-segmentation IDs and verify that simSetSegmentationObjectID works."
    )
    parser.add_argument(
        "--regex",
        default=".*",
        help="Case-insensitive regex used to filter object names from simListInstanceSegmentationObjects().",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Number of matched objects to randomize. Use 0 to randomize all matched objects. Default is 25, or all objects when --screen-only is used.",
    )
    parser.add_argument(
        "--screen-only",
        action="store_true",
        help="Randomize only object IDs whose current segmentation color is visible in the requested camera image.",
    )
    parser.add_argument(
        "--camera-name",
        default="frontcamera",
        help="Camera used with --screen-only. Default matches the segmentation examples: frontcamera.",
    )
    parser.add_argument(
        "--vehicle-name",
        default="",
        help="Vehicle name passed to simGetImages when --screen-only is used.",
    )
    parser.add_argument(
        "--min-pixels",
        type=int,
        default=1,
        help="Minimum pixel count for a segmentation color to count as visible with --screen-only.",
    )
    parser.add_argument(
        "--keep-landscape-components",
        action="store_true",
        help="Do not collapse landscape component names into one landscape update group.",
    )
    parser.add_argument(
        "--id-min",
        type=int,
        default=1,
        help="Minimum object ID to assign. Default is 1 so the infrared view does not stay black.",
    )
    parser.add_argument(
        "--id-max",
        type=int,
        default=255,
        help="Maximum object ID to assign. Default is 255 to stay inside the infrared grayscale range.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed so the same assignment can be reproduced.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional CSV output path. Default writes next to this script with a timestamped filename.",
    )
    parser.add_argument(
        "--restore",
        default="",
        help="Restore original IDs from a CSV previously produced by this script instead of randomizing.",
    )
    return parser


def default_output_path(prefix):
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, f"{prefix}_{timestamp}.csv")


def write_rows(csv_path, rows):
    fieldnames = [
        "object_name",
        "old_id",
        "new_id",
        "readback_id",
        "set_call_succeeded",
        "verified",
        "selection_group",
        "visible_pixel_count",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def pick_random_id(rng, min_id, max_id, old_id):
    if min_id == max_id:
        return min_id

    for _ in range(16):
        candidate = rng.randint(min_id, max_id)
        if candidate != old_id:
            return candidate

    return min_id if old_id != min_id else max_id


def restore_ids(client, csv_path):
    rows = []
    with open(csv_path, "r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            object_name = row["object_name"]
            target_id = int(row["old_id"])
            current_id = client.simGetSegmentationObjectID(object_name)
            success = client.simSetSegmentationObjectID(object_name, target_id, False)
            readback_id = client.simGetSegmentationObjectID(object_name)
            rows.append(
                {
                    "object_name": object_name,
                    "old_id": current_id,
                    "new_id": target_id,
                    "readback_id": readback_id,
                    "set_call_succeeded": success,
                    "verified": success and readback_id == target_id,
                    "selection_group": get_selection_group(object_name),
                    "visible_pixel_count": "",
                }
            )
    return rows


def get_effective_limit(args):
    if args.limit is not None:
        return args.limit
    return 0 if args.screen_only else 25


def get_selection_group(object_name, keep_landscape_components=False):
    if keep_landscape_components:
        return object_name

    match = re.match(r"^Landscape_(?P<label>.+)_LandscapeComponent_\d+_-?\d+_-?\d+$", object_name)
    if match:
        return f"Landscape:{match.group('label')}"

    return object_name


def dedupe_landscape_groups(object_names, keep_landscape_components=False):
    selected_by_group = {}
    for object_name in object_names:
        selection_group = get_selection_group(object_name, keep_landscape_components)
        selected_by_group.setdefault(selection_group, object_name)
    return list(selected_by_group.values())


def get_visible_segmentation_id_counts(client, args):
    responses = client.simGetImages(
        [airsim.ImageRequest(args.camera_name, airsim.ImageType.Segmentation, False, False)],
        args.vehicle_name,
    )
    if not responses:
        raise RuntimeError("simGetImages returned no responses for the segmentation request.")

    response = responses[0]
    if response.width <= 0 or response.height <= 0 or len(response.image_data_uint8) == 0:
        raise RuntimeError(
            f"Segmentation image from camera '{args.camera_name}' is empty "
            f"(width={response.width}, height={response.height})."
        )

    image_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    pixel_count = response.width * response.height
    if image_1d.size == pixel_count * 3:
        image_rgb = image_1d.reshape((response.height, response.width, 3))
    elif image_1d.size == pixel_count * 4:
        image_rgb = image_1d.reshape((response.height, response.width, 4))[:, :, :3]
    else:
        raise RuntimeError(
            f"Unexpected segmentation image byte count: got {image_1d.size}, "
            f"expected {pixel_count * 3} or {pixel_count * 4}."
        )

    unique_colors, color_counts = np.unique(image_rgb.reshape((-1, 3)), axis=0, return_counts=True)
    color_map = client.simGetSegmentationColorMap()
    color_to_id = {
        (int(color[0]), int(color[1]), int(color[2])): object_id
        for object_id, color in enumerate(color_map)
    }

    visible_id_counts = {}
    for color, pixel_count_for_color in zip(unique_colors, color_counts):
        if pixel_count_for_color < args.min_pixels:
            continue
        color_key = (int(color[0]), int(color[1]), int(color[2]))
        object_id = color_to_id.get(color_key)
        if object_id is not None:
            visible_id_counts[object_id] = visible_id_counts.get(object_id, 0) + int(pixel_count_for_color)

    return visible_id_counts, response.width, response.height


def randomize_ids(client, args):
    try:
        name_pattern = re.compile(args.regex, re.IGNORECASE)
    except re.error as exc:
        raise ValueError(f"Invalid regex '{args.regex}': {exc}") from exc

    effective_limit = get_effective_limit(args)
    visible_id_counts = None
    image_size = None
    if args.screen_only:
        visible_id_counts, image_width, image_height = get_visible_segmentation_id_counts(client, args)
        image_size = (image_width, image_height)

    all_objects = sorted(client.simListInstanceSegmentationObjects())
    current_id_cache = {}
    matched_objects = []
    for name in all_objects:
        if not name_pattern.search(name):
            continue

        if visible_id_counts is not None:
            current_id = client.simGetSegmentationObjectID(name)
            current_id_cache[name] = current_id
            if current_id not in visible_id_counts:
                continue

        matched_objects.append(name)

    deduped_objects = dedupe_landscape_groups(matched_objects, args.keep_landscape_components)

    if not deduped_objects:
        screen_hint = ""
        if args.screen_only:
            screen_hint = (
                f" Visible segmentation IDs in camera '{args.camera_name}': "
                f"{sorted(visible_id_counts.keys()) if visible_id_counts else []}."
            )
        raise RuntimeError(
            f"No instance-segmentation objects matched regex '{args.regex}'. "
            "Run the script with '--regex .* --limit 10' first to inspect the scene."
            + screen_hint
        )

    rng = random.Random(args.seed)
    if effective_limit > 0 and effective_limit < len(deduped_objects):
        selected_objects = rng.sample(deduped_objects, effective_limit)
    else:
        selected_objects = deduped_objects

    rows = []
    for object_name in selected_objects:
        old_id = current_id_cache.get(object_name)
        if old_id is None:
            old_id = client.simGetSegmentationObjectID(object_name)
        new_id = pick_random_id(rng, args.id_min, args.id_max, old_id)
        success = client.simSetSegmentationObjectID(object_name, new_id, False)
        readback_id = client.simGetSegmentationObjectID(object_name)
        rows.append(
            {
                "object_name": object_name,
                "old_id": old_id,
                "new_id": new_id,
                "readback_id": readback_id,
                "set_call_succeeded": success,
                "verified": success and readback_id == new_id,
                "selection_group": get_selection_group(object_name, args.keep_landscape_components),
                "visible_pixel_count": visible_id_counts.get(old_id, "") if visible_id_counts is not None else "",
            }
        )
    return rows, {
        "total_count": len(all_objects),
        "matched_count": len(matched_objects),
        "deduped_count": len(deduped_objects),
        "selected_count": len(selected_objects),
        "effective_limit": effective_limit,
        "screen_only": args.screen_only,
        "visible_id_count": len(visible_id_counts) if visible_id_counts is not None else 0,
        "image_size": image_size,
    }


def print_preview(rows, max_rows=12):
    for row in rows[:max_rows]:
        print(
            f"{row['object_name']}: "
            f"{row['old_id']} -> {row['new_id']} "
            f"(readback={row['readback_id']}, ok={row['verified']})"
        )
    if len(rows) > max_rows:
        print(f"... {len(rows) - max_rows} more rows omitted")


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.id_min < 0 or args.id_max < 0:
        parser.error("--id-min and --id-max must be non-negative.")
    if args.id_min > args.id_max:
        parser.error("--id-min must be smaller than or equal to --id-max.")
    if args.limit is not None and args.limit < 0:
        parser.error("--limit must be 0 or larger.")
    if args.min_pixels < 1:
        parser.error("--min-pixels must be at least 1.")

    client = airsim.VehicleClient()
    client.confirmConnection()

    if args.restore:
        rows = restore_ids(client, args.restore)
        output_path = args.output or default_output_path("segmentation_restore_result")
        write_rows(output_path, rows)
        verified_count = sum(1 for row in rows if row["verified"])
        print(f"Restored {verified_count}/{len(rows)} objects from {args.restore}")
        print_preview(rows)
        print(f"Restore report saved to {output_path}")
        return 0 if verified_count == len(rows) else 2

    try:
        rows, stats = randomize_ids(client, args)
    except (RuntimeError, ValueError) as exc:
        print(str(exc))
        return 1

    output_path = args.output or default_output_path("segmentation_randomized_ids")
    write_rows(output_path, rows)

    verified_count = sum(1 for row in rows if row["verified"])
    mismatch_count = len(rows) - verified_count

    print(f"Found {stats['total_count']} instance-segmentation objects in the scene.")
    print(f"Matched {stats['matched_count']} objects with regex '{args.regex}'.")
    if not args.keep_landscape_components:
        print(f"Collapsed landscape components to {stats['deduped_count']} update groups.")
    if stats["screen_only"]:
        print(
            f"Screen filter used camera '{args.camera_name}' "
            f"({stats['image_size'][0]}x{stats['image_size'][1]}) and found "
            f"{stats['visible_id_count']} visible segmentation IDs."
        )
    print(f"Randomized {stats['selected_count']} object IDs in range [{args.id_min}, {args.id_max}].")
    if args.seed is not None:
        print(f"Random seed: {args.seed}")

    print_preview(rows)
    print(f"Verified {verified_count}/{len(rows)} API updates.")
    print(f"CSV report saved to {output_path}")

    if mismatch_count > 0:
        print(f"{mismatch_count} objects did not read back with the requested ID.")
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main())
