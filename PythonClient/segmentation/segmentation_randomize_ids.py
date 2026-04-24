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
        default=25,
        help="Number of matched objects to randomize. Use 0 to randomize all matched objects.",
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
                }
            )
    return rows


def randomize_ids(client, args):
    try:
        name_pattern = re.compile(args.regex, re.IGNORECASE)
    except re.error as exc:
        raise ValueError(f"Invalid regex '{args.regex}': {exc}") from exc

    all_objects = client.simListInstanceSegmentationObjects()
    matched_objects = [name for name in all_objects if name_pattern.search(name)]

    if not matched_objects:
        raise RuntimeError(
            f"No instance-segmentation objects matched regex '{args.regex}'. "
            "Run the script with '--regex .* --limit 10' first to inspect the scene."
        )

    rng = random.Random(args.seed)
    if args.limit > 0 and args.limit < len(matched_objects):
        selected_objects = rng.sample(matched_objects, args.limit)
    else:
        selected_objects = matched_objects

    rows = []
    for object_name in selected_objects:
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
            }
        )
    return rows, len(all_objects), len(matched_objects), len(selected_objects)


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
    if args.limit < 0:
        parser.error("--limit must be 0 or larger.")

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
        rows, total_count, matched_count, selected_count = randomize_ids(client, args)
    except (RuntimeError, ValueError) as exc:
        print(str(exc))
        return 1

    output_path = args.output or default_output_path("segmentation_randomized_ids")
    write_rows(output_path, rows)

    verified_count = sum(1 for row in rows if row["verified"])
    mismatch_count = len(rows) - verified_count

    print(f"Found {total_count} instance-segmentation objects in the scene.")
    print(f"Matched {matched_count} objects with regex '{args.regex}'.")
    print(f"Randomized {selected_count} object IDs in range [{args.id_min}, {args.id_max}].")
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
