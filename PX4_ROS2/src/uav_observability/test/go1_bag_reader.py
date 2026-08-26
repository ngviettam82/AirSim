#!/usr/bin/env python3
"""G-O1 (P10.3) bag readback: message count + newest per-message stamp.

Opens the bag directory directly through rosbag2_py (same library `ros2
bag` itself uses) rather than trusting `ros2 bag info`'s metadata.yaml --
that file is only written on a clean Writer::close(), so this is the
measurement path that still has a chance after an unclean kill -9.

The stamp read back is whatever rosbag_manager_node passed as the `time`
argument to Writer::write(), i.e. rclcpp::Node::now() at write time --
system clock in this domain-99 rig (no /clock topic, use_sim_time unset),
directly comparable to the caller's wall-clock T_kill.

Usage: go1_bag_reader.py <bag_dir> <storage_id>
Stdout on success (always both lines):
  MSG_COUNT=<int>
  LAST_STAMP_SEC=<float|NAN>
Exit 1 + stderr READ_ERROR=<...> if the bag could not be opened/iterated
at all -- caller records that round as unreadable via this path.
"""
import sys

import rosbag2_py


def main() -> None:
    bag_dir, storage_id = sys.argv[1], sys.argv[2]
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    count = 0
    last_stamp_ns = None
    while reader.has_next():
        _topic, _data, stamp_ns = reader.read_next()
        count += 1
        if last_stamp_ns is None or stamp_ns > last_stamp_ns:
            last_stamp_ns = stamp_ns

    print(f'MSG_COUNT={count}')
    print('LAST_STAMP_SEC=' + ('NAN' if last_stamp_ns is None else f'{last_stamp_ns / 1e9:.6f}'))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # any open/parse failure means "unreadable"
        print(f'READ_ERROR={exc}', file=sys.stderr)
        sys.exit(1)
