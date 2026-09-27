#!/usr/bin/env python3
"""Repair non-monotonic header timestamps in a recorded ROS2 bag.

The rover bridge stamps every message with the Raspberry Pi's clock. During the
room_run_05 recording the Pi delivered ~10% of its telemetry/camera packets
late and out of order, so many messages carry a header stamp earlier than the
previous one (regressions of up to ~8 s). RTAB-Map treats those as odometry
resets, increments the map id, and the graph fragments.

This tool rewrites the bag in receive order and drops any message whose
original header stamp regresses relative to the newest stamp already seen for
that topic. Surviving messages keep their original stamps (now monotonic) and
payload; the file is otherwise a faithful copy.

Usage:
    python3 tools/repair_bag.py recordings/room_run_05 recordings/room_run_05_clean
"""
import argparse
import sys
from collections import defaultdict

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
)
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

TOPIC_TYPES = {
    "nav_msgs/msg/Odometry": Odometry,
    "sensor_msgs/msg/BatteryState": BatteryState,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/CompressedImage": CompressedImage,
    "std_msgs/msg/String": String,
    "geometry_msgs/msg/Twist": Twist,
    "tf2_msgs/msg/TFMessage": TFMessage,
}

SKIP_TOPICS = {"/tf_static"}


def converter(input_format="cdr", output_format="cdr"):
    return ConverterOptions(
        input_serialization_format=input_format,
        output_serialization_format=output_format,
    )


def open_reader(uri):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=uri, storage_id="mcap"),
        converter(),
    )
    return reader


def header_stamps(msg):
    """Representative header stamp(s) in seconds, or None if the message has none."""
    if isinstance(msg, TFMessage):
        if not msg.transforms:
            return None
        return [t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 for t in msg.transforms]
    if hasattr(msg, "header"):
        s = msg.header.stamp
        return [s.sec + s.nanosec * 1e-9]
    return None


def repair(in_bag, out_bag, eps=1e-9):
    reader = open_reader(in_bag)
    topic_class = {}
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_bag, storage_id="mcap"), converter())
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)
        topic_class[meta.name] = TOPIC_TYPES.get(meta.type)

    max_stamp = {}
    total = defaultdict(int)
    dropped = defaultdict(int)
    untyped = set()

    while reader.has_next():
        topic, data, t = reader.read_next()
        total[topic] += 1
        cls = topic_class.get(topic)
        if topic in SKIP_TOPICS or cls is None:
            if cls is None:
                untyped.add(topic)
            writer.write(topic, data, t)
            continue

        stamps = header_stamps(deserialize_message(data, cls))
        if not stamps:
            writer.write(topic, data, t)
            continue

        lo, hi = min(stamps), max(stamps)
        previous = max_stamp.get(topic)
        if previous is not None and lo < previous - eps:
            dropped[topic] += 1
            continue
        if previous is None or hi > previous:
            max_stamp[topic] = hi
        writer.write(topic, data, t)

    print(f"repaired {in_bag} -> {out_bag}")
    for topic in sorted(total):
        kept = total[topic] - dropped[topic]
        note = " (kept as-is)" if topic in untyped or topic in SKIP_TOPICS else ""
        print(f"  {topic:45s} kept={kept:6d} dropped={dropped[topic]:5d}{note}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    args = parser.parse_args()
    repair(args.input_bag, args.output_bag)
    return 0


if __name__ == "__main__":
    sys.exit(main())