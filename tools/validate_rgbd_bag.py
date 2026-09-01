#!/usr/bin/env python3
"""Validate the raw RGB-D and motion topics required by the first map."""

import argparse
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def topic_type(reader, topic):
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise ValueError(f"missing required topic: {topic}")
    return types[topic]


def validate(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    required = {
        args.rgb_topic: "sensor_msgs/msg/Image",
        args.depth_topic: "sensor_msgs/msg/Image",
        args.camera_info_topic: "sensor_msgs/msg/CameraInfo",
        args.odom_topic: "nav_msgs/msg/Odometry",
        args.tf_topic: "tf2_msgs/msg/TFMessage",
    }
    for topic, expected_type in required.items():
        actual_type = topic_type(reader, topic)
        if actual_type != expected_type:
            raise ValueError(
                f"{topic} has type {actual_type}, expected {expected_type}"
            )
    available_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    tf_static_type = available_types.get(args.tf_static_topic)
    if tf_static_type is not None and tf_static_type != "tf2_msgs/msg/TFMessage":
        raise ValueError(
            f"{args.tf_static_topic} has type {tf_static_type}, expected tf2_msgs/msg/TFMessage"
        )
    if args.require_tf_static and tf_static_type is None:
        raise ValueError(f"missing required topic: {args.tf_static_topic}")

    message_types = {
        topic: get_message(message_type)
        for topic, message_type in required.items()
    }
    counts = {topic: 0 for topic in required}
    first_ns = {topic: None for topic in required}
    last_ns = {topic: None for topic in required}
    previous_ns = {topic: None for topic in required}
    rgb_depth_pairs = []
    latest_rgb_ns = None
    latest_depth_ns = None
    camera_info = None
    static_camera_found = False
    static_type = get_message(tf_static_type) if tf_static_type else None

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == args.tf_static_topic and static_type is not None:
            message = deserialize_message(data, static_type)
            for transform in message.transforms:
                if (
                    transform.header.frame_id == args.base_frame
                    and transform.child_frame_id == args.camera_frame
                ):
                    static_camera_found = True
            continue
        if topic not in required:
            continue
        message = deserialize_message(data, message_types[topic])
        counts[topic] += 1
        current_ns = stamp_ns(message)
        if previous_ns[topic] is not None and current_ns < previous_ns[topic]:
            raise ValueError(f"non-monotonic header timestamp on {topic}")
        previous_ns[topic] = current_ns
        first_ns[topic] = current_ns if first_ns[topic] is None else first_ns[topic]
        last_ns[topic] = current_ns
        if topic == args.rgb_topic:
            latest_rgb_ns = current_ns
            if latest_depth_ns is not None:
                rgb_depth_pairs.append(abs(current_ns - latest_depth_ns))
        elif topic == args.depth_topic:
            latest_depth_ns = current_ns
            if latest_rgb_ns is not None:
                rgb_depth_pairs.append(abs(latest_rgb_ns - current_ns))
        elif topic == args.camera_info_topic and camera_info is None:
            camera_info = message

    for topic, count in counts.items():
        if count == 0:
            raise ValueError(f"required topic has no messages: {topic}")
    if camera_info.width <= 0 or camera_info.height <= 0:
        raise ValueError("camera info has invalid dimensions")
    intrinsic_values = (camera_info.k[0], camera_info.k[2], camera_info.k[4], camera_info.k[5])
    if not all(math.isfinite(value) and value > 0.0 for value in (intrinsic_values[0], intrinsic_values[2])):
        raise ValueError("camera info has invalid fx or fy")
    if not all(math.isfinite(value) for value in intrinsic_values):
        raise ValueError("camera info has non-finite principal point")
    if args.require_tf_static and not static_camera_found:
        raise ValueError(
            f"missing {args.base_frame} -> {args.camera_frame} on {args.tf_static_topic}"
        )
    camera_start = min(first_ns[args.rgb_topic], first_ns[args.depth_topic])
    camera_end = max(last_ns[args.rgb_topic], last_ns[args.depth_topic])
    for topic in (args.odom_topic, args.tf_topic):
        if first_ns[topic] > camera_start or last_ns[topic] < camera_end:
            raise ValueError(f"{topic} does not cover the full camera interval")
    if rgb_depth_pairs and max(rgb_depth_pairs) > args.sync_tolerance_ns:
        raise ValueError(
            f"RGB/depth timestamp separation exceeds {args.sync_tolerance_ms:.0f} ms"
        )

    print(f"VALID: {args.bag}")
    for topic, count in counts.items():
        span_s = max((last_ns[topic] - first_ns[topic]) / 1e9, 1e-9)
        print(f"  {topic}: {count} messages, {count / span_s:.2f} Hz")
    print(
        f"  camera: {camera_info.width}x{camera_info.height}, "
        f"fx={camera_info.k[0]:.2f}, fy={camera_info.k[4]:.2f}, "
        f"cx={camera_info.k[2]:.2f}, cy={camera_info.k[5]:.2f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--rgb-topic", default="/oak/rgb/image_raw")
    parser.add_argument("--depth-topic", default="/oak/stereo/image_raw")
    parser.add_argument("--camera-info-topic", default="/oak/rgb/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--sync-tolerance-ms", type=float, default=100.0)
    parser.add_argument("--require-tf-static", action="store_true")
    args = parser.parse_args()
    args.sync_tolerance_ns = int(args.sync_tolerance_ms * 1e6)
    try:
        validate(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())