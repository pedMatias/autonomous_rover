"""Bounded-bandwidth OAK-D RGB/depth stream for the ROS2 PC.

Frames are sent as a length-prefixed binary packet over TCP. Each packet is:

    4-byte big-endian JSON-header length
    JSON header with timestamps, shapes, dtypes, and payload lengths
    JPEG RGB payload
    PNG depth payload (uint16 millimetres)

The ROS2 PC can decode the packets and publish sensor_msgs/Image and
sensor_msgs/CameraInfo. PNG depth avoids sending raw 640x400 uint16 frames at
15 FPS over Wi-Fi. The OAK-D performs stereo matching onboard; this process
only compresses and forwards the resulting frames.
"""
import argparse
import json
import logging
import os
import socket
import struct
import time

import cv2

from recorder.oak_camera import OakCamera


REPO_ROOT = os.path.dirname(os.path.realpath(__file__))


def send_packet(client, header, rgb_bytes, depth_bytes):
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    packet = struct.pack(">I", len(header_bytes)) + header_bytes + rgb_bytes + depth_bytes
    client.sendall(packet)


def serve(args):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    camera = OakCamera(
        rgb_size=(args.rgb_width, args.rgb_height),
        mono_size=(640, 400),
        fps=args.fps,
    )
    camera.start()
    camera.warmup(args.warmup)
    camera_info = camera.get_camera_info()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind_host, args.port))
        server.listen(1)
        server.settimeout(1.0)
        logging.info("OAK-D stream listening on %s:%s", args.bind_host, args.port)
        try:
            while True:
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                logging.info("Camera client connected: %s", address)
                try:
                    client.settimeout(2.0)
                    while True:
                        result = camera.read(timeout=2.0)
                        if result is None:
                            continue
                        timestamp, rgb_frame, depth_frame = result
                        ok_rgb, rgb_encoded = cv2.imencode(
                            ".jpg", rgb_frame,
                            [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                        )
                        ok_depth, depth_encoded = cv2.imencode(
                            ".png", depth_frame,
                            [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression],
                        )
                        if not ok_rgb or not ok_depth:
                            logging.warning("Failed to encode an RGB/depth frame")
                            continue
                        rgb_bytes = rgb_encoded.tobytes()
                        depth_bytes = depth_encoded.tobytes()
                        send_packet(client, {
                            "type": "rgbd",
                            "ts": timestamp,
                            "rgb_encoding": "jpeg_bgr8",
                            "depth_encoding": "png_16uc1_mm",
                            "rgb_width": rgb_frame.shape[1],
                            "rgb_height": rgb_frame.shape[0],
                            "depth_width": depth_frame.shape[1],
                            "depth_height": depth_frame.shape[0],
                            "rgb_bytes": len(rgb_bytes),
                            "depth_bytes": len(depth_bytes),
                            "camera_info": camera_info,
                        }, rgb_bytes, depth_bytes)
                except (ConnectionError, OSError, socket.timeout) as exc:
                    logging.info("Camera client disconnected: %s", exc)
                finally:
                    client.close()
        finally:
            camera.stop()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--png-compression", type=int, default=3)
    return parser.parse_args()


if __name__ == "__main__":
    serve(parse_args())