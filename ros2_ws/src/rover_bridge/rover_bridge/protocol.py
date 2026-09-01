"""Wire-protocol helpers shared by the ROS2 bridge and its tests."""

import json
import math
import struct


MAX_HEADER_BYTES = 1024 * 1024
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
COMPRESSED_DEPTH_HEADER_SIZE = 12
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def encode_cmd(seq, vx, wz):
    """Encode one production command as newline-delimited JSON."""
    if not isinstance(seq, int):
        raise ValueError("seq must be an integer")
    if not isinstance(vx, (int, float)) or not math.isfinite(vx):
        raise ValueError("vx must be a finite number")
    if not isinstance(wz, (int, float)) or not math.isfinite(wz):
        raise ValueError("wz must be a finite number")
    message = {"type": "cmd_vel", "seq": seq, "vx": float(vx), "wz": float(wz)}
    return (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")


def recv_exact(sock, size):
    """Read exactly size bytes or raise ConnectionError at EOF."""
    if size < 0:
        raise ValueError("size must not be negative")
    result = bytearray()
    while len(result) < size:
        chunk = sock.recv(size - len(result))
        if not chunk:
            raise ConnectionError("socket closed while reading a packet")
        result.extend(chunk)
    return bytes(result)


def read_rgbd_packet(sock):
    """Read one camera_stream.py packet and return (header, rgb, depth)."""
    header_size = struct.unpack(">I", recv_exact(sock, 4))[0]
    if header_size == 0 or header_size > MAX_HEADER_BYTES:
        raise ValueError(f"invalid RGB-D header size: {header_size}")

    try:
        header = json.loads(recv_exact(sock, header_size).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid RGB-D header: {exc}") from exc
    if not isinstance(header, dict) or header.get("type") != "rgbd":
        raise ValueError("RGB-D header type must be rgbd")

    rgb_size = header.get("rgb_bytes")
    depth_size = header.get("depth_bytes")
    if not isinstance(rgb_size, int) or not isinstance(depth_size, int):
        raise ValueError("RGB-D payload sizes must be integers")
    if rgb_size < 0 or depth_size < 0 or rgb_size + depth_size > MAX_PAYLOAD_BYTES:
        raise ValueError("RGB-D payload is too large")

    return header, recv_exact(sock, rgb_size), recv_exact(sock, depth_size)


def depth_png_bytes(depth_payload):
    """Return PNG bytes from either a ROS compressedDepth or legacy payload."""
    if depth_payload.startswith(PNG_SIGNATURE):
        return depth_payload
    if (
        len(depth_payload) > COMPRESSED_DEPTH_HEADER_SIZE
        and depth_payload[COMPRESSED_DEPTH_HEADER_SIZE:].startswith(PNG_SIGNATURE)
    ):
        return depth_payload[COMPRESSED_DEPTH_HEADER_SIZE:]
    raise ValueError("depth payload is neither PNG nor ROS compressedDepth PNG")
