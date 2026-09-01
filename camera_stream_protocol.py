"""Wire-format helpers for the Raspberry Pi camera stream."""

import struct


_COMPRESSED_DEPTH_HEADER = struct.Struct("<iff")


def build_compressed_depth_payload(depth_png):
    """Prefix a PNG depth image with the ROS compressedDepth config header."""
    if not isinstance(depth_png, (bytes, bytearray, memoryview)):
        raise TypeError("depth_png must be bytes-like")
    return _COMPRESSED_DEPTH_HEADER.pack(0, 0.0, 0.0) + bytes(depth_png)
