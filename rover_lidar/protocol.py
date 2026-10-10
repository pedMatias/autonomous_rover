"""Versioned, bounded lidar transport: JSON metadata and uncompressed samples."""

from dataclasses import dataclass
import json
import math
import struct


MAX_HEADER = 4096
MAX_SAMPLES = 4096


@dataclass(frozen=True)
class Scan:
    sequence: int
    stamp: float
    scan_time: float
    range_min: float
    range_max: float
    ranges_mm: tuple
    intensities: tuple

    def validate(self):
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        for value in (self.stamp, self.scan_time, self.range_min, self.range_max):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("scan metadata must contain finite numbers")
        if self.stamp <= 0 or not 0 < self.scan_time <= 1:
            raise ValueError("invalid acquisition timestamp or scan duration")
        if not 0 < self.range_min < self.range_max <= 65.535:
            raise ValueError("invalid range limits")
        count = len(self.ranges_mm)
        if not 90 <= count <= MAX_SAMPLES or len(self.intensities) != count:
            raise ValueError("invalid sample count")
        if any(type(v) is not int or not 0 <= v <= 65535 for v in self.ranges_mm):
            raise ValueError("ranges must be uint16 millimetres")
        if any(type(v) is not int or not 0 <= v <= 255 for v in self.intensities):
            raise ValueError("intensities must be uint8")

    def ranges_metres(self):
        return [
            v / 1000.0 if v and self.range_min <= v / 1000.0 <= self.range_max
            else math.nan
            for v in self.ranges_mm
        ]


def encode_scan(scan):
    scan.validate()
    count = len(scan.ranges_mm)
    header = json.dumps({
        "type": "scan",
        "version": 1,
        "encoding": "uint16_le_mm_uint8_intensity",
        "sequence": scan.sequence,
        "stamp": scan.stamp,
        "scan_time": scan.scan_time,
        "range_min": scan.range_min,
        "range_max": scan.range_max,
        "count": count,
    }, separators=(",", ":"), allow_nan=False).encode("ascii")
    return (
        struct.pack(">I", len(header)) + header
        + struct.pack(f"<{count}H", *scan.ranges_mm) + bytes(scan.intensities)
    )


def recv_exact(client, size):
    result = bytearray()
    while len(result) < size:
        chunk = client.recv(size - len(result))
        if not chunk:
            raise ConnectionError("lidar stream closed during packet")
        result.extend(chunk)
    return bytes(result)


def read_scan(client):
    size = struct.unpack(">I", recv_exact(client, 4))[0]
    if not 0 < size <= MAX_HEADER:
        raise ValueError("invalid lidar header length")
    try:
        header = json.loads(recv_exact(client, size))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid lidar JSON header") from exc
    if not isinstance(header, dict):
        raise ValueError("lidar header must be an object")
    if (
        header.get("type") != "scan"
        or type(header.get("version")) is not int
        or header["version"] != 1
        or header.get("encoding") != "uint16_le_mm_uint8_intensity"
    ):
        raise ValueError("unsupported lidar protocol")
    count = header.get("count")
    if type(count) is not int or not 90 <= count <= MAX_SAMPLES:
        raise ValueError("invalid lidar sample count")
    required = ("sequence", "stamp", "scan_time", "range_min", "range_max")
    if any(key not in header for key in required):
        raise ValueError("missing lidar metadata")
    # Validate metadata before accepting a payload.
    metadata = [header[key] for key in required]
    Scan(*metadata, (0,) * count, (0,) * count).validate()
    ranges = struct.unpack(f"<{count}H", recv_exact(client, count * 2))
    intensities = tuple(recv_exact(client, count))
    return Scan(*metadata, ranges, intensities)
