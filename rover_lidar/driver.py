"""LD06/LD19-style 47-byte decoder; live D500/STL-19P CP2102 stream validated."""

import logging
import math
import struct

from .protocol import Scan


LOG = logging.getLogger(__name__)


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D if crc & 0x80 else crc << 1) & 0xFF
    return crc


class PacketDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_packets = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\x54\x2c":
                del self.buffer[0]
                continue
            if len(self.buffer) < 47:
                break
            packet = bytes(self.buffer[:47])
            if crc8(packet[:-1]) != packet[-1]:
                self.bad_packets += 1
                if self.bad_packets == 1 or self.bad_packets % 100 == 0:
                    LOG.warning("Rejected lidar CRC failures: %s", self.bad_packets)
                del self.buffer[0]
                continue
            del self.buffer[:47]
            speed, start = struct.unpack_from("<HH", packet, 2)
            end = struct.unpack_from("<H", packet, 42)[0]
            span = ((end - start) % 36000) / 100.0
            if not 0 < speed <= 10000 or start >= 36000 or end >= 36000 or span > 30:
                LOG.warning("Rejected invalid lidar speed/angle fields")
                continue
            samples = []
            for index in range(12):
                distance, confidence = struct.unpack_from("<HB", packet, 6 + 3 * index)
                angle = (start / 100.0 + span * index / 11.0) % 360.0
                samples.append((angle, distance, confidence))
            packets.append((speed, samples))
        return packets


class ScanAssembler:
    def __init__(self, bins=360, range_min=0.05, range_max=12.0,
                 clockwise=True, min_confidence=1, min_coverage=0.75):
        Scan(0, 1.0, 0.1, range_min, range_max, (0,) * bins, (0,) * bins).validate()
        if not 0 <= min_confidence <= 255 or not 0 < min_coverage <= 1:
            raise ValueError("invalid confidence or coverage threshold")
        self.bins = bins
        self.range_min = range_min
        self.range_max = range_max
        self.clockwise = clockwise
        self.min_confidence = min_confidence
        self.min_coverage = min_coverage
        self.sequence = 0
        self.previous_angle = None
        self.started = None
        self._reset()

    def _reset(self):
        self.ranges = [0] * self.bins
        self.intensities = [0] * self.bins
        self.observed = set()

    def add(self, speed, samples, received_at):
        scans = []
        # Estimate sample acquisition time from packet reception and motor speed.
        span = (samples[-1][0] - samples[0][0]) % 360
        for index, (angle, distance, confidence) in enumerate(samples):
            stamp = received_at - span * (11 - index) / (11 * speed)
            if self.previous_angle is not None and self.previous_angle - angle > 180:
                if self.started is not None:
                    duration = stamp - self.started
                    coverage = len(self.observed) / self.bins
                    if 0.02 <= duration <= 0.5 and coverage >= self.min_coverage:
                        scans.append(Scan(
                            self.sequence, self.started, duration,
                            self.range_min, self.range_max,
                            tuple(self.ranges), tuple(self.intensities),
                        ))
                        self.sequence += 1
                    else:
                        LOG.warning("Dropped incomplete/late scan: coverage=%.2f duration=%.3f",
                                    coverage, duration)
                self.started = stamp
                self._reset()
            self.previous_angle = angle
            if self.started is None:
                continue
            ros_angle = math.radians(-angle if self.clockwise else angle)
            slot = round(((ros_angle + math.pi) % (2 * math.pi))
                         * self.bins / (2 * math.pi)) % self.bins
            self.observed.add(slot)
            if confidence < self.min_confidence or not distance:
                continue
            if not self.ranges[slot] or distance < self.ranges[slot]:
                self.ranges[slot] = distance
                self.intensities[slot] = confidence
        return scans
