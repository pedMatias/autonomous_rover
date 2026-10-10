"""Independent Pi lidar service. No camera, motor UART, or ROS dependencies."""

import argparse
import logging
import signal
import socket
import threading
import time

import serial

from .driver import PacketDecoder, ScanAssembler
from .protocol import encode_scan


LOG = logging.getLogger(__name__)


class LatestScan:
    def __init__(self):
        self.condition = threading.Condition()
        self.scan = None
        self.generation = 0

    def put(self, scan):
        with self.condition:
            self.scan = scan
            self.generation += 1
            self.condition.notify_all()

    def clear(self):
        with self.condition:
            self.scan = None
            self.condition.notify_all()

    def wait(self, generation, timeout):
        with self.condition:
            self.condition.wait_for(
                lambda: self.generation != generation and self.scan is not None,
                timeout,
            )
            return self.generation, self.scan


def acquire(args, latest, stop):
    while not stop.is_set():
        try:
            with serial.Serial(args.device, args.baud, timeout=0.1, exclusive=True) as port:
                port.reset_input_buffer()
                decoder = PacketDecoder()
                assembler = ScanAssembler(
                    args.bins, args.range_min, args.range_max,
                    args.direction == "clockwise", args.min_confidence,
                )
                LOG.info("Lidar port opened: %s (%s baud)", args.device, args.baud)
                last_packet = time.monotonic()
                while not stop.is_set():
                    pending = port.in_waiting
                    if pending > 4096:
                        raise TimeoutError("lidar serial backlog; discarding stale samples")
                    data = port.read(max(1, pending))
                    received_at = time.time()
                    packets = decoder.feed(data)
                    if packets:
                        last_packet = time.monotonic()
                    if time.monotonic() - last_packet > 2.0:
                        raise TimeoutError("no valid lidar packets for 2 seconds")
                    durations = [
                        ((samples[-1][0] - samples[0][0]) % 360) * 12 / (11 * speed)
                        for speed, samples in packets
                    ]
                    remaining = sum(durations)
                    for (speed, samples), duration in zip(packets, durations):
                        remaining -= duration
                        stamp = received_at - remaining
                        for scan in assembler.add(speed, samples, stamp):
                            latest.put(scan)
        except (serial.SerialException, OSError, TimeoutError) as exc:
            latest.clear()
            LOG.error("Lidar acquisition unavailable: %s", exc)
            stop.wait(1.0)


def serve(args, stop=None):
    stop = stop or threading.Event()
    latest = LatestScan()
    reader = threading.Thread(target=acquire, args=(args, latest, stop), daemon=True)
    reader.start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.bind_host, args.port))
            server.listen(1)
            server.settimeout(0.2)
            LOG.info("Lidar stream listening on %s:%s", args.bind_host, args.port)
            while not stop.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                LOG.info("Lidar client connected: %s", address)
                with client:
                    client.settimeout(0.2)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    generation = -1
                    try:
                        while not stop.is_set():
                            new_generation, scan = latest.wait(generation, 0.2)
                            if scan is None or new_generation == generation:
                                continue
                            generation = new_generation
                            if not 0 <= time.time() - scan.stamp <= args.max_age:
                                LOG.warning("Discarding stale/future scan before transmission")
                                continue
                            client.sendall(encode_scan(scan))
                    except (OSError, ConnectionError) as exc:
                        LOG.info("Lidar client disconnected: %s", exc)
    finally:
        stop.set()
        reader.join(timeout=3)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", required=True, choices=["ld06"])
    parser.add_argument("--device", required=True, help="Stable /dev/serial/by-id path")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8771)
    parser.add_argument("--bins", type=int, default=360)
    parser.add_argument("--range-min", type=float, default=0.05)
    parser.add_argument("--range-max", type=float, default=12.0)
    parser.add_argument("--direction", choices=["clockwise", "counterclockwise"],
                        default="clockwise")
    parser.add_argument("--min-confidence", type=int, default=1)
    parser.add_argument("--max-age", type=float, default=0.5)
    args = parser.parse_args(argv)
    if not 0 < args.max_age <= 1 or not 1 <= args.port <= 65535 or args.baud <= 0:
        parser.error("invalid age, port, or baud")
    ScanAssembler(args.bins, args.range_min, args.range_max,
                  min_confidence=args.min_confidence)
    return args


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    serve(args, stop)
