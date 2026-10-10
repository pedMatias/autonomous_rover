import argparse
from dataclasses import replace
import json
import logging
import math
import os
import pty
import socket
import struct
import threading
import time
from unittest.mock import patch

import pytest

from base_ctrl import ReadLine
from rover_lidar.driver import PacketDecoder, ScanAssembler, crc8
from rover_lidar.protocol import Scan, encode_scan, read_scan
from rover_lidar.server import LatestScan, parse_args, serve
from rover_agent import RoverAgent


def packet(start=350, end=1, speed=3600, distance=1000, confidence=100):
    data = bytearray(b"\x54\x2c")
    data.extend(struct.pack("<H", speed))
    data.extend(struct.pack("<H", round(start * 100)))
    for _ in range(12):
        data.extend(struct.pack("<HB", distance, confidence))
    data.extend(struct.pack("<HH", round(end * 100), 1000))
    data.append(crc8(data))
    return bytes(data)


def scan():
    return Scan(7, time.time() - 0.1, 0.1, 0.05, 12.0,
                (1000,) * 360, (100,) * 360)


def test_decoder_partial_packet_noise_and_wrap():
    decoder = PacketDecoder()
    data = packet()
    assert len(data) == 47
    assert decoder.feed(b"noise" + data[:18]) == []
    result = decoder.feed(data[18:])
    speed, samples = result[0]
    assert speed == 3600
    assert samples[0] == (350.0, 1000, 100)
    assert samples[-1] == (1.0, 1000, 100)


def test_decoder_corrupt_crc_resynchronizes(caplog):
    data = bytearray(packet())
    data[8] ^= 1
    decoder = PacketDecoder()
    assert len(decoder.feed(data + packet())) == 1
    assert decoder.bad_packets == 1
    assert "CRC" in caplog.text


@pytest.mark.parametrize("kwargs", [
    {"speed": 0}, {"start": 360}, {"start": 0, "end": 90},
])
def test_decoder_invalid_fields(kwargs, caplog):
    assert PacketDecoder().feed(packet(**kwargs)) == []
    assert "invalid lidar" in caplog.text


def assembled(clockwise=True, confidence=100):
    decoder = PacketDecoder()
    assembler = ScanAssembler(clockwise=clockwise)
    result = []
    for index in range(110):
        start = (index * 10) % 360
        data = packet(start=start, end=(start + 10) % 360, confidence=confidence)
        for speed, samples in decoder.feed(data):
            result.extend(assembler.add(speed, samples, 1000 + index / 360))
    return result


def test_assembler_discards_startup_partial_and_emits_uniform_revolution():
    result = assembled()
    assert len(result) == 2
    assert [value.sequence for value in result] == [0, 1]
    for value in result:
        assert value.scan_time == pytest.approx(0.1)
        assert len(value.ranges_mm) == 360
        assert sum(bool(v) for v in value.ranges_mm) >= 350
        value.validate()


def test_low_confidence_is_unknown_not_clear_space():
    result = assembled(confidence=0)
    assert all(math.isnan(value) for value in result[0].ranges_metres())


def test_assembler_direction_and_nearest_sample():
    clockwise = ScanAssembler(clockwise=True)
    counter = ScanAssembler(clockwise=False)
    samples = [(0, 1000, 100)] * 11 + [(90, 1500, 100)]
    for assembler in (clockwise, counter):
        assembler.started = 1000
        assembler.add(3600, samples, 1000.01)
    assert clockwise.ranges[90] == 1500
    assert counter.ranges[270] == 1500
    clockwise.add(3600, [(90, 1200, 100)] * 12, 1000.02)
    assert clockwise.ranges[90] == 1200


def test_incomplete_or_late_scan_is_dropped(caplog):
    assembler = ScanAssembler()
    for index, angle in enumerate((350, 0, 90, 180, 270, 350, 0)):
        assembler.add(3600, [(angle, 1000, 100)] * 12, 1000 + index)
    assert "Dropped incomplete/late scan" in caplog.text


class FragmentedSocket:
    def __init__(self, data):
        self.data = data

    def recv(self, count):
        count = min(count, 3)
        result, self.data = self.data[:count], self.data[count:]
        return result


def test_protocol_roundtrip_partial_reads_and_bandwidth():
    value = scan()
    encoded = encode_scan(value)
    assert read_scan(FragmentedSocket(encoded)) == value
    assert len(encoded) < 1500
    assert len(encoded) * 10 < 15000


@pytest.mark.parametrize("changes", [
    {"stamp": math.nan}, {"sequence": True}, {"scan_time": 0},
    {"range_min": 15}, {"range_max": math.inf}, {"ranges_mm": (1,)},
    {"ranges_mm": (65536,) * 360}, {"intensities": (256,) * 360},
])
def test_protocol_rejects_invalid_scan(changes):
    with pytest.raises(ValueError):
        encode_scan(replace(scan(), **changes))


@pytest.mark.parametrize("header", [
    {"type": "scan", "version": 2},
    {"type": "scan", "version": 1, "encoding": "uint16_le_mm_uint8_intensity",
     "count": 10**9},
    {"type": "scan", "version": 1, "encoding": "uint16_le_mm_uint8_intensity",
     "count": 360},
    [],
])
def test_protocol_rejects_header_before_payload(header):
    data = json.dumps(header).encode()
    with pytest.raises(ValueError):
        read_scan(FragmentedSocket(struct.pack(">I", len(data)) + data))


def test_protocol_rejects_unbounded_header_and_truncated_payload():
    with pytest.raises(ValueError):
        read_scan(FragmentedSocket(struct.pack(">I", 9999999)))
    with pytest.raises(ConnectionError):
        read_scan(FragmentedSocket(encode_scan(scan())[:-1]))


def test_invalid_distances_become_nan_not_infinity():
    value = replace(scan(), ranges_mm=(0, 1, 13000) + (1000,) * 357)
    assert all(math.isnan(v) for v in value.ranges_metres()[:3])


def test_latest_scan_replaces_backlog_and_clears_on_disconnect():
    latest = LatestScan()
    for index in range(100):
        latest.put(replace(scan(), sequence=index))
    generation, value = latest.wait(-1, 0)
    assert value.sequence == 99
    latest.clear()
    assert latest.wait(generation, 0)[1] is None


def test_cli_requires_explicit_protocol_and_device():
    with pytest.raises(SystemExit):
        parse_args([])
    args = parse_args(["--protocol", "ld06", "--device", "/dev/example"])
    assert args.port == 8771
    assert args.bins == 360


def test_legacy_lidar_port_is_not_opened_when_disabled():
    with patch("base_ctrl.glob.glob", return_value=["/dev/ttyUSB0"]), \
            patch("base_ctrl.serial.Serial") as serial_open:
        ReadLine(object(), use_lidar=False, extra_sensor=True)
    assert serial_open.call_count == 1
    assert serial_open.call_args.args[0] == "/dev/ttyUSB0"


def test_no_optional_serial_ports_opened_for_motor_agent():
    with patch("base_ctrl.serial.Serial") as serial_open:
        reader = ReadLine(object(), use_lidar=False, extra_sensor=False)
    serial_open.assert_not_called()
    assert reader.lidar_ser is None
    assert reader.sensor_data_ser is None


def test_motor_agent_explicitly_disables_optional_serial_readers():
    with patch("rover_agent.BaseController") as controller:
        RoverAgent({}, {}, logging.getLogger("ownership-test"))
    controller.assert_called_once_with(
        "/dev/ttyAMA0", 115200, use_lidar=False, extra_sensor=False
    )


def test_tcp_server_roundtrip_reconnect_and_shutdown():
    def fake_acquire(args, latest, stop):
        sequence = 0
        while not stop.is_set():
            latest.put(replace(scan(), sequence=sequence))
            sequence += 1
            stop.wait(0.03)

    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    args = argparse.Namespace(bind_host="127.0.0.1", port=port, max_age=0.5)
    stop = threading.Event()
    with patch("rover_lidar.server.acquire", side_effect=fake_acquire):
        thread = threading.Thread(target=serve, args=(args, stop))
        thread.start()
        try:
            deadline = time.monotonic() + 3
            while True:
                try:
                    client = socket.create_connection(("127.0.0.1", port), timeout=0.5)
                    break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.02)
            with client:
                value = read_scan(client)
                assert value.ranges_mm[0] == 1000
                second = read_scan(client)
                assert second.sequence > value.sequence
            with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
                assert read_scan(client).scan_time == 0.1
        finally:
            stop.set()
            thread.join(timeout=3)
        assert not thread.is_alive()


def test_real_serial_decoder_to_tcp_over_pseudoterminal():
    master, slave = pty.openpty()
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    args = parse_args([
        "--protocol", "ld06", "--device", os.ttyname(slave),
        "--bind-host", "127.0.0.1", "--port", str(port),
    ])
    stop = threading.Event()

    def sensor():
        index = 0
        while not stop.is_set():
            start = (index * 10) % 360
            os.write(master, packet(start=start, end=(start + 10) % 360))
            index += 1
            stop.wait(0.003)

    server = threading.Thread(target=serve, args=(args, stop))
    writer = threading.Thread(target=sensor)
    try:
        server.start()
        writer.start()
        deadline = time.monotonic() + 3
        while True:
            try:
                client = socket.create_connection(("127.0.0.1", port), timeout=2)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)
        with client:
            value = read_scan(client)
            assert value.ranges_mm.count(1000) >= 350
            assert 0.02 <= value.scan_time <= 0.5
            assert 0 <= time.time() - value.stamp < 0.5
            assert read_scan(client).sequence > value.sequence
    finally:
        stop.set()
        server.join(timeout=3)
        writer.join(timeout=3)
        os.close(master)
        os.close(slave)
    assert not server.is_alive()
    assert not writer.is_alive()
