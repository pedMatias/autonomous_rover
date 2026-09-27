import json
import logging
import socket
import unittest
from unittest.mock import patch

from base_ctrl import BaseController, ReadLine
from rover_agent import RoverAgent, limit_wheel_speeds


class FakeBase:
    def __init__(self):
        self.commands = []

    def base_speed_ctrl(self, left, right):
        self.commands.append((left, right))


class SerialReaderTests(unittest.TestCase):
    def test_line_split_across_reads_is_preserved(self):
        class FakeSerial:
            in_waiting = 0

            def __init__(self):
                self.parts = iter((b'{"T":', b'1003}\n'))

            def read(self, size):
                return next(self.parts)

        reader = ReadLine.__new__(ReadLine)
        reader.s = FakeSerial()
        reader.buf = bytearray()
        self.assertEqual(reader.readline(), b'{"T":1003}\n')

    def test_partial_frame_timeout_returns_no_data(self):
        class FakeSerial:
            in_waiting = 0

            def __init__(self):
                self.reads = 0

            def read(self, size):
                self.reads += 1
                if self.reads > 2:
                    raise AssertionError("serial reader retried after timeout")
                return b'{"T":' if self.reads == 1 else b""

        reader = ReadLine.__new__(ReadLine)
        reader.s = FakeSerial()
        reader.buf = bytearray()
        self.assertEqual(reader.readline(), b"")
        self.assertEqual(reader.s.reads, 2)
        self.assertEqual(reader.buf, b"")

    def test_feedback_timeout_does_not_log_parse_error(self):
        class FakeSerial:
            in_waiting = 0

            def __init__(self):
                self.reads = 0

            def read(self, size):
                self.reads += 1
                if self.reads > 1:
                    raise AssertionError("serial reader retried after timeout")
                return b""

            def reset_input_buffer(self):
                pass

        controller = BaseController.__new__(BaseController)
        controller.rl = ReadLine.__new__(ReadLine)
        controller.rl.s = FakeSerial()
        controller.rl.buf = bytearray()
        with patch("builtins.print") as print_output:
            self.assertIsNone(controller.feedback_data())
        print_output.assert_not_called()


class RoverAgentProtocolTests(unittest.TestCase):
    def test_wheel_acceleration_is_limited(self):
        limited = limit_wheel_speeds((0.0, 0.0), (0.5, -0.5), 0.1, 1.0)

        self.assertEqual(limited, (0.1, -0.1))

    def test_telemetry_echoes_last_wheel_command(self):
        base = FakeBase()
        agent = RoverAgent(
            {
                "max_wheel_speed_mps": 0.5,
                "max_wheel_accel_mps2": 0.0,
                "wheel_separation_m": 0.172,
            },
            {},
            logging.getLogger("test_rover_agent"),
            base=base,
        )
        receiver, sender = socket.socketpair()
        try:
            with agent.telemetry_lock:
                agent.telemetry_client = sender
            agent._handle_command(
                {"type": "cmd_vel", "seq": 7, "vx": 0.2, "wz": 0.0}
            )
            agent._publish_telemetry(
                {
                    "ts": 1710000000.0,
                    "data": {"odl": 1, "odr": 1},
                    "voltage_v": 12.0,
                }
            )

            message = json.loads(receiver.recv(4096).decode("utf-8"))
            self.assertEqual(message["cmd_echo"]["seq"], 7)
            self.assertAlmostEqual(message["cmd_echo"]["left_mps"], 0.2)
            self.assertAlmostEqual(message["cmd_echo"]["right_mps"], 0.2)
            self.assertEqual(base.commands, [(0.2, 0.2)])
        finally:
            receiver.close()
            sender.close()


if __name__ == "__main__":
    unittest.main()