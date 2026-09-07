import json
import logging
import socket
import unittest

from rover_agent import RoverAgent, limit_wheel_speeds


class FakeBase:
    def __init__(self):
        self.commands = []

    def base_speed_ctrl(self, left, right):
        self.commands.append((left, right))


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