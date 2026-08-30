import json
import socket
import struct
import unittest

from rover_bridge.protocol import encode_cmd, read_rgbd_packet


class ProtocolTests(unittest.TestCase):
    def test_encode_cmd(self):
        encoded = encode_cmd(12, 0.2, -0.4)
        self.assertEqual(encoded[-1:], b"\n")
        self.assertEqual(
            json.loads(encoded),
            {"type": "cmd_vel", "seq": 12, "vx": 0.2, "wz": -0.4},
        )

    def test_read_rgbd_packet(self):
        receiver, sender = socket.socketpair()
        try:
            header = {
                "type": "rgbd",
                "stamp": 1710000000.25,
                "seq": 42,
                "ts": 1710000000.25,
                "rgb_bytes": 3,
                "depth_bytes": 2,
                "camera_info": {
                    "width": 640,
                    "height": 480,
                    "distortion_model": "rational_polynomial",
                    "d": [],
                    "k": [1.0] * 9,
                },
            }
            encoded_header = json.dumps(header).encode("utf-8")
            sender.sendall(
                struct.pack(">I", len(encoded_header))
                + encoded_header
                + b"rgb"
                + b"\x01\x02"
            )
            result = read_rgbd_packet(receiver)
            self.assertEqual(result, (header, b"rgb", b"\x01\x02"))
            self.assertEqual(result[0]["stamp"], 1710000000.25)
            self.assertEqual(result[0]["seq"], 42)
        finally:
            receiver.close()
            sender.close()


if __name__ == "__main__":
    unittest.main()
