import struct
import unittest

from camera_stream_protocol import build_compressed_depth_payload


class CameraStreamProtocolTests(unittest.TestCase):
    def test_build_compressed_depth_payload(self):
        payload = build_compressed_depth_payload(b"png")

        self.assertEqual(len(payload), 15)
        self.assertEqual(struct.unpack("<iff", payload[:12]), (0, 0.0, 0.0))
        self.assertEqual(payload[12:], b"png")


if __name__ == "__main__":
    unittest.main()
