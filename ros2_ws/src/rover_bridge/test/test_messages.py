import unittest

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CompressedImage

from rover_bridge.node import build_color_compressed, build_depth_compressed


class CompressedMessageTests(unittest.TestCase):
    def setUp(self):
        self.stamp = Time(sec=1710000000, nanosec=250000000)
        self.frame_id = "oakd_rgb_optical_frame"

    def test_build_color_compressed(self):
        message = build_color_compressed(self.stamp, self.frame_id, b"jpeg")

        self.assertIsInstance(message, CompressedImage)
        self.assertEqual(message.header.stamp, self.stamp)
        self.assertEqual(message.header.frame_id, self.frame_id)
        self.assertEqual(message.format, "jpeg")
        self.assertEqual(bytes(message.data), b"jpeg")

    def test_build_depth_compressed(self):
        message = build_depth_compressed(self.stamp, self.frame_id, b"png")

        self.assertIsInstance(message, CompressedImage)
        self.assertEqual(message.header.stamp, self.stamp)
        self.assertEqual(message.header.frame_id, self.frame_id)
        self.assertEqual(message.format, "16UC1; compressedDepth png")
        self.assertEqual(bytes(message.data), b"png")


if __name__ == "__main__":
    unittest.main()