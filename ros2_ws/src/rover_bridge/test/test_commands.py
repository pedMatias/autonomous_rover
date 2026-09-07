import unittest

from rover_bridge.node import fresh_twist, telemetry_sequence_gap


class CommandSafetyTests(unittest.TestCase):
    def test_missing_command_is_zero(self):
        self.assertEqual(fresh_twist((0.2, 0.1), None, 1.0, 0.5), (0.0, 0.0))

    def test_stale_command_is_zero(self):
        self.assertEqual(fresh_twist((0.2, 0.1), 1.0, 1.6, 0.5), (0.0, 0.0))

    def test_fresh_command_is_forwarded(self):
        self.assertEqual(fresh_twist((0.2, 0.1), 1.0, 1.4, 0.5), (0.2, 0.1))

    def test_telemetry_sequence_starts_without_a_gap(self):
        self.assertFalse(telemetry_sequence_gap(None, 10))

    def test_telemetry_sequence_requires_contiguous_frames(self):
        self.assertFalse(telemetry_sequence_gap(10, 11))
        self.assertTrue(telemetry_sequence_gap(10, 12))
        self.assertTrue(telemetry_sequence_gap(10, 9))

    def test_invalid_telemetry_sequence_is_rejected(self):
        self.assertTrue(telemetry_sequence_gap(10, "11"))
        self.assertTrue(telemetry_sequence_gap(10, None))


if __name__ == "__main__":
    unittest.main()