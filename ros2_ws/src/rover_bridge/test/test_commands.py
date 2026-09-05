import unittest

from rover_bridge.node import fresh_twist


class CommandSafetyTests(unittest.TestCase):
    def test_missing_command_is_zero(self):
        self.assertEqual(fresh_twist((0.2, 0.1), None, 1.0, 0.5), (0.0, 0.0))

    def test_stale_command_is_zero(self):
        self.assertEqual(fresh_twist((0.2, 0.1), 1.0, 1.6, 0.5), (0.0, 0.0))

    def test_fresh_command_is_forwarded(self):
        self.assertEqual(fresh_twist((0.2, 0.1), 1.0, 1.4, 0.5), (0.2, 0.1))


if __name__ == "__main__":
    unittest.main()