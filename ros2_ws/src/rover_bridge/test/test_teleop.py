"""Unit tests for the keyboard teleop resolver and utilities."""
from __future__ import annotations

import math
import unittest

from rover_bridge.teleop import (
    FORWARD_KEY,
    REVERSE_KEY,
    LEFT_KEY,
    RIGHT_KEY,
    NO_STOP_REASON,
    WheelCommand,
    active_keys,
    resolve_wheel_command,
    wheels_to_twist,
    validate_motion_parameters,
)


class ActiveKeysTests(unittest.TestCase):
    def test_empty_input_returns_empty(self):
        self.assertEqual(active_keys({}, 0.0, 0.5), frozenset())

    def test_expired_key_is_excluded(self):
        now = 10.0
        last = {FORWARD_KEY: 9.0}
        self.assertEqual(active_keys(last, now, 0.5), frozenset())

    def test_recent_key_is_included(self):
        now = 10.0
        last = {FORWARD_KEY: 9.6}
        self.assertEqual(active_keys(last, now, 0.5), frozenset({FORWARD_KEY}))

    def test_only_drive_keys_are_considered(self):
        last = {"x": 10.0}
        self.assertEqual(active_keys(last, 10.0, 1.0), frozenset())


class WheelsToTwistTests(unittest.TestCase):
    def test_straight_forward(self):
        vx, wz = wheels_to_twist(0.39, 0.39, 0.172)
        self.assertAlmostEqual(vx, 0.39)
        self.assertAlmostEqual(wz, 0.0)

    def test_diagonal_left_arc(self):
        left = 0.153846 * 0.39
        right = 0.39
        vx, wz = wheels_to_twist(left, right, 0.172)
        self.assertGreater(vx, 0.0)
        self.assertGreater(wz, 0.0)

    def test_in_place_spin_is_refused_later(self):
        left, right = -0.39, 0.39
        vx, wz = wheels_to_twist(left, right, 0.172)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(wz, (right - left) / 0.172)


class ResolveWheelCommandTests(unittest.TestCase):
    def test_no_keys_is_stop(self):
        cmd = resolve_wheel_command(frozenset())
        self.assertEqual(cmd.left, 0.0)
        self.assertEqual(cmd.right, 0.0)
        self.assertEqual(cmd.vx, 0.0)
        self.assertEqual(cmd.wz, 0.0)
        self.assertEqual(cmd.reason, NO_STOP_REASON)

    def test_forward_is_straight(self):
        cmd = resolve_wheel_command(frozenset({FORWARD_KEY}))
        self.assertGreater(cmd.left, 0.0)
        self.assertAlmostEqual(cmd.left, cmd.right)
        self.assertGreater(cmd.vx, 0.0)
        self.assertAlmostEqual(cmd.wz, 0.0)

    def test_reverse_is_negative_straight(self):
        cmd = resolve_wheel_command(frozenset({REVERSE_KEY}))
        self.assertLess(cmd.left, 0.0)
        self.assertAlmostEqual(cmd.left, cmd.right)
        self.assertLess(cmd.vx, 0.0)
        self.assertAlmostEqual(cmd.wz, 0.0)

    def test_left_creep_forward_diagonal(self):
        cmd = resolve_wheel_command(frozenset({LEFT_KEY}))
        self.assertGreater(cmd.left, 0.0)
        self.assertGreater(cmd.right, 0.0)
        self.assertGreater(cmd.vx, 0.0)
        self.assertGreater(cmd.wz, 0.0)
        self.assertLess(abs(cmd.left), abs(cmd.right))

    def test_right_creep_forward_diagonal(self):
        cmd = resolve_wheel_command(frozenset({RIGHT_KEY}))
        self.assertGreater(cmd.left, 0.0)
        self.assertGreater(cmd.right, 0.0)
        self.assertGreater(cmd.vx, 0.0)
        self.assertLess(cmd.wz, 0.0)
        self.assertLess(abs(cmd.right), abs(cmd.left))

    def test_forward_plus_left_is_diagonal(self):
        cmd = resolve_wheel_command(frozenset({FORWARD_KEY, LEFT_KEY}))
        self.assertGreater(cmd.left, 0.0)
        self.assertGreater(cmd.right, 0.0)
        self.assertGreater(cmd.vx, 0.0)
        self.assertGreater(cmd.wz, 0.0)
        self.assertLess(abs(cmd.left), abs(cmd.right))
        self.assertEqual(cmd.reason, "diagonal")

    def test_forward_plus_right_is_diagonal(self):
        cmd = resolve_wheel_command(frozenset({FORWARD_KEY, RIGHT_KEY}))
        self.assertGreater(cmd.left, 0.0)
        self.assertGreater(cmd.right, 0.0)
        self.assertGreater(cmd.vx, 0.0)
        self.assertLess(cmd.wz, 0.0)
        self.assertEqual(cmd.reason, "diagonal")

    def test_reverse_plus_left_is_reverse_diagonal(self):
        cmd = resolve_wheel_command(frozenset({REVERSE_KEY, LEFT_KEY}))
        self.assertLess(cmd.left, 0.0)
        self.assertLess(cmd.right, 0.0)
        self.assertLess(cmd.vx, 0.0)
        self.assertGreater(cmd.wz, 0.0)
        self.assertEqual(cmd.reason, "diagonal")

    def test_conflicting_direction_keys_stop(self):
        cmd = resolve_wheel_command(frozenset({FORWARD_KEY, REVERSE_KEY}))
        self.assertEqual(cmd.left, 0.0)
        self.assertEqual(cmd.right, 0.0)

    def test_conflicting_turn_keys_stop(self):
        cmd = resolve_wheel_command(frozenset({LEFT_KEY, RIGHT_KEY}))
        self.assertEqual(cmd.left, 0.0)
        self.assertEqual(cmd.right, 0.0)

    def test_peak_speed_is_clamped(self):
        cmd = resolve_wheel_command(
            frozenset({FORWARD_KEY}), drive_speed_mps=10.0, max_wheel_speed_mps=0.5
        )
        self.assertLessEqual(abs(cmd.left), 0.5 + 1e-6)
        self.assertLessEqual(abs(cmd.right), 0.5 + 1e-6)

    def test_never_produces_opposite_sign_wheels(self):
        samples = (
            frozenset({FORWARD_KEY}),
            frozenset({REVERSE_KEY}),
            frozenset({LEFT_KEY}),
            frozenset({RIGHT_KEY}),
            frozenset({FORWARD_KEY, LEFT_KEY}),
            frozenset({FORWARD_KEY, RIGHT_KEY}),
            frozenset({REVERSE_KEY, LEFT_KEY}),
            frozenset({REVERSE_KEY, RIGHT_KEY}),
        )
        for keys in samples:
            with self.subTest(keys=keys):
                cmd = resolve_wheel_command(keys)
                self.assertGreaterEqual(cmd.left * cmd.right, -1e-9)


class ValidationTests(unittest.TestCase):
    def test_valid_parameters_pass(self):
        validate_motion_parameters(0.39, 0.20, 0.15, 0.7, 20.0, 0.172, 0.5)

    def test_non_positive_drive_speed_raises(self):
        with self.assertRaises(ValueError):
            validate_motion_parameters(0.0, 0.2, 0.15, 0.7, 20.0, 0.172, 0.5)

    def test_turn_ratio_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            validate_motion_parameters(0.39, 0.20, 0.0, 0.7, 20.0, 0.172, 0.5)
        with self.assertRaises(ValueError):
            validate_motion_parameters(0.39, 0.20, 1.0, 0.7, 20.0, 0.172, 0.5)


if __name__ == "__main__":
    unittest.main()

