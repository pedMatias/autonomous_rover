



import math
import unittest

from rover_bridge.odometry import OdometryIntegrator


class OdometryIntegratorTests(unittest.TestCase):
    def make_integrator(self, **kwargs):
        return OdometryIntegrator(
            meters_per_tick_left=1.0,
            meters_per_tick_right=1.0,
            track_width_m=2.0,
            **kwargs,
        )

    def test_first_sample_establishes_baseline(self):
        odometry = self.make_integrator()

        state = odometry.update(100, 200, 10.0)

        self.assertEqual((state.x, state.y, state.theta), (0.0, 0.0, 0.0))
        self.assertFalse(odometry.last_update_reset)

    def test_equal_wheel_motion_moves_forward(self):
        odometry = self.make_integrator()
        odometry.update(0, 0, 0.0)

        state = odometry.update(2, 2, 1.0)

        self.assertAlmostEqual(state.x, 2.0)
        self.assertAlmostEqual(state.y, 0.0)
        self.assertAlmostEqual(state.theta, 0.0)
        self.assertAlmostEqual(state.linear_velocity, 2.0)
        self.assertAlmostEqual(state.angular_velocity, 0.0)

    def test_reverse_encoder_signs_are_supported(self):
        odometry = self.make_integrator(left_encoder_sign=-1, right_encoder_sign=-1)
        odometry.update(0, 0, 0.0)

        state = odometry.update(1, 1, 1.0)

        self.assertAlmostEqual(state.x, -1.0)
        self.assertAlmostEqual(state.theta, 0.0)

    def test_large_jump_resets_baseline_without_pose_spike(self):
        odometry = self.make_integrator(max_encoder_delta_ticks=10.0)
        odometry.update(0, 0, 0.0)
        odometry.update(2, 2, 1.0)

        state = odometry.update(1000, 1000, 2.0)

        self.assertAlmostEqual(state.x, 2.0)
        self.assertAlmostEqual(state.linear_velocity, 0.0)
        self.assertTrue(odometry.last_update_reset)

    def test_timestamp_reset_resets_baseline(self):
        odometry = self.make_integrator()
        odometry.update(0, 0, 2.0)
        odometry.update(1, 1, 3.0)

        state = odometry.update(2, 2, 1.0)

        self.assertAlmostEqual(state.x, 1.0)
        self.assertTrue(odometry.last_update_reset)
        self.assertTrue(math.isfinite(state.theta))


if __name__ == "__main__":
    unittest.main()