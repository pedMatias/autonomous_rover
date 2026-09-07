import unittest

from recorder.odometry import OdometryIntegrator


class RecorderOdometryTests(unittest.TestCase):
    def make_integrator(self, **kwargs):
        kwargs.setdefault("max_distance_delta_m", 10.0)
        kwargs.setdefault("max_dt_s", 10.0)
        kwargs.setdefault("max_linear_velocity_mps", 10.0)
        kwargs.setdefault("max_angular_velocity_rps", 10.0)
        return OdometryIntegrator(
            left_distance_scale=0.01,
            right_distance_scale=0.01,
            track_width_m=0.206,
            **kwargs,
        )

    def test_stationary_nonzero_encoders_do_not_drift(self):
        odometry = self.make_integrator()
        odometry.update(0, 0, 0.0)
        moving_pose = odometry.update(317, 336, 1.0)[:3]

        stationary = odometry.update(317, 336, 1.05)

        self.assertEqual(stationary[:3], moving_pose)
        self.assertEqual(stationary[3:], (0.0, 0.0))

    def test_timestamp_gap_discards_interval(self):
        odometry = self.make_integrator(max_dt_s=0.5)
        odometry.update(0, 0, 0.0)
        odometry.update(10, 10, 0.1)

        state = odometry.update(20, 20, 1.0)

        self.assertAlmostEqual(state[0], 0.1)
        self.assertEqual(state[3:], (0.0, 0.0))
        self.assertTrue(odometry.last_update_reset)


if __name__ == "__main__":
    unittest.main()