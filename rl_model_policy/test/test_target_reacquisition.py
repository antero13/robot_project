import unittest

from rl_model_policy.target_reacquisition import reacquire_angular_velocity


class TargetReacquisitionTest(unittest.TestCase):
    def test_first_phase_turns_toward_last_right_detection(self):
        angular_z = reacquire_angular_velocity(1.0, 0.5, 1.5, 0.18)
        self.assertAlmostEqual(angular_z, -0.18)

    def test_first_phase_turns_toward_last_left_detection(self):
        angular_z = reacquire_angular_velocity(-1.0, 0.5, 1.5, 0.18)
        self.assertAlmostEqual(angular_z, 0.18)

    def test_second_phase_sweeps_the_opposite_direction(self):
        angular_z = reacquire_angular_velocity(1.0, 1.5, 1.5, 0.18)
        self.assertAlmostEqual(angular_z, 0.18)

    def test_negative_elapsed_time_is_rejected(self):
        with self.assertRaises(ValueError):
            reacquire_angular_velocity(1.0, -0.1, 1.5, 0.18)


if __name__ == "__main__":
    unittest.main()
