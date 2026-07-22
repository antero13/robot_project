import unittest

from rl_model_policy.target_reacquisition import (
    reacquire_angular_velocity,
    reacquire_duration_for_evidence,
)


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

    def test_unconfirmed_candidate_never_reverses(self):
        angular_z = reacquire_angular_velocity(
            1.0,
            1.5,
            0.75,
            0.18,
            reverse_enabled=False,
        )
        self.assertAlmostEqual(angular_z, -0.18)

    def test_single_detection_gets_short_reacquisition(self):
        duration = reacquire_duration_for_evidence(1, False, 0.4, 0.7, 1.5)
        self.assertAlmostEqual(duration, 0.4)

    def test_two_detections_get_longer_one_way_reacquisition(self):
        duration = reacquire_duration_for_evidence(2, False, 0.4, 0.7, 1.5)
        self.assertAlmostEqual(duration, 0.7)

    def test_confirmed_target_keeps_full_reacquisition(self):
        duration = reacquire_duration_for_evidence(1, True, 0.4, 0.7, 1.5)
        self.assertAlmostEqual(duration, 1.5)

    def test_negative_elapsed_time_is_rejected(self):
        with self.assertRaises(ValueError):
            reacquire_angular_velocity(1.0, -0.1, 1.5, 0.18)


if __name__ == "__main__":
    unittest.main()
