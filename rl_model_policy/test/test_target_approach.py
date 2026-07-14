import unittest

from rl_model_policy.target_approach import aligned_approach_command


class TargetApproachTest(unittest.TestCase):
    def test_centered_target_drives_forward(self):
        command = aligned_approach_command(
            target_x=0.0,
            target_y=0.30,
            center_tolerance=0.18,
            grab_area_ratio=0.50,
            linear_speed=0.06,
            angular_gain=0.8,
            max_angular_speed=0.12,
        )

        self.assertIsNotNone(command)
        self.assertAlmostEqual(command.linear_x, 0.06)
        self.assertAlmostEqual(command.angular_z, 0.0)

    def test_small_alignment_error_is_corrected_while_driving(self):
        left = aligned_approach_command(-0.10, 0.30, 0.18, 0.50, 0.06, 0.8, 0.12)
        right = aligned_approach_command(0.10, 0.30, 0.18, 0.50, 0.06, 0.8, 0.12)

        self.assertGreater(left.angular_z, 0.0)
        self.assertLess(right.angular_z, 0.0)

    def test_target_outside_center_band_stays_under_rl_control(self):
        command = aligned_approach_command(
            0.19, 0.30, 0.18, 0.50, 0.06, 0.8, 0.12
        )
        self.assertIsNone(command)

    def test_close_target_is_left_to_grab_sequence(self):
        command = aligned_approach_command(
            0.0, 0.50, 0.18, 0.50, 0.06, 0.8, 0.12
        )
        self.assertIsNone(command)

    def test_angular_correction_is_clamped(self):
        command = aligned_approach_command(
            -0.18, 0.30, 0.18, 0.50, 0.06, 4.0, 0.12
        )
        self.assertAlmostEqual(command.angular_z, 0.12)


if __name__ == "__main__":
    unittest.main()
