import unittest

from rl_model_policy.target_alignment_pd import TargetAlignmentPD


class TargetAlignmentPDTest(unittest.TestCase):
    def make_controller(self):
        return TargetAlignmentPD(
            proportional_gain=0.8,
            derivative_gain=0.12,
            derivative_limit=0.25,
            center_deadband=0.06,
            max_angular_z=0.45,
        )

    def test_positive_image_error_turns_right(self):
        command = self.make_controller().command(0.30, 1.0, 1.0)

        self.assertAlmostEqual(command.angular_z, -0.24)

    def test_negative_image_error_turns_left(self):
        command = self.make_controller().command(-0.30, 1.0, 1.0)

        self.assertAlmostEqual(command.angular_z, 0.24)

    def test_center_deadband_stops_rotation(self):
        command = self.make_controller().command(0.05, 1.0, 1.0)

        self.assertEqual(command.angular_z, 0.0)

    def test_derivative_reduces_turn_as_error_converges(self):
        controller = self.make_controller()
        controller.command(0.30, 1.0, 1.0)

        command = controller.command(0.20, 1.1, 1.0)

        self.assertAlmostEqual(command.derivative, -0.25)
        self.assertAlmostEqual(command.angular_z, -0.13)

    def test_output_is_limited(self):
        command = self.make_controller().command(1.0, 1.0, 1.0)

        self.assertAlmostEqual(command.angular_z, -0.45)

    def test_reset_discards_derivative_history(self):
        controller = self.make_controller()
        controller.command(0.30, 1.0, 1.0)
        controller.reset()

        command = controller.command(0.20, 1.1, 1.0)

        self.assertEqual(command.derivative, 0.0)
        self.assertAlmostEqual(command.angular_z, -0.16)


if __name__ == "__main__":
    unittest.main()
