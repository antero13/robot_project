import unittest

from rl_model_policy.deterministic_pickup_controller import (
    DeterministicPickupController,
)


class DeterministicPickupControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = DeterministicPickupController()

    def test_rotates_in_place_until_target_is_centered(self):
        command = self.controller.command(now_s=1.0, target_x=0.4, target_y=0.3)

        self.assertEqual(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, -0.32)

    def test_centered_approach_slows_as_target_gets_closer(self):
        far = self.controller.command(now_s=1.0, target_x=0.0, target_y=0.2)
        near = self.controller.command(now_s=2.0, target_x=0.0, target_y=0.65)

        self.assertGreater(far.linear_x, near.linear_x)
        self.assertAlmostEqual(near.linear_x, 0.03)

    def test_avoidance_turns_toward_lower_cost_side(self):
        command = self.controller.command(
            now_s=1.0,
            target_x=0.0,
            target_y=0.4,
            avoid_required=True,
            avoid_bins=(0.8, 0.5, 0.1),
        )

        self.assertEqual(command.state, self.controller.AVOID_TURN)
        self.assertLess(command.angular_z, 0.0)

    def test_avoidance_runs_turn_then_forward_without_a_visible_target(self):
        self.controller.command(
            now_s=1.0,
            target_x=0.0,
            target_y=0.4,
            avoid_required=True,
            avoid_bins=(0.1, 0.8, 0.8),
        )

        turning = self.controller.command(now_s=1.3)
        forwarding = self.controller.command(now_s=1.6)
        finished = self.controller.command(now_s=2.5)

        self.assertEqual(turning.state, self.controller.AVOID_TURN)
        self.assertEqual(forwarding.state, self.controller.AVOID_FORWARD)
        self.assertGreater(forwarding.linear_x, 0.0)
        self.assertEqual(finished.state, self.controller.TRACK)
        self.assertEqual((finished.linear_x, finished.angular_z), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
