import unittest

from rl_model_policy.pickup_trigger import pickup_is_ready


class PickupTriggerTest(unittest.TestCase):
    def test_close_centered_target_starts_pickup(self):
        self.assertTrue(pickup_is_ready(0.0, 0.70, 0.18, 0.70))

    def test_rl_keeps_control_until_target_is_close(self):
        self.assertFalse(pickup_is_ready(0.0, 0.69, 0.18, 0.70))

    def test_close_target_must_still_be_centered(self):
        self.assertFalse(pickup_is_ready(0.19, 0.80, 0.18, 0.70))

    def test_threshold_boundaries_are_inclusive(self):
        self.assertTrue(pickup_is_ready(-0.18, 0.70, 0.18, 0.70))

    def test_invalid_ratio_is_rejected(self):
        with self.assertRaises(ValueError):
            pickup_is_ready(0.0, 0.7, 0.18, 1.1)


if __name__ == "__main__":
    unittest.main()
