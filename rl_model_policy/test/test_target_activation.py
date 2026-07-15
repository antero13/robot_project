import unittest

from rl_model_policy.target_activation import target_is_close_enough


class TargetActivationTest(unittest.TestCase):
    def test_rejects_missing_or_distant_bbox_center(self):
        self.assertFalse(target_is_close_enough(None, 0.30))
        self.assertFalse(target_is_close_enough(0.299, 0.30))

    def test_accepts_bbox_center_at_threshold(self):
        self.assertTrue(target_is_close_enough(0.30, 0.30))
        self.assertTrue(target_is_close_enough(0.50, 0.30))

    def test_rejects_invalid_threshold(self):
        with self.assertRaises(ValueError):
            target_is_close_enough(0.5, 1.1)


if __name__ == "__main__":
    unittest.main()
