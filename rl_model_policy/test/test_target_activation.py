import unittest

from rl_model_policy.target_activation import (
    coverage_phase_allows_target_search,
    storage_repickup_guard_is_active,
    target_is_close_enough,
    target_is_eligible,
)


class TargetActivationTest(unittest.TestCase):
    def test_target_search_is_blocked_during_main_road_lane_shift(self):
        self.assertFalse(coverage_phase_allows_target_search("SHIFT_TO_NEXT_LANE"))
        self.assertFalse(coverage_phase_allows_target_search("ENTER_FIRST_LANE"))

    def test_target_search_starts_when_lane_scan_starts(self):
        self.assertTrue(coverage_phase_allows_target_search("SCAN_LANE_UP"))
        self.assertTrue(coverage_phase_allows_target_search("SCAN_LANE_DOWN"))

    def test_target_search_stays_blocked_during_main_road_alignment(self):
        self.assertFalse(
            coverage_phase_allows_target_search(
                "SCAN_LANE_DOWN",
                main_road_alignment_active=True,
            )
        )

    def test_rejects_missing_or_distant_bbox_center(self):
        self.assertFalse(target_is_close_enough(None, 0.30))
        self.assertFalse(target_is_close_enough(0.299, 0.30))

    def test_accepts_bbox_center_at_threshold(self):
        self.assertTrue(target_is_close_enough(0.30, 0.30))
        self.assertTrue(target_is_close_enough(0.50, 0.30))

    def test_rejects_invalid_threshold(self):
        with self.assertRaises(ValueError):
            target_is_close_enough(0.5, 1.1)

    def test_tracking_hysteresis_retains_target_below_entry_threshold(self):
        self.assertFalse(target_is_eligible(0.26, 0.30, 0.22, False))
        self.assertTrue(target_is_eligible(0.26, 0.30, 0.22, True))

    def test_tracking_threshold_must_not_exceed_entry_threshold(self):
        with self.assertRaises(ValueError):
            target_is_eligible(0.5, 0.30, 0.31, True)

    def test_storage_guard_blocks_lane_four_descent_after_delivery(self):
        self.assertTrue(
            storage_repickup_guard_is_active(
                enabled=True,
                delivered_count=4,
                lane_number=4,
                coverage_phase="SCAN_LANE_DOWN",
                robot_y=-1.05,
                start_y=-0.95,
            )
        )

    def test_storage_guard_does_not_block_before_first_delivery(self):
        self.assertFalse(
            storage_repickup_guard_is_active(
                enabled=True,
                delivered_count=0,
                lane_number=4,
                coverage_phase="SCAN_LANE_DOWN",
                robot_y=-1.05,
                start_y=-0.95,
            )
        )

    def test_storage_guard_does_not_block_other_lanes_or_upper_lane_four(self):
        common = {
            "enabled": True,
            "delivered_count": 4,
            "coverage_phase": "SCAN_LANE_DOWN",
            "start_y": -0.95,
        }
        self.assertFalse(
            storage_repickup_guard_is_active(
                lane_number=3,
                robot_y=-1.05,
                **common,
            )
        )
        self.assertFalse(
            storage_repickup_guard_is_active(
                lane_number=4,
                robot_y=-0.90,
                **common,
            )
        )


if __name__ == "__main__":
    unittest.main()
