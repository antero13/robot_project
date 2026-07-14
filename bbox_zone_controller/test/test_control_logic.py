import math
import unittest

from bbox_zone_controller.control_logic import (
    Candidate,
    MotionSettings,
    Zone,
    ZoneGeometry,
    decide_motion,
    parse_class_list,
    pickup_is_ready,
    select_largest_candidate,
)


class ControlLogicTest(unittest.TestCase):
    def setUp(self):
        self.geometry = ZoneGeometry()
        self.settings = MotionSettings()
        self.targets = parse_class_list("apple,6")

    def candidate(self, x, y, class_name="cube"):
        return Candidate("99", class_name, 0.9, x, y, 0.1)

    def test_given_points_create_expected_four_zones(self):
        y = 0.85
        left, right = self.geometry.boundaries_at(y)

        self.assertEqual(self.geometry.classify(left - 0.05, y), Zone.OUTER_LEFT)
        self.assertEqual(self.geometry.classify((left + 0.0) * 0.5, y), Zone.INNER_LEFT)
        self.assertEqual(self.geometry.classify(right * 0.5, y), Zone.INNER_RIGHT)
        self.assertEqual(self.geometry.classify(right + 0.05, y), Zone.OUTER_RIGHT)

    def test_selects_largest_bbox_not_highest_confidence(self):
        selected = select_largest_candidate(
            [
                {
                    "class_id": 1,
                    "class_name": "small",
                    "confidence": 0.99,
                    "bbox_xyxy": {"x1": 0, "y1": 0, "x2": 10, "y2": 10},
                },
                {
                    "class_id": 2,
                    "class_name": "large",
                    "confidence": 0.50,
                    "bbox_xyxy": {"x1": 20, "y1": 20, "x2": 80, "y2": 80},
                },
            ],
            100,
            100,
            0.25,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected.class_name, "large")
        self.assertAlmostEqual(selected.area_ratio, 0.36)
        self.assertAlmostEqual(selected.bottom_y, 0.80)

    def test_outer_avoid_objects_drive_straight(self):
        for x in (-0.95, 0.95):
            with self.subTest(x=x):
                decision = decide_motion(
                    self.candidate(x, 0.9), self.targets, self.geometry, self.settings
                )
                self.assertEqual(decision.mode, "avoid_outer_forward")
                self.assertGreater(decision.linear_x, 0.0)
                self.assertEqual(decision.angular_z, 0.0)

    def test_inner_left_turns_right_and_inner_right_turns_left(self):
        inner_left = decide_motion(
            self.candidate(-0.2, 0.85), self.targets, self.geometry, self.settings
        )
        inner_right = decide_motion(
            self.candidate(0.2, 0.85), self.targets, self.geometry, self.settings
        )

        self.assertEqual(inner_left.zone, Zone.INNER_LEFT)
        self.assertLess(inner_left.angular_z, 0.0)
        self.assertEqual(inner_right.zone, Zone.INNER_RIGHT)
        self.assertGreater(inner_right.angular_z, 0.0)

    def test_target_rotates_toward_center_then_drives_forward(self):
        target_right = decide_motion(
            self.candidate(0.4, 0.8, "APPLE"),
            self.targets,
            self.geometry,
            self.settings,
        )
        centered = decide_motion(
            self.candidate(0.05, 0.8, "apple"),
            self.targets,
            self.geometry,
            self.settings,
        )

        self.assertTrue(target_right.is_target)
        self.assertEqual(target_right.linear_x, 0.0)
        self.assertLess(target_right.angular_z, 0.0)
        self.assertEqual(centered.mode, "target_centered_forward")
        self.assertGreater(centered.linear_x, 0.0)
        self.assertEqual(centered.angular_z, 0.0)

    def test_target_can_match_numeric_class_id(self):
        candidate = Candidate("6", "fruit_cube", 0.9, -0.3, 0.8, 0.1)

        decision = decide_motion(
            candidate,
            self.targets,
            self.geometry,
            self.settings,
        )

        self.assertTrue(decision.is_target)
        self.assertEqual(decision.mode, "target_align")

    def test_target_turn_command_is_clamped(self):
        decision = decide_motion(
            self.candidate(-1.0, 0.8, "6"),
            self.targets,
            self.geometry,
            self.settings,
        )

        self.assertTrue(math.isclose(decision.angular_z, self.settings.target_max_angular_z))

    def test_no_object_in_fresh_frame_drives_forward(self):
        decision = decide_motion(None, self.targets, self.geometry, self.settings)

        self.assertEqual(decision.mode, "no_object_forward")
        self.assertGreater(decision.linear_x, 0.0)

    def test_pickup_uses_previous_center_and_bottom_thresholds(self):
        ready = Candidate(
            "6",
            "fruit_cube",
            0.9,
            0.18,
            0.55,
            0.1,
            bottom_y=0.70,
        )

        self.assertTrue(pickup_is_ready(ready, self.targets, 0.18, 0.70))
        self.assertFalse(
            pickup_is_ready(
                Candidate("6", "fruit_cube", 0.9, 0.19, 0.55, 0.1, 0.80),
                self.targets,
                0.18,
                0.70,
            )
        )
        self.assertFalse(
            pickup_is_ready(
                Candidate("6", "fruit_cube", 0.9, 0.0, 0.55, 0.1, 0.69),
                self.targets,
                0.18,
                0.70,
            )
        )

    def test_non_target_never_triggers_pickup(self):
        avoid = Candidate("99", "cube", 0.9, 0.0, 0.55, 0.1, 0.90)

        self.assertFalse(pickup_is_ready(avoid, self.targets, 0.18, 0.70))


if __name__ == "__main__":
    unittest.main()
