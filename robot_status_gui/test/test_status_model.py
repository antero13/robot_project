import math
import unittest

from robot_status_gui.status_model import (
    centered_pose_to_map,
    mode_label,
    parse_json_message,
    quaternion_to_yaw,
    stored_object_label,
)


class StatusModelTest(unittest.TestCase):
    def test_centered_start_pose_maps_to_lower_right_zone(self):
        map_x, map_y = centered_pose_to_map(1.8, -1.8)
        self.assertAlmostEqual(map_x, 3.8)
        self.assertAlmostEqual(map_y, 0.2)

    def test_pause_overrides_mode_label(self):
        state = {"control_mode": "TRACK_TARGET", "motion_paused": True}
        self.assertEqual(mode_label(state), "주행 일시정지")

    def test_grab_state_is_described(self):
        state = {"control_mode": "GRAB_SEQUENCE", "grab_state": "CLOSING"}
        self.assertIn("수납구 닫기", mode_label(state))

    def test_stored_objects_are_counted(self):
        state = {"stored_objects": ["apple", "apple", "banana"]}
        self.assertEqual(stored_object_label(state), "apple × 2, banana")

    def test_invalid_json_uses_empty_dictionary(self):
        self.assertEqual(parse_json_message("not-json"), {})

    def test_quaternion_yaw(self):
        yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertAlmostEqual(yaw, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
