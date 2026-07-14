import math
import unittest

from robot_status_gui.status_model import (
    centered_pose_to_map,
    mapper_diagnostics_label,
    mission_progress_label,
    mission_time_label,
    mode_label,
    parse_json_message,
    quaternion_to_yaw,
    return_reason_label,
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

    def test_storage_return_phase_overrides_low_level_mode(self):
        state = {
            "control_mode": "TRACK_TARGET",
            "mission": {"phase": "RETURN_STAGING"},
        }
        self.assertIn("보관함 복귀", mode_label(state))

    def test_mission_progress_and_timer_are_formatted(self):
        state = {
            "mission": {
                "onboard_count": 3,
                "storage_capacity": 4,
                "delivered_count": 4,
                "total_collected_count": 7,
                "target_object_count": 7,
                "remaining_s": 29.2,
                "return_reason": "TIME_LIMIT",
            }
        }
        self.assertEqual(
            mission_progress_label(state),
            "수집 7/7 · 내부 3/4 · 배출 4/7",
        )
        self.assertEqual(mission_time_label(state), "00:30")
        self.assertEqual(return_reason_label(state), "남은 시간 30초")

    def test_invalid_json_uses_empty_dictionary(self):
        self.assertEqual(parse_json_message("not-json"), {})

    def test_mapper_diagnostics_explain_calibration_and_counts(self):
        state = {
            "mapper_status": "ready",
            "calibration_loaded": True,
            "detection_count": 3,
            "mapped_count": 2,
        }
        self.assertEqual(
            mapper_diagnostics_label(state),
            "보정 준비 · CSV · 감지 3 / 변환 2",
        )

    def test_mapper_diagnostics_expose_calibration_error(self):
        state = {
            "mapper_status": "calibration_error",
            "calibration_loaded": False,
            "calibration_error": "file not found",
        }
        self.assertIn("CSV 보정 오류", mapper_diagnostics_label(state))
        self.assertIn("file not found", mapper_diagnostics_label(state))

    def test_quaternion_yaw(self):
        yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertAlmostEqual(yaw, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
