import math
import unittest

from rl_model_policy.storage_tof_correction import (
    desired_yaw_for_storage_axis,
    make_storage_tof_command,
    measurement_gap_timed_out,
    robot_coordinate_from_min_wall_distance,
    storage_coarse_heading_is_aligned,
)


def make_command(**overrides):
    values = {
        "axis": "x",
        "distance_m": 0.16,
        "measurement_age_s": 0.02,
        "robot_yaw": math.pi,
        "target_coordinate": -1.75,
        "wall_coordinate_m": -2.0,
        "sensor_forward_offset_m": 0.09,
        "transit_speed": 0.25,
        "minimum_speed": 0.05,
        "slowdown_distance_m": 0.20,
        "coordinate_tolerance_m": 0.03,
        "measurement_timeout_s": 0.25,
        "heading_gain": 2.4,
        "max_angular_speed": 1.00,
        "wall_angle_gain": 1.5,
        "wall_angle_max_angular_speed": 0.60,
        "heading_tolerance": 0.12,
        "wall_angle_rad": 0.0,
        "wall_angle_tolerance_rad": 0.05,
    }
    values.update(overrides)
    return make_storage_tof_command(**values)


class StorageTofCorrectionTest(unittest.TestCase):
    def test_successful_axis_corrections_have_cardinal_yaw_resets(self):
        self.assertAlmostEqual(desired_yaw_for_storage_axis("x"), math.pi)
        self.assertAlmostEqual(
            desired_yaw_for_storage_axis("y"),
            -math.pi / 2.0,
        )

    def test_measurement_gap_timeout_requires_one_continuous_second(self):
        self.assertFalse(measurement_gap_timed_out(None, 11.0, 1.0))
        self.assertFalse(measurement_gap_timed_out(10.0, 10.99, 1.0))
        self.assertTrue(measurement_gap_timed_out(10.0, 11.0, 1.0))

    def test_converts_min_wall_range_to_robot_center_coordinate(self):
        coordinate = robot_coordinate_from_min_wall_distance(0.16, -2.0, 0.09)

        self.assertAlmostEqual(coordinate, -1.75)

    def test_x_correction_faces_left_before_reading_tof(self):
        command = make_command(robot_yaw=-math.pi / 2.0, wall_angle_rad=None)

        self.assertEqual(command.phase, "ALIGN_STORAGE_TOF_X")
        self.assertEqual(command.linear_x, 0.0)
        self.assertNotEqual(command.angular_z, 0.0)
        self.assertIsNone(command.measured_coordinate)

    def test_y_correction_faces_bottom_and_uses_bottom_wall(self):
        command = make_command(
            axis="y",
            distance_m=0.16,
            robot_yaw=-math.pi / 2.0,
            target_coordinate=-1.75,
        )

        self.assertTrue(command.reached)
        self.assertEqual(command.phase, "STORAGE_TOF_Y_ALIGNED")
        self.assertAlmostEqual(command.measured_coordinate, -1.75)

    def test_exit_x_check_uses_west_wall_distance(self):
        command = make_command(
            distance_m=0.66,
            target_coordinate=-1.25,
        )

        self.assertTrue(command.reached)
        self.assertEqual(command.phase, "STORAGE_TOF_X_ALIGNED")
        self.assertAlmostEqual(command.measured_coordinate, -1.25)

    def test_stale_measurement_stops_the_robot(self):
        command = make_command(measurement_age_s=0.50)

        self.assertEqual(command.phase, "WAITING_FOR_STORAGE_TOF_X")
        self.assertEqual(command.linear_x, 0.0)
        self.assertFalse(command.reached)

    def test_wall_angle_is_aligned_before_distance_correction(self):
        command = make_command(wall_angle_rad=-0.10)

        self.assertEqual(command.phase, "ALIGN_STORAGE_WALL_ANGLE_X")
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.reached)

    def test_tof_wall_angle_uses_separate_reduced_speed_limits(self):
        proportional = make_command(wall_angle_rad=math.radians(20.0))
        limited = make_command(wall_angle_rad=math.radians(30.0))

        self.assertAlmostEqual(proportional.angular_z, math.radians(30.0))
        self.assertAlmostEqual(limited.angular_z, 0.60)

    def test_coarse_odometry_keeps_original_speed_limit(self):
        command = make_command(robot_yaw=math.pi / 2.0)

        self.assertEqual(command.phase, "ALIGN_STORAGE_TOF_X")
        self.assertAlmostEqual(command.angular_z, 1.0)

    def test_fresh_wall_angle_does_not_override_coarse_odometry_alignment(self):
        command = make_command(robot_yaw=-math.pi / 2.0, wall_angle_rad=-0.10)

        self.assertEqual(command.phase, "ALIGN_STORAGE_TOF_X")
        self.assertLess(command.angular_z, 0.0)

    def test_latched_tof_alignment_does_not_return_to_odometry(self):
        command = make_command(
            robot_yaw=-math.pi / 2.0,
            wall_angle_rad=-0.10,
            coarse_heading_aligned=True,
        )

        self.assertEqual(command.phase, "ALIGN_STORAGE_WALL_ANGLE_X")
        self.assertLess(command.angular_z, 0.0)

    def test_latched_tof_alignment_holds_when_measurement_is_stale(self):
        command = make_command(
            robot_yaw=-math.pi / 2.0,
            measurement_age_s=0.50,
            coarse_heading_aligned=True,
        )

        self.assertEqual(command.phase, "WAITING_FOR_STORAGE_TOF_X")
        self.assertEqual(command.angular_z, 0.0)

    def test_coarse_heading_helper_uses_selected_axis(self):
        self.assertTrue(storage_coarse_heading_is_aligned(math.pi, "x", 0.12))
        self.assertTrue(
            storage_coarse_heading_is_aligned(-math.pi / 2.0, "y", 0.12)
        )
        self.assertFalse(
            storage_coarse_heading_is_aligned(-math.pi / 2.0, "x", 0.12)
        )

    def test_x_entry_advances_until_tof_becomes_available(self):
        command = make_command(
            distance_m=None,
            measurement_age_s=None,
            transit_speed=0.30,
            advance_without_measurement=True,
        )

        self.assertEqual(command.phase, "APPROACH_STORAGE_TOF_X")
        self.assertAlmostEqual(command.linear_x, 0.30)
        self.assertIsNone(command.measured_coordinate)
        self.assertFalse(command.reached)

    def test_drives_toward_min_wall_when_coordinate_is_too_large(self):
        command = make_command(distance_m=0.25)

        self.assertEqual(command.phase, "TOF_CORRECT_STORAGE_X")
        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.coordinate_error, 0.0)

    def test_x_entry_can_hold_full_speed_until_tolerance(self):
        command = make_command(
            distance_m=0.20,
            transit_speed=0.30,
            minimum_speed=0.30,
            slowdown_distance_m=0.03,
        )

        self.assertFalse(command.reached)
        self.assertAlmostEqual(command.linear_x, 0.30)

    def test_reverses_after_crossing_target_coordinate(self):
        command = make_command(distance_m=0.10)

        self.assertLess(command.linear_x, 0.0)
        self.assertLess(command.coordinate_error, 0.0)

    def test_rejects_unknown_axis(self):
        with self.assertRaises(ValueError):
            make_command(axis="z")


if __name__ == "__main__":
    unittest.main()
