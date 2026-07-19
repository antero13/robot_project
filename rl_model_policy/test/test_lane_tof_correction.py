import math
import unittest

from rl_model_policy.lane_tof_correction import (
    make_lane_tof_command,
    robot_x_from_left_wall_distance,
    robot_x_from_right_wall_distance,
    should_run_lane_tof_fine_alignment,
)


def make_command(**overrides):
    values = {
        "distance_m": 2.15,
        "measurement_age_s": 0.02,
        "robot_yaw": math.pi,
        "target_x": 0.25,
        "left_wall_x_m": -2.0,
        "right_wall_x_m": 2.0,
        "wall_side": "left",
        "sensor_forward_offset_m": 0.10,
        "transit_speed": 0.30,
        "minimum_speed": 0.08,
        "slowdown_distance_m": 0.20,
        "x_tolerance_m": 0.03,
        "measurement_timeout_s": 0.25,
        "heading_gain": 2.4,
        "max_angular_speed": 1.0,
        "heading_tolerance": 0.08,
        "wall_angle_rad": 0.0,
        "wall_angle_tolerance_rad": 0.05,
    }
    values.update(overrides)
    return make_lane_tof_command(**values)


class LaneTofCorrectionTest(unittest.TestCase):

    def test_tof_fine_alignment_stays_off_before_waypoint_arrival(self):
        active = should_run_lane_tof_fine_alignment(
            enabled=True,
            leg_phase="SHIFT_TO_NEXT_LANE",
            waypoint_reached=False,
            alignment_active=False,
        )

        self.assertFalse(active)

    def test_tof_fine_alignment_starts_at_waypoint_and_stays_latched(self):
        starting = should_run_lane_tof_fine_alignment(
            enabled=True,
            leg_phase="SHIFT_TO_NEXT_LANE",
            waypoint_reached=True,
            alignment_active=False,
        )
        after_odom_moves_outside_tolerance = should_run_lane_tof_fine_alignment(
            enabled=True,
            leg_phase="SHIFT_TO_NEXT_LANE",
            waypoint_reached=False,
            alignment_active=True,
        )

        self.assertTrue(starting)
        self.assertTrue(after_odom_moves_outside_tolerance)

    def test_tof_fine_alignment_does_not_run_on_scan_legs(self):
        active = should_run_lane_tof_fine_alignment(
            enabled=True,
            leg_phase="SCAN_LANE_UP",
            waypoint_reached=True,
            alignment_active=True,
        )

        self.assertFalse(active)

    def test_converts_left_wall_range_to_robot_center_x(self):
        robot_x = robot_x_from_left_wall_distance(1.15, -2.0, 0.10)

        self.assertAlmostEqual(robot_x, -0.75)

    def test_real_chassis_sensor_offset_recovers_body_center_x(self):
        robot_x = robot_x_from_left_wall_distance(2.16, -2.0, 0.09)

        self.assertAlmostEqual(robot_x, 0.25)

    def test_converts_right_wall_range_to_robot_center_x(self):
        robot_x = robot_x_from_right_wall_distance(2.66, 2.0, 0.09)

        self.assertAlmostEqual(robot_x, -0.75)

    def test_rotates_before_using_wall_distance(self):
        command = make_command(robot_yaw=math.pi / 2.0, wall_angle_rad=None)

        self.assertEqual(command.phase, "ALIGN_TOF_NEXT_LANE")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertIsNone(command.measured_robot_x)

    def test_stale_measurement_stops_instead_of_using_odometry(self):
        command = make_command(measurement_age_s=0.40)

        self.assertEqual(command.phase, "WAITING_FOR_LANE_TOF")
        self.assertEqual(command.linear_x, 0.0)
        self.assertFalse(command.reached)

    def test_wall_angle_is_aligned_before_distance_correction(self):
        command = make_command(wall_angle_rad=0.10)

        self.assertEqual(command.phase, "ALIGN_LANE_WALL_ANGLE")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertFalse(command.reached)

    def test_fresh_wall_angle_does_not_override_coarse_odometry_alignment(self):
        command = make_command(robot_yaw=math.pi / 2.0, wall_angle_rad=0.10)

        self.assertEqual(command.phase, "ALIGN_TOF_NEXT_LANE")
        self.assertGreater(command.angular_z, 0.0)

    def test_drives_left_until_next_lane_is_reached(self):
        command = make_command(distance_m=2.40)

        self.assertEqual(command.phase, "TOF_SHIFT_TO_NEXT_LANE")
        self.assertGreater(command.linear_x, 0.0)
        self.assertAlmostEqual(command.measured_robot_x, 0.50)

    def test_reverses_after_overshooting_lane_center(self):
        command = make_command(distance_m=2.00)

        self.assertLess(command.linear_x, 0.0)
        self.assertLess(command.x_error, 0.0)

    def test_reports_alignment_inside_x_tolerance(self):
        command = make_command(distance_m=2.15)

        self.assertTrue(command.reached)
        self.assertEqual(command.phase, "TOF_LANE_ALIGNED")
        self.assertEqual(command.linear_x, 0.0)

    def test_reverse_route_faces_right_wall_and_drives_east(self):
        command = make_command(
            wall_side="right",
            robot_yaw=0.0,
            target_x=-0.75,
            distance_m=2.91,
            sensor_forward_offset_m=0.09,
        )

        self.assertEqual(command.phase, "TOF_SHIFT_TO_NEXT_LANE")
        self.assertAlmostEqual(command.measured_robot_x, -1.0)
        self.assertGreater(command.linear_x, 0.0)

    def test_reverse_route_reverses_after_crossing_lane_center(self):
        command = make_command(
            wall_side="right",
            robot_yaw=0.0,
            target_x=-0.75,
            distance_m=2.41,
            sensor_forward_offset_m=0.09,
        )

        self.assertAlmostEqual(command.measured_robot_x, -0.5)
        self.assertLess(command.linear_x, 0.0)

    def test_reverse_route_rotates_to_face_right_wall_first(self):
        command = make_command(
            wall_side="right",
            robot_yaw=math.pi,
            wall_angle_rad=None,
        )

        self.assertEqual(command.phase, "ALIGN_TOF_NEXT_LANE")
        self.assertEqual(command.linear_x, 0.0)
        self.assertNotEqual(command.angular_z, 0.0)


if __name__ == "__main__":
    unittest.main()
