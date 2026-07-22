import math
import unittest

from rl_model_policy.main_road_tof_correction import (
    make_main_road_tof_command,
    robot_y_from_south_wall_distance,
)


def make_command(**overrides):
    values = {
        "distance_m": 0.51,
        "wall_angle_rad": 0.0,
        "measurement_age_s": 0.02,
        "robot_yaw": -math.pi / 2.0,
        "target_y": -1.40,
        "south_wall_y_m": -2.0,
        "sensor_forward_offset_m": 0.09,
        "transit_speed": 0.24,
        "minimum_speed": 0.05,
        "slowdown_distance_m": 0.20,
        "y_tolerance_m": 0.03,
        "measurement_timeout_s": 0.25,
        "angle_alignment_active": False,
        "angle_trigger_rad": math.radians(10.0),
        "angle_release_rad": math.radians(5.0),
        "angle_gain": 2.4,
        "max_angular_speed": 1.0,
        "heading_tolerance": 0.08,
    }
    values.update(overrides)
    return make_main_road_tof_command(**values)


class MainRoadTofCorrectionTest(unittest.TestCase):
    def test_coarse_odometry_heading_precedes_tof(self):
        command = make_command(robot_yaw=0.0)

        self.assertEqual(command.phase, "ALIGN_MAIN_ROAD_SOUTH_ODOMETRY")
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)

    def test_converts_south_wall_range_to_robot_center_y(self):
        self.assertAlmostEqual(
            robot_y_from_south_wall_distance(0.51, -2.0, 0.09),
            -1.40,
        )

    def test_waits_for_a_fresh_measurement(self):
        command = make_command(measurement_age_s=0.30)

        self.assertEqual(command.phase, "WAITING_FOR_MAIN_ROAD_SOUTH_TOF")
        self.assertFalse(command.reached)

    def test_corrects_distance_before_a_large_wall_angle(self):
        command = make_command(
            distance_m=0.70,
            wall_angle_rad=math.radians(20.0),
        )

        self.assertEqual(command.phase, "TOF_CORRECT_MAIN_ROAD_DISTANCE")
        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)
        self.assertFalse(command.angle_alignment_active)

    def test_angle_below_ten_degrees_does_not_start_alignment(self):
        command = make_command(wall_angle_rad=math.radians(9.0))

        self.assertEqual(command.phase, "MAIN_ROAD_SOUTH_TOF_ALIGNED")
        self.assertTrue(command.reached)

    def test_ten_degrees_starts_alignment(self):
        command = make_command(wall_angle_rad=math.radians(10.0))

        self.assertEqual(command.phase, "ALIGN_MAIN_ROAD_SOUTH_WALL_ANGLE")
        self.assertGreater(command.angular_z, 0.0)
        self.assertTrue(command.angle_alignment_active)

    def test_active_alignment_continues_below_trigger(self):
        command = make_command(
            wall_angle_rad=math.radians(-7.0),
            angle_alignment_active=True,
        )

        self.assertEqual(command.phase, "ALIGN_MAIN_ROAD_SOUTH_WALL_ANGLE")
        self.assertLess(command.angular_z, 0.0)
        self.assertTrue(command.angle_alignment_active)

    def test_active_alignment_finishes_at_five_degrees(self):
        command = make_command(
            wall_angle_rad=math.radians(5.0),
            angle_alignment_active=True,
        )

        self.assertEqual(command.phase, "MAIN_ROAD_SOUTH_TOF_ALIGNED")
        self.assertEqual(command.angular_z, 0.0)
        self.assertFalse(command.angle_alignment_active)
        self.assertTrue(command.reached)

    def test_rejects_reversed_angle_thresholds(self):
        with self.assertRaises(ValueError):
            make_command(
                angle_trigger_rad=math.radians(5.0),
                angle_release_rad=math.radians(10.0),
            )


if __name__ == "__main__":
    unittest.main()
