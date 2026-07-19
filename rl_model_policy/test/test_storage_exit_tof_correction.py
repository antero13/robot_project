import math
import unittest

from rl_model_policy.storage_exit_tof_correction import (
    make_storage_exit_tof_command,
)


def make_command(**overrides):
    values = {
        "distance_m": 0.66,
        "wall_angle_rad": 0.0,
        "measurement_age_s": 0.02,
        "robot_yaw": math.pi,
        "target_x": -1.25,
        "west_wall_x_m": -2.0,
        "sensor_forward_offset_m": 0.09,
        "transit_speed": 0.25,
        "minimum_speed": 0.05,
        "slowdown_distance_m": 0.20,
        "x_tolerance_m": 0.03,
        "measurement_timeout_s": 0.25,
        "angle_alignment_active": False,
        "angle_trigger_rad": math.radians(10.0),
        "angle_release_rad": math.radians(5.0),
        "angle_gain": 1.5,
        "max_angular_speed": 0.60,
        "heading_tolerance": 0.12,
    }
    values.update(overrides)
    return make_storage_exit_tof_command(**values)


class StorageExitTofCorrectionTest(unittest.TestCase):
    def test_coarse_west_heading_precedes_tof(self):
        command = make_command(robot_yaw=-math.pi / 2.0)

        self.assertEqual(command.phase, "ALIGN_STORAGE_EXIT_WEST_ODOMETRY")
        self.assertEqual(command.linear_x, 0.0)
        self.assertNotEqual(command.angular_z, 0.0)

    def test_waits_for_fresh_tof(self):
        command = make_command(measurement_age_s=0.30)

        self.assertEqual(command.phase, "WAITING_FOR_STORAGE_TOF_X")
        self.assertFalse(command.reached)

    def test_corrects_distance_before_large_angle(self):
        command = make_command(
            distance_m=0.75,
            wall_angle_rad=math.radians(20.0),
        )

        self.assertEqual(command.phase, "TOF_CORRECT_STORAGE_EXIT_DISTANCE")
        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)
        self.assertFalse(command.angle_alignment_active)

    def test_distance_correction_reverses_after_overshoot(self):
        command = make_command(distance_m=0.55)

        self.assertLess(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_angle_below_ten_degrees_does_not_start_alignment(self):
        command = make_command(wall_angle_rad=math.radians(9.0))

        self.assertEqual(command.phase, "STORAGE_EXIT_TOF_ALIGNED")
        self.assertTrue(command.reached)

    def test_ten_degrees_starts_alignment(self):
        command = make_command(wall_angle_rad=math.radians(10.0))

        self.assertEqual(command.phase, "ALIGN_STORAGE_EXIT_WEST_WALL_ANGLE")
        self.assertGreater(command.angular_z, 0.0)
        self.assertTrue(command.angle_alignment_active)

    def test_active_alignment_continues_until_five_degrees(self):
        command = make_command(
            wall_angle_rad=math.radians(-7.0),
            angle_alignment_active=True,
        )

        self.assertEqual(command.phase, "ALIGN_STORAGE_EXIT_WEST_WALL_ANGLE")
        self.assertLess(command.angular_z, 0.0)
        self.assertTrue(command.angle_alignment_active)

    def test_active_alignment_finishes_at_five_degrees(self):
        command = make_command(
            wall_angle_rad=math.radians(5.0),
            angle_alignment_active=True,
        )

        self.assertEqual(command.phase, "STORAGE_EXIT_TOF_ALIGNED")
        self.assertFalse(command.angle_alignment_active)
        self.assertTrue(command.reached)


if __name__ == "__main__":
    unittest.main()
