import math
import unittest

from rl_model_policy.leave_start import make_leave_start_command


class LeaveStartTest(unittest.TestCase):
    def test_drives_forward_and_turns_right_from_west_toward_north(self):
        command = make_leave_start_command(
            origin_x=1.8,
            origin_y=-1.8,
            desired_yaw=math.pi / 2.0,
            robot_x=1.8,
            robot_y=-1.8,
            robot_yaw=math.pi,
            distance_m=0.55,
            linear_speed=0.25,
            heading_gain=2.4,
            max_angular_speed=0.6,
        )
        self.assertAlmostEqual(command.linear_x, 0.25)
        self.assertAlmostEqual(command.angular_z, -0.6)
        self.assertFalse(command.complete)

    def test_corrects_heading_while_continuing_forward(self):
        command = make_leave_start_command(
            origin_x=0.0,
            origin_y=0.0,
            desired_yaw=0.0,
            robot_x=0.1,
            robot_y=0.0,
            robot_yaw=0.2,
            distance_m=0.55,
            linear_speed=0.25,
            heading_gain=1.5,
            max_angular_speed=0.4,
        )
        self.assertGreater(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.complete)

    def test_does_not_finish_distance_until_final_heading_is_reached(self):
        command = make_leave_start_command(
            origin_x=0.0,
            origin_y=0.0,
            desired_yaw=math.pi / 2.0,
            robot_x=0.56,
            robot_y=0.0,
            robot_yaw=math.pi,
            distance_m=0.55,
            linear_speed=0.25,
            heading_gain=1.5,
            max_angular_speed=0.4,
        )
        self.assertGreater(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.complete)

    def test_stops_after_requested_distance_and_heading(self):
        command = make_leave_start_command(
            origin_x=0.0,
            origin_y=0.0,
            desired_yaw=math.pi / 2.0,
            robot_x=0.56,
            robot_y=0.0,
            robot_yaw=math.pi / 2.0,
            distance_m=0.55,
            linear_speed=0.25,
            heading_gain=1.5,
            max_angular_speed=0.4,
        )
        self.assertEqual((command.linear_x, command.angular_z), (0.0, 0.0))
        self.assertTrue(command.complete)


if __name__ == "__main__":
    unittest.main()
