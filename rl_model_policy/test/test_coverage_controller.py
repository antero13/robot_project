import math
import unittest

from rl_model_policy.coverage_controller import (
    CoverageController,
    generate_coverage_legs,
    normalize_angle,
)


def make_legs():
    return generate_coverage_legs(
        min_x=-0.75,
        max_x=1.25,
        main_road_y=-1.33,
        scan_end_y=1.0,
        lane_spacing=1.0,
        scan_speed=0.10,
        transit_speed=0.20,
        return_speed=0.18,
    )


class CoverageControllerTest(unittest.TestCase):
    def test_generates_main_road_lane_search_pattern(self):
        legs = make_legs()

        self.assertEqual(len(legs), 9)
        self.assertEqual(
            [(leg.target_x, leg.target_y, leg.phase) for leg in legs[:4]],
            [
                (1.25, -1.33, "ENTER_FIRST_LANE"),
                (1.25, 1.0, "SCAN_LANE"),
                (1.25, -1.33, "RETURN_MAIN_ROAD"),
                (0.25, -1.33, "SHIFT_TO_NEXT_LANE"),
            ],
        )
        self.assertLess(legs[2].speed, 0.0)

    def test_rotates_before_driving_when_heading_error_is_large(self):
        controller = CoverageController(make_legs())

        command = controller.command(1.25, -1.60, 0.0)

        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertTrue(command.phase.startswith("ALIGN_"))

    def test_drives_when_aligned_with_waypoint(self):
        controller = CoverageController(make_legs())

        command = controller.command(1.25, -1.60, math.pi / 2.0)

        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.phase, "ENTER_FIRST_LANE")

    def test_reached_waypoint_advances_to_scan_leg(self):
        controller = CoverageController(make_legs())

        command = controller.command(1.25, -1.33, math.pi / 2.0)

        self.assertEqual(command.leg_index, 1)
        self.assertEqual(command.phase, "SCAN_LANE")

    def test_return_leg_uses_reverse_velocity_while_facing_north(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 2

        command = controller.command(1.25, 0.80, math.pi / 2.0)

        self.assertLess(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0)

    def test_front_obstacle_overrides_positive_drive_but_not_reverse(self):
        controller = CoverageController(make_legs(), avoid_danger_threshold=0.2)
        forward = controller.command(
            1.25,
            -1.60,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )
        controller.leg_index = 2
        reverse = controller.command(
            1.25,
            0.80,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )

        self.assertEqual(forward.phase, "AVOID_OBJECT")
        self.assertEqual(forward.linear_x, 0.0)
        self.assertLess(forward.angular_z, 0.0)
        self.assertLess(reverse.linear_x, 0.0)

    def test_last_leg_wraps_to_a_new_cycle(self):
        controller = CoverageController(make_legs())
        controller.leg_index = len(controller.legs) - 1

        command = controller.command(-0.75, -1.33, 0.0)

        self.assertEqual(command.leg_index, 0)
        self.assertEqual(command.cycle_count, 1)

    def test_default_route_completes_within_match_time_without_obstacles(self):
        legs = generate_coverage_legs(
            min_x=-1.75,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.14,
            transit_speed=0.18,
            return_speed=0.20,
        )
        controller = CoverageController(legs)
        x, y, yaw = 1.8, -1.8, math.pi / 2.0
        dt = 0.05

        for step in range(int(180.0 / dt)):
            command = controller.command(x, y, yaw)
            x += command.linear_x * math.cos(yaw) * dt
            y += command.linear_x * math.sin(yaw) * dt
            yaw = normalize_angle(yaw + command.angular_z * dt)
            if controller.cycle_count == 1:
                break

        self.assertEqual(controller.cycle_count, 1)
        self.assertLess(step * dt, 180.0)

    def test_normalize_angle_wraps_at_pi(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)


if __name__ == "__main__":
    unittest.main()
