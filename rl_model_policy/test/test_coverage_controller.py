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

        self.assertEqual(len(legs), 6)
        self.assertEqual(
            [(leg.target_x, leg.target_y, leg.phase) for leg in legs[:4]],
            [
                (1.25, -1.33, "ENTER_FIRST_LANE"),
                (1.25, 1.0, "SCAN_LANE_UP"),
                (0.25, 1.0, "SHIFT_TO_NEXT_LANE"),
                (0.25, -1.33, "SCAN_LANE_DOWN"),
            ],
        )
        self.assertTrue(all(leg.speed > 0.0 for leg in legs))

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
        self.assertEqual(command.phase, "SCAN_LANE_UP")

    def test_alternating_lane_scans_drive_forward_in_both_directions(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        command = controller.command(0.25, 0.80, -math.pi / 2.0)

        self.assertGreater(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0)

    def test_front_obstacle_runs_turn_then_forward_bypass(self):
        controller = CoverageController(
            make_legs(),
            avoid_danger_threshold=0.2,
            avoid_turn_angle=0.5,
            avoid_pass_distance=0.4,
            avoid_forward_speed=0.16,
        )
        turning = controller.command(
            1.25,
            -1.60,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )
        bypass_yaw = controller.avoid_target_yaw
        passing = controller.command(1.25, -1.60, bypass_yaw)
        completed = controller.command(
            1.25 + 0.41 * math.cos(bypass_yaw),
            -1.60 + 0.41 * math.sin(bypass_yaw),
            bypass_yaw,
        )

        self.assertEqual(turning.phase, "AVOID_TURN")
        self.assertEqual(turning.linear_x, 0.0)
        self.assertNotEqual(turning.angular_z, 0.0)
        self.assertEqual(passing.phase, "AVOID_PASS")
        self.assertGreater(passing.linear_x, 0.0)
        self.assertEqual(completed.phase, "AVOID_COMPLETE")

    def test_repeated_blockage_skips_leg_instead_of_looping_forever(self):
        controller = CoverageController(
            make_legs(),
            avoid_turn_angle=0.5,
            avoid_pass_distance=0.1,
            max_avoid_attempts_per_leg=2,
        )
        x, y, yaw = 1.25, -1.60, math.pi / 2.0
        for _ in range(2):
            controller.command(x, y, yaw, avoid_center=1.0)
            yaw += 0.5
            controller.command(x, y, yaw)
            x += 0.11 * math.cos(yaw)
            y += 0.11 * math.sin(yaw)
            controller.command(x, y, yaw)

        previous_leg = controller.leg_index
        skipped = controller.command(x, y, yaw, avoid_center=1.0)

        self.assertEqual(skipped.phase, "SKIP_BLOCKED_LEG")
        self.assertNotEqual(controller.leg_index, previous_leg)

    def test_avoidance_turns_inward_near_left_wall(self):
        controller = CoverageController(make_legs())

        command = controller.command(
            -1.75,
            0.0,
            -math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=1.0,
            avoid_right=0.1,
        )

        self.assertEqual(command.phase, "AVOID_TURN")
        self.assertGreater(command.angular_z, 0.0)

    def test_cancelling_avoidance_preserves_coverage_progress(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 4
        controller.avoid_phase = "PASS"

        controller.cancel_avoidance()

        self.assertEqual(controller.leg_index, 4)
        self.assertIsNone(controller.avoid_phase)

    def test_last_leg_wraps_to_a_new_cycle(self):
        controller = CoverageController(make_legs())
        controller.leg_index = len(controller.legs) - 1
        final_leg = controller.current_leg

        command = controller.command(final_leg.target_x, final_leg.target_y, 0.0)

        self.assertEqual(command.leg_index, 0)
        self.assertEqual(command.cycle_count, 1)

    def test_default_route_completes_within_match_time_without_obstacles(self):
        legs = generate_coverage_legs(
            min_x=-1.75,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.22,
            transit_speed=0.28,
            return_speed=0.24,
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
        self.assertLess(step * dt, 90.0)

    def test_normalize_angle_wraps_at_pi(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)


if __name__ == "__main__":
    unittest.main()
