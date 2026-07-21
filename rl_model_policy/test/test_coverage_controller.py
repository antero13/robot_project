import math
import unittest

from rl_model_policy.coverage_controller import (
    CoverageController,
    curved_rejoin_exceeds_coordinate_limit,
    generate_coverage_legs,
    normalize_angle,
    signed_lane_shift_speed,
    wall_side_for_lane_number,
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
            [(leg.target_x, leg.target_y, leg.phase) for leg in legs[:5]],
            [
                (1.25, -1.33, "ENTER_FIRST_LANE"),
                (1.25, 1.0, "SCAN_LANE_UP"),
                (1.25, -1.33, "SCAN_LANE_DOWN"),
                (0.25, -1.33, "SHIFT_TO_NEXT_LANE"),
                (0.25, 1.0, "SCAN_LANE_UP"),
            ],
        )
        shift_legs = [leg for leg in legs if leg.phase == "SHIFT_TO_NEXT_LANE"]
        self.assertEqual([leg.lane_number for leg in shift_legs], [2, 3])
        self.assertEqual([leg.wall_side for leg in shift_legs], ["right", "left"])
        self.assertLess(shift_legs[0].speed, 0.0)
        self.assertGreater(shift_legs[1].speed, 0.0)

    def test_generates_four_lanes_in_reverse_order(self):
        legs = generate_coverage_legs(
            min_x=-1.25,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.24,
            transit_speed=0.30,
            return_speed=0.24,
            reverse_order=True,
        )

        scan_lane_x_positions = [
            leg.target_x for leg in legs if leg.phase == "SCAN_LANE_UP"
        ]

        self.assertEqual(len(legs), 12)
        self.assertEqual(scan_lane_x_positions, [-1.25, -0.75, 0.25, 1.25])

    def test_normal_route_lane_1_to_2_faces_east_wall_and_reverses(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        command = controller.command(0.75, -1.33, 0.0)

        self.assertEqual(controller.current_leg.lane_number, 2)
        self.assertEqual(controller.current_shift_wall_side(), "right")
        self.assertEqual(command.phase, "SHIFT_TO_NEXT_LANE")
        self.assertLess(command.linear_x, 0.0)

    def test_normal_route_lane_2_to_3_faces_west_wall_and_drives_forward(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 6

        command = controller.command(-0.25, -1.33, math.pi)

        self.assertEqual(controller.current_leg.lane_number, 3)
        self.assertEqual(controller.current_shift_wall_side(), "left")
        self.assertGreater(command.linear_x, 0.0)

    def test_reverse_route_lane_4_to_3_faces_west_wall_and_reverses(self):
        legs = generate_coverage_legs(
            min_x=-1.25,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.24,
            transit_speed=0.30,
            return_speed=0.24,
            reverse_order=True,
        )
        controller = CoverageController(legs)
        controller.leg_index = 3

        command = controller.command(-1.0, -1.3343, math.pi)

        self.assertEqual(controller.current_leg.lane_number, 3)
        self.assertEqual(controller.current_shift_wall_side(), "left")
        self.assertLess(command.linear_x, 0.0)

    def test_reverse_route_lane_3_to_2_faces_east_wall_and_drives_forward(self):
        legs = generate_coverage_legs(
            min_x=-1.25,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.24,
            transit_speed=0.30,
            return_speed=0.24,
            reverse_order=True,
        )
        controller = CoverageController(legs)
        controller.leg_index = 6

        command = controller.command(-0.25, -1.3343, 0.0)

        self.assertEqual(controller.current_leg.lane_number, 2)
        self.assertEqual(controller.current_shift_wall_side(), "right")
        self.assertGreater(command.linear_x, 0.0)

    def test_lane_shift_uses_waypoint_motion_before_tof_fine_alignment(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        command = controller.command(0.75, -1.33, 0.0)

        self.assertEqual(command.phase, "SHIFT_TO_NEXT_LANE")
        self.assertEqual(controller.leg_index, 3)
        self.assertLess(command.linear_x, 0.0)

    def test_lane_shift_uses_configured_point_four_speed(self):
        legs = generate_coverage_legs(
            min_x=-0.75,
            max_x=1.25,
            main_road_y=-1.33,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.24,
            transit_speed=0.40,
            return_speed=0.24,
        )
        controller = CoverageController(legs)
        controller.leg_index = 3

        command = controller.command(0.75, -1.33, 0.0)

        self.assertAlmostEqual(command.linear_x, -0.40)

    def test_lane_1_to_2_turns_left_to_face_north_after_tof_completion(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        self.assertTrue(controller.complete_current_leg("SHIFT_TO_NEXT_LANE"))
        command = controller.command(0.25, -1.33, 0.0)

        self.assertEqual(command.phase, "ALIGN_SCAN_LANE_UP")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_lane_wall_assignment_uses_east_for_1_2_and_west_for_3_4(self):
        self.assertEqual(
            [wall_side_for_lane_number(number) for number in range(1, 5)],
            ["right", "right", "left", "left"],
        )

    def test_signed_shift_speed_matches_requested_examples(self):
        self.assertLess(
            signed_lane_shift_speed(
                current_x=1.25,
                target_x=0.25,
                wall_side="right",
                transit_speed=0.30,
            ),
            0.0,
        )
        self.assertGreater(
            signed_lane_shift_speed(
                current_x=0.25,
                target_x=-0.75,
                wall_side="left",
                transit_speed=0.30,
            ),
            0.0,
        )
        self.assertLess(
            signed_lane_shift_speed(
                current_x=-1.25,
                target_x=-0.75,
                wall_side="left",
                transit_speed=0.30,
            ),
            0.0,
        )
        self.assertGreater(
            signed_lane_shift_speed(
                current_x=-0.75,
                target_x=0.25,
                wall_side="right",
                transit_speed=0.30,
            ),
            0.0,
        )

    def test_current_leg_reached_checks_waypoint_without_advancing(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        self.assertFalse(controller.current_leg_reached(0.40, -1.33))
        self.assertTrue(controller.current_leg_reached(0.30, -1.33))
        self.assertEqual(controller.leg_index, 3)

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

    def test_downward_lane_scan_turns_then_drives_forward(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 2

        command = controller.command(1.25, 0.80, -math.pi / 2.0)

        self.assertGreater(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0)

    def test_close_object_steers_while_still_moving_forward(self):
        controller = CoverageController(
            make_legs(),
            avoid_danger_threshold=0.2,
            avoid_angular_speed=0.5,
            avoid_linear_scale=0.7,
        )
        controller.leg_index = 1
        clear = controller.command(1.25, -0.5, math.pi / 2.0)
        avoiding = controller.command(
            1.25,
            -0.5,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )

        self.assertEqual(avoiding.phase, "CURVE_AVOID_RIGHT")
        self.assertGreater(avoiding.linear_x, 0.0)
        self.assertLess(avoiding.angular_z, 0.0)
        self.assertLess(avoiding.linear_x, clear.linear_x)

    def test_small_heading_error_is_corrected_while_moving(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 1

        command = controller.command(1.25, -0.5, math.pi / 2.0 - 0.2)

        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_lane_avoidance_waits_until_heading_error_is_within_tolerance(self):
        controller = CoverageController(
            make_legs(),
            avoid_heading_tolerance=0.14,
        )
        controller.leg_index = 1

        command = controller.command(
            1.25,
            -0.5,
            math.pi / 2.0 - 0.20,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )

        self.assertEqual(command.phase, "SCAN_LANE_UP")
        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_close_side_object_also_triggers_moving_correction(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 1

        command = controller.command(
            1.25,
            -0.5,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.0,
            avoid_right=0.1,
        )

        self.assertEqual(command.phase, "CURVE_AVOID_RIGHT")
        self.assertGreater(command.linear_x, 0.0)

    def test_top_of_lane_starts_fast_180_degree_turn(self):
        controller = CoverageController(
            make_legs(),
            heading_gain=2.4,
            max_angular_speed=1.0,
            turn_in_place_threshold=0.65,
        )
        controller.leg_index = 1

        command = controller.command(
            1.25,
            1.0,
            math.pi / 2.0,
            avoid_left=0.8,
            avoid_center=0.9,
            avoid_right=0.1,
        )

        self.assertEqual(command.phase, "ALIGN_SCAN_LANE_DOWN")
        self.assertEqual(command.linear_x, 0.0)
        self.assertAlmostEqual(abs(command.angular_z), 1.0)

    def test_cancelling_avoidance_preserves_coverage_progress(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 4

        controller.cancel_avoidance()

        self.assertEqual(controller.leg_index, 4)

    def test_rejoin_aligns_45_degrees_then_drives_toward_current_lane(self):
        controller = CoverageController(make_legs(), rejoin_speed=0.16)
        controller.leg_index = 1

        self.assertTrue(controller.begin_rejoin(robot_y=-0.25))
        aligning = controller.command(1.75, -0.25, math.pi / 2.0)
        driving = controller.command(1.75, -0.25, 3.0 * math.pi / 4.0)

        self.assertEqual(aligning.phase, "ALIGN_CURVED_REJOIN")
        self.assertEqual(aligning.linear_x, 0.0)
        self.assertGreater(aligning.angular_z, 0.0)
        self.assertEqual(driving.phase, "CURVE_REJOIN_LANE")
        self.assertAlmostEqual(driving.linear_x, 0.16)
        self.assertAlmostEqual(driving.angular_z, 0.0)
        self.assertAlmostEqual(driving.waypoint_x, 1.25)
        self.assertAlmostEqual(driving.waypoint_y, -0.25)

    def test_rejoin_keeps_moving_while_blending_back_to_lane_heading(self):
        controller = CoverageController(
            make_legs(),
            rejoin_speed=0.16,
            rejoin_blend_distance=0.45,
        )
        controller.leg_index = 1
        controller.begin_rejoin(robot_y=-0.25)
        controller.command(1.75, -0.25, 3.0 * math.pi / 4.0)

        command = controller.command(1.45, -0.05, 3.0 * math.pi / 4.0)

        self.assertEqual(command.phase, "CURVE_REJOIN_LANE")
        self.assertGreater(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)

    def test_rejoin_uses_perpendicular_path_when_curve_would_leave_arena(self):
        controller = CoverageController(
            make_legs(),
            rejoin_speed=0.16,
            rejoin_coordinate_limit=1.8,
        )
        controller.leg_index = 1
        controller.begin_rejoin(robot_y=1.10)

        aligning = controller.command(1.75, 1.10, math.pi / 2.0)
        driving = controller.command(1.75, 1.10, math.pi)

        self.assertEqual(aligning.phase, "ALIGN_PERPENDICULAR_REJOIN")
        self.assertEqual(aligning.linear_x, 0.0)
        self.assertEqual(driving.phase, "PERPENDICULAR_REJOIN_LANE")
        self.assertAlmostEqual(driving.linear_x, 0.16)
        self.assertAlmostEqual(driving.angular_z, 0.0)
        self.assertAlmostEqual(driving.waypoint_x, 1.25)
        self.assertAlmostEqual(driving.waypoint_y, 1.10)

    def test_rejoin_keeps_curved_path_when_prediction_stays_inside_arena(self):
        controller = CoverageController(
            make_legs(),
            rejoin_speed=0.16,
            rejoin_coordinate_limit=1.8,
        )
        controller.leg_index = 1
        controller.begin_rejoin(robot_y=0.50)

        command = controller.command(1.75, 0.50, 3.0 * math.pi / 4.0)

        self.assertEqual(command.phase, "CURVE_REJOIN_LANE")

    def test_coordinate_at_limit_is_unsafe_for_curved_rejoin(self):
        self.assertTrue(
            curved_rejoin_exceeds_coordinate_limit(
                robot_x=1.75,
                robot_y=1.80,
                target_x=1.25,
                scan_direction=1.0,
                approach_angle=math.pi / 4.0,
                blend_distance=0.45,
                waypoint_tolerance=0.10,
                coordinate_limit=1.80,
            )
        )

    def test_rejoin_completion_resumes_existing_scan_leg(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 1
        controller.begin_rejoin(robot_y=-0.25)

        command = controller.command(1.25, -0.25, math.pi / 2.0)

        self.assertFalse(controller.rejoin_active)
        self.assertEqual(command.leg_index, 1)
        self.assertEqual(command.phase, "SCAN_LANE_UP")
        self.assertGreater(command.linear_x, 0.0)

    def test_rejoin_is_not_started_during_main_road_shift(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        self.assertFalse(controller.begin_rejoin(robot_y=-1.10))
        self.assertFalse(controller.rejoin_active)

    def test_external_reference_can_complete_only_expected_shift_leg(self):
        controller = CoverageController(make_legs())
        controller.leg_index = 3

        self.assertFalse(controller.complete_current_leg("SCAN_LANE_UP"))
        self.assertEqual(controller.leg_index, 3)
        self.assertTrue(controller.complete_current_leg("SHIFT_TO_NEXT_LANE"))

        hold = controller.hold_command("TOF_LANE_ALIGNED")
        self.assertEqual(controller.leg_index, 4)
        self.assertEqual(hold.phase, "TOF_LANE_ALIGNED")
        self.assertEqual(hold.linear_x, 0.0)
        self.assertEqual(hold.waypoint_x, 0.25)

    def test_last_leg_wraps_to_a_new_cycle(self):
        controller = CoverageController(make_legs())
        controller.leg_index = len(controller.legs) - 1
        final_leg = controller.current_leg

        command = controller.command(final_leg.target_x, final_leg.target_y, 0.0)

        self.assertEqual(command.leg_index, 0)
        self.assertEqual(command.cycle_count, 1)

    def test_default_route_completes_within_match_time_without_obstacles(self):
        legs = generate_coverage_legs(
            min_x=-1.25,
            max_x=1.25,
            main_road_y=-1.3343,
            scan_end_y=1.0,
            lane_spacing=1.0,
            scan_speed=0.24,
            transit_speed=0.30,
            return_speed=0.24,
        )
        controller = CoverageController(
            legs,
            waypoint_tolerance=0.10,
            heading_tolerance=0.08,
            heading_gain=2.4,
            max_angular_speed=1.0,
            turn_in_place_threshold=0.65,
        )
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
        self.assertLess(step * dt, 140.0)

    def test_normalize_angle_wraps_at_pi(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)


if __name__ == "__main__":
    unittest.main()
