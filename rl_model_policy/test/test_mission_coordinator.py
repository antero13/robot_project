import math
import unittest

from rl_model_policy.mission_coordinator import (
    fixed_heading_dash_command,
    MissionCoordinator,
    MissionPhase,
    normalize_angle,
    ReturnReason,
    StorageCurveAvoidanceController,
    reverse_storage_x_exit_command,
    storage_dash_heading,
    storage_mirrored_repush_heading,
    storage_pose_bounds_required,
    storage_second_repush_required,
    storage_side_shift_heading,
    storage_phase_after_staging_x,
    storage_return_start_phase,
    storage_staging_coordinates,
    storage_visit_dash_heading,
    storage_visit_number,
    waypoint_avoidance_required,
    waypoint_command,
)


def simulate_waypoint(start, target, speed, tolerance=0.04):
    x, y, yaw = (float(value) for value in start)
    dt = 0.05
    for _ in range(2000):
        command = waypoint_command(
            robot_x=x,
            robot_y=y,
            robot_yaw=yaw,
            target_x=target[0],
            target_y=target[1],
            speed=speed,
            waypoint_tolerance=tolerance,
            heading_tolerance=0.14,
            heading_gain=1.5,
            max_angular_speed=0.60,
        )
        if command.reached:
            return (x, y, yaw)
        x += command.linear_x * math.cos(yaw) * dt
        y += command.linear_x * math.sin(yaw) * dt
        yaw = math.atan2(
            math.sin(yaw + command.angular_z * dt),
            math.cos(yaw + command.angular_z * dt),
        )
    raise AssertionError("waypoint simulation did not converge")


class MissionCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.mission = MissionCoordinator(
            storage_capacity=4,
            target_object_count=7,
            mission_duration_s=180.0,
            force_return_remaining_s=30.0,
        )
        self.mission.start(10.0)

    def test_waypoint_avoidance_is_disabled_during_final_yaw_alignment(self):
        self.assertFalse(
            waypoint_avoidance_required(
                robot_x=-1.25,
                robot_y=-1.40,
                target_x=-1.25,
                target_y=-1.40,
                waypoint_tolerance=0.10,
                linear_x=0.0,
                avoid_center=0.70,
                danger_threshold=0.20,
            )
        )

    def test_waypoint_avoidance_is_disabled_during_in_place_heading_alignment(self):
        self.assertFalse(
            waypoint_avoidance_required(
                robot_x=-1.25,
                robot_y=0.0,
                target_x=-1.25,
                target_y=-1.40,
                waypoint_tolerance=0.10,
                linear_x=0.0,
                avoid_center=0.70,
                danger_threshold=0.20,
            )
        )

    def test_waypoint_avoidance_remains_enabled_during_translation(self):
        self.assertTrue(
            waypoint_avoidance_required(
                robot_x=-1.25,
                robot_y=0.0,
                target_x=-1.25,
                target_y=-1.40,
                waypoint_tolerance=0.10,
                linear_x=0.25,
                avoid_center=0.70,
                danger_threshold=0.20,
            )
        )

    def test_storage_avoidance_curves_forward_away_from_left_obstacle(self):
        controller = StorageCurveAvoidanceController(
            danger_threshold=0.20,
            release_threshold=0.12,
            linear_scale=0.70,
            angular_speed=0.45,
            direction_hold_s=0.8,
        )
        base = waypoint_command(0.0, 0.0, -math.pi / 2.0, 0.0, -1.0, 0.40)

        command = controller.command(
            now_s=1.0,
            base_command=base,
            nominal_speed=0.40,
            avoid_left=0.80,
            avoid_center=0.80,
            avoid_right=0.0,
            allow_start=True,
        )

        self.assertAlmostEqual(command.linear_x, 0.28)
        self.assertAlmostEqual(command.angular_z, -0.45)
        self.assertTrue(controller.active)

    def test_storage_avoidance_direction_stays_latched_during_heading_error(self):
        controller = StorageCurveAvoidanceController(
            danger_threshold=0.20,
            release_threshold=0.12,
            linear_scale=0.70,
            angular_speed=0.45,
            direction_hold_s=0.8,
        )
        translating = waypoint_command(
            0.0, 0.0, -math.pi / 2.0, 0.0, -1.0, 0.40
        )
        controller.command(
            now_s=1.0,
            base_command=translating,
            nominal_speed=0.40,
            avoid_left=0.80,
            avoid_center=0.80,
            avoid_right=0.0,
            allow_start=True,
        )
        aligning = waypoint_command(0.0, 0.0, -2.0, 0.0, -1.0, 0.40)

        command = controller.command(
            now_s=1.2,
            base_command=aligning,
            nominal_speed=0.40,
            avoid_left=0.0,
            avoid_center=0.0,
            avoid_right=0.80,
            allow_start=False,
        )

        self.assertAlmostEqual(command.linear_x, 0.28)
        self.assertAlmostEqual(command.angular_z, -0.45)
        self.assertTrue(controller.active)

    def test_storage_avoidance_releases_after_hold_and_three_clear_samples(self):
        controller = StorageCurveAvoidanceController(
            danger_threshold=0.20,
            release_threshold=0.12,
            linear_scale=0.70,
            angular_speed=0.45,
            direction_hold_s=0.8,
        )
        base = waypoint_command(0.0, 0.0, -math.pi / 2.0, 0.0, -1.0, 0.40)
        controller.command(
            now_s=1.0,
            base_command=base,
            nominal_speed=0.40,
            avoid_left=0.80,
            avoid_center=0.80,
            avoid_right=0.0,
            allow_start=True,
        )
        for now_s in (1.8, 1.9):
            command = controller.command(
                now_s=now_s,
                base_command=base,
                nominal_speed=0.40,
                avoid_left=0.0,
                avoid_center=0.0,
                avoid_right=0.0,
                allow_start=False,
            )
            self.assertTrue(controller.active)

        command = controller.command(
            now_s=2.0,
            base_command=base,
            nominal_speed=0.40,
            avoid_left=0.0,
            avoid_center=0.0,
            avoid_right=0.0,
            allow_start=False,
        )

        self.assertFalse(controller.active)
        self.assertEqual(command, base)

    def test_storage_avoidance_has_a_maximum_curve_duration(self):
        controller = StorageCurveAvoidanceController(
            danger_threshold=0.20,
            release_threshold=0.12,
            linear_scale=0.70,
            angular_speed=0.45,
            direction_hold_s=0.8,
            max_duration_s=1.5,
        )
        base = waypoint_command(0.0, 0.0, -math.pi / 2.0, 0.0, -1.0, 0.40)
        controller.command(
            now_s=1.0,
            base_command=base,
            nominal_speed=0.40,
            avoid_left=0.80,
            avoid_center=0.80,
            avoid_right=0.0,
            allow_start=True,
        )

        command = controller.command(
            now_s=2.5,
            base_command=base,
            nominal_speed=0.40,
            avoid_left=0.80,
            avoid_center=0.80,
            avoid_right=0.0,
            allow_start=True,
        )

        self.assertFalse(controller.active)
        self.assertEqual(command, base)

    def test_fourth_pickup_triggers_capacity_return(self):
        for index in range(3):
            self.assertIsNone(self.mission.record_pickup(f"object-{index}", 20.0))
        reason = self.mission.record_pickup("object-3", 21.0)
        self.assertEqual(reason, ReturnReason.CAPACITY)
        self.assertEqual(self.mission.phase, MissionPhase.RETURN_MAIN_ROAD)
        self.assertEqual(self.mission.onboard_count, 4)

    def test_thirty_second_rule_does_not_return_empty_robot(self):
        result = self.mission.update_time(160.0)
        self.assertIsNone(result)
        self.assertEqual(self.mission.phase, MissionPhase.COLLECTING)

    def test_thirty_second_return_waits_for_pickup_sequence_to_finish(self):
        self.mission.onboard_objects.append("already-stored")

        result = self.mission.update_time(
            160.0,
            defer_storage_return=True,
        )

        self.assertIsNone(result)
        self.assertEqual(self.mission.phase, MissionPhase.COLLECTING)
        reason = self.mission.record_pickup("current-target", 160.1)
        self.assertEqual(reason, ReturnReason.TIME_LIMIT)
        self.assertTrue(self.mission.is_storage_phase())

    def test_pickup_deferral_does_not_override_mission_timeout(self):
        self.mission.onboard_objects.append("already-stored")

        result = self.mission.update_time(
            190.0,
            defer_storage_return=True,
        )

        self.assertEqual(result, MissionPhase.TIMEOUT)
        self.assertEqual(self.mission.phase, MissionPhase.TIMEOUT)

    def test_storage_return_starts_with_main_road_waypoint(self):
        self.assertEqual(
            storage_return_start_phase(),
            MissionPhase.RETURN_MAIN_ROAD,
        )

    def test_only_lanes_one_and_two_repeat_y_alignment_at_storage(self):
        self.assertEqual(
            storage_phase_after_staging_x(1),
            MissionPhase.CORRECT_STORAGE_STAGING_Y,
        )
        self.assertEqual(
            storage_phase_after_staging_x(2),
            MissionPhase.CORRECT_STORAGE_STAGING_Y,
        )
        self.assertEqual(
            storage_phase_after_staging_x(3),
            MissionPhase.OPEN_STORAGE_ENTRY,
        )
        self.assertEqual(
            storage_phase_after_staging_x(4),
            MissionPhase.OPEN_STORAGE_ENTRY,
        )

    def test_storage_visit_routes_use_distinct_staging_coordinates(self):
        self.assertEqual(storage_visit_number(0), 1)
        self.assertEqual(storage_visit_number(4), 2)
        self.assertEqual(
            storage_staging_coordinates(1, -1.25, -1.70, -1.60, -1.40),
            (-1.25, -1.70),
        )
        self.assertEqual(
            storage_staging_coordinates(2, -1.25, -1.70, -1.60, -1.40),
            (-1.60, -1.40),
        )

    def test_storage_dash_headings_are_selected_by_visit(self):
        first_heading = storage_visit_dash_heading(1, -165.0, -113.0)
        second_heading = storage_visit_dash_heading(2, -165.0, -113.0)

        self.assertAlmostEqual(math.degrees(first_heading), -165.0)
        self.assertAlmostEqual(math.degrees(second_heading), -113.0)

    def test_timed_storage_motion_does_not_require_bounded_xy(self):
        self.assertFalse(storage_pose_bounds_required(MissionPhase.ENTER_STORAGE))
        self.assertFalse(storage_pose_bounds_required(MissionPhase.EXIT_STORAGE))
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.REPUSH_STORAGE)
        )
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.EXIT_STORAGE_REPUSH)
        )
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.SHIFT_STORAGE_SIDE_BACKWARD)
        )
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.SHIFT_STORAGE_SIDE_FORWARD)
        )
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.REPUSH_STORAGE_SIDE)
        )
        self.assertFalse(
            storage_pose_bounds_required(MissionPhase.EXIT_STORAGE_SIDE_REPUSH)
        )
        self.assertTrue(
            storage_pose_bounds_required(MissionPhase.ALIGN_STORAGE_DASH)
        )
        self.assertTrue(
            storage_pose_bounds_required(MissionPhase.RETURN_STAGING)
        )

    def test_slow_closed_gripper_repush_only_runs_on_second_visit(self):
        self.assertFalse(storage_second_repush_required(1))
        self.assertTrue(storage_second_repush_required(2))

    def test_side_shift_is_right_ninety_and_repush_is_mirrored(self):
        entry_heading = storage_visit_dash_heading(2, -165.0, -113.0)
        shift_heading = storage_side_shift_heading(entry_heading)
        repush_heading = storage_mirrored_repush_heading(entry_heading)

        self.assertAlmostEqual(math.degrees(shift_heading), 157.0)
        self.assertAlmostEqual(
            normalize_angle(shift_heading - entry_heading),
            -math.pi / 2.0,
        )
        self.assertAlmostEqual(math.degrees(repush_heading), -157.0)
        self.assertAlmostEqual(
            math.cos(repush_heading),
            math.sin(entry_heading),
        )
        self.assertAlmostEqual(
            math.sin(repush_heading),
            math.cos(entry_heading),
        )
        right_turn = waypoint_command(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=entry_heading,
            target_x=0.0,
            target_y=0.0,
            speed=0.0,
            final_yaw=shift_heading,
        )
        left_turn = waypoint_command(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=shift_heading,
            target_x=0.0,
            target_y=0.0,
            speed=0.0,
            final_yaw=entry_heading,
        )
        mirrored_right_turn = waypoint_command(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=entry_heading,
            target_x=0.0,
            target_y=0.0,
            speed=0.0,
            final_yaw=repush_heading,
        )
        self.assertLess(right_turn.angular_z, 0.0)
        self.assertGreater(left_turn.angular_z, 0.0)
        self.assertLess(mirrored_right_turn.angular_z, 0.0)

    def test_pickup_inside_final_thirty_seconds_returns_immediately(self):
        reason = self.mission.record_pickup("target", 161.0)
        self.assertEqual(reason, ReturnReason.TIME_LIMIT)
        self.assertTrue(self.mission.is_storage_phase())

    def test_storage_tof_correction_phases_remain_storage_phases(self):
        self.mission.set_phase(MissionPhase.REJOIN_STORAGE_LANE, 19.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_MAIN_ROAD_SOUTH, 19.5)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_X, 20.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_STAGING_X, 20.5)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_STAGING_Y, 20.8)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_Y, 21.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_EXIT_X, 21.5)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_ENTRY, 22.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.OPEN_STORAGE_ENTRY, 23.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_DASH, 23.5)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_EXIT_WEST, 23.8)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CLOSE_STORAGE_EXIT, 24.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.RETURN_FROM_STORAGE, 25.0)
        self.assertTrue(self.mission.is_storage_phase())

    def test_deposit_then_resume_until_seven_objects_are_delivered(self):
        for index in range(4):
            self.mission.record_pickup(f"first-{index}", 20.0)
        self.mission.record_deposit(30.0)
        self.assertEqual(self.mission.delivered_count, 4)
        self.assertEqual(self.mission.onboard_count, 0)
        self.assertEqual(
            self.mission.finish_storage_exit(35.0),
            MissionPhase.COLLECTING,
        )

        for index in range(3):
            self.mission.record_pickup(f"second-{index}", 50.0)
        self.assertEqual(self.mission.return_reason, ReturnReason.TARGET_COUNT)
        self.mission.record_deposit(60.0)
        self.assertEqual(
            self.mission.finish_storage_exit(65.0),
            MissionPhase.COMPLETE,
        )
        self.assertEqual(self.mission.delivered_count, 7)

    def test_mission_times_out_at_three_minutes(self):
        self.assertEqual(self.mission.update_time(190.0), MissionPhase.TIMEOUT)
        self.assertEqual(self.mission.phase, MissionPhase.TIMEOUT)

    def test_waypoint_controller_aligns_before_driving(self):
        command = waypoint_command(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=math.pi / 2.0,
            target_x=1.0,
            target_y=0.0,
            speed=0.2,
        )
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.reached)

    def test_storage_entry_and_road_return_turn_clockwise(self):
        west = waypoint_command(
            -1.25, -1.75, -math.pi / 2.0, -1.25, -1.75, 0.0, final_yaw=math.pi
        )
        north = waypoint_command(-1.25, -1.75, math.pi, -1.25, -1.40, 0.25)

        self.assertEqual((west.linear_x, north.linear_x), (0.0, 0.0))
        self.assertLess(west.angular_z, 0.0)
        self.assertLess(north.angular_z, 0.0)

    def test_reverse_storage_exit_stops_at_x_minus_1_25(self):
        moving = reverse_storage_x_exit_command(
            -1.75, math.pi, -1.25, math.pi, 0.40, x_tolerance=0.04
        )
        stopped = reverse_storage_x_exit_command(
            -1.24, math.pi, -1.25, math.pi, 0.40, x_tolerance=0.04
        )

        self.assertLess(moving.linear_x, 0.0)
        self.assertAlmostEqual(moving.angular_z, 0.0)
        self.assertTrue(stopped.reached)

    def test_negative_waypoint_speed_reverses_along_the_entry_path(self):
        entry_heading = math.atan2(-1.75 + 1.40, -1.75 + 1.25)
        command = waypoint_command(
            robot_x=-1.75,
            robot_y=-1.75,
            robot_yaw=entry_heading,
            target_x=-1.25,
            target_y=-1.40,
            speed=-0.40,
        )

        self.assertLess(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0)
        self.assertFalse(command.reached)

    def test_fast_diagonal_entry_and_reverse_return_to_main_road_staging(self):
        staging = (-1.25, -1.40)
        center = (-1.75, -1.75)
        entered = simulate_waypoint(
            (staging[0], staging[1], math.pi),
            center,
            speed=0.40,
        )
        returned = simulate_waypoint(
            entered,
            staging,
            speed=-0.40,
        )

        self.assertLess(
            math.hypot(entered[0] - center[0], entered[1] - center[1]),
            0.04,
        )
        self.assertLess(
            math.hypot(returned[0] - staging[0], returned[1] - staging[1]),
            0.04,
        )

    def test_storage_dash_heading_uses_configured_fixed_angle(self):
        heading = storage_dash_heading(-139.26)

        self.assertAlmostEqual(math.degrees(heading), -139.26, places=3)

    def test_fixed_heading_dash_does_not_stop_to_chase_pose(self):
        desired_yaw = storage_dash_heading(-139.26)
        command = fixed_heading_dash_command(
            robot_yaw=desired_yaw + 0.10,
            desired_yaw=desired_yaw,
            speed=0.40,
            elapsed_s=0.50,
            duration_s=2.50,
            heading_gain=1.5,
            max_angular_speed=0.30,
        )

        self.assertAlmostEqual(command.linear_x, 0.40)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.reached)

    def test_storage_exit_rotates_to_west_before_tof(self):
        dash_yaw = storage_dash_heading(-139.26)
        command = waypoint_command(
            robot_x=-1.25,
            robot_y=-1.40,
            robot_yaw=dash_yaw,
            target_x=-1.25,
            target_y=-1.40,
            speed=0.0,
            waypoint_tolerance=0.04,
            heading_tolerance=0.14,
            heading_gain=1.5,
            max_angular_speed=0.60,
            final_yaw=math.pi,
            final_yaw_tolerance=0.12,
        )

        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertFalse(command.reached)

        aligned = waypoint_command(
            robot_x=-1.25,
            robot_y=-1.40,
            robot_yaw=math.pi,
            target_x=-1.25,
            target_y=-1.40,
            speed=0.0,
            waypoint_tolerance=0.04,
            final_yaw=math.pi,
            final_yaw_tolerance=0.12,
        )
        self.assertTrue(aligned.reached)

    def test_fixed_heading_reverse_uses_same_heading_until_timer_finishes(self):
        desired_yaw = storage_dash_heading(-139.26)
        reversing = fixed_heading_dash_command(
            robot_yaw=desired_yaw,
            desired_yaw=desired_yaw,
            speed=-0.40,
            elapsed_s=1.45,
            duration_s=1.50,
        )
        stopped = fixed_heading_dash_command(
            robot_yaw=desired_yaw,
            desired_yaw=desired_yaw,
            speed=-0.40,
            elapsed_s=1.50,
            duration_s=1.50,
        )

        self.assertAlmostEqual(reversing.linear_x, -0.40)
        self.assertAlmostEqual(reversing.angular_z, 0.0)
        self.assertFalse(reversing.reached)
        self.assertEqual((stopped.linear_x, stopped.angular_z), (0.0, 0.0))
        self.assertTrue(stopped.reached)


if __name__ == "__main__":
    unittest.main()
