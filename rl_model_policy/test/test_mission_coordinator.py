import math
import unittest

from rl_model_policy.mission_coordinator import (
    MissionCoordinator,
    MissionPhase,
    ReturnReason,
    reverse_storage_x_exit_command,
    storage_return_start_phase,
    waypoint_command,
)


class MissionCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.mission = MissionCoordinator(
            storage_capacity=4,
            target_object_count=7,
            mission_duration_s=180.0,
            force_return_remaining_s=30.0,
        )
        self.mission.start(10.0)

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

    def test_storage_return_starts_with_direct_y_descent(self):
        self.assertEqual(
            storage_return_start_phase(True),
            MissionPhase.CORRECT_STORAGE_Y,
        )
        self.assertEqual(
            storage_return_start_phase(False),
            MissionPhase.MOVE_TO_STORAGE_Y,
        )

    def test_pickup_inside_final_thirty_seconds_returns_immediately(self):
        reason = self.mission.record_pickup("target", 161.0)
        self.assertEqual(reason, ReturnReason.TIME_LIMIT)
        self.assertTrue(self.mission.is_storage_phase())

    def test_storage_tof_correction_phases_remain_storage_phases(self):
        self.mission.set_phase(MissionPhase.REJOIN_STORAGE_LANE, 19.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_X, 20.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.CORRECT_STORAGE_Y, 21.0)
        self.assertTrue(self.mission.is_storage_phase())
        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_ENTRY, 22.0)
        self.assertTrue(self.mission.is_storage_phase())

        self.mission.set_phase(MissionPhase.OPEN_STORAGE_ENTRY, 23.0)
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
        north = waypoint_command(-1.25, -1.75, math.pi, -1.25, -1.3343, 0.25)

        self.assertEqual((west.linear_x, north.linear_x), (0.0, 0.0))
        self.assertLess(west.angular_z, 0.0)
        self.assertLess(north.angular_z, 0.0)

    def test_reverse_storage_exit_stops_at_x_minus_1_25(self):
        moving = reverse_storage_x_exit_command(
            -1.75, math.pi, -1.25, math.pi, 0.25, x_tolerance=0.04
        )
        stopped = reverse_storage_x_exit_command(
            -1.24, math.pi, -1.25, math.pi, 0.25, x_tolerance=0.04
        )

        self.assertLess(moving.linear_x, 0.0)
        self.assertAlmostEqual(moving.angular_z, 0.0)
        self.assertTrue(stopped.reached)


if __name__ == "__main__":
    unittest.main()
