import math
import unittest

from rl_model_policy.mission_coordinator import (
    MissionCoordinator,
    MissionPhase,
    ReturnReason,
    reverse_exit_command,
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

    def test_pickup_inside_final_thirty_seconds_returns_immediately(self):
        reason = self.mission.record_pickup("target", 161.0)
        self.assertEqual(reason, ReturnReason.TIME_LIMIT)
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

    def test_reverse_exit_stops_after_crossing_staging_y(self):
        moving = reverse_exit_command(-1.7, -math.pi / 2.0, -1.25, -math.pi / 2.0, 0.1)
        stopped = reverse_exit_command(-1.2, -math.pi / 2.0, -1.25, -math.pi / 2.0, 0.1)
        self.assertLess(moving.linear_x, 0.0)
        self.assertTrue(stopped.reached)


if __name__ == "__main__":
    unittest.main()
