import math
import unittest

from rl_model_policy.observation import (
    OBSERVATION_DIM,
    estimate_target_world_bearing,
    make_pose_observation,
    pose_is_usable,
    quaternion_to_yaw,
    validate_observation,
)


class ObservationTest(unittest.TestCase):
    def test_pose_observation_matches_18_input_contract(self):
        pose = make_pose_observation(
            pose_valid=True,
            robot_x=1.0,
            robot_y=-1.0,
            yaw=math.pi / 2.0,
            yaw_rate=0.4,
            last_target_world_bearing=math.pi,
            arena_half_extent_m=2.0,
            max_angular_speed=0.8,
        )

        expected = [1.0, 0.5, -0.5, 1.0, 0.0, 0.5, 1.0, 0.0]
        for actual, wanted in zip(pose, expected):
            self.assertAlmostEqual(actual, wanted)
        self.assertEqual(len(validate_observation([0.0] * 10 + pose)), OBSERVATION_DIM)

    def test_invalid_pose_zeros_all_pose_inputs(self):
        pose = make_pose_observation(False, 1.0, 1.0, 0.4, 0.2, 0.7, 2.0, 0.8)
        self.assertEqual(pose, [0.0] * 8)

    def test_target_bearing_uses_ros_and_image_sign_conventions(self):
        bearing = estimate_target_world_bearing(0.0, -1.0, math.pi / 2.0)
        self.assertAlmostEqual(bearing, math.pi / 4.0)

    def test_quaternion_to_yaw(self):
        yaw = math.pi / 3.0
        actual = quaternion_to_yaw(
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )
        self.assertAlmostEqual(actual, yaw)

    def test_observation_length_is_enforced(self):
        with self.assertRaisesRegex(ValueError, "Expected 18 observations"):
            validate_observation([0.0] * 10)

    def test_pose_outside_arena_is_not_usable(self):
        self.assertTrue(pose_is_usable(1.9, -1.9, 2.0, 0.25))
        self.assertFalse(pose_is_usable(2.8, 0.9, 2.0, 0.25))


if __name__ == "__main__":
    unittest.main()
