import math
import unittest

from robot_pose_tracker.pose_estimator import PoseEstimator, normalize_angle


class PoseEstimatorTest(unittest.TestCase):
    def test_drives_straight_for_one_meter(self):
        estimator = PoseEstimator()

        for _ in range(10):
            estimator.step(0.1, 1.0, 0.0)

        self.assertAlmostEqual(estimator.x, 1.0)
        self.assertAlmostEqual(estimator.y, 0.0)
        self.assertAlmostEqual(estimator.yaw, 0.0)

    def test_initial_heading_rotates_translation_into_map_frame(self):
        estimator = PoseEstimator(yaw=math.pi / 2.0)

        estimator.step(1.0, 1.0, 0.0)

        self.assertAlmostEqual(estimator.x, 0.0)
        self.assertAlmostEqual(estimator.y, 1.0)

    def test_integrates_rotation_and_wraps_angle(self):
        estimator = PoseEstimator(yaw=math.radians(170.0))

        estimator.step(1.0, 0.0, math.radians(30.0))

        self.assertAlmostEqual(estimator.yaw, math.radians(-160.0))
        self.assertAlmostEqual(
            estimator.rotation_travelled,
            math.radians(30.0),
        )

    def test_reset_restores_requested_pose_and_counters(self):
        estimator = PoseEstimator()
        estimator.step(1.0, 1.0, 0.5)

        estimator.reset(3.8, 0.2, math.pi)

        self.assertAlmostEqual(estimator.x, 3.8)
        self.assertAlmostEqual(estimator.y, 0.2)
        self.assertAlmostEqual(estimator.yaw, -math.pi)
        self.assertEqual(estimator.distance_travelled, 0.0)
        self.assertEqual(estimator.rotation_travelled, 0.0)

    def test_correct_x_preserves_y_yaw_and_travel_counters(self):
        estimator = PoseEstimator(x=1.0, y=-0.5, yaw=0.7)
        estimator.step(0.5, 0.2, 0.1)
        previous_y = estimator.y
        previous_yaw = estimator.yaw
        previous_distance = estimator.distance_travelled
        previous_rotation = estimator.rotation_travelled

        estimator.correct_x(0.25)

        self.assertAlmostEqual(estimator.x, 0.25)
        self.assertAlmostEqual(estimator.y, previous_y)
        self.assertAlmostEqual(estimator.yaw, previous_yaw)
        self.assertAlmostEqual(estimator.distance_travelled, previous_distance)
        self.assertAlmostEqual(estimator.rotation_travelled, previous_rotation)

    def test_correct_x_rejects_non_finite_values(self):
        estimator = PoseEstimator(x=1.0)

        with self.assertRaises(ValueError):
            estimator.correct_x(float('nan'))

        self.assertEqual(estimator.x, 1.0)

    def test_correct_y_preserves_x_yaw_and_travel_counters(self):
        estimator = PoseEstimator(x=1.0, y=-0.5, yaw=0.7)
        estimator.step(0.5, 0.2, 0.1)
        previous_x = estimator.x
        previous_yaw = estimator.yaw
        previous_distance = estimator.distance_travelled
        previous_rotation = estimator.rotation_travelled

        estimator.correct_y(-1.25)

        self.assertAlmostEqual(estimator.x, previous_x)
        self.assertAlmostEqual(estimator.y, -1.25)
        self.assertAlmostEqual(estimator.yaw, previous_yaw)
        self.assertAlmostEqual(estimator.distance_travelled, previous_distance)
        self.assertAlmostEqual(estimator.rotation_travelled, previous_rotation)

    def test_correct_y_rejects_non_finite_values(self):
        estimator = PoseEstimator(y=-0.5)

        with self.assertRaises(ValueError):
            estimator.correct_y(float('inf'))

        self.assertEqual(estimator.y, -0.5)

    def test_normalize_angle_uses_ros_yaw_range(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)


if __name__ == '__main__':
    unittest.main()
