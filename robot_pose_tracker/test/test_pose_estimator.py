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

    def test_normalize_angle_uses_ros_yaw_range(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)


if __name__ == '__main__':
    unittest.main()
