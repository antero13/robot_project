import math
import unittest

from rl_model_policy.object_localization import (
    GridObjectLocalizer,
    generate_candidate_points,
)


class GridObjectLocalizerTest(unittest.TestCase):
    def setUp(self):
        self.localizer = GridObjectLocalizer()

    def test_generates_all_42_competition_points(self):
        points = generate_candidate_points()

        self.assertEqual(len(points), 42)
        self.assertEqual((points[0].x, points[0].y), (-1.5, -1.0))
        self.assertEqual((points[-1].x, points[-1].y), (1.5, 1.5))

    def test_recovers_visible_candidate_from_predicted_center(self):
        observed = self.localizer.predict_observation(
            object_x=0.0,
            object_y=-1.0,
            robot_x=0.0,
            robot_y=-1.8,
            robot_yaw=math.pi / 2.0,
        )

        estimate = self.localizer.localize(
            observed[0],
            observed[1],
            robot_x=0.0,
            robot_y=-1.8,
            robot_yaw=math.pi / 2.0,
        )

        self.assertEqual(estimate.method, "candidate_grid")
        self.assertEqual((estimate.x, estimate.y), (0.0, -1.0))

    def test_image_right_is_negative_ros_bearing(self):
        observed = self.localizer.predict_observation(
            object_x=0.5,
            object_y=-1.0,
            robot_x=0.0,
            robot_y=-1.8,
            robot_yaw=math.pi / 2.0,
        )

        self.assertGreater(observed[0], 0.0)

    def test_candidate_behind_camera_is_rejected(self):
        observed = self.localizer.predict_observation(
            object_x=0.0,
            object_y=-1.0,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=math.pi / 2.0,
        )

        self.assertIsNone(observed)

    def test_invalid_camera_height_is_rejected(self):
        with self.assertRaises(ValueError):
            GridObjectLocalizer(
                camera_height_m=0.04,
                object_center_height_m=0.04,
            )


if __name__ == "__main__":
    unittest.main()
