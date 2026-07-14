import csv
import math
from pathlib import Path
import unittest

from rl_model_policy.object_localization import CalibrationObjectLocalizer


CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "distance_normalized_points.csv"
)


class CalibrationObjectLocalizerTest(unittest.TestCase):
    def setUp(self):
        self.localizer = CalibrationObjectLocalizer(CALIBRATION_PATH)

    def test_recovers_all_measured_camera_positions(self):
        with CALIBRATION_PATH.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))

        self.assertEqual(len(rows), 51)
        for row in rows:
            with self.subTest(row=row):
                estimate = self.localizer.interpolate_camera_position(
                    float(row["norm_x"]),
                    float(row["norm_y"]),
                )
                self.assertIsNotNone(estimate)
                lateral_m, forward_m, _ = estimate
                self.assertAlmostEqual(lateral_m, float(row["real_x_m"]), places=6)
                self.assertAlmostEqual(forward_m, float(row["distance_m"]), places=6)

    def test_interpolates_between_measured_distances(self):
        estimate = self.localizer.interpolate_camera_position(
            image_x=0.0,
            image_y=(0.30038 + 0.269279) * 0.5,
        )

        lateral_m, forward_m, span_m = estimate
        self.assertAlmostEqual(lateral_m, 0.0, places=6)
        self.assertAlmostEqual(forward_m, 0.65, places=6)
        self.assertAlmostEqual(span_m, 0.1, places=6)

    def test_world_transform_uses_camera_right_axis(self):
        estimate = self.localizer.localize(
            image_x=0.32389,
            image_y=0.353612,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=math.pi / 2.0,
        )

        self.assertAlmostEqual(estimate.forward_m, 0.5, places=6)
        self.assertAlmostEqual(estimate.lateral_m, 0.2, places=6)
        self.assertAlmostEqual(estimate.x, 0.2, places=6)
        self.assertAlmostEqual(estimate.y, 0.5, places=6)

    def test_bbox_center_outside_calibrated_range_is_rejected(self):
        estimate = self.localizer.interpolate_camera_position(
            image_x=0.0,
            image_y=0.75,
        )

        self.assertIsNone(estimate)

    def test_world_position_outside_arena_is_rejected(self):
        estimate = self.localizer.localize(
            image_x=0.0,
            image_y=0.143219,
            robot_x=1.0,
            robot_y=1.0,
            robot_yaw=0.0,
        )

        self.assertIsNone(estimate)


if __name__ == "__main__":
    unittest.main()
