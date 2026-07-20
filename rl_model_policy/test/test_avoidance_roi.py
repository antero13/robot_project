import unittest

from rl_model_policy.avoidance_roi import (
    interpolate_roi_x,
    point_is_inside_trapezoid_roi,
)


CALIBRATED_ROI = {
    "left_far_x": -0.2649,
    "left_far_y": 0.2576,
    "left_near_x": -0.6563,
    "left_near_y": 0.7483,
    "right_far_x": 0.0951,
    "right_far_y": 0.2567,
    "right_near_x": 0.4620,
    "right_near_y": 0.6992,
}


class AvoidanceRoiTest(unittest.TestCase):
    def test_accepts_points_in_calibrated_robot_corridor(self):
        self.assertTrue(point_is_inside_trapezoid_roi(0.0, 0.30, **CALIBRATED_ROI))
        self.assertTrue(point_is_inside_trapezoid_roi(0.0, 0.50, **CALIBRATED_ROI))

    def test_rejects_points_beside_or_beyond_corridor(self):
        self.assertFalse(point_is_inside_trapezoid_roi(-0.70, 0.50, **CALIBRATED_ROI))
        self.assertFalse(point_is_inside_trapezoid_roi(0.50, 0.50, **CALIBRATED_ROI))
        self.assertFalse(point_is_inside_trapezoid_roi(0.0, 0.20, **CALIBRATED_ROI))
        self.assertFalse(point_is_inside_trapezoid_roi(0.0, 0.80, **CALIBRATED_ROI))

    def test_includes_interpolated_side_boundaries(self):
        y = 0.50
        left_x = interpolate_roi_x(
            y,
            CALIBRATED_ROI["left_far_x"],
            CALIBRATED_ROI["left_far_y"],
            CALIBRATED_ROI["left_near_x"],
            CALIBRATED_ROI["left_near_y"],
        )
        self.assertTrue(point_is_inside_trapezoid_roi(left_x, y, **CALIBRATED_ROI))
        self.assertFalse(
            point_is_inside_trapezoid_roi(left_x - 0.001, y, **CALIBRATED_ROI)
        )

    def test_rejects_non_finite_detection_coordinates(self):
        self.assertFalse(
            point_is_inside_trapezoid_roi(float("nan"), 0.5, **CALIBRATED_ROI)
        )


if __name__ == "__main__":
    unittest.main()
