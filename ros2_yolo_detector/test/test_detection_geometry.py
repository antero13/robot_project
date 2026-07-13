from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros2_yolo_detector.detection_geometry import bbox_to_normalized_point


class DetectionGeometryTest(unittest.TestCase):
    def test_uses_bounding_box_center_for_policy_coordinates(self):
        point = bbox_to_normalized_point(160, 120, 320, 360, 640, 480)

        self.assertAlmostEqual(point.x, -0.25)
        self.assertAlmostEqual(point.y, 0.50)
        self.assertAlmostEqual(point.bottom_y, 0.75)

    def test_clamps_boxes_that_extend_outside_the_image(self):
        point = bbox_to_normalized_point(-100, -50, 800, 600, 640, 480)

        self.assertGreaterEqual(point.x, -1.0)
        self.assertLessEqual(point.x, 1.0)
        self.assertGreaterEqual(point.y, 0.0)
        self.assertLessEqual(point.y, 1.0)
        self.assertEqual(point.bottom_y, 1.0)

    def test_rejects_invalid_image_or_box_dimensions(self):
        invalid_arguments = [
            (0, 0, 10, 10, 0, 480),
            (0, 0, 10, 10, 640, 0),
            (10, 0, 10, 10, 640, 480),
            (0, 10, 10, 10, 640, 480),
        ]
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    bbox_to_normalized_point(*arguments)


if __name__ == "__main__":
    unittest.main()
