import math
import unittest

from wall_distance_sensor.wall_geometry import calculate_wall_measurement


class WallGeometryTest(unittest.TestCase):
    def test_equal_ranges_mean_perpendicular_wall(self):
        measurement = calculate_wall_measurement(0.5, 0.5, 0.29)

        self.assertAlmostEqual(measurement.distance_m, 0.5)
        self.assertAlmostEqual(measurement.angle_rad, 0.0)
        self.assertAlmostEqual(measurement.min_distance_m, 0.5)

    def test_right_sensor_farther_gives_positive_angle(self):
        measurement = calculate_wall_measurement(0.4, 0.5, 0.29)

        self.assertGreater(measurement.angle_rad, 0.0)
        self.assertAlmostEqual(
            measurement.angle_rad,
            math.atan2(0.1, 0.29),
        )
        self.assertAlmostEqual(measurement.min_distance_m, 0.4)

    def test_rejects_invalid_geometry(self):
        with self.assertRaises(ValueError):
            calculate_wall_measurement(float("nan"), 0.5, 0.29)
        with self.assertRaises(ValueError):
            calculate_wall_measurement(0.5, 0.5, 0.0)
        with self.assertRaises(ValueError):
            calculate_wall_measurement(-0.1, 0.5, 0.29)


if __name__ == "__main__":
    unittest.main()
