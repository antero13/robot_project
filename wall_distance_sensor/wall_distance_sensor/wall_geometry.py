from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WallMeasurement:
    distance_m: float
    angle_rad: float
    min_distance_m: float


def calculate_wall_measurement(left_distance_m, right_distance_m, separation_m):
    left_distance_m = float(left_distance_m)
    right_distance_m = float(right_distance_m)
    separation_m = float(separation_m)
    if not all(
        math.isfinite(value)
        for value in (left_distance_m, right_distance_m, separation_m)
    ):
        raise ValueError("wall geometry values must be finite")
    if left_distance_m <= 0.0 or right_distance_m <= 0.0:
        raise ValueError("wall distances must be positive")
    if separation_m <= 0.0:
        raise ValueError("sensor separation must be positive")

    angle_rad = math.atan2(
        right_distance_m - left_distance_m,
        separation_m,
    )
    distance_m = (
        0.5 * (left_distance_m + right_distance_m) * math.cos(angle_rad)
    )
    return WallMeasurement(
        distance_m=distance_m,
        angle_rad=angle_rad,
        min_distance_m=min(left_distance_m, right_distance_m),
    )
