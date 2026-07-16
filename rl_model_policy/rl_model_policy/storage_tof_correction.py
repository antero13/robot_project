from dataclasses import dataclass
import math
from typing import Optional

from rl_model_policy.coverage_controller import clamp, normalize_angle


@dataclass(frozen=True)
class StorageTofCommand:
    linear_x: float
    angular_z: float
    phase: str
    measured_coordinate: Optional[float]
    coordinate_error: Optional[float]
    reached: bool


def robot_coordinate_from_min_wall_distance(
    distance_m,
    wall_coordinate_m,
    sensor_forward_offset_m,
):
    """Convert forward range to robot-center x/y from a minimum arena wall."""
    distance_m = float(distance_m)
    wall_coordinate_m = float(wall_coordinate_m)
    sensor_forward_offset_m = float(sensor_forward_offset_m)
    if not all(
        math.isfinite(value)
        for value in (
            distance_m,
            wall_coordinate_m,
            sensor_forward_offset_m,
        )
    ):
        raise ValueError("wall-distance geometry values must be finite")
    if distance_m <= 0.0:
        raise ValueError("wall distance must be positive")
    if sensor_forward_offset_m < 0.0:
        raise ValueError("sensor_forward_offset_m cannot be negative")
    return wall_coordinate_m + distance_m + sensor_forward_offset_m


def measurement_gap_timed_out(missing_started_at_s, now_s, timeout_s):
    """Return true after one continuous interval without a usable measurement."""
    if missing_started_at_s is None:
        return False
    missing_started_at_s = float(missing_started_at_s)
    now_s = float(now_s)
    timeout_s = float(timeout_s)
    if not all(math.isfinite(v) for v in (missing_started_at_s, now_s, timeout_s)):
        raise ValueError("measurement timeout values must be finite")
    if timeout_s < 0.0:
        raise ValueError("measurement timeout cannot be negative")
    return max(0.0, now_s - missing_started_at_s) >= timeout_s


def make_storage_tof_command(
    *,
    axis,
    distance_m,
    measurement_age_s,
    robot_yaw,
    target_coordinate,
    wall_coordinate_m,
    sensor_forward_offset_m,
    transit_speed,
    minimum_speed,
    slowdown_distance_m,
    coordinate_tolerance_m,
    measurement_timeout_s,
    heading_gain,
    max_angular_speed,
    heading_tolerance,
    advance_without_measurement=False,
):
    """Align one storage-staging axis using the left or bottom arena wall."""
    axis = str(axis).strip().lower()
    if axis == "x":
        desired_yaw = math.pi
        axis_name = "X"
    elif axis == "y":
        desired_yaw = -math.pi / 2.0
        axis_name = "Y"
    else:
        raise ValueError("axis must be 'x' or 'y'")

    heading_error = normalize_angle(desired_yaw - float(robot_yaw))
    angular_z = clamp(
        float(heading_gain) * heading_error,
        -float(max_angular_speed),
        float(max_angular_speed),
    )
    if abs(heading_error) > float(heading_tolerance):
        return StorageTofCommand(
            linear_x=0.0,
            angular_z=angular_z,
            phase=f"ALIGN_STORAGE_TOF_{axis_name}",
            measured_coordinate=None,
            coordinate_error=None,
            reached=False,
        )

    measurement_is_fresh = (
        distance_m is not None
        and measurement_age_s is not None
        and math.isfinite(float(distance_m))
        and math.isfinite(float(measurement_age_s))
        and float(distance_m) > 0.0
        and 0.0 <= float(measurement_age_s) <= float(measurement_timeout_s)
    )
    if not measurement_is_fresh:
        if bool(advance_without_measurement):
            return StorageTofCommand(
                linear_x=float(transit_speed),
                angular_z=angular_z,
                phase=f"APPROACH_STORAGE_TOF_{axis_name}",
                measured_coordinate=None,
                coordinate_error=None,
                reached=False,
            )
        return StorageTofCommand(
            linear_x=0.0,
            angular_z=angular_z,
            phase=f"WAITING_FOR_STORAGE_TOF_{axis_name}",
            measured_coordinate=None,
            coordinate_error=None,
            reached=False,
        )

    measured_coordinate = robot_coordinate_from_min_wall_distance(
        distance_m,
        wall_coordinate_m,
        sensor_forward_offset_m,
    )
    coordinate_error = measured_coordinate - float(target_coordinate)
    if abs(coordinate_error) <= float(coordinate_tolerance_m):
        return StorageTofCommand(
            linear_x=0.0,
            angular_z=0.0,
            phase=f"STORAGE_TOF_{axis_name}_ALIGNED",
            measured_coordinate=measured_coordinate,
            coordinate_error=coordinate_error,
            reached=True,
        )

    slowdown_distance_m = max(
        float(slowdown_distance_m),
        float(coordinate_tolerance_m),
    )
    speed_scale = clamp(abs(coordinate_error) / slowdown_distance_m, 0.0, 1.0)
    speed = max(float(minimum_speed), float(transit_speed) * speed_scale)
    # Both reference walls are the minimum-coordinate walls. While facing the
    # wall, positive base velocity decreases the corresponding map coordinate.
    linear_x = math.copysign(speed, coordinate_error)
    return StorageTofCommand(
        linear_x=linear_x,
        angular_z=angular_z,
        phase=f"TOF_CORRECT_STORAGE_{axis_name}",
        measured_coordinate=measured_coordinate,
        coordinate_error=coordinate_error,
        reached=False,
    )
