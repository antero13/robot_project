from dataclasses import dataclass
import math
from typing import Optional

from rl_model_policy.coverage_controller import clamp, normalize_angle
from rl_model_policy.storage_tof_correction import (
    robot_coordinate_from_min_wall_distance,
)


@dataclass(frozen=True)
class StorageExitTofCommand:
    linear_x: float
    angular_z: float
    phase: str
    measured_robot_x: Optional[float]
    x_error: Optional[float]
    angle_alignment_active: bool
    reached: bool


def make_storage_exit_tof_command(
    *,
    distance_m,
    wall_angle_rad,
    measurement_age_s,
    robot_yaw,
    target_x,
    west_wall_x_m,
    sensor_forward_offset_m,
    transit_speed,
    minimum_speed,
    slowdown_distance_m,
    x_tolerance_m,
    measurement_timeout_s,
    angle_alignment_active,
    angle_trigger_rad,
    angle_release_rad,
    angle_gain,
    max_angular_speed,
    heading_tolerance,
    coarse_heading_aligned=False,
    coarse_heading_gain=None,
    coarse_max_angular_speed=None,
):
    """Correct storage-exit x first, then conditionally square to west wall."""
    angle_trigger_rad = float(angle_trigger_rad)
    angle_release_rad = float(angle_release_rad)
    if not 0.0 <= angle_release_rad < angle_trigger_rad:
        raise ValueError("angle thresholds must satisfy 0 <= release < trigger")
    if coarse_heading_gain is None:
        coarse_heading_gain = angle_gain
    if coarse_max_angular_speed is None:
        coarse_max_angular_speed = max_angular_speed

    heading_error = normalize_angle(math.pi - float(robot_yaw))
    if (
        not bool(coarse_heading_aligned)
        and abs(heading_error) > float(heading_tolerance)
    ):
        return StorageExitTofCommand(
            linear_x=0.0,
            angular_z=clamp(
                float(coarse_heading_gain) * heading_error,
                -float(coarse_max_angular_speed),
                float(coarse_max_angular_speed),
            ),
            phase="ALIGN_STORAGE_EXIT_WEST_ODOMETRY",
            measured_robot_x=None,
            x_error=None,
            angle_alignment_active=False,
            reached=False,
        )

    measurement_is_fresh = (
        distance_m is not None
        and wall_angle_rad is not None
        and measurement_age_s is not None
        and math.isfinite(float(distance_m))
        and math.isfinite(float(wall_angle_rad))
        and math.isfinite(float(measurement_age_s))
        and float(distance_m) > 0.0
        and 0.0 <= float(measurement_age_s) <= float(measurement_timeout_s)
    )
    if not measurement_is_fresh:
        return StorageExitTofCommand(
            linear_x=0.0,
            angular_z=0.0,
            phase="WAITING_FOR_STORAGE_TOF_X",
            measured_robot_x=None,
            x_error=None,
            angle_alignment_active=False,
            reached=False,
        )

    measured_robot_x = robot_coordinate_from_min_wall_distance(
        distance_m,
        west_wall_x_m,
        sensor_forward_offset_m,
    )
    x_error = measured_robot_x - float(target_x)

    # The robot is looking toward the west wall. Positive base velocity lowers
    # map x. Keep angular velocity at zero until distance correction finishes.
    if abs(x_error) > float(x_tolerance_m):
        slowdown_distance_m = max(
            float(slowdown_distance_m),
            float(x_tolerance_m),
        )
        speed_scale = clamp(abs(x_error) / slowdown_distance_m, 0.0, 1.0)
        speed = max(float(minimum_speed), float(transit_speed) * speed_scale)
        return StorageExitTofCommand(
            linear_x=math.copysign(speed, x_error),
            angular_z=0.0,
            phase="TOF_CORRECT_STORAGE_EXIT_DISTANCE",
            measured_robot_x=measured_robot_x,
            x_error=x_error,
            angle_alignment_active=False,
            reached=False,
        )

    wall_angle_rad = float(wall_angle_rad)
    should_align_angle = bool(angle_alignment_active) or (
        abs(wall_angle_rad) >= angle_trigger_rad
    )
    if should_align_angle and abs(wall_angle_rad) > angle_release_rad:
        return StorageExitTofCommand(
            linear_x=0.0,
            angular_z=clamp(
                float(angle_gain) * wall_angle_rad,
                -float(max_angular_speed),
                float(max_angular_speed),
            ),
            phase="ALIGN_STORAGE_EXIT_WEST_WALL_ANGLE",
            measured_robot_x=measured_robot_x,
            x_error=x_error,
            angle_alignment_active=True,
            reached=False,
        )

    return StorageExitTofCommand(
        linear_x=0.0,
        angular_z=0.0,
        phase="STORAGE_EXIT_TOF_ALIGNED",
        measured_robot_x=measured_robot_x,
        x_error=x_error,
        angle_alignment_active=False,
        reached=True,
    )
