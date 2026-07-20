from dataclasses import dataclass
import math
from typing import Optional

from rl_model_policy.coverage_controller import clamp, normalize_angle


@dataclass(frozen=True)
class LaneTofCommand:
    linear_x: float
    angular_z: float
    phase: str
    measured_robot_x: Optional[float]
    x_error: Optional[float]
    reached: bool


def should_run_lane_tof_fine_alignment(
    *,
    enabled,
    leg_phase,
    waypoint_reached,
    alignment_active,
):
    """Use ToF only after odometry reaches a lane-shift waypoint."""
    return (
        bool(enabled)
        and str(leg_phase) == "SHIFT_TO_NEXT_LANE"
        and (bool(alignment_active) or bool(waypoint_reached))
    )


def desired_yaw_for_wall(wall_side):
    wall_side = str(wall_side).strip().lower()
    if wall_side == "left":
        return math.pi
    if wall_side == "right":
        return 0.0
    raise ValueError("wall_side must be 'left' or 'right'")


def coarse_heading_is_aligned(robot_yaw, wall_side, heading_tolerance):
    desired_yaw = desired_yaw_for_wall(wall_side)
    heading_error = normalize_angle(desired_yaw - float(robot_yaw))
    return abs(heading_error) <= float(heading_tolerance)


def robot_x_from_left_wall_distance(
    distance_m,
    left_wall_x_m,
    sensor_forward_offset_m,
):
    """Convert a front ToF wall distance to robot-center x while facing left."""
    distance_m = float(distance_m)
    left_wall_x_m = float(left_wall_x_m)
    sensor_forward_offset_m = float(sensor_forward_offset_m)
    if not all(
        math.isfinite(value)
        for value in (distance_m, left_wall_x_m, sensor_forward_offset_m)
    ):
        raise ValueError("wall-distance geometry values must be finite")
    if distance_m <= 0.0:
        raise ValueError("wall distance must be positive")
    if sensor_forward_offset_m < 0.0:
        raise ValueError("sensor_forward_offset_m cannot be negative")
    return left_wall_x_m + distance_m + sensor_forward_offset_m


def robot_x_from_right_wall_distance(
    distance_m,
    right_wall_x_m,
    sensor_forward_offset_m,
):
    """Convert a front ToF wall distance to robot-center x while facing right."""
    distance_m = float(distance_m)
    right_wall_x_m = float(right_wall_x_m)
    sensor_forward_offset_m = float(sensor_forward_offset_m)
    if not all(
        math.isfinite(value)
        for value in (distance_m, right_wall_x_m, sensor_forward_offset_m)
    ):
        raise ValueError("wall-distance geometry values must be finite")
    if distance_m <= 0.0:
        raise ValueError("wall distance must be positive")
    if sensor_forward_offset_m < 0.0:
        raise ValueError("sensor_forward_offset_m cannot be negative")
    return right_wall_x_m - distance_m - sensor_forward_offset_m


def make_lane_tof_command(
    *,
    distance_m,
    measurement_age_s,
    robot_yaw,
    target_x,
    left_wall_x_m,
    sensor_forward_offset_m,
    transit_speed,
    minimum_speed,
    slowdown_distance_m,
    x_tolerance_m,
    measurement_timeout_s,
    heading_gain,
    max_angular_speed,
    heading_tolerance,
    wall_side="left",
    right_wall_x_m=2.0,
    wall_angle_rad=None,
    wall_angle_tolerance_rad=0.05,
    coarse_heading_aligned=False,
):
    """Align to the next lane after facing the wall in the shift direction."""
    wall_side = str(wall_side).strip().lower()
    desired_yaw = desired_yaw_for_wall(wall_side)
    heading_error = normalize_angle(desired_yaw - float(robot_yaw))
    angular_z = clamp(
        float(heading_gain) * heading_error,
        -float(max_angular_speed),
        float(max_angular_speed),
    )

    measurement_is_fresh = (
        distance_m is not None
        and measurement_age_s is not None
        and math.isfinite(float(distance_m))
        and math.isfinite(float(measurement_age_s))
        and float(distance_m) > 0.0
        and 0.0 <= float(measurement_age_s) <= float(measurement_timeout_s)
        and (
            wall_angle_rad is None
            or math.isfinite(float(wall_angle_rad))
        )
    )
    # Odometry is only a one-time coarse gate. Once both sensors face the wall,
    # ToF remains authoritative so odometry drift cannot pull alignment back.
    if (
        not bool(coarse_heading_aligned)
        and abs(heading_error) > float(heading_tolerance)
    ):
        return LaneTofCommand(
            linear_x=0.0,
            angular_z=angular_z,
            phase="ALIGN_TOF_NEXT_LANE",
            measured_robot_x=None,
            x_error=None,
            reached=False,
        )

    if not measurement_is_fresh:
        return LaneTofCommand(
            linear_x=0.0,
            angular_z=angular_z,
            phase="WAITING_FOR_LANE_TOF",
            measured_robot_x=None,
            x_error=None,
            reached=False,
        )

    if (
        wall_angle_rad is not None
        and abs(float(wall_angle_rad)) > float(wall_angle_tolerance_rad)
    ):
        return LaneTofCommand(
            linear_x=0.0,
            angular_z=clamp(
                float(heading_gain) * float(wall_angle_rad),
                -float(max_angular_speed),
                float(max_angular_speed),
            ),
            phase="ALIGN_LANE_WALL_ANGLE",
            measured_robot_x=None,
            x_error=None,
            reached=False,
        )

    wall_angular_z = (
        angular_z
        if wall_angle_rad is None
        else clamp(
            float(heading_gain) * float(wall_angle_rad),
            -float(max_angular_speed),
            float(max_angular_speed),
        )
    )

    if wall_side == "left":
        measured_robot_x = robot_x_from_left_wall_distance(
            distance_m,
            left_wall_x_m,
            sensor_forward_offset_m,
        )
    else:
        measured_robot_x = robot_x_from_right_wall_distance(
            distance_m,
            right_wall_x_m,
            sensor_forward_offset_m,
        )
    x_error = measured_robot_x - float(target_x)
    if abs(x_error) <= float(x_tolerance_m):
        return LaneTofCommand(
            linear_x=0.0,
            angular_z=0.0,
            phase="TOF_LANE_ALIGNED",
            measured_robot_x=measured_robot_x,
            x_error=x_error,
            reached=True,
        )

    slowdown_distance_m = max(float(slowdown_distance_m), float(x_tolerance_m))
    speed_scale = clamp(abs(x_error) / slowdown_distance_m, 0.0, 1.0)
    speed = max(
        float(minimum_speed),
        float(transit_speed) * speed_scale,
    )
    # Positive base velocity points toward the selected wall. Reverse if the
    # robot crossed the requested lane center.
    error_in_forward_direction = x_error if wall_side == "left" else -x_error
    linear_x = math.copysign(speed, error_in_forward_direction)
    return LaneTofCommand(
        linear_x=linear_x,
        angular_z=wall_angular_z,
        phase="TOF_SHIFT_TO_NEXT_LANE",
        measured_robot_x=measured_robot_x,
        x_error=x_error,
        reached=False,
    )
