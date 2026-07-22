from dataclasses import dataclass
import math
from typing import Optional

from rl_model_policy.coverage_controller import clamp, normalize_angle


@dataclass(frozen=True)
class MainRoadTofCommand:
    linear_x: float
    angular_z: float
    phase: str
    measured_robot_y: Optional[float]
    y_error: Optional[float]
    angle_alignment_active: bool
    reached: bool


def robot_y_from_south_wall_distance(
    distance_m,
    south_wall_y_m,
    sensor_forward_offset_m,
):
    """Convert a south-facing range into the robot-center map y coordinate."""
    distance_m = float(distance_m)
    south_wall_y_m = float(south_wall_y_m)
    sensor_forward_offset_m = float(sensor_forward_offset_m)
    values = (distance_m, south_wall_y_m, sensor_forward_offset_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("south-wall geometry values must be finite")
    if distance_m <= 0.0:
        raise ValueError("wall distance must be positive")
    if sensor_forward_offset_m < 0.0:
        raise ValueError("sensor_forward_offset_m cannot be negative")
    return south_wall_y_m + distance_m + sensor_forward_offset_m


def make_main_road_tof_command(
    *,
    distance_m,
    wall_angle_rad,
    measurement_age_s,
    robot_yaw,
    target_y,
    south_wall_y_m,
    sensor_forward_offset_m,
    transit_speed,
    minimum_speed,
    slowdown_distance_m,
    y_tolerance_m,
    measurement_timeout_s,
    angle_alignment_active,
    angle_trigger_rad,
    angle_release_rad,
    angle_gain,
    max_angular_speed,
    heading_tolerance,
):
    """Correct main-road y first, then conditionally square to the south wall.

    Separate trigger/release values can provide hysteresis. Equal values use
    one fixed angle tolerance: correction runs above it and completes within it.
    """
    angle_trigger_rad = float(angle_trigger_rad)
    angle_release_rad = float(angle_release_rad)
    if not 0.0 <= angle_release_rad <= angle_trigger_rad:
        raise ValueError("angle thresholds must satisfy 0 <= release <= trigger")

    desired_yaw = -math.pi / 2.0
    heading_error = normalize_angle(desired_yaw - float(robot_yaw))
    if abs(heading_error) > float(heading_tolerance):
        return MainRoadTofCommand(
            linear_x=0.0,
            angular_z=clamp(
                float(angle_gain) * heading_error,
                -float(max_angular_speed),
                float(max_angular_speed),
            ),
            phase="ALIGN_MAIN_ROAD_SOUTH_ODOMETRY",
            measured_robot_y=None,
            y_error=None,
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
        return MainRoadTofCommand(
            linear_x=0.0,
            angular_z=0.0,
            phase="WAITING_FOR_MAIN_ROAD_SOUTH_TOF",
            measured_robot_y=None,
            y_error=None,
            angle_alignment_active=False,
            reached=False,
        )

    measured_robot_y = robot_y_from_south_wall_distance(
        distance_m,
        south_wall_y_m,
        sensor_forward_offset_m,
    )
    y_error = measured_robot_y - float(target_y)

    # The robot is already facing south after SCAN_LANE_DOWN/RETURN_MAIN_ROAD.
    # Positive base velocity therefore decreases map y. Do not use wall angle
    # until the requested distance correction has completed.
    if abs(y_error) > float(y_tolerance_m):
        slowdown_distance_m = max(
            float(slowdown_distance_m),
            float(y_tolerance_m),
        )
        speed_scale = clamp(abs(y_error) / slowdown_distance_m, 0.0, 1.0)
        speed = max(float(minimum_speed), float(transit_speed) * speed_scale)
        return MainRoadTofCommand(
            linear_x=math.copysign(speed, y_error),
            angular_z=0.0,
            phase="TOF_CORRECT_MAIN_ROAD_DISTANCE",
            measured_robot_y=measured_robot_y,
            y_error=y_error,
            angle_alignment_active=False,
            reached=False,
        )

    wall_angle_rad = float(wall_angle_rad)
    should_align_angle = bool(angle_alignment_active) or (
        abs(wall_angle_rad) >= angle_trigger_rad
    )
    if should_align_angle and abs(wall_angle_rad) > angle_release_rad:
        return MainRoadTofCommand(
            linear_x=0.0,
            angular_z=clamp(
                float(angle_gain) * wall_angle_rad,
                -float(max_angular_speed),
                float(max_angular_speed),
            ),
            phase="ALIGN_MAIN_ROAD_SOUTH_WALL_ANGLE",
            measured_robot_y=measured_robot_y,
            y_error=y_error,
            angle_alignment_active=True,
            reached=False,
        )

    return MainRoadTofCommand(
        linear_x=0.0,
        angular_z=0.0,
        phase="MAIN_ROAD_SOUTH_TOF_ALIGNED",
        measured_robot_y=measured_robot_y,
        y_error=y_error,
        angle_alignment_active=False,
        reached=True,
    )
