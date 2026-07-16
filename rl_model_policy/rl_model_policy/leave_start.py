import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LeaveStartCommand:
    linear_x: float
    angular_z: float
    traveled_m: float
    complete: bool


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def make_leave_start_command(
    origin_x,
    origin_y,
    desired_yaw,
    robot_x,
    robot_y,
    robot_yaw,
    distance_m,
    linear_speed,
    heading_gain,
    max_angular_speed,
):
    distance_m = float(distance_m)
    linear_speed = float(linear_speed)
    heading_gain = float(heading_gain)
    max_angular_speed = float(max_angular_speed)
    if distance_m <= 0.0:
        raise ValueError("distance_m must be positive")
    if linear_speed <= 0.0:
        raise ValueError("linear_speed must be positive")
    if heading_gain < 0.0:
        raise ValueError("heading_gain must not be negative")
    if max_angular_speed < 0.0:
        raise ValueError("max_angular_speed must not be negative")

    traveled_m = math.hypot(
        float(robot_x) - float(origin_x),
        float(robot_y) - float(origin_y),
    )
    if traveled_m >= distance_m:
        return LeaveStartCommand(0.0, 0.0, traveled_m, True)

    heading_error = _normalize_angle(float(desired_yaw) - float(robot_yaw))
    angular_z = max(
        -max_angular_speed,
        min(max_angular_speed, heading_gain * heading_error),
    )
    return LeaveStartCommand(linear_speed, angular_z, traveled_m, False)
