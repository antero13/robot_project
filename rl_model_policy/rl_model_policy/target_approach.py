from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachCommand:
    linear_x: float
    angular_z: float


def aligned_approach_command(
    target_x,
    target_y,
    center_tolerance,
    grab_area_ratio,
    linear_speed,
    angular_gain,
    max_angular_speed,
):
    """Drive toward a centered target until the pickup sequence can take over."""
    if center_tolerance < 0.0:
        raise ValueError("center_tolerance must be non-negative")
    if linear_speed < 0.0:
        raise ValueError("linear_speed must be non-negative")
    if angular_gain < 0.0 or max_angular_speed < 0.0:
        raise ValueError("angular correction values must be non-negative")

    target_x = float(target_x)
    target_y = float(target_y)
    if abs(target_x) > center_tolerance:
        return None
    if target_y >= grab_area_ratio:
        return None

    angular_z = max(
        -max_angular_speed,
        min(max_angular_speed, -target_x * angular_gain),
    )
    return ApproachCommand(float(linear_speed), angular_z)
