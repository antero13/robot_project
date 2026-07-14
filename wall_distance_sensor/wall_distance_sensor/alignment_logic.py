import math


def alignment_angular_command(
    wall_angle_rad: float,
    tolerance_rad: float,
    gain: float,
    minimum_angular_z: float,
    maximum_angular_z: float,
) -> float:
    if not math.isfinite(wall_angle_rad):
        raise ValueError("wall_angle_rad must be finite")
    if tolerance_rad < 0.0:
        raise ValueError("tolerance_rad cannot be negative")
    if gain <= 0.0:
        raise ValueError("gain must be positive")
    if minimum_angular_z < 0.0:
        raise ValueError("minimum_angular_z cannot be negative")
    if maximum_angular_z <= 0.0:
        raise ValueError("maximum_angular_z must be positive")
    if minimum_angular_z > maximum_angular_z:
        raise ValueError("minimum_angular_z cannot exceed maximum_angular_z")

    if abs(wall_angle_rad) <= tolerance_rad:
        return 0.0

    command = max(
        -maximum_angular_z,
        min(maximum_angular_z, gain * wall_angle_rad),
    )
    if abs(command) < minimum_angular_z:
        command = math.copysign(minimum_angular_z, command)
    return command
