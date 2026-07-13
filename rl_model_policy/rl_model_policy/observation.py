import math


OBSERVATION_NAMES = (
    "target_visible",
    "target_x",
    "target_y",
    "time_since_target_seen_norm",
    "last_target_direction",
    "avoid_left",
    "avoid_center",
    "avoid_right",
    "nearest_avoid_x",
    "nearest_avoid_y",
    "pose_valid",
    "robot_x_norm",
    "robot_y_norm",
    "sin_yaw",
    "cos_yaw",
    "imu_yaw_rate_norm",
    "last_target_bearing_sin",
    "last_target_bearing_cos",
)
OBSERVATION_DIM = len(OBSERVATION_NAMES)


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_to_yaw(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def estimate_target_world_bearing(yaw, target_x, camera_horizontal_fov_rad):
    # Image x is positive to the right; ROS yaw is positive counter-clockwise.
    return yaw - clamp(target_x, -1.0, 1.0) * camera_horizontal_fov_rad * 0.5


def make_pose_observation(
    pose_valid,
    robot_x,
    robot_y,
    yaw,
    yaw_rate,
    last_target_bearing,
    arena_half_extent_m,
    max_angular_speed,
):
    if not pose_valid:
        return [0.0] * 8

    extent = max(float(arena_half_extent_m), 1e-6)
    angular_speed = max(abs(float(max_angular_speed)), 1e-6)
    bearing = yaw if last_target_bearing is None else last_target_bearing
    return [
        1.0,
        clamp(float(robot_x) / extent, -1.0, 1.0),
        clamp(float(robot_y) / extent, -1.0, 1.0),
        math.sin(float(yaw)),
        math.cos(float(yaw)),
        clamp(float(yaw_rate) / angular_speed, -1.0, 1.0),
        math.sin(float(bearing)),
        math.cos(float(bearing)),
    ]


def validate_observation(observation):
    if len(observation) != OBSERVATION_DIM:
        raise ValueError(
            f"Expected {OBSERVATION_DIM} observations, got {len(observation)}"
        )
    if not all(math.isfinite(float(value)) for value in observation):
        raise ValueError("Observation contains a non-finite value")
    return observation
