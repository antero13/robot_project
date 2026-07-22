import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PoseEstimator:
    """Integrate planar linear and angular velocity into an x, y, yaw pose."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.reset(x, y, yaw)

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = normalize_angle(float(yaw))
        self.distance_travelled = 0.0
        self.rotation_travelled = 0.0

    def correct_x(self, x):
        """Apply an absolute x landmark without changing y, yaw, or counters."""
        x = float(x)
        if not math.isfinite(x):
            raise ValueError('corrected x must be finite')
        self.x = x

    def correct_y(self, y):
        """Apply an absolute y landmark without changing x, yaw, or counters."""
        y = float(y)
        if not math.isfinite(y):
            raise ValueError('corrected y must be finite')
        self.y = y

    def correct_yaw(self, yaw):
        """Apply an absolute yaw landmark without changing position or counters."""
        yaw = float(yaw)
        if not math.isfinite(yaw):
            raise ValueError('corrected yaw must be finite')
        self.yaw = normalize_angle(yaw)

    def step(
        self,
        dt,
        linear_velocity,
        angular_velocity,
        integrate_position=True,
    ):
        if dt <= 0.0:
            return

        linear_velocity = float(linear_velocity)
        angular_velocity = float(angular_velocity)
        delta_yaw = angular_velocity * dt

        # Midpoint integration is more accurate than using only the old heading on arcs.
        midpoint_yaw = self.yaw + 0.5 * delta_yaw
        distance = linear_velocity * dt if integrate_position else 0.0
        self.x += distance * math.cos(midpoint_yaw)
        self.y += distance * math.sin(midpoint_yaw)
        self.yaw = normalize_angle(self.yaw + delta_yaw)

        self.distance_travelled += abs(distance)
        self.rotation_travelled += abs(delta_yaw)
