from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CandidatePoint:
    column: int
    row: int
    x: float
    y: float


@dataclass(frozen=True)
class ObjectEstimate:
    x: float
    y: float
    method: str
    error: float
    candidate: CandidatePoint | None


def generate_candidate_points(
    columns=7,
    rows=6,
    first_x=-1.5,
    first_y=-1.0,
    spacing=0.5,
):
    return [
        CandidatePoint(
            column=column,
            row=row,
            x=float(first_x) + column * float(spacing),
            y=float(first_y) + row * float(spacing),
        )
        for row in range(int(rows))
        for column in range(int(columns))
    ]


class GridObjectLocalizer:
    def __init__(
        self,
        candidates=None,
        horizontal_fov_deg=80.0,
        vertical_fov_deg=50.0,
        camera_height_m=0.18,
        camera_pitch_deg=15.0,
        object_center_height_m=0.04,
        max_range_m=4.5,
        max_x_error=0.35,
        max_y_error=0.22,
        fallback_snap_distance_m=0.36,
        arena_half_extent_m=2.0,
    ):
        self.candidates = list(candidates or generate_candidate_points())
        self.horizontal_fov = math.radians(float(horizontal_fov_deg))
        self.vertical_fov = math.radians(float(vertical_fov_deg))
        self.camera_height = float(camera_height_m)
        self.camera_pitch = math.radians(float(camera_pitch_deg))
        self.object_center_height = float(object_center_height_m)
        self.max_range = float(max_range_m)
        self.max_x_error = float(max_x_error)
        self.max_y_error = float(max_y_error)
        self.fallback_snap_distance = float(fallback_snap_distance_m)
        self.arena_half_extent = float(arena_half_extent_m)
        self._validate()

    def _validate(self):
        if not self.candidates:
            raise ValueError("at least one object candidate point is required")
        if self.horizontal_fov <= 0.0 or self.vertical_fov <= 0.0:
            raise ValueError("camera fields of view must be positive")
        if self.camera_height <= self.object_center_height:
            raise ValueError("camera must be above the detected object center")
        if self.max_range <= 0.0:
            raise ValueError("max_range_m must be positive")
        if self.max_x_error <= 0.0 or self.max_y_error <= 0.0:
            raise ValueError("image matching tolerances must be positive")

    def localize(self, image_x, image_y, robot_x, robot_y, robot_yaw):
        image_x = float(image_x)
        image_y = float(image_y)
        robot_x = float(robot_x)
        robot_y = float(robot_y)
        robot_yaw = float(robot_yaw)

        matches = []
        for candidate in self.candidates:
            predicted = self.predict_observation(
                candidate.x,
                candidate.y,
                robot_x,
                robot_y,
                robot_yaw,
            )
            if predicted is None:
                continue
            predicted_x, predicted_y = predicted
            x_error = abs(predicted_x - image_x)
            y_error = abs(predicted_y - image_y)
            if x_error > self.max_x_error or y_error > self.max_y_error:
                continue
            score = math.hypot(
                x_error / self.max_x_error,
                y_error / self.max_y_error,
            )
            matches.append((score, candidate))

        if matches:
            score, candidate = min(matches, key=lambda item: item[0])
            return ObjectEstimate(
                x=candidate.x,
                y=candidate.y,
                method="candidate_grid",
                error=score,
                candidate=candidate,
            )

        return self._continuous_estimate(
            image_x,
            image_y,
            robot_x,
            robot_y,
            robot_yaw,
        )

    def predict_observation(
        self,
        object_x,
        object_y,
        robot_x,
        robot_y,
        robot_yaw,
    ):
        dx = float(object_x) - float(robot_x)
        dy = float(object_y) - float(robot_y)
        distance = math.hypot(dx, dy)
        if distance <= 1e-6 or distance > self.max_range:
            return None

        relative_bearing = normalize_angle(math.atan2(dy, dx) - float(robot_yaw))
        half_horizontal_fov = self.horizontal_fov * 0.5
        if abs(relative_bearing) > half_horizontal_fov * 1.15:
            return None

        image_x = -math.tan(relative_bearing) / math.tan(half_horizontal_fov)
        down_angle = math.atan2(
            self.camera_height - self.object_center_height,
            distance,
        )
        image_y = 0.5 + 0.5 * (
            math.tan(down_angle - self.camera_pitch)
            / math.tan(self.vertical_fov * 0.5)
        )
        if image_y < -0.1 or image_y > 1.1:
            return None
        return image_x, image_y

    def _continuous_estimate(
        self,
        image_x,
        image_y,
        robot_x,
        robot_y,
        robot_yaw,
    ):
        down_angle = self.camera_pitch + math.atan(
            (image_y * 2.0 - 1.0) * math.tan(self.vertical_fov * 0.5)
        )
        if down_angle <= math.radians(1.0):
            return None

        distance = (
            self.camera_height - self.object_center_height
        ) / math.tan(down_angle)
        if distance <= 0.0 or distance > self.max_range:
            return None

        bearing = robot_yaw - math.atan(
            image_x * math.tan(self.horizontal_fov * 0.5)
        )
        estimate_x = robot_x + distance * math.cos(bearing)
        estimate_y = robot_y + distance * math.sin(bearing)
        if (
            abs(estimate_x) > self.arena_half_extent
            or abs(estimate_y) > self.arena_half_extent
        ):
            return None

        nearest = min(
            self.candidates,
            key=lambda point: math.hypot(
                point.x - estimate_x,
                point.y - estimate_y,
            ),
        )
        snap_distance = math.hypot(
            nearest.x - estimate_x,
            nearest.y - estimate_y,
        )
        if snap_distance <= self.fallback_snap_distance:
            return ObjectEstimate(
                x=nearest.x,
                y=nearest.y,
                method="projected_and_snapped",
                error=snap_distance,
                candidate=nearest,
            )
        return ObjectEstimate(
            x=estimate_x,
            y=estimate_y,
            method="ground_projection",
            error=0.0,
            candidate=None,
        )


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))
