import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class CalibrationPoint:
    norm_x: float
    norm_y: float
    lateral_m: float
    forward_m: float


@dataclass(frozen=True)
class LayerSample:
    forward_m: float
    expected_norm_y: float
    lateral_m: float


@dataclass(frozen=True)
class ObjectEstimate:
    x: float
    y: float
    lateral_m: float
    forward_m: float
    method: str
    interpolation_span_m: float


class CalibrationObjectLocalizer:
    """Interpolate camera-relative object position from measured bbox centers."""

    REQUIRED_COLUMNS = {"distance_m", "real_x_m", "norm_x", "norm_y"}

    def __init__(
        self,
        calibration_path,
        arena_half_extent_m=2.0,
        horizontal_extrapolation_margin=0.015,
        vertical_extrapolation_margin=0.012,
    ):
        self.calibration_path = Path(calibration_path)
        self.arena_half_extent = float(arena_half_extent_m)
        self.horizontal_margin = float(horizontal_extrapolation_margin)
        self.vertical_margin = float(vertical_extrapolation_margin)
        self.layers = self._load_layers(self.calibration_path)
        self._validate()

    def _validate(self):
        if len(self.layers) < 2:
            raise ValueError("calibration requires at least two distance layers")
        if self.arena_half_extent <= 0.0:
            raise ValueError("arena_half_extent_m must be positive")
        if self.horizontal_margin < 0.0 or self.vertical_margin < 0.0:
            raise ValueError("calibration extrapolation margins cannot be negative")
        for forward_m, points in self.layers:
            if forward_m <= 0.0 or len(points) < 2:
                raise ValueError(
                    "each calibration distance must contain at least two points"
                )

    @classmethod
    def _load_layers(cls, path):
        grouped = {}
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            columns = set(reader.fieldnames or [])
            missing = cls.REQUIRED_COLUMNS - columns
            if missing:
                raise ValueError(
                    "calibration CSV is missing columns: "
                    + ", ".join(sorted(missing))
                )
            for row_number, row in enumerate(reader, start=2):
                try:
                    point = CalibrationPoint(
                        norm_x=float(row["norm_x"]),
                        norm_y=float(row["norm_y"]),
                        lateral_m=float(row["real_x_m"]),
                        forward_m=float(row["distance_m"]),
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"invalid calibration value on CSV row {row_number}"
                    ) from exc
                grouped.setdefault(point.forward_m, []).append(point)

        return [
            (forward_m, sorted(points, key=lambda point: point.norm_x))
            for forward_m, points in sorted(grouped.items())
        ]

    def localize(self, image_x, image_y, robot_x, robot_y, robot_yaw):
        camera_estimate = self.interpolate_camera_position(image_x, image_y)
        if camera_estimate is None:
            return None
        lateral_m, forward_m, interpolation_span_m = camera_estimate

        yaw = float(robot_yaw)
        world_x = (
            float(robot_x)
            + forward_m * math.cos(yaw)
            + lateral_m * math.sin(yaw)
        )
        world_y = (
            float(robot_y)
            + forward_m * math.sin(yaw)
            - lateral_m * math.cos(yaw)
        )
        if (
            abs(world_x) > self.arena_half_extent
            or abs(world_y) > self.arena_half_extent
        ):
            return None

        return ObjectEstimate(
            x=world_x,
            y=world_y,
            lateral_m=lateral_m,
            forward_m=forward_m,
            method="calibration_interpolation",
            interpolation_span_m=interpolation_span_m,
        )

    def interpolate_camera_position(self, image_x, image_y):
        image_x = float(image_x)
        image_y = float(image_y)
        samples = []
        for forward_m, points in self.layers:
            interpolated = self._interpolate_layer(points, image_x)
            if interpolated is None:
                continue
            expected_norm_y, lateral_m = interpolated
            samples.append(
                LayerSample(
                    forward_m=forward_m,
                    expected_norm_y=expected_norm_y,
                    lateral_m=lateral_m,
                )
            )

        if not samples:
            return None
        exact = min(samples, key=lambda sample: abs(sample.expected_norm_y - image_y))
        if abs(exact.expected_norm_y - image_y) <= 1e-9:
            return exact.lateral_m, exact.forward_m, 0.0

        brackets = []
        for first, second in zip(samples, samples[1:]):
            low_y = min(first.expected_norm_y, second.expected_norm_y)
            high_y = max(first.expected_norm_y, second.expected_norm_y)
            if low_y <= image_y <= high_y:
                span = abs(first.forward_m - second.forward_m)
                brackets.append((span, first, second))

        if brackets:
            _, first, second = min(brackets, key=lambda item: item[0])
            return self._interpolate_between_layers(first, second, image_y)

        nearest = min(samples, key=lambda sample: abs(sample.expected_norm_y - image_y))
        if abs(nearest.expected_norm_y - image_y) <= self.vertical_margin:
            return nearest.lateral_m, nearest.forward_m, 0.0
        return None

    def _interpolate_layer(self, points, image_x):
        minimum = points[0].norm_x
        maximum = points[-1].norm_x
        if image_x < minimum - self.horizontal_margin:
            return None
        if image_x > maximum + self.horizontal_margin:
            return None

        if image_x <= minimum:
            first, second = points[0], points[1]
        elif image_x >= maximum:
            first, second = points[-2], points[-1]
        else:
            first = points[0]
            second = points[1]
            for left, right in zip(points, points[1:]):
                if left.norm_x <= image_x <= right.norm_x:
                    first, second = left, right
                    break

        denominator = second.norm_x - first.norm_x
        if abs(denominator) <= 1e-12:
            return None
        ratio = (image_x - first.norm_x) / denominator
        expected_y = _lerp(first.norm_y, second.norm_y, ratio)
        lateral_m = _lerp(first.lateral_m, second.lateral_m, ratio)
        return expected_y, lateral_m

    @staticmethod
    def _interpolate_between_layers(first, second, image_y):
        denominator = second.expected_norm_y - first.expected_norm_y
        if abs(denominator) <= 1e-12:
            ratio = 0.5
        else:
            ratio = (image_y - first.expected_norm_y) / denominator
        ratio = min(1.0, max(0.0, ratio))
        lateral_m = _lerp(first.lateral_m, second.lateral_m, ratio)
        forward_m = _lerp(first.forward_m, second.forward_m, ratio)
        return lateral_m, forward_m, abs(second.forward_m - first.forward_m)


def _lerp(first, second, ratio):
    return float(first) + (float(second) - float(first)) * float(ratio)
