from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Iterable


class Zone(str, Enum):
    OUTER_LEFT = "outer_left"
    INNER_LEFT = "inner_left"
    INNER_RIGHT = "inner_right"
    OUTER_RIGHT = "outer_right"


@dataclass(frozen=True)
class Candidate:
    class_id: str
    class_name: str
    confidence: float
    center_x: float
    center_y: float
    area_ratio: float

    @property
    def class_keys(self) -> set[str]:
        return {
            self.class_id.strip().casefold(),
            self.class_name.strip().casefold(),
        }


@dataclass(frozen=True)
class ZoneGeometry:
    point1: tuple[float, float] = (-0.8600, 0.9900)
    point2: tuple[float, float] = (-0.7600, 0.8333)
    point3: tuple[float, float] = (0.7375, 0.9933)
    point4: tuple[float, float] = (0.5825, 0.7367)

    def __post_init__(self) -> None:
        for point in (self.point1, self.point2, self.point3, self.point4):
            if len(point) != 2 or not all(math.isfinite(value) for value in point):
                raise ValueError("zone points must contain two finite coordinates")
        if math.isclose(self.point1[1], self.point2[1]):
            raise ValueError("points 1 and 2 must have different y coordinates")
        if math.isclose(self.point3[1], self.point4[1]):
            raise ValueError("points 3 and 4 must have different y coordinates")
        for y in (0.0, 1.0):
            left, right = self.boundaries_at(y)
            if not left < 0.0 < right:
                raise ValueError("zone lines must remain on opposite sides of x=0")

    def boundaries_at(self, y: float) -> tuple[float, float]:
        left = line_x_at_y(self.point1, self.point2, y)
        right = line_x_at_y(self.point3, self.point4, y)
        return left, right

    def classify(self, x: float, y: float) -> Zone:
        left, right = self.boundaries_at(y)
        if x < left:
            return Zone.OUTER_LEFT
        if x < 0.0:
            return Zone.INNER_LEFT
        if x <= right:
            return Zone.INNER_RIGHT
        return Zone.OUTER_RIGHT


@dataclass(frozen=True)
class MotionSettings:
    straight_linear_x: float = 0.10
    avoid_turn_linear_x: float = 0.06
    avoid_turn_angular_z: float = 0.45
    target_forward_linear_x: float = 0.08
    target_center_tolerance: float = 0.10
    target_angular_gain: float = 0.80
    target_min_angular_z: float = 0.10
    target_max_angular_z: float = 0.45

    def __post_init__(self) -> None:
        nonnegative = (
            self.straight_linear_x,
            self.avoid_turn_linear_x,
            self.avoid_turn_angular_z,
            self.target_forward_linear_x,
            self.target_center_tolerance,
            self.target_min_angular_z,
            self.target_max_angular_z,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
            raise ValueError("motion speeds and tolerances must be finite and nonnegative")
        if not math.isfinite(self.target_angular_gain) or self.target_angular_gain <= 0.0:
            raise ValueError("target_angular_gain must be finite and positive")
        if self.target_center_tolerance > 1.0:
            raise ValueError("target_center_tolerance cannot exceed 1.0")
        if self.target_min_angular_z > self.target_max_angular_z:
            raise ValueError("target_min_angular_z cannot exceed target_max_angular_z")


@dataclass(frozen=True)
class MotionDecision:
    linear_x: float
    angular_z: float
    mode: str
    is_target: bool = False
    zone: Zone | None = None


def parse_class_list(value: Any) -> set[str]:
    if value is None:
        return set()
    if isinstance(value, (list, tuple, set)):
        values: Iterable[Any] = value
    else:
        values = str(value).split(",")
    return {
        str(item).strip().casefold()
        for item in values
        if str(item).strip()
    }


def line_x_at_y(
    first: tuple[float, float],
    second: tuple[float, float],
    y: float,
) -> float:
    if not math.isfinite(y):
        raise ValueError("y must be finite")
    delta_y = second[1] - first[1]
    if math.isclose(delta_y, 0.0):
        raise ValueError("line points must have different y coordinates")
    ratio = (y - first[1]) / delta_y
    return first[0] + ratio * (second[0] - first[0])


def candidate_from_detection(
    detection: dict[str, Any],
    image_width: float,
    image_height: float,
    min_confidence: float,
) -> Candidate | None:
    if image_width <= 0.0 or image_height <= 0.0:
        raise ValueError("image dimensions must be positive")
    try:
        confidence = float(detection.get("confidence", 0.0))
        bbox = detection["bbox_xyxy"]
        x1 = clamp(float(bbox["x1"]), 0.0, image_width)
        y1 = clamp(float(bbox["y1"]), 0.0, image_height)
        x2 = clamp(float(bbox["x2"]), 0.0, image_width)
        y2 = clamp(float(bbox["y2"]), 0.0, image_height)
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (confidence, x1, y1, x2, y2)):
        return None
    if confidence < min_confidence or x2 <= x1 or y2 <= y1:
        return None

    center_x_px = (x1 + x2) * 0.5
    center_y_px = (y1 + y2) * 0.5
    area_ratio = ((x2 - x1) * (y2 - y1)) / (image_width * image_height)
    return Candidate(
        class_id=str(detection.get("class_id", "")),
        class_name=str(detection.get("class_name", detection.get("class_id", ""))),
        confidence=confidence,
        center_x=clamp((center_x_px / image_width) * 2.0 - 1.0, -1.0, 1.0),
        center_y=clamp(center_y_px / image_height, 0.0, 1.0),
        area_ratio=area_ratio,
    )


def select_largest_candidate(
    detections: Any,
    image_width: float,
    image_height: float,
    min_confidence: float,
) -> Candidate | None:
    if not isinstance(detections, list):
        return None
    candidates = [
        candidate
        for detection in detections
        if isinstance(detection, dict)
        for candidate in [
            candidate_from_detection(
                detection,
                image_width,
                image_height,
                min_confidence,
            )
        ]
        if candidate is not None
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item.area_ratio, item.confidence))


def decide_motion(
    candidate: Candidate | None,
    target_classes: set[str],
    geometry: ZoneGeometry,
    settings: MotionSettings,
) -> MotionDecision:
    if candidate is None:
        return MotionDecision(
            linear_x=settings.straight_linear_x,
            angular_z=0.0,
            mode="no_object_forward",
        )

    is_target = bool(candidate.class_keys & target_classes)
    if is_target:
        if abs(candidate.center_x) <= settings.target_center_tolerance:
            return MotionDecision(
                linear_x=settings.target_forward_linear_x,
                angular_z=0.0,
                mode="target_centered_forward",
                is_target=True,
            )
        angular_z = clamp(
            -settings.target_angular_gain * candidate.center_x,
            -settings.target_max_angular_z,
            settings.target_max_angular_z,
        )
        if 0.0 < abs(angular_z) < settings.target_min_angular_z:
            angular_z = math.copysign(settings.target_min_angular_z, angular_z)
        return MotionDecision(
            linear_x=0.0,
            angular_z=angular_z,
            mode="target_align",
            is_target=True,
        )

    zone = geometry.classify(candidate.center_x, candidate.center_y)
    if zone in (Zone.OUTER_LEFT, Zone.OUTER_RIGHT):
        return MotionDecision(
            linear_x=settings.straight_linear_x,
            angular_z=0.0,
            mode="avoid_outer_forward",
            zone=zone,
        )
    if zone == Zone.INNER_LEFT:
        return MotionDecision(
            linear_x=settings.avoid_turn_linear_x,
            angular_z=-settings.avoid_turn_angular_z,
            mode="avoid_inner_left_turn_right",
            zone=zone,
        )
    return MotionDecision(
        linear_x=settings.avoid_turn_linear_x,
        angular_z=settings.avoid_turn_angular_z,
        mode="avoid_inner_right_turn_left",
        zone=zone,
    )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
