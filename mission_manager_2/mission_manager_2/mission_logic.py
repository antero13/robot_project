import json
import math
from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TargetObservation:
    class_name: str
    class_id: str
    confidence: float
    x_error: float
    center_y_ratio: float
    bottom_y_ratio: float
    area_ratio: float
    height_ratio: float
    bbox_xyxy: tuple[float, float, float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_error(target: float, current: float) -> float:
    return normalize_angle(target - current)


def pose_distance(first: Pose2D, second: Pose2D) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_class_list(value: Any) -> set[str]:
    if value is None:
        return set()
    if isinstance(value, str):
        chunks = value.replace(';', ',').split(',')
    elif isinstance(value, Iterable):
        chunks = value
    else:
        chunks = [value]
    return {str(item).strip().lower() for item in chunks if str(item).strip()}


def parse_detection_payload(raw_payload: str | dict[str, Any]) -> dict[str, Any] | None:
    if isinstance(raw_payload, dict):
        return raw_payload
    try:
        payload = json.loads(raw_payload)
    except (json.JSONDecodeError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


def select_target(
    raw_payload: str | dict[str, Any],
    target_classes: set[str],
    min_confidence: float,
    locked_class: str | None = None,
) -> TargetObservation | None:
    payload = parse_detection_payload(raw_payload)
    if payload is None:
        return None

    try:
        image_width = float(payload.get('image_width', 0.0))
        image_height = float(payload.get('image_height', 0.0))
    except (TypeError, ValueError):
        return None
    if image_width <= 0.0 or image_height <= 0.0:
        return None

    observations = []
    detections = payload.get('detections', [])
    if not isinstance(detections, list):
        return None

    locked_key = locked_class.strip().lower() if locked_class else None
    for detection in detections:
        observation = _convert_detection(detection, image_width, image_height)
        if observation is None or observation.confidence < min_confidence:
            continue

        class_keys = {observation.class_name.lower(), observation.class_id.lower()}
        if target_classes and not class_keys.intersection(target_classes):
            continue
        if locked_key and locked_key not in class_keys:
            continue
        observations.append(observation)

    if not observations:
        return None

    # Prefer the visually closest target, with a small penalty for large steering.
    return max(
        observations,
        key=lambda item: (
            item.bottom_y_ratio
            + 0.20 * item.area_ratio
            - 0.20 * abs(item.x_error)
        ),
    )


def target_is_large_enough(
    target: TargetObservation,
    min_area_ratio: float,
    min_height_ratio: float,
) -> bool:
    return (
        target.area_ratio >= min_area_ratio
        and target.height_ratio >= min_height_ratio
    )


def wall_matches_expected(
    measured_distance: float,
    expected_distance: float,
    tolerance: float,
    minimum: float = 0.02,
    maximum: float = 4.0,
) -> bool:
    return (
        math.isfinite(measured_distance)
        and math.isfinite(expected_distance)
        and minimum <= measured_distance <= maximum
        and abs(measured_distance - expected_distance) <= tolerance
    )


def _convert_detection(
    detection: Any,
    image_width: float,
    image_height: float,
) -> TargetObservation | None:
    if not isinstance(detection, dict):
        return None

    bbox = detection.get('bbox_xyxy', {})
    try:
        x1 = clamp(float(bbox['x1']), 0.0, image_width)
        y1 = clamp(float(bbox['y1']), 0.0, image_height)
        x2 = clamp(float(bbox['x2']), 0.0, image_width)
        y2 = clamp(float(bbox['y2']), 0.0, image_height)
        confidence = float(detection.get('confidence', 0.0))
    except (KeyError, TypeError, ValueError):
        return None

    width = x2 - x1
    height = y2 - y1
    if width <= 0.0 or height <= 0.0:
        return None

    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    return TargetObservation(
        class_name=str(detection.get('class_name', detection.get('class_id', ''))),
        class_id=str(detection.get('class_id', '')),
        confidence=confidence,
        x_error=clamp((center_x - image_width * 0.5) / (image_width * 0.5), -1.0, 1.0),
        center_y_ratio=clamp(center_y / image_height, 0.0, 1.0),
        bottom_y_ratio=clamp(y2 / image_height, 0.0, 1.0),
        area_ratio=clamp((width * height) / (image_width * image_height), 0.0, 1.0),
        height_ratio=clamp(height / image_height, 0.0, 1.0),
        bbox_xyxy=(x1, y1, x2, y2),
    )
