from dataclasses import dataclass


@dataclass(frozen=True)
class NormalizedDetectionPoint:
    x: float
    y: float
    bottom_y: float

    @property
    def policy_y(self):
        """RL closeness input; intentionally separate from calibration y."""
        return self.bottom_y


def bbox_to_normalized_point(x1, y1, x2, y2, image_width, image_height):
    image_width = float(image_width)
    image_height = float(image_height)
    if image_width <= 0.0 or image_height <= 0.0:
        raise ValueError("image dimensions must be greater than zero")
    if float(x2) <= float(x1) or float(y2) <= float(y1):
        raise ValueError("bounding box must have positive width and height")

    center_x = (float(x1) + float(x2)) * 0.5
    center_y = (float(y1) + float(y2)) * 0.5
    normalized_x = (center_x - image_width * 0.5) / (image_width * 0.5)
    normalized_y = center_y / image_height
    bottom_y = float(y2) / image_height

    return NormalizedDetectionPoint(
        x=_clamp(normalized_x, -1.0, 1.0),
        y=_clamp(normalized_y, 0.0, 1.0),
        bottom_y=_clamp(bottom_y, 0.0, 1.0),
    )


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, float(value)))
