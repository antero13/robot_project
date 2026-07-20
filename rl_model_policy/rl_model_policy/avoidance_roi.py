import math


def interpolate_roi_x(y, x1, y1, x2, y2):
    y = float(y)
    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    if abs(y2 - y1) < 1e-6:
        return (x1 + x2) * 0.5
    ratio = (y - y1) / (y2 - y1)
    return x1 + ratio * (x2 - x1)


def point_is_inside_trapezoid_roi(
    x,
    y,
    *,
    left_far_x,
    left_far_y,
    left_near_x,
    left_near_y,
    right_far_x,
    right_far_y,
    right_near_x,
    right_near_y,
):
    values = [
        x,
        y,
        left_far_x,
        left_far_y,
        left_near_x,
        left_near_y,
        right_far_x,
        right_far_y,
        right_near_x,
        right_near_y,
    ]
    values = [float(value) for value in values]
    if not all(math.isfinite(value) for value in values):
        return False

    (
        x,
        y,
        left_far_x,
        left_far_y,
        left_near_x,
        left_near_y,
        right_far_x,
        right_far_y,
        right_near_x,
        right_near_y,
    ) = values

    minimum_y = min(left_far_y, left_near_y, right_far_y, right_near_y)
    maximum_y = max(left_far_y, left_near_y, right_far_y, right_near_y)
    if y < minimum_y or y > maximum_y:
        return False

    left_x = interpolate_roi_x(
        y,
        left_far_x,
        left_far_y,
        left_near_x,
        left_near_y,
    )
    right_x = interpolate_roi_x(
        y,
        right_far_x,
        right_far_y,
        right_near_x,
        right_near_y,
    )
    if left_x > right_x:
        left_x, right_x = right_x, left_x
    return left_x <= x <= right_x
