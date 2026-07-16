def target_is_close_enough(center_y, minimum_center_y):
    minimum_center_y = float(minimum_center_y)
    if not 0.0 <= minimum_center_y <= 1.0:
        raise ValueError("minimum_center_y must be between 0 and 1")
    if center_y is None:
        return False
    return float(center_y) >= minimum_center_y


def target_is_eligible(
    center_y,
    entry_minimum_center_y,
    tracking_minimum_center_y,
    tracking_active,
):
    entry_minimum_center_y = float(entry_minimum_center_y)
    tracking_minimum_center_y = float(tracking_minimum_center_y)
    if not 0.0 <= entry_minimum_center_y <= 1.0:
        raise ValueError("entry_minimum_center_y must be between 0 and 1")
    if not 0.0 <= tracking_minimum_center_y <= 1.0:
        raise ValueError("tracking_minimum_center_y must be between 0 and 1")
    if tracking_minimum_center_y > entry_minimum_center_y:
        raise ValueError(
            "tracking_minimum_center_y must not exceed entry_minimum_center_y"
        )
    minimum_center_y = (
        tracking_minimum_center_y
        if tracking_active
        else entry_minimum_center_y
    )
    return target_is_close_enough(center_y, minimum_center_y)
