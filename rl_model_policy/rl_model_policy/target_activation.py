def target_is_close_enough(center_y, minimum_center_y):
    minimum_center_y = float(minimum_center_y)
    if not 0.0 <= minimum_center_y <= 1.0:
        raise ValueError("minimum_center_y must be between 0 and 1")
    if center_y is None:
        return False
    return float(center_y) >= minimum_center_y
