def pickup_is_ready(
    target_x,
    target_y,
    center_tolerance,
    grab_area_ratio,
):
    """Return whether visual alignment and proximity are ready for pickup."""
    if center_tolerance < 0.0:
        raise ValueError("center_tolerance must be non-negative")
    if not 0.0 <= grab_area_ratio <= 1.0:
        raise ValueError("grab_area_ratio must be between 0 and 1")
    return (
        abs(float(target_x)) <= float(center_tolerance)
        and float(target_y) >= float(grab_area_ratio)
    )
