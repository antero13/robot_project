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


def coverage_phase_allows_target_search(
    coverage_phase,
    *,
    coverage_enabled=True,
    leave_start_active=False,
    rejoin_active=False,
    main_road_alignment_active=False,
):
    """Allow target pursuit only while the robot is scanning inside a lane."""
    if (
        bool(leave_start_active)
        or bool(rejoin_active)
        or bool(main_road_alignment_active)
    ):
        return False
    if not bool(coverage_enabled):
        return True
    return str(coverage_phase) in ("SCAN_LANE_UP", "SCAN_LANE_DOWN")


def storage_repickup_guard_is_active(
    *,
    enabled,
    delivered_count,
    lane_number,
    coverage_phase,
    robot_y,
    start_y,
):
    """Block deposited objects once lane 4 passes the final legal object row."""
    return (
        bool(enabled)
        and int(delivered_count) > 0
        and int(lane_number) == 4
        and str(coverage_phase) == "SCAN_LANE_DOWN"
        and float(robot_y) <= float(start_y)
    )
